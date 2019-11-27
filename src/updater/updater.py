#!/usr/bin/env python
from lxml import etree
from time import sleep
import os
import re
import sys
import rosgraph
import rosnode
import yaml

import robin
import srcgen

try:  #python2
    from StringIO import StringIO
except ImportError:  #python3
    from io import StringIO

# DEV = True

def print_(msg):
    print(msg)
    sys.stdout.flush()


class RobinUpdater:
    DEF_NODE_NAME = 'robin'
    DEF_CATKIN_WS = '~/catkin_ws'

    def __init__(self, paths_file='paths.yml', catkin_ws=DEF_CATKIN_WS):
        print_('\nGenerating source code...')
        self.paths = self.get_paths(paths_file)
        self.types_map = self.get_types_map(self.paths['types'])
        self.xml_root = self.get_xml_root(self.paths['xml'])
        self.robins = self.get_robins()
        if 'DEV' in globals() and DEV:
            print_('\n# ROBINS\n' + str(self.robins))
            # raise SystemExit  #DEV
        
        self.src_gen = srcgen.RobinSourceGenerator(self.types_map, self.xml_root, self.robins)
        self.source = self.src_gen.get_source()
        if 'DEV' in globals() and DEV:
            print_('\n# SOURCE\n{}'.format(self.source))
            # raise SystemExit  #DEV
        
        self.write_source()
        print_('\nRecompiling...')
        self.recompile_robin(catkin_ws)

        print_('\nRestarting...')
        self.restart_robin(self.DEF_NODE_NAME, catkin_ws)

        print_('\nUpdate finished.')

    # loads paths
    def get_paths(self, file_path):
        with open(file_path) as file:
            paths = yaml.safe_load(file)
            root_path = paths['folders']['package']['root']
            for file in paths['files']:
                paths[file] = (root_path
                               + paths['folders']['package'][file]
                               + paths['files'][file])
                paths[file + '_tpl'] = (paths['folders']['templates']
                                        + paths['files'][file])
            paths['msg'] = (root_path
                            + paths['folders']['package']['msg'])
            paths['cmakelists'] = (root_path
                                   + 'CMakeLists.txt')
            paths['package'] = (root_path
                                + 'package.xml')
            return paths

    # loads data types map
    def get_types_map(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

    # loads xml
    def get_xml_root(self, file_path):
        with open(file_path, 'r') as file:
            xml = file.read()
            while xml[:5] != '<?xml': xml = xml[1:]  # fix weird first character
            xml = re.sub('<\?xml version=.*encoding=.*\n', '', xml, count=1)  # remove encoding
            xml = re.sub(' xmlns=".*plcopen.org.*"', '', xml, count=1)  # remove namespace
            return etree.fromstring(xml)

    # parses robins from robin objects in xml
    def get_robins(self):
        robins = []
        robin_objs = self.xml_root.xpath('instances//variable[descendant::derived[@name="Robin"]]/@name')
        for obj_name in robin_objs:
            src = self.xml_root.xpath('instances//addData/data/pou/body/ST/*[contains(text(), "{}();")]/text()'.format(obj_name))  #TODO handle spaces
            if len(src) == 0:
                print_("Warning: no source found for robin object '{}'.".format(obj_name))
            elif len(src) > 1:
                raise RuntimeError("Robin object '{}' used in more than one POU.".format(obj_name))
            for line in StringIO(src[0]):
                robins += self.get_robin_from_call(line, obj_name)
        if len(robins) == 0:
            raise RuntimeError('No valid robin objects found.')
        return robins

    # parses robin from robin call
    def get_robin_from_call(self, src, name):
        pat = "^[ ]*{}[ ]*\.[ ]*(read|write)[ ]*\([ ]*'([^ ,]+)'[ ]*,[ ]*([^ ,)]+)[ ]*\)[ ]*;".format(name)
        match = re.search(pat, src)
        if match is None:
            return []
        type_, name, var_name = match.group(1, 2, 3)
        if None in (type_, name, var_name):
            raise RuntimeError("Failed to parse robin call in '{}'.".format(src))
        return [robin.Robin(self.types_map, self.xml_root, type_, name, var_name)]

    # parses msgs/structs and generates source
    def get_source(self):
        self.msg_pkgs_used = set()
        src_gen = SourceGenerator(self.types_map, self.xml_root, self.msg_pkgs_used)
        for robin in self.robins:
            #TODO handle variables outside of POU (eg var_name: GVL.foo)
            var = self.xml_root.xpath('.//variable[@name="{}"]'.format(robin['var_name']))[0]
            var_type = var.xpath('./type/*')[0].tag
            cpp_type, msg_type = None, None
            if var_type == 'derived':  # is not codesys base type
                cpp_type = self.xml_root.xpath('.//variable[@name="{}"]/type/derived/@name'.format(robin['var_name']))[0]
                for msg_pkg in self.types_map['ros']:  # each standard ros message package
                    if cpp_type in self.types_map['ros'][msg_pkg]:  # in message package
                        msg_type = msg_pkg + '::' + cpp_type
                        self.msg_pkgs_used.add(msg_pkg)  # <--!!!
                        break
                else:  # its a custom message
                    msg_type = 'robin::' + cpp_type
                    src_gen.add_struct(cpp_type)  # <--!!!
            elif var_type not in self.types_map['codesys']:  # is unsupported codesys base type
                raise TypeError("CODESYS data type '{}' is not supported.".format(var_type))
            #TODO elif var_type == 'array':
            else:  # its a supported codesys base type
                cpp_type, msg_type = self.types_map['codesys'][var_type][::2]
                self.msg_pkgs_used.add('std_msgs')  # <--!!!
                if var_type == 'string':  # is string
                    size = var.xpath('./type/string/@length')
                    size = size[0] if len(size) > 0 else RobinUpdater.CODESYS_DEF_STR_SIZE  #TODO raise error if len(size) > 1
                    cpp_type = cpp_type.format(size=size)
            src_gen.add_type(cpp_type, msg_type)  # <--!!!
            src_gen.add_robin(robin['name'], robin['type'], cpp_type, msg_type)  # <--!!!
        # self.msg_pkgs_used = self.msg_pkgs_used | src_gen.msg_pkgs_used
        return src_gen.get_source()

    # writes source files
    def write_source(self):
        # write generated source to respective files
        for file in self.paths['files']:
            with open(self.paths[file + '_tpl'], 'r') as template, open(self.paths[file], 'w') as src_file:
                src_file.write(template.read().format(*self.source[file]))
        os.system('rm ' + self.paths['msg'] + '*.msg')
        for msg, src in self.source['msgs'].items():
            with open(self.paths['msg'] + msg + '.msg', 'w') as src_file:
                src_file.write(src)
        self.update_cmakelists(self.paths['cmakelists'], self.src_gen.msg_pkgs, self.source['msgs'])
        self.update_package_xml(self.paths['package'], self.src_gen.msg_pkgs, self.source['msgs'])

    # updates CMakeLists.txt
    @staticmethod
    def update_cmakelists(cmakelists_path, msg_pkgs, msgs):
        with open(cmakelists_path, 'r+') as file:
            content = file.read()
            if len(msg_pkgs) > 0:
                # find_package
                new_src = ('find_package(catkin REQUIRED COMPONENTS\n'
                         + '  roscpp\n'
                         + ''.join(['  ' + pkg + '\n' for pkg in msg_pkgs])
                         +('  message_generation\n' if len(msgs) > 0 else '')
                         + ')')
                content = re.sub('find_package\s?\([^)]*roscpp[^)]*\)', new_src, content)
                # generate_messages
                new_src = ('generate_messages(\n'
                         + '  DEPENDENCIES\n'
                         + ''.join(['  ' + pkg + '\n' for pkg in msg_pkgs])
                         + ')')
                content = re.sub('#? ?generate_messages\s?\([^)]*\n[^)]*\)', new_src, content)
                # catkin_package
                new_src = ('\n  CATKIN_DEPENDS roscpp '
                         + ''.join([pkg + ' ' for pkg in msg_pkgs])
                         + 'message_runtime' if len(msgs) > 0 else '')
                content = re.sub('\n#?\s*CATKIN_DEPENDS roscpp.*', new_src, content)
            if len(msgs) > 0:
                # add_message_files
                new_src = ('add_message_files(\n'
                         + '  FILES\n'
                         + ''.join(['  ' + msg + '.msg\n' for msg in msgs])
                         + ')')
                content = re.sub('#? ?add_message_files\s?\([^)]*\)', new_src, content)
            file.seek(0)
            file.write(content)
            file.truncate()

    # updates package.xml
    @staticmethod
    def update_package_xml(package_xml_path, msg_pkgs, msgs):
        with open(package_xml_path, 'r+') as file:
            content = file.read()
            new_src = ('\n  <depend>roscpp</depend>\n'
                     + ''.join(['  <depend>' + pkg + '</depend>\n' for pkg in msg_pkgs])
                     +('  <build_depend>message_generation</build_depend>\n'
                     + '  <exec_depend>message_runtime</exec_depend>\n' if len(msgs) > 0 else '')
                     + '  <exec_depend>python</exec_depend>')
            content = re.sub('\n  <depend>roscpp<\/depend>[\S\s]*<exec_depend>python<\/exec_depend>', new_src, content)
            file.seek(0)
            file.write(content)
            file.truncate()

    # recompiles robin package
    @staticmethod
    def recompile_robin(catkin_ws=DEF_CATKIN_WS):
        cmd = '''bash -c "
                    cd {} &&
                    . devel/setup.bash &&
                    build_robin()
                    {{
                        if [ -d .catkin_tools ]; then
                            catkin build robin
                        else
                            catkin_make robin
                        fi
                    }}
                    set -o pipefail
                    build_robin 2>&1 >/dev/null |
                    sed 's/\\x1b\[[0-9;]*[mK]//g'
                "'''.format(catkin_ws)
        if os.system(cmd) != 0:
            raise RuntimeError('Failed to recompile robin package.')

    # restarts robin bridge
    @classmethod
    def restart_robin(cls, node_name=DEF_NODE_NAME, catkin_ws=DEF_CATKIN_WS):
        node_path = cls.get_node_path(node_name)
        if node_path == '':
            print_('Robin node is not running.')
        elif node_path is not None:
            if rosnode.rosnode_ping(node_name, max_count=3):  # if node alive
                if node_path not in rosnode.kill_nodes([node_path])[0]:  # kill node
                    raise RuntimeError("Failed to kill robin node '{}'.".format(node_path))
            cls.wait_for(lambda: cls.get_node_path(node_name) == '')
            cmd = '''bash -c "
                        cd {} &&
                        . devel/setup.bash &&
                        rosrun robin robin __ns:={} &
                    " > /dev/null 2>&1'''.format(catkin_ws, node_path[:-len('/' + node_name)])
            if os.system(cmd) != 0:
                raise RuntimeError('Failed to rerun robin node.')
            cls.wait_for(lambda: cls.get_node_path(node_name) != '', timeout=10)

        # try to restart codesyscontrol service
        if os.system('sudo -n systemctl restart codesyscontrol > /dev/null 2>&1') != 0:
            print_('\nFailed to restart codesyscontrol. Please do it manually.')

    # searches for node called node_name
    @staticmethod
    def get_node_path(node_name):
        try:
            for node in rosnode.get_node_names():
                if node[-len('/' + node_name):] == '/' + node_name:
                    return node
            return ''
        except rosnode.ROSNodeIOException:
            print_('ROS master is not running.')
            return None

    # waits for a given condition to become true; interval in msec, timeout in sec
    @staticmethod
    def wait_for(condition, interval=100, timeout=5):
        for i in range(0 ,timeout * 1000, interval):
            sleep(interval / 1000.0)
            if condition():
                return
        raise RuntimeError('Operation timed out.')


if __name__ == '__main__':
    # check catkin workspace was passed as argument
    if len(sys.argv) != 2:
        print_('Usage: ./update.py <path_to_catkin_ws>')
        raise SystemExit
    catkin_ws = sys.argv[1] + ('/' if not sys.argv[1].endswith('/') else '')

    # check catkin workspace exists
    if not os.path.isdir(catkin_ws):
        print_("Folder '{}' not found.".format(catkin_ws))
        raise SystemExit

    # run updater
    RobinUpdater(catkin_ws=catkin_ws)
