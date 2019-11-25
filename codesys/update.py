#!/usr/bin/env python
from collections import OrderedDict
from functools import partial
from lxml import etree
from time import sleep
import os
import re
import sys
import rosgraph
import rosnode
import yaml

try:  #python2
    from StringIO import StringIO
except ImportError:  #python3
    from io import StringIO


class RobinUpdater:
    CODESYS_DEF_STR_SIZE = 80
    NODE_NAME = 'robin'

    def __init__(self, paths_file='paths.yml', catkin_ws='~/catkin_ws'):
        print('\nGenerating source code...')
        sys.stdout.flush()
        self.paths = self.get_paths(paths_file)
        self.types_map = self.get_types_map(self.paths['types'])
        self.xml_root = self.get_xml_root(self.paths['xml'])
        self.robins = self.get_robins()
        # print('# ROBINS\n{}'.format(yaml.dump(self.robins, default_flow_style=False)))  #DEV
        # sys.stdout.flush()  #DEV
        self.source = self.get_source()
        # print('# SOURCE\n{}'.format(yaml.dump(self.source, default_flow_style=False)))  #DEV
        # sys.stdout.flush()  #DEV
        # raise SystemExit   #DEV
        self.write_source()

        print('\nRecompiling...')
        sys.stdout.flush()
        RobinUpdater.recompile(catkin_ws)

        print('\nRestarting...')
        sys.stdout.flush()
        RobinUpdater.restart_robin(RobinUpdater.NODE_NAME, catkin_ws)

        print('\nUpdate finished.')

    # loads paths
    def get_paths(self, file_path):
        with open(file_path) as file:
            paths = yaml.safe_load(file)
            for file in paths['files']:
                paths[file] = paths['folders'][file] + paths['files'][file]
                paths[file + '_tpl'] = paths['folders']['templates'] + paths['files'][file]
            paths['cmakelists'] = paths['folders']['package'] + 'CMakeLists.txt'
            paths['package'] = paths['folders']['package'] + 'package.xml'
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

    # parses robin objects and calls from xml
    def get_robins(self):
        robins = []
        robin_objs = self.xml_root.xpath('instances//variable[descendant::derived[@name="Robin"]]/@name')
        for obj_name in robin_objs:
            robin_src = self.xml_root.xpath('instances//addData/data/pou/body/ST/*[contains(text(), "{}();")]/text()'.format(obj_name))  #TODO handle spaces
            if len(robin_src) > 1:
                raise RuntimeError("Robin object '{}' used in more than one POU.".format(obj_name))
            for line in StringIO(robin_src[0]):
                pat = "^[ ]*{}[ ]*\.[ ]*(read|write)[ ]*\([ ]*'([^ ,]+)'[ ]*,[ ]*([^ ,)]+)[ ]*\)[ ]*;".format(obj_name)
                match = re.search(pat, line)
                if match is not None:
                    type_, name, var_name = match.group(1, 2, 3)
                    if None in (type_, name, var_name):
                        raise RuntimeError("Failed to parse robin in line '{}'.".format(line))
                    robins.append({'type': type_, 'name': name, 'var_name': var_name})
        if len(robins) == 0:
            raise RuntimeError('No valid robin objects found.')
        return robins

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
                        self.msg_pkgs_used.add(msg_pkg)
                        break
                else:  # its a custom message
                    msg_type = 'robin::' + cpp_type
                    src_gen.add_struct(cpp_type)
            elif var_type not in self.types_map['codesys']:  # is unsupported codesys base type
                raise TypeError("CODESYS data type '{}' is not supported.".format(var_type))
            #TODO elif var_type == 'array':
            else:  # its a supported codesys base type
                cpp_type, msg_type = self.types_map['codesys'][var_type][::2]
                self.msg_pkgs_used.add('std_msgs')
                if var_type == 'string':  # is string
                    size = var.xpath('./type/string/@length')
                    size = size[0] if len(size) > 0 else RobinUpdater.CODESYS_DEF_STR_SIZE  #TODO raise error if len(size) > 1
                    cpp_type = cpp_type.format(size=size)
            src_gen.add_type(cpp_type, msg_type)
            src_gen.add_robin(robin['name'], robin['type'], cpp_type, msg_type)
        # self.msg_pkgs_used = self.msg_pkgs_used | src_gen.msg_pkgs_used
        return src_gen.get_source()

    # writes source files
    def write_source(self):
        # write generated source to respective files
        for file in self.paths['files']:
            with open(self.paths[file + '_tpl'], 'r') as template, open(self.paths[file], 'w') as src_file:
                src_file.write(template.read().format(*self.source[file]))
        os.system('rm ' + self.paths['folders']['msg'] + '*.msg')
        for msg, src in self.source['msgs'].items():
            with open(self.paths['folders']['msg'] + msg + '.msg', 'w') as src_file:
                src_file.write(src)

        # update CMakeLists.txt
        with open(self.paths['cmakelists'], 'r+') as file:
            content = file.read()
            if len(self.msg_pkgs_used) > 0:
                # find_package
                new_src = ('find_package(catkin REQUIRED COMPONENTS\n'
                         + '  roscpp\n'
                         + ''.join(['  ' + pkg + '\n' for pkg in self.msg_pkgs_used])
                         +('  message_generation\n' if len(self.source['msgs']) > 0 else '')
                         + ')')
                content = re.sub('find_package\s?\([^)]*roscpp[^)]*\)', new_src, content)
                # generate_messages
                new_src = ('generate_messages(\n'
                         + '  DEPENDENCIES\n'
                         + ''.join(['  ' + pkg + '\n' for pkg in self.msg_pkgs_used])
                         + ')')
                content = re.sub('#? ?generate_messages\s?\([^)]*\n[^)]*\)', new_src, content)
                # catkin_package
                new_src = ('\n  CATKIN_DEPENDS roscpp '
                         + ''.join([pkg + ' ' for pkg in self.msg_pkgs_used])
                         + 'message_runtime' if len(self.source['msgs']) > 0 else '')
                content = re.sub('\n#?\s*CATKIN_DEPENDS roscpp.*', new_src, content)
            if len(self.source['msgs']) > 0:
                # add_message_files
                new_src = ('add_message_files(\n'
                         + '  FILES\n'
                         + ''.join(['  ' + msg + '.msg\n' for msg in self.source['msgs']])
                         + ')')
                content = re.sub('#? ?add_message_files\s?\([^)]*\)', new_src, content)
            file.seek(0)
            file.write(content)
            file.truncate()

        # update package.xml
        with open(self.paths['package'], 'r+') as file:
            content = file.read()
            new_src = ('\n  <depend>roscpp</depend>\n'
                     + ''.join(['  <depend>' + pkg + '</depend>\n' for pkg in self.msg_pkgs_used])
                     +('  <build_depend>message_generation</build_depend>\n'
                     + '  <exec_depend>message_runtime</exec_depend>\n' if len(self.source['msgs']) > 0 else '')
                     + '  <exec_depend>python</exec_depend>')
            content = re.sub('\n  <depend>roscpp<\/depend>[\S\s]*<exec_depend>python<\/exec_depend>', new_src, content)
            file.seek(0)
            file.write(content)
            file.truncate()

    # recompiles source robin package
    @staticmethod
    def recompile(catkin_ws):
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
    @staticmethod
    def restart_robin(node_name, catkin_ws):
        node_path = RobinUpdater.get_node_path(node_name)
        if node_path == '':
            print('Robin node is not running.')
            sys.stdout.flush()
        elif node_path is not None:
            if rosnode.rosnode_ping(node_name, max_count=3):  # if node alive
                if node_path not in rosnode.kill_nodes([node_path])[0]:  # kill node
                    raise RuntimeError("Failed to kill robin node '{}'.".format(node_path))
            RobinUpdater.wait_for(lambda: RobinUpdater.get_node_path(node_name) == '')
            cmd = '''bash -c "
                        cd {} &&
                        . devel/setup.bash &&
                        rosrun robin robin __ns:={} &
                    " > /dev/null 2>&1'''.format(catkin_ws, node_path[:-len('/' + node_name)])
                    # "'''.format(catkin_ws, node_path[:-len('/' + node_name)])
            if os.system(cmd) != 0:
                raise RuntimeError('Failed to rerun robin node.')
            RobinUpdater.wait_for(lambda: RobinUpdater.get_node_path(node_name) != '', timeout=10)

        # try to restart codesyscontrol service
        if os.system('sudo -n systemctl restart codesyscontrol > /dev/null 2>&1') != 0:
            # raise RuntimeError('Failed to restart codesyscontrol.')
            print('\nFailed to restart codesyscontrol. Please do it manually.')

    # searches for node called node_name
    @staticmethod
    def get_node_path(node_name):
        try:
            for node in rosnode.get_node_names():
                if node[-len('/' + node_name):] == '/' + node_name:
                    return node
            return ''
        except rosnode.ROSNodeIOException:
            print('ROS master is not running.')
            sys.stdout.flush()
            return None

    # waits for a given condition to become true; interval in msec, timeout in sec
    @staticmethod
    def wait_for(condition, interval=100, timeout=5):
        for i in range(0 ,timeout * 1000, interval):
            sleep(interval / 1000.0)
            if condition():
                return
        raise RuntimeError('Operation timed out.')


class SourceGenerator:
    def __init__(self, types_map, xml_root, msg_pkgs_used):
        self.types_map = types_map
        self.xml_root = xml_root
        self.types = {}
        self.structs = OrderedDict()
        self.msgs = {}
        self.node = []
        self.robin_inst = {}
        # self.msg_pkgs_used = set()

    def add_type(self, cpp_type, ros_msg):
        if cpp_type not in self.types:
            self.types[cpp_type] = ros_msg

    def add_struct(self, struct_name):
        struct_name = str(struct_name)
        if struct_name not in self.structs:
            struct_src, msg_src = '', ''
            struct = self.xml_root.xpath('.//dataType[@name="{}"]'.format(struct_name))[0]
            for member in struct.xpath('baseType/struct/variable'):
                var_type = member.xpath('type/*')[0].tag
                cpp_type, ros_type = None, None
                if var_type == 'derived':
                    cpp_type, ros_type = [member.xpath('type/derived/@name')[0]] * 2
                    for msg_pkg in self.types_map['ros']:
                        if cpp_type in self.types_map['ros'][msg_pkg]:
                            # self.msg_pkgs_used.add(msg_pkg)
                            msg_pkgs_used.add(msg_pkg)
                            break
                    else:
                        self.add_struct(cpp_type)
                else:
                    if var_type not in self.types_map['codesys']:
                        raise TypeError("CODESYS data type '{}' is not supported.".format(var_type))
                    cpp_type, ros_type = self.types_map['codesys'][var_type][:2]
                struct_src += '\n  {} {};'.format(cpp_type, member.attrib['name'])
                msg_src += '{} {}\n'.format(ros_type, member.attrib['name'])
            self.structs[struct_name] = '\nstruct {}\n{{{}\n}};'.format(struct.attrib['name'], struct_src)
            self.msgs[struct_name] = msg_src

    def add_robin(self, name, type_, var_type, msg_type):
        obj = 'Robin{}<{}, {}>'.format('Subscriber' if type_ == 'read' else 'Publisher',
                                       var_type,
                                       msg_type)
        node_src = '\n  {} {}(nh, "{}");'.format(obj, name, name)
        self.node.append(node_src)
        instantiation = '\ntemplate class {};'.format(obj)
        if instantiation not in self.robin_inst:
            specialization = ''
            if msg_type == 'std_msgs::String':
                specialization = self.types_map['spec_tpls'][msg_type][type_].format(type=var_type)
            self.robin_inst[instantiation] = specialization

    def get_source(self):  #TODO prettify
        sorted_ = partial(sorted, key=lambda s: s.lower())
        includes = ''.join(sorted_(['\n#include "{}.h"'.format(ros_msg.replace('::', '/')) for ros_msg in self.types.values()]))
        node = ''.join(sorted_(self.node))
        robin_inst = ''.join(sorted_(self.robin_inst.values()) + sorted_(self.robin_inst.keys()))  #TODO? eg dict to list, sort only once
        structs = ''.join(self.structs.values())
        return {'node': (includes, node),
                'robin_inst': (includes + robin_inst,),
                'structs': (structs,),
                'msgs': self.msgs}

    def __str__(self):
        return str(yaml.dump(self.get_source()))


if __name__ == '__main__':
    # check catkin workspace was passed as argument
    if len(sys.argv) != 2:
        print('Usage: ./update.py <path_to_catkin_ws>')
        raise SystemExit
    catkin_ws = sys.argv[1] + ('/' if not sys.argv[1].endswith('/') else '')

    # check catkin workspace exists
    if not os.path.isdir(catkin_ws):
        print("Folder '{}' not found.".format(catkin_ws))
        raise SystemExit

    # run updater
    RobinUpdater(catkin_ws=catkin_ws)
