#!/usr/bin/env python
from collections import OrderedDict
from functools import partial
from lxml import etree
from time import sleep
import os
import re
import sys
import rosnode
import yaml

try:  #python2
    from StringIO import StringIO
except ImportError:  #python3
    from io import StringIO


PATHS_FILE = 'paths.yml'
NODE_NAME = 'robin'


# check catkin workspace exists
if len(sys.argv) != 2:
    print('Usage: ./update.py <path_to_catkin_ws>')
    raise SystemExit
ROBIN_WS = sys.argv[1]
if not os.path.isdir(ROBIN_WS):
    print("Catkin workspace '{}' not found.".format(ROBIN_WS))
    raise SystemExit


class SourceGenerator:
    def __init__(self):
        self.types = {}
        self.structs = OrderedDict()
        self.msgs = {}
        self.node = []
        self.robin_inst = set()

    # @staticmethod
    # def sorted(*args, **kwargs):
    #     return sorted(*args, key=lambda s: s.lower(), **kwargs)

    def add_type(self, cpp_type, ros_msg):
        if cpp_type not in self.types:
            self.types[cpp_type] = ros_msg

    def add_struct(self, struct_name):
        if struct_name not in self.structs:
            struct_src, msg_src = ['']*2
            struct = root.xpath('.//dataType[@name="{}"]'.format(struct_name))[0]
            for member in struct.xpath('baseType/struct/variable'):
                var_type = member.xpath('type/*')[0].tag
                cpp_type, ros_type = [None]*2
                if var_type == 'derived':
                    cpp_type, ros_type = [member.xpath('type/derived/@name')[0]] * 2
                    for msg_pkg in TYPES_MAP['ros']:
                        if cpp_type in TYPES_MAP['ros'][msg_pkg]:
                            msg_pkgs_used.add(msg_pkg)
                            break
                    else:
                        self.add_struct(cpp_type)
                else:
                    if var_type not in TYPES_MAP['codesys']:
                        raise TypeError("CODESYS data type '{}' is not supported.".format(var_type))
                    cpp_type, ros_type = TYPES_MAP['codesys'][var_type][:2]
                struct_src += '\n  {} {};'.format(cpp_type, member.attrib['name'])
                msg_src += '{} {}\n'.format(ros_type, member.attrib['name'])
            self.structs[struct_name] = '\nstruct {}\n{{{}\n}};'.format(struct.attrib['name'], struct_src)
            self.msgs[struct_name] = msg_src

    def add_publisher(self, var_name, var_type, msg_type):
        self.node.append('\n  RobinPublisher<{}, {}> {}("{}");'.format(var_type, msg_type, var_name, var_name))
        self.robin_inst.add('\ntemplate class RobinPublisher<{}, {}>;'.format(var_type, msg_type))

    def add_subscriber(self, var_name, var_type, msg_type):
        self.node.append('\n  RobinSubscriber<{}, {}> {}("{}");'.format(var_type, msg_type, var_name, var_name))
        self.robin_inst.add('\ntemplate class RobinSubscriber<{}, {}>;'.format(var_type, msg_type))

    def get_source(self):  #TODO prettify
        sorted_ = partial(sorted, key=lambda s: s.lower())
        includes = ''.join(sorted_(['\n#include "{}.h"'.format(ros_msg.replace('::', '/')) for ros_msg in self.types.values()]))
        node = ''.join(sorted_(self.node))
        robin_inst = list(self.robin_inst)
        robin_inst += ['\ntemplate class Robin<{}, {}>;'.format(cpp_type, ros_msg) for cpp_type, ros_msg in self.types.items()]
        robin_inst = ''.join(sorted_(self.robin_inst))
        shm_inst = ''.join(sorted_(['\ntemplate class SharedMemory<{}>;'.format(cpp_type) for cpp_type in self.types]))
        structs = ''.join(self.structs.values())
        return {'node': (includes, node),
                'robin_inst': (includes + robin_inst,),
                'shm_inst': (shm_inst,),
                'structs': (structs,),
                'msgs': self.msgs}

    def __str__(self):
        return str(self.get_source())

# load paths
def get_paths(file_path):
    with open(file_path) as file:
        paths = yaml.safe_load(file)
        for file in paths['files']:
            paths[file] = paths['folders'][file] + paths['files'][file]
            paths[file + '_tpl'] = paths['folders']['templates'] + paths['files'][file]
        paths['cmakelists'] = paths['folders']['package'] + 'CMakeLists.txt'
        paths['package'] = paths['folders']['package'] + 'package.xml'
        return paths

PATHS = get_paths(PATHS_FILE)


# load data types map
def get_types_map(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

TYPES_MAP = get_types_map(PATHS['types'])


# load xml
def get_xml_root(file_path):
    with open(file_path, 'r') as file:
        xml = file.read()
        while xml[:5] != '<?xml': xml = xml[1:]  # fix weird first character
        xml = re.sub('<\?xml version=.*encoding=.*\n', '', xml, count=1)  # remove encoding
        xml = re.sub(' xmlns=".*plcopen.org.*"', '', xml, count=1)  # remove namespace
        return etree.fromstring(xml)

root = get_xml_root(PATHS['xml'])


# parse robins from xml
robins = []
robin_objs = root.xpath('instances//variable[descendant::derived[@name="Robin"]]/@name')
for obj_name in robin_objs:
    robin_src = root.xpath('instances//addData/data/pou/body/ST/*[contains(text(), "{}();")]/text()'.format(obj_name))  #TODO handle spaces
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
# print('# ROBINS\n{}'.format(robins))
# sys.stdout.flush()


# parse msgs/structs and generate source
src_gen = SourceGenerator()
msg_pkgs_used = set()
for robin in robins:
    var_type = root.xpath('.//variable[@name="{}"]/type/*'.format(robin['var_name']))[0].tag
    cpp_type, msg_type = [None]*2
    if var_type == 'derived':
        cpp_type = root.xpath('.//variable[@name="{}"]/type/derived/@name'.format(robin['var_name']))[0]
        for msg_pkg in TYPES_MAP['ros']:
            if cpp_type in TYPES_MAP['ros'][msg_pkg]:
                msg_type = msg_pkg + '::' + cpp_type
                msg_pkgs_used.add(msg_pkg)
                break
        else:
            msg_type = 'robin::' + cpp_type
            src_gen.add_struct(cpp_type)
    else:
        if var_type not in TYPES_MAP['codesys']:
            raise TypeError("CODESYS data type '{}' is not supported.".format(var_type))
        cpp_type, msg_type = TYPES_MAP['codesys'][var_type][::2]
        msg_pkgs_used.add('std_msgs')
    src_gen.add_type(cpp_type, msg_type)
    if robin['type'] == 'read':
        src_gen.add_subscriber(robin['name'], cpp_type, msg_type)
    else:
        src_gen.add_publisher(robin['name'], cpp_type, msg_type)

source = src_gen.get_source()


# write source files
for file in PATHS['files']:
    with open(PATHS[file + '_tpl'], 'r') as template, open(PATHS[file], 'w') as src_file:
        src_file.write(template.read().format(*source[file]))
os.system('rm ' + PATHS['folders']['msg'] + '*.msg')
for msg, src in source['msgs'].items():
    with open(PATHS['folders']['msg'] + msg + '.msg', 'w') as src_file:
        src_file.write(src)


# update CMakeLists.txt
with open(PATHS['cmakelists'], 'r+') as file:
    content = file.read()
    if len(msg_pkgs_used) > 0:
        # find_package
        new_src = ('find_package(catkin REQUIRED COMPONENTS\n'
                 + '  roscpp\n'
                 + ''.join(['  ' + pkg + '\n' for pkg in msg_pkgs_used])
                 +('  message_generation\n' if len(source['msgs']) > 0 else '')
                 + ')')
        content = re.sub('find_package\s?\([^)]*roscpp[^)]*\)', new_src, content)
        # generate_messages
        new_src = ('generate_messages(\n'
                 + '  DEPENDENCIES\n'
                 + ''.join(['  ' + pkg + '\n' for pkg in msg_pkgs_used])
                 + ')')
        content = re.sub('#? ?generate_messages\s?\([^)]*\n[^)]*\)', new_src, content)
        # catkin_package
        new_src = ('\n  CATKIN_DEPENDS roscpp '
                 + ''.join([pkg + ' ' for pkg in msg_pkgs_used])
                 + 'message_runtime' if len(source['msgs']) > 0 else '')
        content = re.sub('\n#?\s*CATKIN_DEPENDS roscpp.*', new_src, content)
    if len(source['msgs']) > 0:
        # add_message_files
        new_src = ('add_message_files(\n'
                 + '  FILES\n'
                 + ''.join(['  ' + msg + '.msg\n' for msg in source['msgs']])
                 + ')')
        content = re.sub('#? ?add_message_files\s?\([^)]*\)', new_src, content)
    file.seek(0)
    file.write(content)
    file.truncate()


# update package.xml
with open(PATHS['package'], 'r+') as file:
    content = file.read()
    new_src = ('\n  <depend>roscpp</depend>\n'
             + ''.join(['  <depend>' + pkg + '</depend>\n' for pkg in msg_pkgs_used])
             +('  <build_depend>message_generation</build_depend>\n'
             + '  <exec_depend>message_runtime</exec_depend>\n' if len(source['msgs']) > 0 else '')
             + '  <exec_depend>python</exec_depend>')
    content = re.sub('\n  <depend>roscpp<\/depend>[\S\s]*<exec_depend>python<\/exec_depend>', new_src, content)
    file.seek(0)
    file.write(content)
    file.truncate()


# recompile
cmd = '''bash -c "
            cd {} &&
            . devel/setup.bash &&
            if [ -d .catkin_tools ]; then
                catkin build robin
            else
                catkin_make robin
            fi
        "'''.format(ROBIN_WS)
if os.system(cmd) != 0:
    raise RuntimeError('Failed to recompile robin package.')


def get_node_path(node_name):
    try:
        for node in rosnode.get_node_names():
            if node[-len('/' + node_name):] == '/' + node_name:
                return node
    except rosnode.ROSNodeIOException:
        print('ROS master is not running.')
    return None

# waits for a given condition to become true; interval in msec, timeout in sec
def wait_for(condition, interval=100, timeout=5):
    for i in range(0 ,timeout * 1000, interval):
        sleep(interval / 1000.0)
        if condition():
            return
    raise RuntimeError('Operation timed out.')

# if robin running, kill and rerun
node_path = get_node_path(NODE_NAME)
if node_path is None:
    print('Robin node is not running.')
else:
    if node_path not in rosnode.kill_nodes([node_path])[0]:
        raise RuntimeError("Failed to kill robin node '{}'.".format(node_path))
    wait_for(lambda: get_node_path(NODE_NAME) is None)
    cmd = '''bash -c "
                cd {} &&
                . devel/setup.bash &&
                rosrun robin robin __ns:={} &
            " > /dev/null 2>&1'''.format(ROBIN_WS, node_path[:-len('/' + NODE_NAME)])
    if os.system(cmd) != 0:
        raise RuntimeError('Failed to rerun robin node.')
    wait_for(lambda: get_node_path(NODE_NAME) is not None, timeout=10)

# os.system('pgrep codesyscontrol && pkill --signal SIGINT codesyscontrol');
# os.system('cd /var/opt/codesys && /opt/codesys/bin/codesyscontrol.bin /etc/CODESYSControl.cfg > /dev/null 2>&1 &');
# cmd = '''pgrep codesyscontrol && pkill --signal SIGINT codesyscontrol &&
#          cd /var/opt/codesys &&
#          /opt/codesys/bin/codesyscontrol.bin /etc/CODESYSControl.cfg > /dev/null 2>&1 &'''
# if os.system(cmd) != 0:
#     raise RuntimeError('Failed to rerun codesyscontrol.')

print('\nUpdate finished.')
