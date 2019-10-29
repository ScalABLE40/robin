#!/usr/bin/env python
import os
import re
import rosnode
import yaml
from collections import OrderedDict
from lxml import etree


class SourceComposer:
    def __init__(self):
        self.types = {}
        self.structs = OrderedDict()
        self.msgs = {}
        self.node = []
        self.robin_inst = []

    @staticmethod
    def sorted(*args, **kwargs):
        return sorted(*args, key=lambda s: s.lower(), **kwargs)

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
                    #TODO detect standard ros msgs
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
        self.robin_inst.append('\ntemplate class RobinPublisher<{}, {}>;'.format(var_type, msg_type))

    def add_subscriber(self, var_name, var_type, msg_type):
        self.node.append('\n  RobinSubscriber<{}, {}> {}("{}");'.format(var_type, msg_type, var_name, var_name))
        self.robin_inst.append('\ntemplate class RobinSubscriber<{}, {}>;'.format(var_type, msg_type))

    def get_source(self):
        includes = ''.join(SourceComposer.sorted(['\n#include "{}.h"'.format(ros_msg.replace('::', '/')) for ros_msg in self.types.values()]))
        node = ''.join(SourceComposer.sorted(self.node))
        self.robin_inst += ['\ntemplate class Robin<{}, {}>;'.format(cpp_type, ros_msg) for cpp_type, ros_msg in self.types.items()]
        robin_inst = ''.join(SourceComposer.sorted(self.robin_inst))
        shm_inst = ''.join(SourceComposer.sorted(['\ntemplate class SharedMemory<{}>;'.format(cpp_type) for cpp_type in self.types]))
        structs = ''.join(self.structs.values())
        #TODO prettify code
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
#         return yaml.safe_load(file)
        paths = yaml.safe_load(file)
        for file in paths['files']:
            paths[file] = paths['folders'][file] + paths['files'][file]
            paths[file + '_tpl'] = paths['folders']['templates'] + paths['files'][file]
        paths['cmakelists'] = paths['folders']['package'] + 'CMakeLists.txt'
        paths['package'] = paths['folders']['package'] + 'package.xml'
        return paths

PATHS = get_paths('paths.yml')
# print('# PATHS\n{}'.format(PATHS))


# load data types map
def get_types_map(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

TYPES_MAP = get_types_map(PATHS['types'])
# print('# TYPES_MAP\n{}'.format(TYPES_MAP))


# load xml
def get_xml_root(file_path):
    with open(file_path, 'r') as file:
        xml = file.read()
        while xml[:5] != '<?xml': xml = xml[1:]  # fix weird first character
        xml = re.sub('<\?xml version=.*encoding=.*\n', '', xml, count=1)  # remove encoding
        xml = re.sub(' xmlns=".*plcopen.org.*"', '', xml, count=1)  # remove namespace
        return etree.fromstring(xml)

root = get_xml_root(PATHS['xml'])
robins = root.xpath('instances//variable[descendant::derived[@name="RobinReader" or @name="RobinWriter"]]')


# parse xml
composer = SourceComposer()
for robin in robins:
    name = robin.xpath('addData/data/InputAssignments/InputAssignment[1]/Value/text()')[0][1:-1]
    var_name = robin.xpath('addData/data/InputAssignments/InputAssignment[2]/Value/text()')[0][4:-1]
    var_type = root.xpath('.//variable[@name="{}"]/type/*'.format(var_name))[0].tag
    cpp_type, msg_type = [None]*2
    if var_type == 'derived':
        cpp_type = root.xpath('.//variable[@name="{}"]/type/derived/@name'.format(var_name))[0]
        #TODO detect standard ros msgs
#         for msg in ros_msgs:
#             if cpp_type == msg:
#                 msg_type = msg + '::' + cpp_type
#         else:
        composer.add_struct(cpp_type)
        msg_type = 'robin::' + cpp_type
    else:
        if var_type not in TYPES_MAP['codesys']:
            raise TypeError("CODESYS data type '{}' is not supported.".format(var_type))
        cpp_type, msg_type = TYPES_MAP['codesys'][var_type][::2]
    composer.add_type(cpp_type, msg_type)
    if robin.xpath('type/derived/@name')[0] == 'RobinReader':
        composer.add_subscriber(name, cpp_type, msg_type)
    elif robin.xpath('type/derived/@name')[0] == 'RobinWriter':
        composer.add_publisher(name, cpp_type, msg_type)
source = composer.get_source()
# print('# SOURCE\n{}'.format(source))


# write source files
for file in PATHS['files']:
    with open(PATHS[file + '_tpl'], 'r') as template, open(PATHS[file], 'w') as src_file:
        src_file.write(template.read().format(*source[file]))
for msg, src in source['msgs'].items():
    with open(PATHS['folders']['msg'] + msg + '.msg', 'w') as src_file:
        src_file.write(src)


if len(source['msgs']) > 0:
    # update CMakeLists.txt
    with open(PATHS['cmakelists'], 'r+') as file:
        content = file.read()
        new_src = 'add_message_files(\n  FILES\n  {}\n)'.format('\n  '.join([msg + '.msg' for msg in source['msgs']]))
        content = re.sub('add_message_files\([^)]*FILES[^)]*\)', new_src, content)
        file.seek(0)
        file.write(content)
        file.truncate()
    #TODO support ros standard messages
        #TODO update package.xml


# recompile
ret = os.system('''
    bash -c "cd {}../../ &&
    . devel/setup.bash &&
    if [ -d .catkin_tools ]; then
        catkin build robin
    else
        catkin_make robin
    fi"'''.format(PATHS['folders']['package']))
if ret != 0: raise RuntimeError('Failed to recompile robin package.')

# if robin running, kill and rerun
try:
    for node in rosnode.get_node_names():
        if node[-6:] == '/robin':
            namespace = '__ns:={}'.format(node[:-6]) if len(node) > 6 else ''
            if node not in rosnode.kill_nodes([node])[0]:
                raise RuntimeError("Failed to kill robin node ''.".format(node))
            if os.system('''
                    bash -c "cd {}../../ &&
                    . devel/setup.bash &&
                    rosrun robin robin {} &"'''.format(PATHS['folders']['package'], namespace)) != 0:
                raise RuntimeError('Failed to rerun robin node.')
            break
    else:
        print('Robin node is not running.')
except rosnode.ROSNodeIOException as e:
    print('ROS master is not running.')

#TODO? add print statements to provide more feedback
print('\nUpdate finished.')

#TODO encapsulate everything in class(es)
