#!/usr/bin/env python
from collections import OrderedDict
from functools import partial
import yaml

class SourceGenerator:
    """Parses list of robins and generates source code"""
    def __init__(self, types_map, templates, xml_root):
        self.types_map = types_map
        self.templates = templates
        self.xml_root = xml_root

        self.vars = []
        self.robin_vars = []

        self.insts = OrderedDict()
        self.msg_pkgs = []
        self.source = {'node': '', 'insts': '', 'structs': '', 'msgs': OrderedDict()}

    def get_source(self, robins):
        self.parse_robins(robins)
        self.parse_vars()
        return self.source

    def parse_robins(self, robins):
        for robin in robins:
            # add var
            var = robin.var
            self.add_var(var, self.robin_vars)
        
            # assemble robin properties
            props = {'type': robin.ros_type, 'cpp': var.cpp_type,
                      'msg': var.msg_type, 'name': robin.name}

            # add line to node src
            self.source['node'] += self.templates['node']['line'].format(**props)

            # get inst line
            inst = self.templates['insts']['line'].format(**props)

            # add new inst
            if inst not in self.insts:
                # add spec if non-pod  #TODO handle all specializations
                if not var.is_pod:
                    base_cpp = var.members[0].cpp_type if var.type.endswith('array') else ''
                    spec = self.templates['specs'][var.type][robin.type].format(
                        cpp=var.cpp_type, msg=var.msg_type, base_cpp=base_cpp)
                else:
                    spec = ''
                self.insts[inst] = spec

        # sorted() that ignores case
        sorted_ = partial(sorted, key=lambda s: s.lower())
        
        # generate insts source
        includes = [self.templates['includes'].format(
            var.msg_type.replace('::', '/')) for var in self.robin_vars]
        includes_src = ''.join(sorted_(includes))
        self.source['insts'] = includes_src + ''.join(self.insts.values() + self.insts.keys())

    def add_var(self, var, robin_vars=None):
        # add recursively for structs and arrays
        for member in var.members:
            self.add_var(member)

        # add var
        if var not in self.vars:
            self.vars.append(var)
            if robin_vars is not None: robin_vars.append(var)

    def parse_vars(self):
        for var in self.vars:
            # add struct source
            if var.type == 'derived':
                src = ''.join(['  {} {};\n'.format(member.cpp_type, member.name) for member in var.members])
                self.source['structs'] += self.templates['structs']['struct'].format(name=var.cpp_type, src=src)
            
            # add custom message definition
            if var.msg_pkg == 'robin':
                if var.type == 'derived':
                    msg_src = ''.join(['{} {}\n'.format(member.ros_type, member.name) for member in var.members])
                elif var.type.endswith('array'):
                    msg_src = '{} {}\n'.format(var.ros_type, var.members[0].name)
                self.source['msgs'][var.msg_name] = msg_src
            
            # add msg_pkg
            elif var.msg_pkg not in self.msg_pkgs:
                self.msg_pkgs.append(var.msg_pkg)

# class SourceGenerator:
#     """Parses list of Robin objects and generates source code"""
#     def __init__(self, types_map, xml_root, robins):
#         self.types_map = types_map
#         self.xml_root = xml_root
#         self.structs = OrderedDict()
#         self.msgs = {}
#         self.msg_types = set()
#         self.msg_pkgs = set()
#         self.node = []
#         self.robin_inst = {}
#         self.parse_robins(robins)

#     def parse_robins(self, robins):
#         for robin in robins:
#             # add message package
#             msg_pkg = robin.msg_type.split('::')[0]
#             if msg_pkg != 'robin':
#                 self.msg_pkgs.add(msg_pkg)
#             # add struct
#             if robin.var_type == 'derived':
#                 self.add_struct(robin.cpp_type)
#             # handle array type
#             elif robin.var_type.endswith('array'):
#                 if robin.base_type == 'derived':
#                     self.add_struct(robin.base_cpp_type)
#                 msg_name = robin.msg_type.split('::')[1]
#                 self.msgs[msg_name] = '{} {}\n'.format(robin.ros_type, 'data')
#             # add robin
#             self.add_robin(robin)

#     def add_struct(self, struct_name):
#         struct_name = str(struct_name)
#         if struct_name not in self.structs:
#             struct_src, msg_src = '', ''
#             struct = self.xml_root.xpath('.//dataType[@name="{}"]'.format(struct_name))[0]
#             for member in struct.xpath('baseType/struct/variable'):
#                 var_type = member.xpath('type/*')[0].tag
#                 if var_type == 'derived':
#                     cpp_type, ros_type = [member.xpath('type/derived/@name')[0]] * 2
#                     for msg_pkg in self.types_map['ros']:
#                         if cpp_type in self.types_map['ros'][msg_pkg]:
#                             self.msg_pkgs.add(msg_pkg)
#                             break
#                     else:
#                         self.add_struct(cpp_type)
#                 else:
#                     if var_type not in self.types_map['codesys']:
#                         raise TypeError("CODESYS data type '{}' is not supported.".format(var_type))
#                     cpp_type, ros_type = self.types_map['codesys'][var_type][:2]
#                 struct_src += '\n  {} {};'.format(cpp_type, member.attrib['name'])
#                 msg_src += '{} {}\n'.format(ros_type, member.attrib['name'])
#             self.structs[struct_name] = '\nstruct {}\n{{{}\n}};'.format(struct.attrib['name'], struct_src)
#             self.msgs[struct_name] = msg_src

#     def add_robin(self, robin):
#         self.msg_types.add(robin.msg_type)
#         obj = 'Robin{}<{}, {}>'.format(
#             'Subscriber' if robin.type == 'read' else 'Publisher',
#             robin.cpp_type,
#             robin.msg_type)
#         node_src = '\n  {} {}(nh, "{}");'.format(obj, robin.name, robin.name)
#         self.node.append(node_src)
#         # add new specializations
#         inst = '\ntemplate class {};'.format(obj)
#         if inst not in self.robin_inst:
#             # TODO handle all specializations
#             spec = (self.types_map['spec_tpls'][robin.var_type][robin.type]
#                 .format(cpp_type=robin.cpp_type,
#                         msg_type=robin.msg_type,
#                         base_cpp_type=robin.base_cpp_type)
#                 if robin.var_type in self.types_map['spec_tpls'] else '')
#             self.robin_inst[inst] = spec

#     def get_source(self):
#         sorted_ = partial(sorted, key=lambda s: s.lower())
#         includes = ''.join(sorted_(['\n#include "{}.h"'.format(ros_msg.replace('::', '/')) for ros_msg in self.msg_types]))
#         node = ''.join(sorted_(self.node))
#         robin_inst = ''.join(sorted_(self.robin_inst.values()) + sorted_(self.robin_inst.keys()))  #TODO? eg dict to list, sort only once
#         structs = ''.join(self.structs.values())
#         return {'node': (includes, node),
#                 'robin_inst': (includes + robin_inst,),
#                 'structs': (structs,),
#                 'msgs': self.msgs}

#     def __repr__(self):
#         return yaml.dump(self.get_source())
