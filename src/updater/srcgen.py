#!/usr/bin/env python
from collections import OrderedDict
from functools import partial

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
