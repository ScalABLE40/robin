#!/usr/bin/env python
from collections import OrderedDict
from functools import partial

import variable

class SourceGenerator:
    """Parses robins and generates source code"""
    def __init__(self, types_map, templates, xml_root):
        self.types_map = types_map
        self.templates = templates
        self.xml_root = xml_root

        self.vars = []
        self.robin_vars = []

        self.insts = OrderedDict()
        self.source = {'node': '', 'insts': '', 'structs': '',
                       'msgs': OrderedDict(), 'msg_pkgs': []}

    def add_robin(self, robin):
        # add var
        var = robin.var
        self.add_var(var, self.robin_vars)
        # print(var)
    
        # assemble robin properties
        props = {'type': robin.ros_type, 'cpp': var.cpp_type, 'len': var.cpp_len,
                 'msg': var.msg_type, 'name': robin.name}

        # add line to node src
        self.source['node'] += self.templates['node']['line'].format(**props)

        # get inst line
        inst = self.templates['insts']['line'].format(**props)

        # add new inst
        if inst not in self.insts:
            self.insts[inst] = self.get_spec(robin, var)

    def add_var(self, var, robin_vars=None):
        # add recursively for structs and arrays
        for member in var.members:
            self.add_var(member)

        # add var
        if var not in self.vars:
            self.vars.append(var)
            if robin_vars is not None:
                robin_vars.append(var)

    # generates explicit specialization if robin.var is non-pod
    def get_spec(self, robin, var, level=1, path=''):
        tpls = self.templates['specs']
        spec = ''
        if not robin.var.is_pod:
            # base_cpp = robin.var.members[0].cpp_type if robin.var.type.endswith('array') else ''
            # spec = self.templates['specs'][robin.var.type][robin.type].format(
            #     cpp=robin.var.cpp_type, msg=robin.var.msg_type, base_cpp=base_cpp)
            props = {'indent': '  ' * level,
                     'type': robin.type,
                     'cpp': var.cpp_type,
                     'len': var.cpp_len,
                     'msg': var.msg_type}

            # get shm/msg variable namespace paths
            if var.parent is None:
                props['shm_path'] = path
                props['msg_path'] = path + '.data'
            elif isinstance(var.parent, variable.Variable) and var.parent.type.endswith('array'): 
                props['shm_path'] = props['msg_path'] = path + '[i]'
            else:
                props['shm_path'] = props['msg_path'] = path + '.' + var.name

            # get spec
            if var.type == 'string':
                spec = tpls[var.type][robin.type].format(**props)

            elif var.type == 'derived':
                for member in var.members:
                    spec += self.get_spec(robin, member, level=level, path=props['shm_path'])

            elif var.type.endswith('array'):
                base_var = var.members[0]
                props.update({'name': var.name, 'base_cpp': base_var.cpp_type})
                if not base_var.is_pod:
                    props['src'] = self.get_spec(robin, base_var, level=level+1,
                                                 path=props['shm_path'])
                spec = tpls[var.type][robin.type].format(**props)
                # print(var.name)
                # print(var.parent.name if var.parent is not None else 'None')
                # print(var.type)

            else:
                spec = tpls['pod'][robin.type].format(**props)

            # add enclosing stuff
            if var == robin.var:
                spec = tpls['root'][robin.type].format(src=spec, **props)
    
        return spec

    def get_source(self):
        # generate insts source
        includes = [self.templates['include'].format(
            var.msg_type.replace('::', '/')) for var in self.robin_vars]
        includes_src = ''.join(sorted(includes, key=lambda x: x.lower()))
        self.source['insts'] = includes_src + ''.join(self.insts.values() + self.insts.keys())

        self.parse_vars()
        return self.source

    # # sorted() that ignores case
    # def sorted(self, it)
    #      return sorted(it, key=lambda x: x.lower())

    def parse_vars(self):
        # structs_msg_types = set()
        for var in self.vars:
            # add struct source
            if var.type == 'derived':
                # src = ''.join([self.templates['structs']['line'].format(cpp=member.cpp_type, name=member.name) for member in var.members])
                struct_src = ''
                for member in var.members:
                    # if member.msg_pkg != 'robin' and member.type == 'derived':
                    #     structs_msg_types.add(member.msg_type)
                    struct_src += self.templates['structs']['line'].format(
                        cpp=member.cpp_type, name=member.name, len=member.cpp_len)
                self.source['structs'] += self.templates['structs']['struct'].format(
                    name=var.msg_name, src=struct_src)
            
            # add custom message definition
            if var.msg_pkg == 'robin':
                if var.type == 'derived':
                    msg_src = ''
                    for member in var.members:
                        msg_src += self.templates['msgs']['line'].format(
                            ros=member.ros_type, len=member.ros_len, name=member.name)
                    self.source['msgs'][var.msg_name] = msg_src
                elif var.type.endswith('array') and var.parent == None:
                    # msg_src = '{} {}\n'.format(ros=var.ros_type, name=var.members[0].name)
                    msg_src = self.templates['msgs']['line'].format(
                        # ros=var.ros_type, name=var.members[0].name)
                        ros=var.ros_type, len=var.ros_len, name=var.name)
                    self.source['msgs'][var.msg_name] = msg_src
            
            # add msg_pkg
            elif var.msg_pkg not in self.source['msg_pkgs']:
                self.source['msg_pkgs'].append(var.msg_pkg)

        # # generate struct includes
        # structs_includes = []
        # for msg_type in structs_msg_types:
        #     include = msg_type.replace('::', '/')
        #     structs_includes.append(include)

        # # generate struct includes
        # structs_includes = [self.templates['include'].format(
        #     msg_type.replace('::', '/')) for msg_type in structs_msg_types]
        
        # structs_includes = ''.join(sorted(structs_includes, key=lambda x: x.lower()))
        # self.source['structs'] = structs_includes + self.source['structs']
