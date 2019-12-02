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
        self.source = {'node': '', 'insts': '', 'structs': '',
                       'msgs': OrderedDict(), 'msg_pkgs': []}

    def add_robin(self, robin):
        # add var
        self.add_var(robin.var, self.robin_vars)
    
        # assemble robin properties
        props = {'type': robin.ros_type, 'cpp': robin.var.cpp_type,
                 'msg': robin.var.msg_type, 'name': robin.name}

        # add line to node src
        self.source['node'] += self.templates['node']['line'].format(**props)

        # get inst line
        inst = self.templates['insts']['line'].format(**props)
        
        # add new inst
        if inst not in self.insts:
            self.insts[inst] = self.get_spec(robin, robin.var)

    def add_var(self, var, robin_vars=None):
        # add recursively for structs and arrays
        for member in var.members:
            self.add_var(member)

        # add var
        if var not in self.vars:
            self.vars.append(var)
            if robin_vars is not None: robin_vars.append(var)

    # generates spec if robin.var is non-pod  #TODO handle all specializations
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
                     'msg': var.msg_type}

            props['shm_path'] = path + ('.' + var.name if var != robin.var else '')
            props['msg_path'] = props['shm_path'] + ('.data' if var == robin.var else '')

            if var.type == 'string':
                spec = tpls[var.type][robin.type].format(**props)

            elif var.type == 'derived':
                for member in var.members:
                    spec += self.get_spec(robin, member, level=level,
                                          path=props['shm_path'])

            elif var.type.endswith('array'):
                base_var = var.members[0]
                props.update({'name': var.name, 'base_cpp': base_var.cpp_type})
                # props['base_cpp'] = base_var.cpp_type
                if not base_var.is_pod:
                    props['src'] = self.get_spec(robin, base_var, level=level+1,
                                                 path=props['shm_path'])
                spec = tpls[var.type][robin.type].format(**props)

            else:
                spec = tpls['pod'][robin.type].format(**props)

            # add enclosing stuff
            if var == robin.var:
                spec = tpls['root'][robin.type].format(src=spec, **props)
    
        return spec

    def get_source(self):
        # self.parse_robins(robins)

        # sorted() that ignores case
        sorted_ = partial(sorted, key=lambda s: s.lower())
        
        # generate insts source
        includes = [self.templates['includes'].format(
            var.msg_type.replace('::', '/')) for var in self.robin_vars]
        includes_src = ''.join(sorted_(includes))
        self.source['insts'] = includes_src + ''.join(self.insts.values() + self.insts.keys())

        self.parse_vars()
        
        return self.source

    def parse_vars(self):
        for var in self.vars:
            # add struct source
            if var.type == 'derived':
                # src = ''.join([self.templates['structs']['line'].format(cpp=member.cpp_type, name=member.name) for member in var.members])
                struct_src = ''
                for member in var.members:
                    struct_src += self.templates['structs']['line'].format(
                        cpp=member.cpp_type, name=member.name)
                self.source['structs'] += self.templates['structs']['struct'].format(
                    name=var.cpp_type, src=struct_src)
            
            # add custom message definition
            if var.msg_pkg == 'robin':
                if var.type == 'derived':
                    # msg_src = ''.join(['{} {}\n'.format(member.ros_type, member.name) for member in var.members])
                    msg_src = ''
                    for member in var.members:
                        msg_src += self.templates['msgs']['line'].format(
                            ros=member.ros_type, name=member.name)
                elif var.type.endswith('array'):
                    # msg_src = '{} {}\n'.format(ros=var.ros_type, name=var.members[0].name)
                    msg_src = self.templates['msgs']['line'].format(
                        # ros=var.ros_type, name=var.members[0].name)
                        ros=var.ros_type, name=var.name)
                self.source['msgs'][var.msg_name] = msg_src
            
            # add msg_pkg
            elif var.msg_pkg not in self.source['msg_pkgs']:
                self.source['msg_pkgs'].append(var.msg_pkg)
