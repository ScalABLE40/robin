"""
Copyright 2019 INESC TEC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import collections
import time

import variable


class SourceGenerator:
    """Parses robins and generates source code"""
    def __init__(self, types_map, templates, xml_root):
        self.types_map = types_map
        self.templates = templates
        self.xml_root = xml_root

        self.vars = []
        self.robin_vars = []

        self.insts = collections.OrderedDict()
        self.source = {'node': '', 'insts': '', 'structs': '',
                       'msgs': collections.OrderedDict(), 'msg_pkgs': []}

    def add_robin(self, robin):
        # add var
        var = robin.var
        self.add_var(var, self.robin_vars)
    
        # assemble robin properties
        props = {'type': robin.ros_type, 'cpp': var.cpp_type_len, 'len': var.cpp_len,
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

    # generates explicit specialization
    def get_spec(self, robin, var, indent=1, idx=None, shm_path='', msg_path=''):
        if var.parent is None:
            # specialization not needed for root pod variables
            if var.is_pod:
                return ''
            # initialize idx list
            idx = [-1]

        # update index for shmlen, msglen and i variables
        if var.type in ['varlen_array', 'nonpod_array', 'nonpod_varlen_array']:
            idx += [-1] * (indent - len(idx))
            idx[indent-1] += 1

        # get indent and idx template fields as strings
        fields = {'indent': '  ' * indent,
                  'idx': '_'.join(map(str, idx))}

        # get shm/msg variable namespace paths
        if var.parent is None:
            msg_path += '.data' if var.type != 'derived' else ''
            path_suffix = ''
        elif var.parent.xml_type == 'array':
            path_suffix = '[i_{}]'.format(fields['idx'])
        else:
            path_suffix = '.' + var.name
        shm_path += path_suffix
        msg_path += path_suffix

        # get specialization source
        spec = ''
        tpls = self.templates['specs']
        if var.type == 'derived' and not var.is_pod:
            # handle structs
            for member in var.members:
                spec += self.get_spec(robin, member, indent=indent, idx=idx,
                                      shm_path=shm_path, msg_path=msg_path)
                for i in range(indent, len(idx)):
                    idx.pop()
        else:
            # handle arrays
            if var.xml_type == 'array':
                base_var = var.members[0]
                fields['base_cpp'] = base_var.cpp_type_len
                if not base_var.is_pod:
                    fields['src'] = self.get_spec(robin, base_var, indent=indent+1, idx=idx,
                                                  shm_path=shm_path, msg_path=msg_path)
            # get specialization source
            spec = tpls[var.type][robin.ros_type].format(shm_path=shm_path, msg_path=msg_path, **fields)

        # add enclosing source
        if spec != '' and var.parent is None:  #TODO first condition needed? empty struct possible?
            spec = tpls['root'][robin.ros_type].format(cpp=var.cpp_type_len, msg=var.msg_type, src=spec)

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
            if var.xml_type == 'derived':
                # src = ''.join([self.templates['structs']['line'].format(cpp=member.cpp_type, name=member.name) for member in var.members])
                struct_src = ''
                for member in var.members:
                    # if member.msg_pkg != 'robin' and member.xml_type == 'derived':
                    #     structs_msg_types.add(member.msg_type)
                    struct_src += self.templates['structs']['line'].format(
                        cpp=member.cpp_type, name=member.name, len=member.cpp_len)
                self.source['structs'] += self.templates['structs']['struct'].format(
                    name=var.msg_name, src=struct_src)
            
            # add custom message definition
            if var.msg_pkg == 'robin_bridge':
                if var.xml_type == 'derived':
                    msg_src = ''
                    for member in var.members:
                        msg_src += self.templates['msgs']['line'].format(
                            ros=member.ros_type, len=member.ros_len, name=member.name)
                    self.source['msgs'][var.msg_name] = msg_src
                elif var.xml_type.endswith('array') and var.parent == None:
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
