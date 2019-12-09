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
    def get_spec(self, robin, var, level=1, path='', stamp=None):
        tpls = self.templates['specs']
        spec = ''

        # base_cpp = robin.var.members[0].cpp_type if robin.var.xml_type == 'array' else ''
        # spec = self.templates['specs'][robin.var.type][robin.type].format(
        #     cpp=robin.var.cpp_type, msg=robin.var.msg_type, base_cpp=base_cpp)
        props = {'indent': '  ' * level,
                 'type': robin.type,
                 'cpp': var.cpp_type,
                 'len': var.cpp_len,
                 'msg': var.msg_type}
        props['stamp'] = stamp if stamp is not None else int(time.time() * 10**6) % 10**8

        # get shm/msg variable namespace paths
        if var.parent is None:
            props['shm_path'] = path
            props['msg_path'] = path + '.data'
        # elif var.parent is not None and var.parent.xml_type == 'array':
        elif var.parent is not None and var.parent.xml_type == 'array':
            props['shm_path'] = props['msg_path'] = path + '[i_{}_{}]'.format(var.name, props['stamp'])
        else:
            props['shm_path'] = props['msg_path'] = path + '.' + var.name

        # if not var.is_pod:
            # get spec
            # if var.xml_type == 'string':
            #     spec = tpls[var.type][robin.type].format(**props)

            # elif var.xml_type == 'derived':
        if var.type == 'derived':# and not var.is_pod:
            for member in var.members:
                spec += self.get_spec(robin, member, level=level, path=props['shm_path'])
        else:
            if var.xml_type == 'array':
                base_var = var.members[0]
                props.update({'name': var.name, 'base_cpp': base_var.cpp_type})
                if not base_var.is_pod:
                    props['src'] = self.get_spec(robin, base_var, level=level+1,
                                                 path=props['shm_path'], stamp=props['stamp'])
                # spec = tpls[var.type][robin.type].format(**props)

            # else:
            #     spec = tpls['pod'][robin.type].format(**props)
        # elif var.parent is not None:
        #     spec = tpls['pod'][robin.type].format(**props)
        # else:#elif var.type in ['string', 'ros_type'] or var.parent is not None:
            spec = tpls[var.type][robin.type].format(**props)

        # add enclosing stuff
        if spec != '' and var.parent is None:
            props['src'] = spec
            spec = tpls['root'][robin.type].format(**props)

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
