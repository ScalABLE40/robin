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

import variable


class SourceGenerator:
    """Parses robin publishers/subscribers and generates source code

    :param types_map: Mapping of xml/cpp/ros/msg types
    :type types_map: dict
    :param templates: Templates to generate source code
    :type templates: dict
    """
    def __init__(self, types_map, templates):
        self._types_map = types_map
        self._templates = templates

        self.vars = []
        self._robin_vars = []

        self._insts = collections.OrderedDict()
        self._source = {'node': '', 'insts': '', 'structs': '',
                       'msgs': collections.OrderedDict(), 'msg_pkgs': []}

    def add_robin(self, robin):
        """Stores variable being passed and generates source for robin publisher/subscriber."""
        # add var
        var = robin.var
        self._add_var(var, self._robin_vars)
    
        # assemble robin properties
        props = {'type': robin.ros_type, 'cpp': var.cpp_type_len, 'len': var.cpp_len,
                 'msg': var.msg_type, 'name': robin.name}

        # add line to node src
        self._source['node'] += self._templates['node']['line'].format(**props)

        # get inst line
        inst = self._templates['insts']['line'].format(**props)

        # add new inst
        if inst not in self._insts:
            self._insts[inst] = self._get_spec(var, robin.ros_type)

    def _add_var(self, var, robin_vars=None):
        """Stores variable, keeping track of root variables."""
        # add recursively for structs and arrays
        for member in var.members:
            self._add_var(member)

        # add var
        if var not in self.vars:
            self.vars.append(var)
            if robin_vars is not None:
                robin_vars.append(var)

    def _get_spec(self, var, robin_type, indent=1, idx=None, shm_path='', msg_path=''):
        """Generates template specialization for read/write() function if needed.

        :param var: Variable to process 
        :type var: variable.Variable
        :param robin_type: Robin type (publisher/subscriber)
        :type robin_type: str
        :param indent: Indentation level for current variable
        :type indent: int
        :param idx: Indexes for shm_len, msg_len and i variables
        :type idx: list
        :param shm_path: Shared memory variable namespace path
        :type shm_path: str
        :param msg_path: ROS message variable namespace path
        :type msg_path: str
        :return: Specialization for var and ros_type
        :rtype: str
        """
        if var.parent is None:
            if var.is_pod:  # specialization not needed for root pod variables
                return ''
            idx = [-1]  # initialize idx list

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
        tpls = self._templates['specs']
        if var.type == 'derived' and not var.is_pod:
            # handle structs
            for member in var.members:
                spec += self._get_spec(member, robin_type, indent=indent, idx=idx,
                                      shm_path=shm_path, msg_path=msg_path)
                # for i in range(indent, len(idx)):
                #     idx.pop()
                idx = idx[:indent]
        else:
            # handle arrays
            if var.xml_type == 'array':
                base_var = var.members[0]
                fields['base_cpp'] = base_var.cpp_type_len
                if not base_var.is_pod:
                    fields['src'] = self._get_spec(base_var, robin_type, indent=indent+1, idx=idx,
                                                  shm_path=shm_path, msg_path=msg_path)
            # get specialization source
            spec = tpls[var.type][robin_type].format(shm_path=shm_path, msg_path=msg_path, **fields)

        # add enclosing source
        if spec != '' and var.parent is None:  #TODO first condition needed? empty struct possible?
            spec = tpls['root'][robin_type].format(cpp=var.cpp_type_len, msg=var.msg_type, src=spec)

        return spec

    def get_source(self):
        """Returns dictionary with source code components."""
        # generate insts source
        includes = [self._templates['include'].format(
            var.msg_type.replace('::', '/')) for var in self._robin_vars]
        includes_src = ''.join(sorted(includes, key=lambda x: x.lower()))
        self._source['insts'] = includes_src + ''.join(self._insts.values() + self._insts.keys())

        self._parse_vars()
        return self._source

    def _parse_vars(self):
        """Generates source for structs and custom messages and stores ROS message packages used."""
        for var in self.vars:
            # add struct source
            if var.xml_type == 'derived':
                # src = ''.join([self._templates['structs']['line'].format(cpp=member.cpp_type, name=member.name) for member in var.members])
                struct_src = ''
                for member in var.members:
                    struct_src += self._templates['structs']['line'].format(
                        cpp=member.cpp_type, name=member.name, len=member.cpp_len)
                self._source['structs'] += self._templates['structs']['struct'].format(
                    name=var.msg_name, src=struct_src)
            
            # add custom message definition
            if var.msg_pkg == 'robin_bridge':
                if var.xml_type == 'derived':
                    msg_src = ''
                    for member in var.members:
                        msg_src += self._templates['msgs']['line'].format(
                            ros=member.ros_type, len=member.ros_len, name=member.name)
                    self._source['msgs'][var.msg_name] = msg_src
                elif var.xml_type.endswith('array') and var.parent is None:
                    msg_src = self._templates['msgs']['line'].format(
                        ros=var.ros_type, len=var.ros_len, name=var.name)
                    self._source['msgs'][var.msg_name] = msg_src
            
            # add msg_pkg
            elif var.msg_pkg not in self._source['msg_pkgs']:
                self._source['msg_pkgs'].append(var.msg_pkg)
