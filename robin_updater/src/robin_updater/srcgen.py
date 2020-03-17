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
        self._types_map = types_map  # expanded path for types.yml
        self._templates = templates  # expanded path for templates.yml

        self.vars = []               # all the variables types to be passed (see _add_var())
        self._robin_vars = []        # variables associated with the robin publishers/subscribers

        # dict {instacing the class (always mandatory) : implementation (can be empty)}
        self._insts = collections.OrderedDict()

        # 'node' -> src code for robin_node.cpp
        # 'insts' -> src code for robin_inst.cpp
        # 'structs' -> src code for include/structs.h (C++ corresponding to the ROS msgs)
        # 'msgs' -> dicionary with the content for each msg file (msg name as keys FOR CUSTOM MSGS)
        # 'msg_pkgs' -> ROS msgs packages
        self._source = {'node': '', 'insts': '', 'structs': '',
                        'msgs': collections.OrderedDict(), 'msg_pkgs': []}

    def add_robin(self, robin):
        """Stores variable being passed and generates source for robin publisher/subscriber."""
        
        # add var
        var = robin.var
        self._add_var(var, self._robin_vars)

        ############################################################################################
        #   ASSEMBLE ROBIN PROPERTIES
        #       'type': Publisher/Subscriber
        #       'cpp': for arrays/strings, for iec equals cpp_type
        #       'len': for arrays/strings, for iec/structs(msgs) equals ''
        #       'msg': for arrays/derived checks (ex: std_msgs::Bool | robin_bridge::xpto if custom)
        # for iec corresponding ROS msg (BOOL -> std_msgs::Bool)
        #       'name': name of the robin publisher/subscriber
        #
        #   EXAMPLES:
        # custom:      {'cpp':'InputBits','msg':'robin_bridge::InputBits',
        #               'type':'Subscriber','name':'yaskawa_bits','len':''}
        # ROS msg:     {'cpp':'AccelStamped','msg':'geometry_msgs::AccelStamped',
        #               'type': 'Publisher','name':'struct_to_ros','len': ''}
        # iec:         {'cpp': 'uint8_t', 'msg': 'std_msgs::Bool', 
        #               'type': 'Publisher', 'name': 'part_for_robot', 'len': ''}
        # string :     {'cpp':'char[81]','msg':'std_msgs::String',
        #               'type':'Publisher','name':'string_varlen_array_to_ros','len':'[81]'}
        # ROS msg arr: {'cpp':'AccelStamped[2]','msg':'robin_bridge::AccelStampedArray',
        #               'type': 'Publisher','name':'strc2_arr2_ros','len':'[2]'}
        #
        ############################################################################################
        
        props = {'type': robin.ros_type, 'cpp': var.cpp_type_len, 'len': var.cpp_len,
                 'msg': var.msg_type, 'name': robin.name}

        # gets line from template, maps the parameters and adds to the dictionary for the 
        # 'node' src code - robin_node.cpp
        # **props unpacks items in dictionary, passes them as arguments
        self._source['node'] += self._templates['node']['line'].format(**props)

        # gets line from template, maps the parameters and adds to the dictionary for the 
        # 'insts' src code - robin_inst.cpp
        inst = self._templates['insts']['line'].format(**props)

        # gets implementation for the 'insts' if it does not exist
        if inst not in self._insts:
            self._insts[inst] = self._get_spec(var, robin.ros_type)

    def _add_var(self, var, robin_vars=None):
        """Stores variable, keeping track of root variables."""

        ############################################################################################
        #   FOR EXAMPLE: For yaskawa_bits(robin_bridge::YaskawaBits -> BOOL, BOOL, BYTE)
        #
        #       1 Adds first BOOL to self.vars, second BOOL does not enter
        #       2 Adds BYTE to self.vars
        #       3 Exists for loop and adds YaskawaBits to self.vars
        #       4 Also adds YaskawaBits to robin_vars
        #
        #   When there is a NORMAL CODESYS VARIABLE enters in the elif adding to robin_vars
        #
        ############################################################################################

        # add recursively for structs and arrays (the variables w/ members)
        for member in var.members:
            self._add_var(member)

        # add all the vars if not already in list
        if var not in self.vars:
            self.vars.append(var)

            # if it is associated with the robin publishers/subscribers
            if robin_vars is not None:
                robin_vars.append(var)

        # TODO: Verify conditions to make sure they cover every scenario
        # if not a struct/array
        elif not var.members:
        
            # if it is associated with the robin publishers/subscriber and not in robin_vars
            if robin_vars is not None and var not in robin_vars:
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

        # if the variable does not have a father
        if var.parent is None:
        
            # specialization not needed for root pod variables
            if var.is_pod:
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

        ############################################################################################
        #       EXAMPLE 'includes_src'
        #
        #       #include "robin_bridge/InputBits.h"                                                 
        #       #include "std_msgs/Bool.h"
        #
        ############################################################################################

        # gets template with the 'include' part (std_msgs::Bool --> std_msgs/Bool)
        includes = [self._templates['include'].format(
            var.msg_type.replace('::', '/')) for var in self._robin_vars]

        # sorts and joins everything (as the template has the newline)
        includes_src = ''.join(sorted(includes, key=lambda x: x.lower()))

        # completes code 'robin_inst.cpp' adding intanciations(keys) and implementations(values)
        self._source['insts'] = includes_src + ''.join(self._insts.values() + self._insts.keys())

        self._parse_vars()

        return self._source

    def _parse_vars(self):
        """Generates source for structs and custom messages and stores ROS message packages used."""

        # for the variables (ex: [BOOL, BYTE, YaskawaBits, AccelStamped])
        for var in self.vars:

            # add struct source (ex: AccelStamped)
            if var.xml_type == 'derived':
            
                struct_src = ''

                for member in var.members:

                    # "  {cpp} {name}{len};\n"
                    # maps with template content
                    struct_src += self._templates['structs']['line'].format(cpp=member.cpp_type, 
                                                                            name=member.name,
                                                                            len=member.cpp_len)

                # "struct {name}\n{{\n{src}}};\n"
                self._source['structs'] += self._templates['structs']['struct'].format(
                                            name=var.msg_name, src=struct_src)
            
            # add custom message definition (ex: YaskawaBits)
            if var.msg_pkg == 'robin_bridge':

                # no array
                if var.xml_type == 'derived':
                    msg_src = ''

                    for member in var.members:
                        msg_src += self._templates['msgs']['line'].format(ros=member.ros_type,
                                                                          len=member.ros_len,
                                                                          name=member.name)

                    # separating msg content acording to the msg
                    self._source['msgs'][var.msg_name] = msg_src

                # array case
                elif var.xml_type.endswith('array') and var.parent is None:

                    msg_src = self._templates['msgs']['line'].format(ros=var.ros_type,
                                                                     len=var.ros_len,
                                                                     name=var.name)

                    self._source['msgs'][var.msg_name] = msg_src
            
            # add msg_pkg to src if not already added for another var
            elif var.msg_pkg not in self._source['msg_pkgs']:

                self._source['msg_pkgs'].append(var.msg_pkg)
