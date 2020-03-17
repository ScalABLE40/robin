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

import variable


class Robin:
    """Stores information about a robin publisher/subscriber.

    :param types_map: Mapping of xml/cpp/ros/msg types
    :type types_map: dict
    :param xml_roots: XML root nodes to search for variable
    :type xml_roots: list
    :param type_: Type of the robin call (read/write)
    :type type_: str
    :param name: Name of the robin publisher/subscriber
    :type name: str
    :param var_name: Name of the variable being passed
    :type var_name: str
    """
    def __init__(self, types_map, xml_roots, type_, name, var_name):
        self.type = type_
        self.ros_type = 'Subscriber' if self.type == 'read' else 'Publisher'
        self.name = name
        self.var = variable.Variable(types_map, xml_roots, var_name)

    def __repr__(self):
        return '\nname: {}\ntype: {}\nvar:({})'.format(self.name, self.type, self.var)
