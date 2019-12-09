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
    def __init__(self, types_map, xml_roots, type_, name, var_name):
        self.type = type_
        self.ros_type = 'Subscriber' if self.type == 'read' else 'Publisher'
        self.name = name
        self.var = variable.Variable(types_map, xml_roots, name=var_name)
        print(self.var)

    def __repr__(self):
        return '\nname: {}\ntype: {}\nvar:({})'.format(self.name, self.type, self.var)
