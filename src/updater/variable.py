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

class Variable:
    """Extracts types for given variable from xml"""  
    CODESYS_DEF_STR_SIZE = 81
    VAR_LEN_ATTRIB = 'robin_var_len'

    # def __init__(self, types_map, xml_root, xml_node=None, name='data', xml_scope=None):
    def __init__(self, types_map, xml_root, name=None, xml_node=None, xml_scope=None, parent=None):
        self.types_map = types_map
        self.xml_root = xml_root
        self.name = name
        self.parent = parent

        if xml_scope is None:
            xml_scope = self.xml_root

        # get xml_node from name  #TODO handle variables outside of POU (eg var_name: GVL.foo)
        self.xml_node = xml_node if xml_node is not None else xml_scope.xpath(
            './/variable[@name="{}"]/type/*'.format(self.name))[0]
        
        self.members = []
        self.cpp_len = ''
        self.ros_len = ''

        self.get_types()

    def get_types(self):
        # get type from xml_node
        self.type = self.xml_node.tag

        # type can be iec, derived or array
        if self.type in self.types_map['codesys']:
            self.get_iec_types()
        elif self.type == 'derived':
            self.get_derived_types()
        elif self.type == 'array':
            self.get_array_types()
            self.is_pod = False
        else:
            raise TypeError("CODESYS data type '{}' is not supported.".format(self.var_type))
        
        # prepend msg_pkg to ros_type for custom messages
        if self.msg_pkg != 'robin' and self.type == 'derived':
            self.ros_type = self.msg_pkg + '/' + self.ros_type
        
        # # get cpp/ros declarations
        # self.cpp_decl = '{} {}{};\n'.format(self.cpp_type, self.name, self.cpp_len)
        # self.ros_decl = '{}{} {}\n'.format(self.ros_type, self.ros_len, self.name)

        # # append cpp_len to cpp_type for strings/arrays
        # self.cpp_type += self.cpp_len
    
    def get_iec_types(self):
        # get types from typemap
        self.cpp_type, self.ros_type, self.msg_type = self.types_map['codesys'][self.type]
        self.msg_pkg, self.msg_name = self.msg_type.split('::')
        
        # handle strings
        if self.type == 'string':
            attribs = self.xml_node.attrib
            str_len = int(attribs['length']) + 1 if 'length' in attribs else self.CODESYS_DEF_STR_SIZE
            self.cpp_len = '[{}]'.format(str_len)
            # self.cpp_type = self.cpp_type.format(str_len=str_len)
            self.is_pod = False
        else:
            self.is_pod = True

    def get_derived_types(self):
        base_type = self.xml_node.attrib['name']
        self.cpp_type = self.ros_type = self.msg_name = base_type

        # get msg_type
        msg_pkgs = self.types_map['ros']
        self.msg_pkg = next((pkg for pkg in msg_pkgs if base_type in msg_pkgs[pkg]), 'robin')
        self.msg_type = self.msg_pkg + '::' + self.msg_name

        # get members from struct definition  #TODO? handle array members
        xml_struct_def = self.xml_root.xpath('.//dataType[@name="{}"]'.format(base_type))[0]
        for xml_member in xml_struct_def.xpath('./baseType/struct/variable'):
            member = Variable(self.types_map, self.xml_root, xml_member.attrib['name'],
                              xml_scope=xml_struct_def, parent=self)
            self.members.append(member)

        # check for non-pod member
        self.is_pod = False not in (member.is_pod for member in self.members)

    def get_array_types(self):
        if self.parent == None:
            self.name = 'data'

        # get base var
        base_var_xml_node = self.xml_node.xpath('./baseType/*')[0]
        base_var = Variable(self.types_map, self.xml_root, xml_node=base_var_xml_node, parent=self)
        self.members.append(base_var)
        
        # get array dimensions
        dims = self.xml_node.xpath('./dimension')

        # # TODO handle multidimensional arrays
        # if base_var.type == 'array':
        #     pass
        # elif len(dims) > 1:
        #     pass

        # get array length  #TODO get upper_bound from xml; use general function to get variable value
        lower_bound = int(dims[0].attrib['lower'])
        try:
            upper_bound = int(dims[0].attrib['upper'])
        except ValueError:
            upper_bound = 20
        self.cpp_len = '[{}]'.format(upper_bound - lower_bound + 1)

        # handle variable length arrays
        is_var_len = self.xml_node.xpath(
            'count(../..//Attribute[@Name="{}"])'.format(self.VAR_LEN_ATTRIB)) == 1
        self.ros_len = self.cpp_len if not is_var_len else '[]'

        # get types
        if not base_var.is_pod:
            self.type = 'nonpod_array'
        elif is_var_len:
            self.type = 'varlen_array'
        self.cpp_type = base_var.cpp_type
        self.ros_type = base_var.ros_type
        self.msg_pkg = 'robin'
        self.msg_name = base_var.msg_name + ('VarLen' if is_var_len else '') + 'Array'
        self.msg_type = self.msg_pkg + '::' + self.msg_name

    def __eq__(self, other):
        return (isinstance(other, Variable)
                and self.type == other.type
                and self.ros_type == other.ros_type
                and self.ros_len == other.ros_len)

    def __ne__(self, other):
        return not self == other

    def __repr__(self):
        parent = self.parent.name if self.parent is not None else 'None'
        repr_ = ('\nname: {}\ntype: {}\nparent: {}\n'
                 'cpp_type: {}\nros_type: {}\nmsg_type: {}\n'
                 .format(self.name, self.type, parent,
                         self.cpp_type, self.ros_type, self.msg_type))
        repr_ += 'members:{}'.format(self.members) if len(self.members) > 0 else ''
        return repr_
