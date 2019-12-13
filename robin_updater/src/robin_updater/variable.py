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
    """Stores information about a variable and its members (for structs) or its base variable (for arrays).

    :param types_map: Mapping of xml/cpp/ros/msg types
    :type types_map: dict
    :param xml_roots: XML root nodes to search for variable
    :type xml_roots: list
    :param name: Name of the variable
    :type name: str
    :param xml_node: XML node of the variable
    :type xml_node: lxml.etree._Element
    :param xml_scopes: XML parent nodes to search for variable
    :type xml_scopes: list
    :param parent: Parent variable
    :type parent: variable.Variable
    """
    CODESYS_DEF_STR_SIZE = 81
    VAR_LEN_ATTRIB = 'robin_var_len'

    # def __init__(self, types_map, xml_roots, xml_node=None, name='data', xml_scope=None):
    def __init__(self, types_map, xml_roots, name, xml_node=None, xml_scopes=None, parent=None):
        self.types_map = types_map
        self.xml_roots = xml_roots
        self.name = name
        self.parent = parent

        if xml_scopes is None:
            xml_scopes = self.xml_roots

        # get xml_node from name  #TODO handle variables outside of POU (eg var_name: GVL.foo)
        self.xml_node = xml_node if xml_node is not None else self.get_xml_node(
            xml_scopes, './/variable[@name="{}"]/type/*'.format(self.name))
        
        self.members = []
        self.cpp_len = ''
        self.cpp_type_len = ''
        self.ros_len = ''

        self.get_types()

    def get_xml_node(self, xml_scopes, xpath_str):
        """Searches for a variable in xml_scopes using xpath_str and returns its XML node.

        :param xml_scopes: XML parent nodes to search for node
        :type xml_scopes: list
        :param xpath_str: XPath string specifying node to be found
        :type xpath_str: str
        :raises RuntimeError: When no XML node is found
        :return: XML node found
        :rtype: lxml.etree._Element
        """
        for scope in xml_scopes:
            xml_node = scope.xpath(xpath_str)
            if len(xml_node) == 1:
                return xml_node[0]
        raise RuntimeError('XML node not found.')

    def get_types(self):
        """Gets the cpp, ros and msg types as well as some helper types for the variable."""
        # get type from xml_node
        self.xml_type = self.type = self.xml_node.tag

        # xml_type can be derived, array or iec
        if self.xml_type == 'derived':
            self.get_derived_types()
        elif self.xml_type == 'array':
            self.get_array_types()
            self.is_pod = False
        elif self.xml_type in self.types_map['codesys']:
            self.get_iec_types()
        else:
            raise TypeError("CODESYS data type '{}' is not supported.".format(self.var_type))
        
        # prepend msg_pkg to ros_type for custom messages
        if self.xml_type == 'derived' and self.msg_pkg != 'robin_bridge':
            self.ros_type = self.msg_pkg + '/' + self.ros_type

        if self.cpp_type_len == '':
            self.cpp_type_len = self.cpp_type
    
    def get_iec_types(self):
        """Gets types for IEC variables."""
        # get types from typemap
        self.cpp_type, self.ros_type, self.msg_type = self.types_map['codesys'][self.xml_type]
        self.msg_pkg, self.msg_name = self.msg_type.split('::')
        
        # handle strings
        if self.xml_type == 'string':
            attribs = self.xml_node.attrib
            str_len = int(attribs['length']) + 1 if 'length' in attribs else self.CODESYS_DEF_STR_SIZE
            self.cpp_len = '[{}]'.format(str_len)
            self.cpp_type_len = self.cpp_type + self.cpp_len
            self.is_pod = False
        else:
            self.is_pod = True
            self.type = 'basic'

    def get_derived_types(self):
        """Gets types for derived variables (structs)."""
        self.base_type = self.xml_node.attrib['name']

        # get members from struct definition  #TODO? handle array members
        xml_struct_def = self.get_xml_node(self.xml_roots, './/dataType[@name="{}"]'.format(self.base_type))
        for xml_member in xml_struct_def.xpath('./baseType/struct/variable'):
            member = Variable(self.types_map, self.xml_roots, xml_member.attrib['name'],
                              xml_scopes=xml_struct_def, parent=self)
            self.members.append(member)

        # check for non-pod member
        self.is_pod = False not in (member.is_pod for member in self.members)

        # get types
        if self.base_type in self.types_map['codesys']['derived']:
            self.cpp_type, self.ros_type, self.msg_type = self.types_map['codesys']['derived'][self.base_type]
            self.msg_pkg, self.msg_name = self.msg_type.split('::')
        else:
            # if self.is_pod:
            #     self.type = 'pod'
            self.cpp_type = self.ros_type = self.msg_name = self.base_type
            msg_pkgs = self.types_map['ros']
            self.msg_pkg = next((pkg for pkg in msg_pkgs if self.base_type in msg_pkgs[pkg]), 'robin_bridge')
            self.msg_type = self.msg_pkg + '::' + self.msg_name

    def get_array_types(self):
        """Gets types for array variables."""
        if self.parent is None:
            self.name = 'data'

        # get base var
        base_var_xml_node = self.xml_node.xpath('./baseType/*')[0]
        base_var_name = self.name + '_base_var'
        base_var = Variable(self.types_map, self.xml_roots, base_var_name, xml_node=base_var_xml_node, parent=self)
        self.members.append(base_var)
        
        # get array dimensions
        dims = self.xml_node.xpath('./dimension')

        # TODO handle multidimensional arrays
        if base_var.xml_type == 'array' or len(dims) > 1:
            raise RuntimeError('Multidimensional arrays are not supported.')

        # get array length  #TODO get upper_bound from xml; create general function to get variable value from XML
        lower_bound = int(dims[0].attrib['lower'])
        try:
            upper_bound = int(dims[0].attrib['upper'])
        except ValueError:
            upper_bound = 20
        self.cpp_len = '[{}]'.format(upper_bound - lower_bound + 1)

        # handle variable length arrays
        is_varlen = self.xml_node.xpath(
            'count(../..//Attribute[@Name="{}"])'.format(self.VAR_LEN_ATTRIB)) == 1
        self.ros_len = self.cpp_len if not is_varlen else '[]'

        # get types
        self.type = 'varlen_' + self.type if is_varlen else self.type
        self.type = 'nonpod_' + self.type if not base_var.is_pod else self.type
        self.cpp_type = base_var.cpp_type
        self.cpp_type_len = base_var.cpp_type + self.cpp_len + base_var.cpp_len
        self.ros_type = base_var.ros_type
        self.msg_pkg = 'robin_bridge'
        self.msg_name = base_var.msg_name + ('VarLen' if is_varlen else '') + 'Array'
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
        repr_ = ('\nname: {}\ntype: {}\nparent: {}\nis_pod: {}\n'
                 'cpp_type: {}\ncpp_type_len: {}\nros_type: {}\nmsg_type: {}\n'
                 .format(self.name, self.type, parent, self.is_pod,
                         self.cpp_type, self.cpp_type_len, self.ros_type, self.msg_type))
        repr_ += 'members:{}'.format(self.members) if len(self.members) > 0 else ''
        return repr_
