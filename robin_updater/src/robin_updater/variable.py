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

CODESYS_DEF_STR_SIZE = 81                   # Default size given by CODESYS
VAR_LEN_ATTRIB = 'robin_var_len'            # Attribute for var len arrays


class Variable:
    """Stores information about a variable and its members (for structs) or its base variable (for arrays).

    :param types_map: Mapping of xml/cpp/ros/msg types
    :type types_map: dict
    :param xml_roots: XML root nodes to search for variable
    :type xml_roots: list
    :param name: Name of the variable
    :type name: str
    :param xml_node: XML node of the variable. (Used only on recursive calls)
    :type xml_node: lxml.etree._Element
    :param xml_scopes: XML parent nodes to search for variable (Used only on recursive calls)
    :type xml_scopes: list
    :param parent: Parent variable (Used only on recursive calls)
    :type parent: variable.Variable
    """
    def __init__(self, types_map, xml_roots, name, xml_node=None, xml_scopes=None, parent=None):

        self._types_map = types_map                     # expanded path for types.yml
        self._xml_roots = xml_roots

        self.name = name                                # name of the variable being passed
        self.parent = parent                            # name of the variable's parent

        # If not scope (used on recusive), scope equals to root
        if xml_scopes is None:
            xml_scopes = self._xml_roots

        # TODO handle variables outside of POU (eg var_name: GVL.foo)
        # get xml_node <type> using var_name ('derived', 'array' or ex: 'BOOL')
        self._xml_node = xml_node if xml_node is not None else self._get_xml_node(
            xml_scopes, './/variable[@name="{}"]/type/*'.format(self.name))
        
        self.members = []           # list of Variables inside Variable
        self.cpp_len = ''           # used for arrays and strings (char[])
        self.cpp_type_len = ''      # used for arrays and strings (char[])
        self.ros_len = ''

        self._get_types()

    def _get_xml_node(self, xml_scopes, xpath_str):
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

    def _get_types(self):
        """Gets the cpp, ros and msg types as well as some helper types for the variable."""

        # TODO: Initialize class variables in the constructor
        # get type from xml_node
        self.xml_type = self.type = self._xml_node.tag

        # xml_type can be derived, array or iec (order is important)
        if self.xml_type == 'derived':              # msgs variables
            self._get_derived_types()
            
        elif self.xml_type == 'array':
            self._get_array_types()
            self.is_pod = False

        elif self.xml_type in self._types_map['codesys']:
            self._get_iec_types()

        else:
            raise TypeError("CODESYS data type '{}' is not supported.".format(self.xml_type))
        
        # prepend msg_pkg to ros_type (ex: std_msgs/Bool)
        if self.xml_type == 'derived' and self.msg_pkg != 'robin_bridge_generated':
            self.ros_type = self.msg_pkg + '/' + self.ros_type

        if self.cpp_type_len == '':
            self.cpp_type_len = self.cpp_type
    
    def _get_iec_types(self):
        """Gets types for IEC variables."""

        # gets info from types.yml (cpp_type: uint8_t, ros_type: bool, msg_type: std_msgs::Bool)
        self.cpp_type, self.ros_type, self.msg_type = self._types_map['codesys'][self.xml_type]
        self.msg_pkg, self.msg_name = self.msg_type.split('::')
        
        # handle special case (strings)
        if self.xml_type == 'string':

            # gets attributes of xml node string
            attribs = self._xml_node.attrib

            # verifies if it has a defined length otherwise sets default 
            # (lenght in CODESYS is inclusive so there is the need to +1 for C++)
            str_len = int(attribs['length']) + 1 if 'length' in attribs else CODESYS_DEF_STR_SIZE

            self.cpp_len = '[{}]'.format(str_len)
            self.cpp_type_len = self.cpp_type + self.cpp_len             # ex: char[5]
            self.is_pod = False

        # all the other variables from IEC enter here
        else:
            self.is_pod = True
            self.type = 'basic'                  # this 'basic' will be later used for getting the template

    def _get_derived_types(self):
        """Gets types for derived variables (structs)."""

        # TODO: Initialize class variables in the constructor
        # Struct Type
        self.base_type = self._xml_node.attrib['name']

        # gets node 'dataType' from struct
        xml_struct_def = self._get_xml_node(self._xml_roots, './/dataType[@name="{}"]'.format(self.base_type))
        
        # get members from struct definition  #TODO? handle array members
        for xml_member in xml_struct_def.xpath('./baseType/struct/variable'):

            # creates new Variable limiting the scope to the current 'dataType' node 
            member = Variable(self._types_map, self._xml_roots, xml_member.attrib['name'],
                              xml_scopes=xml_struct_def, parent=self)
            self.members.append(member)

        # check for non-pod member. If every member is pod itself is a pod
        self.is_pod = False not in (member.is_pod for member in self.members)

        # if it is a codesys derived variable
        if self.base_type in self._types_map['codesys']['derived']:

            # TODO: Initialize class variables in the constructor
            # gets info from types.yml (cpp_type: Time, ros_type: time, msg_type: std_msgs::Time)
            self.cpp_type, self.ros_type, self.msg_type = self._types_map['codesys']['derived'][self.base_type]
            self.msg_pkg, self.msg_name = self.msg_type.split('::')

        # ros_msg or custom ros_msg     
        else:

            # TODO: Initialize class variables in the constructor
            self.cpp_type = self.ros_type = self.msg_name = self.base_type

            # gets all ros_msg packages from types.yaml
            msg_pkgs = self._types_map['ros']
            
            # checks if it is a ROS package msg, otherwise is a custom msg so robin_bridge_generated
            # return first package match from types.yml
            self.msg_pkg = next((pkg for pkg in msg_pkgs if self.base_type in msg_pkgs[pkg]), 'robin_bridge_generated')

            # for C++ code (ex: std_msgs::Bool)
            self.msg_type = self.msg_pkg + '::' + self.msg_name

    def _get_array_types(self):
        """Gets types for array variables. (custom)"""

        # array example: var_struct_varlen_array of ByteMultiArray for example

        # for compliance with ROS standards
        if self.parent is None:
            self.name = 'data'

        # get base var
        base_var_xml_node = self._xml_node.xpath('./baseType/*')[0]     # ex: ByteMultiArray
        base_var_name = self.name + '_base_var'
        base_var = Variable(self._types_map, self._xml_roots, base_var_name, xml_node=base_var_xml_node, parent=self)
        self.members.append(base_var)
        
        # get array dimensions
        dims = self._xml_node.xpath('./dimension')

        # CODESYS (multidimensional arrays). In codesys the for loops go until the last and not until last - 1
        # var_name : ARRAY[1..2, 1..6] OF TYPE;
        # var_name[1,3]
        # var_name : ARRAY[1..2] OF ARRAY OF TYPE; 
        # var_name[1][3]

        # TODO handle multidimensional arrays
        if base_var.xml_type == 'array' or len(dims) > 1:
            raise TypeError('Multidimensional arrays are not supported.')

        
        # get array length  #TODO get upper_bound from xml; create general function to get variable value from XML
        lower_bound = int(dims[0].attrib['lower'])
        try:
            upper_bound = int(dims[0].attrib['upper'])
        except ValueError:
            upper_bound = 20
        
        # For C++ code
        self.cpp_len = '[{}]'.format(upper_bound - lower_bound + 1)

        # handle variable length arrays ( for attribute 'robin_var_len'. ignored by compiler and it is still fixed size to codesys)
        is_varlen = self._xml_node.xpath(
            'count(../..//Attribute[@Name="{}"])'.format(VAR_LEN_ATTRIB)) == 1
        self.ros_len = self.cpp_len if not is_varlen else '[]'

        # get types
        self.type = 'varlen_' + self.type if is_varlen else self.type                       # to get templates for specifications
        self.type = 'nonpod_' + self.type if not base_var.is_pod else self.type

        self.cpp_type = base_var.cpp_type
        self.cpp_type_len = base_var.cpp_type + self.cpp_len + base_var.cpp_len
        self.ros_type = base_var.ros_type

        self.msg_pkg = 'robin_bridge_generated'                                                       # it's custom msg
        self.msg_name = base_var.msg_name + ('VarLen' if is_varlen else '') + 'Array'
        self.msg_type = self.msg_pkg + '::' + self.msg_name

    def __eq__(self, other):

        # Used in src generator
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
