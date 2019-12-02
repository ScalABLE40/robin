#!/usr/bin/env python
import sys
class Variable:
    """Extracts types for given variable from xml"""  
    CODESYS_DEF_STR_SIZE = 80
    VAR_LEN_ATTRIB = 'robin_var_len'

    # def __init__(self, types_map, xml_root, xml_node=None, name='data', xml_scope=None):
    def __init__(self, types_map, xml_root, name, xml_node=None, xml_scope=None):
        self.types_map = types_map
        self.xml_root = xml_root
        self.name = name
        print(self.name)

        if xml_scope is None: xml_scope = self.xml_root

        # get xml_node from name  #TODO handle variables outside of POU (eg var_name: GVL.foo)
        self.xml_node = xml_node if xml_node is not None else xml_scope.xpath(
            './/variable[@name="{}"]/type/*'.format(self.name))[0]
        
        self.members = []

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

    def get_iec_types(self):
        # get types from typemap
        self.cpp_type, self.ros_type, self.msg_type = self.types_map['codesys'][self.type]
        self.msg_pkg, self.msg_name = self.msg_type.split('::')
        
        # handle strings
        if self.type == 'string':
            attribs = self.xml_node.attrib
            str_len = attribs['length'] if 'length' in attribs else self.CODESYS_DEF_STR_SIZE
            self.cpp_type = self.cpp_type.format(str_len=str_len)
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
        print(base_type)
        xml_struct_def = self.xml_root.xpath('.//dataType[@name="{}"]'.format(base_type))[0]
        for xml_member in xml_struct_def.xpath('./baseType/struct/variable'):
            # member = Variable(self.types_map, self.xml_root, name=xml_member.attrib['name'], xml_scope=xml_struct_def)
            member = Variable(self.types_map, self.xml_root, xml_member.attrib['name'], xml_scope=xml_struct_def)
            self.members.append(member)

        # check for non-pod member
        self.is_pod = False not in (member.is_pod for member in self.members)

    def get_array_types(self):
        base_xml_node = self.xml_node.xpath('./baseType/*')[0]
        base_type = base_xml_node.tag
        dims = self.xml_node.xpath('./dimension')

        # # TODO handle multidimensional arrays
        # if base_type == 'array':
        #     pass
        # elif len(dims) > 1:
        #     pass

        # get array length
        lower_bound = int(dims[0].attrib['lower'])
        try:
            upper_bound = int(dims[0].attrib['upper'])
        except ValueError:
            upper_bound = 20  #TODO get value from xml
        arr_len = upper_bound - lower_bound + 1
        is_var_len = self.xml_node.xpath(
            'count(../..//Attribute[@Name="{}"])'.format(self.VAR_LEN_ATTRIB)) == 1

        # get member var
        # base_var = Variable(self.types_map, self.xml_root, xml_node=base_xml_node)
        base_var = Variable(self.types_map, self.xml_root, self.name, xml_node=base_xml_node)
        self.members.append(base_var)

        # get types
        self.cpp_type = base_var.cpp_type + '[{}]'.format(arr_len)
        self.ros_type = base_var.ros_type + '[{}]'.format(arr_len if not is_var_len else '')
        self.msg_pkg = 'robin'
        self.msg_name = base_var.msg_name + '{}Array'.format('VarLen' if is_var_len else '')
        self.msg_type = self.msg_pkg + '::' + self.msg_name

        if not base_var.is_pod:
            self.type = 'nonpod_array'
        elif is_var_len:
            self.type = 'varlen_array'

    def __eq__(self, other):
        return isinstance(other, Variable) and self.ros_type == other.ros_type

    def __ne__(self, other):
        return not self == other

    def __repr__(self):
        repr_ = ('\nname: {}\ntype: {}\n'
                 'cpp_type: {}\nros_type: {}\nmsg_type: {}\n'
                 .format(self.name, self.type,
                         self.cpp_type, self.ros_type, self.msg_type))
        repr_ += 'members:{}'.format(self.members) if len(self.members) > 0 else ''
        return repr_
