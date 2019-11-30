#!/usr/bin/env python
import variable

class Robin:
    def __init__(self, types_map, xml_root, type_, name, var_name):
        self.type = type_
        self.ros_type = 'Subscriber' if self.type == 'read' else 'Publisher'
        self.name = name
        self.var = variable.Variable(types_map, xml_root, name=var_name)

    def __repr__(self):
        return '\nname: {}\ntype: {}\nvar:({})'.format(self.name, self.type, self.var)


# class Robin:
#     """Extracts variable and message types for given robin object call from xml"""  
#     CODESYS_DEF_STR_SIZE = 80
#     VAR_LEN_ATTRIB = 'robin_var_len'

#     def __init__(self, types_map, xml_root, type_, name, var_name):  #TODO necessary to save all args?
#         self.types_map = types_map
#         self.type = type_
#         self.name = name
#         #TODO handle variables outside of POU (eg var_name: GVL.foo)
#         xml_var_type = xml_root.xpath('.//variable[@name="{}"]/type/*'.format(var_name))[0]
#         self.var_type = xml_var_type.tag
#         self.base_type, self.base_cpp_type = '', ''
#         self.cpp_type, self.ros_type, self.msg_type = self.get_types(xml_var_type)

#     def get_types(self, xml_var_type):
#         if xml_var_type.tag in self.types_map['codesys']:  # if supported codesys base type
#             return self.get_codesys_types(xml_var_type)
#         elif xml_var_type.tag == 'derived':
#             return self.get_derived_types(xml_var_type)
#         elif xml_var_type.tag == 'array':
#             return self.get_array_types(xml_var_type)
#         else:
#             raise TypeError("CODESYS data type '{}' is not supported.".format(var_type))

#     def get_codesys_types(self, xml_var_type):
#         cpp_type, ros_type, msg_type = self.types_map['codesys'][xml_var_type.tag]
#         if xml_var_type.tag == 'string':
#             str_size = xml_var_type.xpath('./@length')
#             str_size = size[0] if len(str_size) == 1 else self.CODESYS_DEF_STR_SIZE
#             cpp_type = cpp_type.format(size=str_size)
#         return cpp_type, ros_type, msg_type

#     def get_derived_types(self, xml_var_type):
#         cpp_type, ros_type = [str(xml_var_type.xpath('./@name')[0])] * 2
#         for msg_pkg in self.types_map['ros']:  # each standard ros message package
#             if cpp_type in self.types_map['ros'][msg_pkg]:  # in message package
#                 return cpp_type, ros_type, msg_pkg + '::' + cpp_type
#         else:  # its a custom message
#             return cpp_type, ros_type, 'robin::' + cpp_type

#     def get_array_types(self, xml_var_type):
#         # get array length
#         lower_bound = int(xml_var_type.xpath('./dimension/@lower')[0])
#         upper_bound = int(xml_var_type.xpath('./dimension/@upper')[0])
#         array_len = upper_bound - lower_bound + 1
#         is_var_len = xml_var_type.xpath(('count(../..//Attribute[@Name="{}"])'
#                                          .format(self.VAR_LEN_ATTRIB))) == 1
#         # get array base type
#         xml_base_type = xml_var_type.xpath('./baseType/*')[0]
#         self.base_type = xml_base_type.tag
#         # get cpp type and base cpp type
#         self.base_cpp_type, base_ros_type, base_msg_type = self.get_types(xml_base_type)  #TODO handle multidimensional arrays
#         cpp_type = '{}[{}]'.format(self.base_cpp_type, array_len)
#         ros_type = '{}[{}]'.format(base_ros_type, '' if is_var_len else array_len)
#         # get msg type
#         # self.variable_length = xml_var_type.xpath(
#         #     'count(../..//Attribute[@Name="{}"])'.format(self.VAR_LEN_ATTRIB)) == 1
#         msg_type = 'robin::' + base_msg_type.split('::')[1]
#         if is_var_len:
#             self.var_type = 'vlarray'
#             msg_type += 'VarLen'
#         # # msg_type = 'robin::' + base_msg_type.split('::')[1]
#         msg_type += 'Array'
#         # msg_type += 'VarLenArray' if is_var_len else 'Array'
#         return cpp_type, ros_type, msg_type

#     def __repr__(self):
#         repr_ = ('\nname: {}\ntype: {}\nvar_type: {}\n'
#                  'cpp_type: {}\nros_type: {}\nmsg_type: {}\n'
#                  .format(self.name, self.type, self.var_type,
#                          self.cpp_type, self.ros_type, self.msg_type))
#         if self.var_type.endswith('array'):
#             repr_ += ('base_type: {}\nbase_cpp_type: {}\n'
#                 .format(self.base_type, self.base_cpp_type))
#         return repr_
