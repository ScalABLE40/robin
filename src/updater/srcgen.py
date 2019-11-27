from collections import OrderedDict
from functools import partial
import yaml

class RobinSourceGenerator:
    """Parses list of Robin objects and generates source code"""
    def __init__(self, types_map, xml_root, robins):
        self.types_map = types_map
        self.xml_root = xml_root
        self.structs = OrderedDict()
        self.msgs = {}
        self.msg_types = set()
        self.msg_pkgs = set()
        self.node = []
        self.robin_inst = {}
        self.parse_robins(robins)

    def parse_robins(self, robins):
        for robin in robins:
            # add message package
            msg_pkg = robin.msg_type.split('::')[0]
            if msg_pkg != 'robin':
                self.msg_pkgs.add(msg_pkg)
            # add struct
            if robin.var_type == 'derived':
                self.add_struct(robin.cpp_type)
            # handle array type
            elif robin.var_type.endswith('array'):
                if robin.base_type == 'derived':
                    self.add_struct(robin.base_cpp_type)
                msg_name = robin.msg_type.split('::')[1]
                self.msgs[msg_name] = '{} {}\n'.format(robin.ros_type, 'data')
            # add robin
            self.add_robin(robin)

    def add_struct(self, struct_name):
        struct_name = str(struct_name)
        if struct_name not in self.structs:
            struct_src, msg_src = '', ''
            struct = self.xml_root.xpath('.//dataType[@name="{}"]'.format(struct_name))[0]
            for member in struct.xpath('baseType/struct/variable'):
                var_type = member.xpath('type/*')[0].tag
                if var_type == 'derived':
                    cpp_type, ros_type = [member.xpath('type/derived/@name')[0]] * 2
                    for msg_pkg in self.types_map['ros']:
                        if cpp_type in self.types_map['ros'][msg_pkg]:
                            self.msg_pkgs.add(msg_pkg)
                            break
                    else:
                        self.add_struct(cpp_type)
                else:
                    if var_type not in self.types_map['codesys']:
                        raise TypeError("CODESYS data type '{}' is not supported.".format(var_type))
                    cpp_type, ros_type = self.types_map['codesys'][var_type][:2]
                struct_src += '\n  {} {};'.format(cpp_type, member.attrib['name'])
                msg_src += '{} {}\n'.format(ros_type, member.attrib['name'])
            self.structs[struct_name] = '\nstruct {}\n{{{}\n}};'.format(struct.attrib['name'], struct_src)
            self.msgs[struct_name] = msg_src

    def add_robin(self, robin):
        self.msg_types.add(robin.msg_type)
        obj = 'Robin{}<{}, {}>'.format(
            'Subscriber' if robin.type == 'read' else 'Publisher',
            robin.cpp_type,
            robin.msg_type)
        node_src = '\n  {} {}(nh, "{}");'.format(obj, robin.name, robin.name)
        self.node.append(node_src)
        # add new specializations
        inst = '\ntemplate class {};'.format(obj)
        if inst not in self.robin_inst:
            # TODO handle all specializations
            spec = (self.types_map['spec_tpls'][robin.var_type][robin.type]
                .format(cpp_type=robin.cpp_type,
                        msg_type=robin.msg_type,
                        base_cpp_type=robin.base_cpp_type)
                if robin.var_type in self.types_map['spec_tpls'] else '')
            self.robin_inst[inst] = spec

    def get_source(self):
        sorted_ = partial(sorted, key=lambda s: s.lower())
        includes = ''.join(sorted_(['\n#include "{}.h"'.format(ros_msg.replace('::', '/')) for ros_msg in self.msg_types]))
        node = ''.join(sorted_(self.node))
        robin_inst = ''.join(sorted_(self.robin_inst.values()) + sorted_(self.robin_inst.keys()))  #TODO? eg dict to list, sort only once
        structs = ''.join(self.structs.values())
        return {'node': (includes, node),
                'robin_inst': (includes + robin_inst,),
                'structs': (structs,),
                'msgs': self.msgs}

    def __repr__(self):
        return yaml.dump(self.get_source())
