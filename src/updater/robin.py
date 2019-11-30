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
