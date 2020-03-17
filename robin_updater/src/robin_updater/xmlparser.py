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

import re

import lxml.etree

import robin
import srcgen

try:  #python2
    from StringIO import StringIO
except ImportError:  #python3
    from io import StringIO


class XMLParser:
    """Parses the PLCopenXML file exported from CODESYS.

    :param types_map: Mapping of xml/cpp/ros/msg types
    :type types_map: dict
    :param templates: Templates for srcgen.SourceGenerator
    :type templates: dict
    """
    def __init__(self, types_map, templates):
        self._types_map = types_map                 # expanded path for types.yml
        self._templates = templates                 # expanded path for templates.yml

    def get_src_from_xml(self, file_paths):
        """Parses XML file and returns dict with source components."""

        # xml roots for project and library
        self._xml_roots = [self._get_xml_root(path) for path in file_paths]

        # creates SourceGenerator
        self._src_gen = srcgen.SourceGenerator(self._types_map, self._templates)
        self._parse_robins()

        return self._src_gen.get_source()

    @staticmethod
    def _get_xml_root(file_path):
        """Loads XML file. Remove garbage from xml files from project and library xmls"""
        with open(file_path, 'r') as file:
            xml = file.read()

            # fix weird first character
            while xml[:5] != '<?xml': xml = xml[1:]

            xml = re.sub('<\?xml version=.*encoding=.*\n', '', xml, count=1)  # remove encoding
            xml = re.sub(' xmlns=".*plcopen.org.*"', '', xml, count=1)  # remove namespace

            return lxml.etree.fromstring(xml)

    def _parse_robins(self):
        """Parses robin publishers/subscribers from Robin objects in XML file."""

        # gets class Robin name
        robin_objs = self._xml_roots[0].xpath('instances//variable[descendant::derived[@name="Robin"]]/@name')

        # TODO: check if this is supported (more than one robin class?)
        for obj_name in robin_objs:

            # retrives all text from ST from POUs with obj_name (list with all text from ST)
            src = self._xml_roots[0].xpath('instances//addData/data/pou/body/ST/*[contains(text(), "{}();")]/text()'.format(obj_name))  #TODO handle spaces
            # src = self._xml_roots[0].xpath('instances//addData/data/pou/body/ST/node()[contains(text(), "{}();")]/text()'.format(obj_name))  #TODO try
            
            # each obj_name can only appear in one place (this has to be like this)
            if len(src) == 0:
                print_("Warning: no source found for robin object '{}'.".format(obj_name))
            elif len(src) > 1:
                raise RuntimeError("Robin object '{}' used in more than one POU.".format(obj_name))

            # for each code line in src
            for line in StringIO(src[0]):
                robin = self._parse_robin_from_call(line, obj_name)

                # generates src code
                if robin is not None:
                    self._src_gen.add_robin(robin)

        if len(self._src_gen.vars) == 0:
            raise RuntimeError('No valid robin objects found.')

    def _parse_robin_from_call(self, src, name):
        """Parses robin publisher/subscriber from a read/write() call of a Robin object."""

        # (groups)
        pat = "^[ ]*{}[ ]*\.[ ]*(read|write)[ ]*\([ ]*'([^ ,]+)'[ ]*,[ ]*([^ ,)]+)[ ]*\)[ ]*;".format(name)

        # searches for matches in the 'line'
        match = re.search(pat, src)
        if match is None:
            return None

        # tuple separated according to groups: type_of_operation (read/write), name (topic name), var_name
        props = match.group(1, 2, 3)  # type, name, var_name

        if None in props:
            raise RuntimeError("Failed to parse robin call in '{}'.".format(src))

        # creates robin object
        return robin.Robin(self._types_map, self._xml_roots, *props)
