#!/usr/bin/env python

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

import os
import re
import sys
import time

import rosgraph
import rosnode
import yaml

import xmlparser


# DEV = True  #DEV uncomment to stop before recompilation

DEF_NODE_NAME = 'robin_node'
DEF_CATKIN_WS = '~/catkin_ws'


def print_(msg):
    """Prints string msg to console immediately."""
    print(msg)
    sys.stdout.flush()


class Updater:
    """Main class that handles the update process.

    :param paths_file: Path to file containing paths for configuration files
    :type paths_file: str
    """

    def __init__(self, paths_file='../../cfg/paths.yml'):
        # load config files
        self._paths = self._load_yaml(paths_file, self._parse_paths)                # expanded paths (all)
        self._types_map = self._load_yaml(self._paths['config']['types'])           # expanded path for types.yml 
        self._templates = self._load_yaml(self._paths['config']['templates'])       # expanded path for templates.yml

    def update(self, catkin_ws=DEF_CATKIN_WS):
        """Main function.

        :param catkin_ws: Path to catkin workspace
        :type catkin_ws: str
        """
        print_('\nGenerating source code...')

        # paths for xmls (project and library)
        xmls = [self._paths['config']['proj'], self._paths['config']['lib']]


        self._source = xmlparser.XMLParser(self._types_map, self._templates).get_src_from_xml(xmls)
        
        # writes source files. updates CMakeLists.txt and package.xml files
        self._rewrite_source()

        if 'DEV' in globals() and DEV:
            raise SystemExit  #DEV

        # recompiles robin pkg
        self._recompile_robin(catkin_ws)
        self._restart_robin(DEF_NODE_NAME, catkin_ws)

        print_('\nUpdate finished.')

    @staticmethod
    def _load_yaml(file_path, parse=lambda x: x):
        """Loads yaml file and parses it with parse(). 
        Parsing configuration file on a custom way"""

        with open(file_path, 'r') as file:
            return parse(yaml.safe_load(file))

    @staticmethod
    def _parse_paths(paths):
        """Returns paths dict with expanded paths."""
        
        # TODO: clean this logic
        # get root folders
        cfg_root = paths['config'].pop('root')
        pkg_root = paths['package'].pop('root')

        # expand config files paths
        for file in paths['config']:
            paths['config'][file] = cfg_root + paths['config'][file]

        # expand source files path
        for file in paths['src_files']:
            paths['src_files'][file] = pkg_root + paths['package'][file] + paths['src_files'][file]
            paths['package'].pop(file)

        # expand package files paths
        for file in paths['package']:
            paths['package'][file] = pkg_root + paths['package'][file]

        return paths

    def _rewrite_source(self):
        """Writes source files. Updates CMakeLists.txt and package.xml files"""

        # TODO: clean this logic (key, file in self._paths['src_files'].items())
        # write generated source to respective files
        for file in self._paths['src_files']:
            with open(self._paths['src_files'][file], 'w') as src_file:
                src_file.write(self._templates[file]['file'].format(self._source[file]))

        # delete and rewrite msg files
        os.system('rm ' + self._paths['package']['msg'] + '*.msg 2>/dev/null')
        for msg, src in self._source['msgs'].items():
            with open(self._paths['package']['msg'] + msg + '.msg', 'w') as src_file:
                src_file.write(src)
                
        # updates CMakeLists.txt and package.xml files
        self._update_cmakelists(self._paths['package']['cmakelists'], self._source['msg_pkgs'], self._source['msgs'])
        self._update_package_xml(self._paths['package']['package_xml'], self._source['msg_pkgs'], self._source['msgs'])

    @staticmethod
    def _update_cmakelists(path, msg_pkgs, msgs):
        """Updates robin_bridge/CMakeLists.txt.

        :param path: Path to CMakeLists.txt file
        :type path: str
        :param msg_pkgs: Standard ROS message packages used
        :type msg_pkgs: list
        :param msgs: Custom ROS messages defined
        :type msgs: collections.OrderedDict()
        """
        # reads CMakeLists.txt
        with open(path, 'r+') as file:
            content = file.read()

            # with msg_pkgs (ROSmsgs like std_msgs, CUSTOM excluded)
            if len(msg_pkgs) > 0:
                
                # builds new find_package()
                new_src = ('find_package(catkin REQUIRED COMPONENTS\n'
                         + '  roscpp\n'
                         + ''.join(['  ' + pkg + '\n' for pkg in msg_pkgs])
                         +('  message_generation\n' if len(msgs) > 0 else '')
                         + ')')

                # searches and replaces       
                content = re.sub('find_package\s?\([^)]*roscpp[^)]*\)', new_src, content)

                # catkin_package
                # TODO: This does not seem to be doing anything
                new_src = ('\n  CATKIN_DEPENDS roscpp '
                         + ''.join([pkg + ' ' for pkg in msg_pkgs])
                         + 'message_runtime' if len(msgs) > 0 else '')
                content = re.sub('\n#?\s*CATKIN_DEPENDS roscpp.*', new_src, content)

            # if there are CUSTOM msgs
            if len(msgs) > 0:

                # builds new add_message_files()
                new_src = ('add_message_files(\n'
                         + '  FILES\n'
                         + ''.join(['  ' + msg + '.msg\n' for msg in msgs])
                         + ')')
                
                # searches and replaces 
                content = re.sub('#? ?add_message_files\s?\([^)]*\)', new_src, content)

                # builds new generate_messages()
                new_src = ('generate_messages(\n'
                         + '  DEPENDENCIES\n'
                         + ''.join(['  ' + pkg + '\n' for pkg in msg_pkgs])
                         + ')')
                
                # searches and replaces 
                content = re.sub('#? ?generate_messages\s?\([^)]*\n[^)]*\)', new_src, content)
            
            # if there are no CUSTOM msgs there is the need to comment
            else:

                # comment add_message_files()
                new_src = ('# add_message_files(\n'
                         + '#  FILES\n'
                         + '#)')

                content = re.sub('#? ?add_message_files\s?\([^)]*\)', new_src, content)

                # comment generate_messages()
                new_src = ('# generate_messages(\n'
                         + '#  DEPENDENCIES\n'
                         + '#)')

                content = re.sub('#? ?generate_messages\s?\([^)]*\n[^)]*\)', new_src, content)

            file.seek(0)
            file.write(content)
            file.truncate()

    @staticmethod
    def _update_package_xml(path, msg_pkgs, msgs):
        """Updates robin_bridge/package.xml.

        :param path: Path to package.xml file
        :type path: str
        :param msg_pkgs: Standard ROS message packages used
        :type msg_pkgs: list
        :param msgs: Custom ROS messages defined
        :type msgs: collections.OrderedDict()
        """

        # reads package.xml
        with open(path, 'r+') as file:
            content = file.read()

            # builds new part of package.xml
            new_src = ('\n  <depend>roscpp</depend>\n'
                     + ''.join(['  <depend>' + pkg + '</depend>\n' for pkg in msg_pkgs])
                     +('  <build_depend>message_generation</build_depend>\n'
                     + '  <exec_depend>message_runtime</exec_depend>\n' if len(msgs) > 0 else '')
                     + '  <exec_depend>python</exec_depend>')

            # searches and replaces       
            content = re.sub('\n  <depend>roscpp<\/depend>[\S\s]*<exec_depend>python<\/exec_depend>', new_src, content)

            file.seek(0)
            file.write(content)
            file.truncate()

    @staticmethod
    def _recompile_robin(catkin_ws=DEF_CATKIN_WS):
        """Recompiles robin_bridge package."""

        print_('\nRecompiling...')

        # bash command: sources setup.bash, creates method and calls it     
        cmd = '''bash -c "
                    cd {} &&
                    . *devel*/setup.bash &&
                    build_robin()
                    {{
                        if [ -d .catkin_tools ]; then
                            catkin build robin_bridge
                        else
                            catkin_make robin_bridge
                        fi
                    }}
                    set -o pipefail
                    build_robin 2>&1 >/dev/null |
                    sed 's/\\x1b\[[0-9;]*[mK]//g'
                "'''.format(catkin_ws)  # sed is used to filter special characters from build output
        
        # exit code different from 0
        if os.system(cmd) != 0:
            raise RuntimeError('Failed to recompile robin_bridge package.')

    @classmethod
    def _restart_robin(cls, node_name=DEF_NODE_NAME, catkin_ws=DEF_CATKIN_WS):
        """Restarts robin bridge."""

        print_('\nRestarting...')

        # get the first robin node
        node_path = cls._get_node_path(node_name)

        if node_path == '':
            print_('Robin node is not running.')

        # restart tobin node   
        elif node_path is not None:

            cls._restart_robin_node(node_path)

        # try to restart codesyscontrol service
        if os.system('sudo -n systemctl restart codesyscontrol > /dev/null 2>&1') != 0:
            print_('\nFailed to restart codesyscontrol via systemctl. Please do it manually.')

        if os.system('sudo -n service codesyscontrol start > /dev/null 2>&1') != 0:
            print_('\nFailed to restart codesyscontrol via service. Please do it manually.')

    @staticmethod
    def _get_node_path(node_name):  #TODO allow multiple robin nodes
        """Returns node_path for first node node_name found."""

        try:
            # between the nodes running
            for node_path in rosnode.get_node_names():

                # get the first robin node
                if node_path.split('/')[-1] == node_name:
                    return node_path

            return ''

        except rosnode.ROSNodeIOException:
            print_('ROS master is not running.')
            return None

    
    @classmethod
    def _restart_robin_node(cls, node_path):  #TODO try to simplify
        """Restarts robin bridge node."""

        # if node alive
        if rosnode.rosnode_ping(node_path, max_count=3):

            # kill node 
            if node_path not in rosnode.kill_nodes([node_path])[0]:  
                raise RuntimeError("Failed to kill robin node '{}'.".format(node_path))

        else:
            
            master = rosgraph.Master(rosnode.ID)
            rosnode.cleanup_master_blacklist(master, [node_path])
        node_name = node_path.split('/')[-1]
        cls._wait_for(lambda: cls._get_node_path(node_name) == '')
        namespace = '/'.join(node_path.split('/')[:-1])
        cmd = '''bash -c "
                    cd {} &&
                    . *devel*/setup.bash &&
                    rosrun robin_bridge robin_node __ns:={} &
                " > /dev/null 2>&1'''.format(catkin_ws, namespace)
                # " > /dev/null 2>&1'''.format(catkin_ws, node_path[:-len('/' + node_name)])
        if os.system(cmd) != 0:
            raise RuntimeError('Failed to rerun robin node.')
        cls._wait_for(lambda: cls._get_node_path(node_name) != '', timeout=10)

    @staticmethod
    def _wait_for(condition, interval=100, timeout=5):
        """Waits for a given condition() to return True.

        :param condition: Function to evaluate condition
        :type condition: function
        :param interval: Interval between tries in msec
        :type interval: int
        :param timeout: Maximum time to wait in sec
        :type timeout: int
        :raises RuntimeError: When operation times out.
        """
        for i in range(0, timeout * 1000, interval):
            time.sleep(interval / 1000.0)
            if condition():
                return
        raise RuntimeError('Operation timed out.')


if __name__ == '__main__':
    # check catkin workspace was passed as argument
    if len(sys.argv) != 2:
        print_('Usage: ./update.py <path_to_catkin_ws>')
        raise SystemExit
    catkin_ws = sys.argv[1] + ('/' if not sys.argv[1].endswith('/') else '')

    # check catkin workspace exists
    if not os.path.isdir(catkin_ws):
        print_("Folder '{}' not found.".format(catkin_ws))
        raise SystemExit

    # run updater
    Updater().update(catkin_ws=catkin_ws)
