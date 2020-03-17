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
import urllib2

############################################################################
##                                                                        ##
##                       DEVELOPER CONFIGURATIONS                         ##
##                                                                        ##
############################################################################
DEV = False

# location = 'hdomingos@localhost:catkin_ws/'                             # ROSworkspace location WSL
# target = 'hdomingos@localhost'                                          # target machine WSL
# pass_ = '5dpo'                                                          # target password WSL
# catkin_ws = '~/catkin_ws/'                                              # ROSworkspace path WSL
# port = 2222                                                             # ssh port is WSL has to be changed

############################################################################
##                                                                        ##
##                                SETUP                                   ##
##                                                                        ##
############################################################################

XML_PATH = 'codesys_project.xml'

# set working directory and check write permission
work_dir = os.popen('echo %TMP%').read().strip()                        # path for TMP folder
work_dir = work_dir + '\\' if work_dir[-1] != '\\' else work_dir
work_dir += 'robin_ros_codesys_bridge' + '\\'
try:
    os.mkdir(work_dir)
except OSError:
    pass

###########################################################################    

# get plink if not in working directory (python2 only)
# plink - PuTTY link - for the ssh connection
# If any error occur in this part please use the 'url' below to download the 
# plink.exe. Place it under C:/Users/<usr>/AppData/Local/Temp/robin_ros_codesys_bridge
# If this path does not exist please print 'work_dir' in the CODESYS by 
# uncommenting the next line and place plink.exe inside
# system.write_message(Severity.Information, "PLINK PATH: %s" % work_dir)
work_dir_contents = os.listdir(work_dir)
if 'plink.exe' not in work_dir_contents:
    url = 'https://the.earth.li/~sgtatham/putty/latest/w32/plink.exe'
    data = urllib2.urlopen(url).read()
    with open(work_dir + 'plink.exe', 'wb') as file:
        file.write(data)

############################################################################
##                                                                        ##
##                               CODESYS                                  ##
##                                                                        ##
############################################################################

# Logger CODESYS (https://forge.codesys.com/tol/scripting/home/Snippets/?version=2)
class ER(ExportReporter):

    def error(self, object, message):
        system.write_message(Severity.Error, "Error exporting %s: %s" % (object, message))

    def warning(self, object, message):
        system.write_message(Severity.Warning, "Warning exporting %s: %s" % (object, message))

    def nonexportable(self, object):
        system.write_message(Severity.Information, "Object not exportable: %s" % object)

    @property
    def aborting(self):
        return False;

###########################################################################  

# the variable 'projects' comes from CODESYS
# selects current project, saves and exports it to XML
project = projects.primary
# TODO: Find out what dirty flag means
if project.dirty:

    res = system.ui.prompt("The project needs to be saved first. Proceed?", PromptChoice.YesNo, PromptResult.No);

    if res != PromptResult.Yes:
        system.ui.error('Update aborted.')
        raise SystemExit

    project.save()

# TODO: minify XML to save space
project.export_xml(ER(), project.get_children(True), work_dir + XML_PATH, True)

############################################################################
##                                                                        ##
##                            ACCESSING TARGET                            ##
##                                                                        ##
############################################################################

# when not in developer mode
if 'DEV' not in globals() or not DEV:

    # get catkin workspace location
    location = ''

    # confirms that the location is valid
    while location.find('@') == -1 or location.find(':') == -1:

        # prompts for location
        location = system.ui.query_string("Catkin workspace location:\n( <user>@<address>:<path> )", cancellable=True)

        # when empty location, will ask again
        if location == '':
            system.ui.error('Please provide location in the form: <user>@<address>:<path>')
        
        # cancelled/closed case
        if location is None:
            system.ui.error('Update aborted.')
            raise SystemExit

    # parse location
    location = location.replace('\\', '/')
    location = location + '/' if not location.endswith('/') else location

    # gets target and catkin_ws from location
    target, catkin_ws = location.split(':')
    catkin_ws = '~/' + catkin_ws if not catkin_ws.startswith(('/', '~')) else catkin_ws

    # gets user from target
    user, _ = target.split('@')

    # prompts for password
    pass_ = system.ui.query_password("Password for user '{}':".format(user), cancellable=True)
    
    # default ssh port
    port = 22

    # cancelled/closed case
    if pass_ is None:
        system.ui.error('Update aborted.')
        raise SystemExit

# the variable 'online' comes from CODESYS
# download project to softplc
onlineapp = online.create_online_application()

# verifies if was logged in
if onlineapp.is_logged_in:
    onlineapp.logout()

onlineapp.login(OnlineChangeOption.Never, True)

# always logout otherwise error will come up when restarting codesyscontrol service
onlineapp.logout()

# run update script
cmd = ' '.join(('cmd /c "',
                    'set RET=0',
                    '& echo.',
                    '& echo * * * * * * * * * * * * *',
                    '& echo * * * Robin Updater * * *',
                    '& echo * * * * * * * * * * * * *',
                    '& echo.',
                    '& echo Connecting...',
                    '& {wd}plink.exe -ssh -batch -P {port} -pw {pwd} {tgt} < {wd}{xml} "',
                        'cat > {ws}{xml}',
                        '&& cd {ws}',
                        '&& . *devel*/setup.bash',
                        '&& mv {xml} $(rospack find robin_updater)/cfg/',
                        '&& roscd robin_updater/src/robin_updater',
                        '&& ./updater.py {ws}',
                    '" || set RET=1',
                    '& echo.',
                    '& echo.',
                    '& pause',
                    '& exit %RET%',
                '"')).format(wd=work_dir, xml=XML_PATH, tgt=target,
                             pwd=pass_, ws=catkin_ws, port=port)

if os.system(cmd) == 0:
    system.ui.info('Update finished successfully!')
else:
    system.ui.error('Update failed.')