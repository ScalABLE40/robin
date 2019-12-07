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


DEV = True
XML_PATH = 'codesys_project.xml'


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


# set working directory and check write permission
work_dir = os.popen('echo %TMP%').read().strip()
work_dir = work_dir + '\\' if work_dir[-1] != '\\' else work_dir
work_dir += 'robin_ros_codesys_bridge' + '\\'
try:
    os.mkdir(work_dir)
except OSError:
    pass

# get plink (python2 only)
work_dir_contents = os.listdir(work_dir)
if 'plink.exe' not in work_dir_contents:
    url = 'https://the.earth.li/~sgtatham/putty/latest/w32/plink.exe'
    data = urllib2.urlopen(url).read()
    with open(work_dir + 'plink.exe', 'wb') as file:
        file.write(data)

# save project
project = projects.primary
if project.dirty:
    res = system.ui.prompt("The project needs to be saved first. Proceed?", PromptChoice.YesNo, PromptResult.No);
    if res != PromptResult.Yes:
        system.ui.error('Update aborted.')
        raise SystemExit
    project.save()
# project.export_xml(ER(), project.get_children(False), work_dir + XML_PATH, True)
project.export_xml(ER(), project.get_children(True), work_dir + XML_PATH, True)
# raise SystemExit  #DEV

# DEV
location = 'criis@robin.local:catkin_ws/'
pass_ = '5dpo'
target = 'criis@robin.local'
catkin_ws = '~/catkin_ws/'

if 'DEV' not in globals() or not DEV:  #DEV
    # get catkin workspace location
    location = ''
    while location.find('@') == -1 or location.find(':') == -1:
        if location != '':
            system.ui.error('Please provide location in the form: <user>@<address>:<path>')
        location = system.ui.query_string("Catkin workspace location:\n( <user>@<address>:<path> )", cancellable=True) 
        if location is None:
            system.ui.error('Update aborted.')
            raise SystemExit

    # parse location
    location = location.replace('\\', '/')
    location = location + '/' if not location.endswith('/') else location
    target, catkin_ws = location.split(':')
    catkin_ws = '~/' + catkin_ws if not catkin_ws.startswith(('/', '~')) else catkin_ws
    user, _ = target.split('@')

    # get password
    pass_ = system.ui.query_password("Password for user '{}':".format(user), cancellable=True)
    if pass_ is None:
        system.ui.error('Update aborted.')
        raise SystemExit

# online update / download
onlineapp = online.create_online_application()
# was_logged_in = onlineapp.is_logged_in
# if was_logged_in:
if onlineapp.is_logged_in:
    onlineapp.logout()
onlineapp.login(OnlineChangeOption.Never, True)
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
                    '& {wd}plink.exe -ssh -batch -pw {pwd} {tgt} < {wd}{xml} "',
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
                             pwd=pass_, ws=catkin_ws)
if os.system(cmd) == 0:
    system.ui.info('Update finished successfully!')
else:
    system.ui.error('Update failed.')

# # log back in
# if was_logged_in:
#     onlineapp.login(OnlineChangeOption.Keep, False)
