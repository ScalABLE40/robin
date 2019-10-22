import subprocess

filename = 'robin.project'
filename_tmp = filename + '.tmp'

subprocess.run(['cp', filename, filename_tmp])


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

reporter = ER()
proj = projects.open(filename_tmp)
proj.export_xml(reporter, proj.get_children(False), tempname, recursive = True)
proj.close()


subprocess.run(['rm', filename_tmp])
