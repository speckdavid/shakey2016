#!/usr/bin/env python
import sys
import math
import rospkg

def print_usage():
    print "Usage: %s <package> <model_filename> <object_name> <launch_filename>" % sys.argv[0]
    print "Creates a launch file in package/launch/launch_filename for a model in package/modeles/model_filename."

def get_path(pkg, filedir, filename):
    r = rospkg.RosPack()
    p = r.get_path(pkg)
    p += "/" + filedir + "/" + filename
    return p

def write_launch(model_file, object_name, pkg, launch_file):
    launch_template_path = get_path("manipulation_objects", "templates", "spawn_object.launch.tmpl")
    with open(launch_template_path) as f:
        launch_template = f.read()
    launch = launch_template % (object_name, pkg, model_file)
    launch_path = get_path(pkg, "launch", launch_file)
    with open(launch_path, "w") as f:
        print >> f, launch

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print_usage()
        sys.exit(1)

    pkg = sys.argv[1]
    model_file = sys.argv[2]
    object_name = sys.argv[3]
    launch_filename = sys.argv[4]

    write_launch(model_file, object_name, pkg, launch_filename)

