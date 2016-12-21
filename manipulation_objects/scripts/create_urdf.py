#!/usr/bin/env python
import sys
import math
import rospkg
from manipulation_objects import meshlab

def print_usage():
    print "Usage: %s <package> <mesh_filename> <object_name> <mass> <urdf_filename>" % sys.argv[0]
    print "Creates a URDF file in package/models/urdf_filename for a mesh in package/meshes/mesh_filename."

def get_path(pkg, filedir, filename):
    r = rospkg.RosPack()
    p = r.get_path(pkg)
    p += "/" + filedir + "/" + filename
    return p

def compute_mesh_geometry(mesh_path):
    script_path = get_path("manipulation_objects", "templates", "compute_geometry.mlx")
    volume, com, inertia = meshlab.call_meshlab(mesh_path, script_path)
    return volume, com

def compute_inertia(mesh_path, scale):
    script_path = get_path("manipulation_objects", "templates", "compute_inertia.mlx")
    volume, com, inertia = meshlab.call_meshlab(mesh_path, script_path, scale, scale, scale)
    return inertia

def write_urdf(mesh_file, object_name, mass, center_of_mass, inertia, pkg, urdf_file):
    urdf_template_path = get_path("manipulation_objects", "templates", "model.urdf")
    with open(urdf_template_path) as f:
        urdf_template = f.read()
    # URDF from template
    urdf = urdf_template % (object_name, object_name,
            mass, center_of_mass[0], center_of_mass[1], center_of_mass[2],
            inertia[0][0], inertia[0][1], inertia[0][2],
            inertia[1][1], inertia[1][2],
            inertia[2][2],
            pkg, mesh_file, pkg, mesh_file,
            object_name)
    urdf_path = get_path(pkg, "models", urdf_file)
    with open(urdf_path, "w") as f:
        print >> f, urdf

if __name__ == "__main__":
    if len(sys.argv) != 6:
        print_usage()
        sys.exit(1)

    pkg = sys.argv[1]
    mesh_file = sys.argv[2]
    object_name = sys.argv[3]
    mass = float(sys.argv[4])
    urdf_filename = sys.argv[5]
    mesh_path = get_path(pkg, "meshes", mesh_file)

    mesh_volume, center_of_mass = compute_mesh_geometry(mesh_path)
    density = mass/mesh_volume

    lin_scale = math.pow(mesh_volume, -1.0/3.0) # scale to unit volume for better inertia output numerics
    inertia_scaled = compute_inertia(mesh_path, lin_scale)  # TODO check actual scale
    # scaled mesh has volume (-> s^3) and inertia itself (-> s^2) scaled
    # meshlab assumes unit density, thus the s^3 is also in the inertia
    # correct for both and use the actual density
    scale_fix = density * math.pow(lin_scale, -5.0)
    inertia = [[scale_fix * x for x in y] for y in inertia_scaled]
    write_urdf(mesh_file, object_name, mass, center_of_mass, inertia, pkg, urdf_filename)

