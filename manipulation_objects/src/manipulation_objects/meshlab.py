#!/usr/bin/env python

import sys
import subprocess

def call_meshlab(mesh_path, template_path, *args):
    """ Call meshlabserver for path using template with args
        and return the parsed information."""
    # First create the script
    with open(template_path) as f:
        template = f.read()
    script = template % args

    script_path = "/tmp/meshlabscript.mlx"
    with open(script_path, "w") as f:
        print >> f, script

    log_path = "/tmp/meshlabserver.log"
    with open(log_path, "w") as f:  # make it empty
        pass

    cmd = "meshlabserver -i %s -s %s -l %s" % (mesh_path, script_path, log_path)
    subprocess.call(cmd.split())

    entries = parse_meshlab(log_path)
    print
    print entries
    assert len(entries) == 1
    return entries[0]

def parse_meshlab(fname):
    """ Expected Format:
Mesh Bounding Box Size 0.057200 0.057200 0.145000
Mesh Bounding Box Diag 0.166038 
Mesh Volume  is 0.000366
Mesh Surface is 0.029995
Thin shell barycenter  -0.000000  -0.000000   0.072500
Center of Mass  is -0.000000 -0.000000 0.072500
Inertia Tensor is :
    |  0.000001   0.000000   0.000000 |
    |  0.000000   0.000001   0.000000 |
    |  0.000000   0.000000   0.000000 |
Principal axes are :
    |  0.707107   0.707107  -0.000000 |
    | -0.707107   0.707107  -0.000000 |
    |  0.000000   0.000000   1.000000 |
axis momenta are :
    |  0.000001   0.000001   0.000000 |
    """
    with open(fname) as f:
        entries = []

        center_of_mass = None
        volume = None
        inertia = []
        in_inertia = False
        for line in f:
            if line.startswith("Mesh Volume  is"):
                line = line[len("Mesh Volume  is"):].strip()
                volume = float(line)
            elif line.startswith("Center of Mass  is"):
                line = line[len("Center of Mass  is"):].strip()
                parts = line.split()
                assert len(parts) == 3
                center_of_mass = [float(x) for x in parts]
            elif line.startswith("Inertia Tensor is :"):
                in_inertia = True
            elif line.startswith("Principal axes are :"):
                in_inertia = False
            elif in_inertia:
                line = line.strip()
                assert line.startswith("|") and line.endswith("|")
                line = line[1:-1]
                parts = line.split()
                assert len(parts) == 3
                inertia.append([float(x) for x in parts])
            elif line.startswith("Mesh Bounding Box") and volume is not None:
                # extra entry
                entries.append((volume, center_of_mass, inertia))
                volume = None
                center_of_mass = None
                inertia = []
        if volume is not None:
            entries.append((volume, center_of_mass, inertia))
        return entries


if __name__ == "__main__":
    ml = parse_meshlab(sys.argv[1])
    for m in ml:
        print m
