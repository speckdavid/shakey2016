How to create/add an object for Gazebo/ORK:

1. Create STL model, e.g. with blender
(face normals seem off with blender for some reason -> meshlab recompute face normals helps)

2. Call add_new_object script and provide pkg to add this to + STL file and name of the entry.
For gazebo this also needs a mass estimate.
Inertia is computed automatically assuming uniform mass distribution (using meshlab).

3. URDF file for gazebo ends up in models/, a launch file for adding to gazebo is also created.
The object is optinally also added to the ORK database for recognition.

