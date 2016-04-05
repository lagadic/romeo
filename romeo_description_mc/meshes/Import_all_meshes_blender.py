import bpy.ops
import os

def fileList(path): # Would be nice is this was built into python!
    for dirpath, dirnames, filenames in os.walk(path):
        for filename in filenames:
            yield os.path.join(dirpath, filename)

for f in fileList("/home/jokla/catkin_ws/src/romeo/romeo_description_mc/meshes"):
    if f.lower().endswith(".dae"):
        bpy.ops.wm.collada_import(filepath=f, import_units=True)
        print('importing')
