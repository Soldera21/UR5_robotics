##
# @file scale_legos.py
#
#
# @section module_authors Author(s)
# - Created by Marco Morandin
#
import os
import xml.etree.ElementTree as ET

# Global Constants
## Path to the world models sdf(s)
MODELS_PATH = os.getcwd().replace('utils', '') + 'locosim/ros_impedance_controller/worlds/models'
## Path to the spawn models sdf(s)
SPAWN_PATH = os.getcwd().replace('utils', '') + 'spawnLego/models'

## The new value for the scale of the legos
NEW_SCALE_FACTOR = 0.8
## The value that already is used to scale legos
OLD_SCALE_FACTOR = 0.9

def scale_legos():
    """! This function scale legos modifying their sdf file. This was done in order to increase
    the spacing between legos and enhance recognition performance.
    """
    for dirname in os.listdir(MODELS_PATH):
        if ('X' and 'Y' and 'Z') in dirname:
            content = ''
            with open(MODELS_PATH + '/' + dirname + '/' + 'model.sdf', 'r') as file:
                content = file.read()
            file.close()
            content = content.replace('<scale>' + ((str(OLD_SCALE_FACTOR) + ' ') * 3).rstrip() + '</scale>',
                                    '<scale>' + ((str(NEW_SCALE_FACTOR) + ' ') * 3).rstrip() + '</scale>')
            with open(MODELS_PATH + '/' + dirname + '/' + 'model.sdf', 'w') as file:
                file.write(content)
            file.close()

    for dirname in os.listdir(SPAWN_PATH):
        if ('X' and 'Y' and 'Z') in dirname:
            content = ''
            with open(SPAWN_PATH + '/' + dirname + '/' + 'model.sdf', 'r') as file:
                content = file.read()
            file.close()
            content = content.replace('<scale>' + ((str(OLD_SCALE_FACTOR) + ' ') * 3).rstrip() + '</scale>',
                                    '<scale>' + ((str(NEW_SCALE_FACTOR) + ' ') * 3).rstrip() + '</scale>')
            with open(SPAWN_PATH + '/' + dirname + '/' + 'model.sdf', 'w') as file:
                file.write(content)
            file.close()

if __name__ == '__main__':
    scale_legos()