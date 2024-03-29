#!/usr/bin/env python3
# wing rib creation settings file

# name of a file to log diagnostics and errors
LOGFILE = '/Users/andy/logs/script_output.log'

# -----------------------------------------
# Settings used by Wing Ribs script
# -----------------------------------------

# name of the wing body - used by WingRibs and CreateSpars
WING_BODY = 'skin'


RIB_ROOT_SKETCH = 'root-profile'

# name to give the created component
RIB_COMPONENT_NAME = 'ribs'

# spanwise thickness of rib
RIB_THICKNESS_CM = 0.12

# inset of rib from surface
RIB_INSET_CM = 0.08

# rib locations in cm spanwise at root.
RIB_STATIONS_CM = [0, 2, 4, 6, 8, 10 - RIB_THICKNESS_CM]

# are rib post locations proportional to chord, (e.g. for tapered wings) or absolute
RIB_POST_LOC_TYPE='absolute'


# chordwise positions of rib verticals posts in cm (at root position. locations proportional elsewhere)
# RIB_POST_ROOT_LOCS_CM = [2.5, 5.0, 7.5 , 10, 12.5]
RIB_POST_ROOT_LOCS_CM = [1.0, 2, 3, 4, 5]

# chordwise width of rib posts
RIB_POST_WIDTH_CM = 0.1

# length of side of the triangular pieces on top and bottom of rib posts
RIB_POST_TRIANGLE_LEN_CM = 0.2
