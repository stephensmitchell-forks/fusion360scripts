#!/usr/bin/env python3
# wing creation settings file

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

# rib locations in cm spanwise at root.
RIB_STATIONS_CM = [0, 3, 6 - 0.12]

# spanwise thickness of rib
RIB_THICKNESS = "1.2 mm"

# inset of rib from surface
RIB_INSET = "0.8 mm"

# chordwise positions of rib verticals posts in cm (at root position. locations proportional elsewhere)
RIB_POST_ROOT_LOCS_CM = [2.5, 5.0, 7.5, 10, 12.5]

# chordwise width of rib posts
RIB_POST_WIDTH_CM = 0.1

# length of side of the triangular pieces on top and bottom of rib posts
RIB_POST_TRIANGLE_LEN_CM = 0.5

# -----------------------------------------
# settings for CreateSpars
# -----------------------------------------
SPARS_SKETCH = 'spars'
SPARS_COMPONENT_NAME = 'spars'
SPAR_THICKNESS_CM = 0.1
SPAR_CIRCLE_DIAMETER_CM = 0.8
SPAR_CIRCLE_SPACING_CM = 0.8
