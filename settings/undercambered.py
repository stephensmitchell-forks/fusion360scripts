#!/usr/bin/env python3
# wing rib creation settings file

# name of a file to log diagnostics and errors
LOGFILE = '/Users/andy/logs/create-ribs.log'

# name to give the created component
CREATE_COMPONENT_NAME = 'ribs'

ROOT_SKETCH = 'root-profile'
TIP_SKETCH = 'tip-profile'
WING_BODY = 'skin'


# rib locations in cm spanwise from root.
RIB_STATIONS_CM = [0, 2, 4, 6, 8, 10 - 0.12]

# spanwise thickness of rib
RIB_THICKNESS = "1.2 mm"

# inset of rib from surface
RIB_INSET = "0.8 mm"

# chordwise positions of rib verticals posts in cm (at root position. locations proportional elsewhere)
# RIB_POST_ROOT_LOCS_CM = [2.5, 5.0, 7.5 , 10, 12.5]
RIB_POST_ROOT_LOCS_CM = [1.0, 2, 3, 4, 5]

# chordwise width of rib posts
RIB_POST_WIDTH_CM = 0.1

# length of side of the triangular pieces on top and bottom of rib posts
RIB_POST_TRIANGLE_LEN_CM = 0.2
