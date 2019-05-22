3D Labprint-style wings
------------------------

Wing Recipe
------------

1. Create a sketch and draw the root wing section. I find the spline tool useful.
2. for simple parallel chord wings:
    - extrude the section to create the wing
3. for tapered or variable section wings
    - create a construction plane the correct spanwise distance from the root.
    - create a second sketch on this plane for the tip section. 
    - You can project the root section onto this tip sketch, then scale, rotate or offset it, or just draw a completely new wing section on this sketch
    - use the loft tool to join between the root and tip sections.
4. you should now have a solid, representing the wing. This will eventually be turned into a thin shell using the Shell Tool, however this should be done later, since having a solid filled wing object makes creation of the internal support structure (ribs and spars) easier. They can be made by creating parallel construction planes and using the boundaryFill tool to fill the region between the planes that intersects with the wing body.
5. adjust the wing object further in any way necessary, e.g. making cut-outs for ailerons etc.
6. create ribs and spars using the API scripts detailed below.
7. finally to make the wing thin skinned, use the "shell tool", in the mode where you select the root and tip faces to be removed, and select an internal thickness. I find 0.5mm works well.


Ensure slic3r settings are set to 1 perimeter, no top, bottom if wanted. Can have solid or filled infill now.
Cura seems slice such thin objects much better. Follow the notes on the 3D Labprint website for slicing models with Cura.



Slicing & Extrusion Settings:
---
Many of these are taken from 3DLabprint Gcode files:

- Temperature: nozzle 230/ bed 60. No cooling fan on nozzle.
- Layer Height 0.25 
- Moves 7800 mm/min  (130mm/s)
- Extrude 2520 mm/min = (42mm/s)
- Fill = 48mm/s

1st Layer speeds (0.5x speed during extrusion. Moves at full speed):
- 1st Layer Height 0.225
- Moves 7800 mm/min (130mm/s)
- Perimeter 1260 mm/min (21mm/s)

General:
- Turn on thin wall detection.
- extrusion multiplier 1.02-1.15 (1.02)
- retraction 0.7-1.5 (0.8)
- extra restart distance 0.05-0.12 (0.2)

---------------------

### Older Recipes
Kept for reference, but weren't as successful as approach above.

Wing-skin - create solid - using zero-infill takes care of it being printed as just the skin. 

Export 1 STL containing just the skins + a bounding box

Spars - also create solid, export all spars etc as a second STL, also including the same bounding box.

Exporting the same bounding box (which can be just a thin rectangular rim or similar), is important to ensure that the STLs naturally align in Slic3r

Simply turning off visibility of the objects in Fusion 360 before using the export STL menu works to control what's exported.

### Spar Recipe (Old)

1. Create Offset planes - LE, TE, then at offset distance from them
2. Create Sketch - project skin body into sketch - note this is the bounding box of skin.
- sketch rectange to this bounding box, and extrude it.
- to trim it to intersection with skin, use Modify/Combine Bodies: target = spar, tool = skin, type = intersect, keep tools = true

Load both parts simultaneously into slicer, and slice with 0% infill, 0 top layers, 0 bottom layers.

Limitations:
- all parts are vertical skins only - would be nice to have solid spars, or solid flanges for ends


