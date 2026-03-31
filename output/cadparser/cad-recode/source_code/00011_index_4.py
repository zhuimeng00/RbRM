import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,24))
r=w0.workplane(offset=-61/2).cylinder(61,93).union(w0.sketch().circle(78).circle(39,mode='s').finalize().extrude(-9)).union(w0.sketch().circle(78).circle(39,mode='s').finalize().extrude(13))