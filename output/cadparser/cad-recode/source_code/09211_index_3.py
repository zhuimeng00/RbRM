import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,-46))
r=w0.workplane(offset=-40/2).cylinder(40,51).union(w0.workplane(offset=127/2).cylinder(127,58)).union(w0.workplane(offset=132/2).cylinder(132,29))