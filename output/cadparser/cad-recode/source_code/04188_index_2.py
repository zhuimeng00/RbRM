import cadquery as cq
w0=cq.Workplane('YZ',origin=(-33,0,0))
w1=cq.Workplane('YZ',origin=(-19,0,0))
r=w0.workplane(offset=-19/2).cylinder(19,66).union(w0.workplane(offset=128/2).cylinder(128,37)).union(w1.workplane(offset=-77/2).cylinder(77,37))