import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,51))
w1=cq.Workplane('XY',origin=(0,0,17))
r=w0.sketch().circle(87).circle(67,mode='s').circle(34).circle(15,mode='s').finalize().extrude(-101).union(w1.workplane(offset=-34/2).cylinder(34,86))