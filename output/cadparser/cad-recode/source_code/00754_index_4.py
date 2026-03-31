import cadquery as cq
w0=cq.Workplane('YZ',origin=(-94,0,0))
w1=cq.Workplane('YZ',origin=(-100,0,0))
r=w0.sketch().circle(27).circle(16,mode='s').finalize().extrude(173).union(w1.workplane(offset=103/2).cylinder(103,23))