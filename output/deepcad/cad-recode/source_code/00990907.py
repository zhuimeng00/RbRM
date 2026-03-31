import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,-12))
r=w0.workplane(offset=5/2).cylinder(5,55).union(w0.sketch().rect(140,136).rect(112,106,mode='s').finalize().extrude(33))