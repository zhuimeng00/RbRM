import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,-33))
r=w0.workplane(offset=23/2).moveTo(0,-1).cylinder(23,94).union(w0.sketch().push([(0,-1)]).circle(70).circle(58,mode='s').finalize().extrude(82))