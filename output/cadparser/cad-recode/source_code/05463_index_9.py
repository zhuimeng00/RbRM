import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,-99))
w1=cq.Workplane('ZX',origin=(0,-3,0))
r=w0.workplane(offset=10/2).cylinder(10,12).union(w0.workplane(offset=178/2).cylinder(178,10)).union(w1.sketch().push([(74,0)]).circle(26).circle(20,mode='s').finalize().extrude(10))