import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,-3,0))
r=w0.workplane(offset=6/2).cylinder(6,100)