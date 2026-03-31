import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,-8,0))
r=w0.workplane(offset=8/2).cylinder(8,100).union(w0.workplane(offset=21/2).cylinder(21,75))