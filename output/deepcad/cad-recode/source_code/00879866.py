import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,38,0))
w1=cq.Workplane('ZX',origin=(0,77,0))
r=w0.workplane(offset=-27/2).cylinder(27,65).union(w1.workplane(offset=-164/2).cylinder(164,49))