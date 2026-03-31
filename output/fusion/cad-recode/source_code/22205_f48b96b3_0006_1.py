import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,56,0))
r=w0.workplane(offset=-154/2).cylinder(154,18).union(w0.workplane(offset=23/2).cylinder(23,26)).union(w0.workplane(offset=44/2).cylinder(44,26))