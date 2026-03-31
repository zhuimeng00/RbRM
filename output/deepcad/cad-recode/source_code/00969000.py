import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,86,0))
r=w0.workplane(offset=-173/2).box(14,98,173).union(w0.sketch().segment((-7,-49),(7,-49)).segment((7,49)).segment((-7,49)).segment((-7,20)).arc((-3,17),(-7,14)).close().assemble().finalize().extrude(3))