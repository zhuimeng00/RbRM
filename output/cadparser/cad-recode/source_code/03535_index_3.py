import cadquery as cq
w0=cq.Workplane('YZ',origin=(-96,0,0))
r=w0.sketch().arc((-2,-1),(0,-2),(2,-1)).segment((0,-1)).segment((0,1)).segment((2,1)).arc((0,2),(-2,1)).segment((-1,1)).segment((-1,-1)).close().assemble().finalize().extrude(-4).union(w0.workplane(offset=196/2).cylinder(196,2))