import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,42,0))
r=w0.workplane(offset=-94/2).moveTo(-12.5,0).box(47,142,94).union(w0.sketch().segment((-32,-70),(58,-70)).segment((58,-46)).arc((11,2),(58,47)).segment((58,71)).segment((-32,71)).close().assemble().finalize().extrude(-47))