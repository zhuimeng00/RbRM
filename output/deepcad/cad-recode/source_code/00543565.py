import cadquery as cq
w0=cq.Workplane('YZ',origin=(-73,0,0))
r=w0.sketch().segment((-5,-28),(5,-28)).segment((5,-24)).arc((0,-25),(-5,-24)).close().assemble().finalize().extrude(-2).union(w0.workplane(offset=148/2).moveTo(-1.5,0).box(121,56,148))