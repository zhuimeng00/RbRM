import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,50,0))
r=w0.workplane(offset=-149/2).cylinder(149,14).union(w0.sketch().segment((-21,-12),(1,-25)).segment((21,-12)).segment((21,12)).segment((-1,25)).close().assemble().finalize().extrude(18)).union(w0.workplane(offset=28/2).box(42,26,28))