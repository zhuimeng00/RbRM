import cadquery as cq
w0=cq.Workplane('YZ',origin=(88,0,0))
r=w0.workplane(offset=-177/2).box(88,26,177).union(w0.sketch().segment((-44,-9),(-34,-9)).arc((0,-34),(34,-9)).segment((44,-9)).segment((44,9)).segment((34,9)).arc((0,34),(-34,9)).segment((-44,9)).close().assemble().finalize().extrude(1))