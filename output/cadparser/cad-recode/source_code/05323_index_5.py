import cadquery as cq
w0=cq.Workplane('YZ',origin=(-74,0,0))
r=w0.workplane(offset=148/2).moveTo(-2,0).box(126,2,148).union(w0.sketch().segment((61,9),(67,4)).segment((67,10)).arc((64,9),(61,9)).assemble().finalize().extrude(148))