import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,-92,0))
r=w0.sketch().segment((-22,-21),(22,-21)).segment((22,21)).segment((19,21)).segment((19,18)).segment((16,18)).segment((16,21)).segment((-22,21)).close().assemble().finalize().extrude(175).union(w0.workplane(offset=192/2).cylinder(192,16))