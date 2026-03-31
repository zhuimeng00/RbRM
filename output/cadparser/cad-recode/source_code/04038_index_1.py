import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,-32,0))
r=w0.sketch().segment((-91,-4),(-15,-4)).segment((-15,-38)).segment((74,-38)).segment((74,28)).segment((-91,28)).close().assemble().finalize().extrude(64)