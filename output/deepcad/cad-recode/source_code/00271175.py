import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,-55,0))
r=w0.sketch().segment((-46,-39),(64,-39)).segment((64,16)).arc((-4,21),(-46,71)).close().assemble().finalize().extrude(110)