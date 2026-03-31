import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,14,0))
r=w0.sketch().segment((-27,92),(-26,92)).arc((17,-97),(-10,95)).segment((-10,94)).segment((-12,94)).segment((-12,95)).arc((-19,94),(-25,92)).close().assemble().push([(-1,16)]).circle(22,mode='s').push([(0,65)]).circle(13,mode='s').finalize().extrude(-28)