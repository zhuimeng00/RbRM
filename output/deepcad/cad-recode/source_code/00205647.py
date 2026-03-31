import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,52))
r=w0.sketch().arc((-28,81),(1,-84),(29,79)).arc((24,69),(14,62)).segment((26,55)).segment((15,36)).arc((15,-6),(-14,29)).segment((-26,25)).close().assemble().finalize().extrude(-105).union(w0.sketch().segment((-30,80),(-20,67)).arc((-23,74),(-30,80)).assemble().finalize().extrude(-94))