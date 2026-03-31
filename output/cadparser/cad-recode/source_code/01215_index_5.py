import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,26,0))
w1=cq.Workplane('XY',origin=(0,0,-18))
r=w0.sketch().push([(-7,-2)]).circle(21).circle(9,mode='s').finalize().extrude(-115).union(w0.sketch().segment((-32,-3),(-20,-24)).segment((19,-3)).segment((6,20)).segment((-20,20)).arc((-26,8),(-32,-3)).assemble().finalize().extrude(-84)).union(w0.sketch().push([(-7,-2)]).circle(19).circle(10,mode='s').finalize().extrude(36)).union(w1.sketch().arc((-3,64),(4,63),(10,58)).segment((10,52)).segment((23,52)).arc((15,68),(0,75)).arc((-2,70),(-3,64)).assemble().finalize().extrude(82))