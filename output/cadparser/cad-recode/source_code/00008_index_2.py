import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,13))
w1=cq.Workplane('XY',origin=(0,0,18))
r=w0.workplane(offset=-109/2).cylinder(109,29).union(w0.sketch().circle(37).circle(27,mode='s').finalize().extrude(-11)).union(w0.sketch().circle(50).reset().face(w0.sketch().segment((-25,-15),(-20,-15)).arc((0,-28),(20,-15)).segment((25,-15)).segment((25,15)).segment((20,15)).arc((0,28),(-20,15)).segment((-25,15)).close().assemble(),mode='s').finalize().extrude(40)).union(w1.sketch().circle(49).circle(46,mode='s').finalize().extrude(35))