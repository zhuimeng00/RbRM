import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,-54))
r=w0.workplane(offset=-2/2).cylinder(2,36).union(w0.sketch().segment((-60,-40),(-39,-40)).segment((-40,-60)).segment((40,-60)).segment((39,-40)).segment((60,-40)).segment((60,40)).segment((39,40)).segment((40,60)).segment((-40,60)).segment((-39,40)).segment((-60,40)).close().assemble().finalize().extrude(116))