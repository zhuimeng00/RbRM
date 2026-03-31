import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,-43))
r=w0.sketch().segment((-90,-2),(-45,-77)).segment((-32,-77)).segment((45,-77)).segment((45,-70)).arc((67,-40),(77,-4)).segment((90,1)).segment((45,77)).segment((45,79)).segment((-45,79)).segment((-45,72)).arc((-67,42),(-77,4)).close().assemble().finalize().extrude(61).union(w0.workplane(offset=91/2).cylinder(91,73))