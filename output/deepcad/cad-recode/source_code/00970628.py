import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,-65))
r=w0.sketch().segment((-69,-28),(64,-28)).segment((64,-17)).arc((60,-16),(64,-15)).segment((64,29)).segment((-69,29)).close().assemble().push([(48,4)]).circle(10,mode='s').finalize().extrude(132).union(w0.workplane(offset=132/2).moveTo(0,0).box(120,56,132))