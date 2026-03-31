import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,-24,0))
w1=cq.Workplane('XY',origin=(0,0,29))
r=w0.sketch().segment((-94,-20),(-87,-24)).arc((-50,-73),(10,-85)).segment((17,-89)).segment((21,-81)).arc((53,-52),(64,-10)).segment((69,-2)).segment((62,2)).arc((25,51),(-35,63)).segment((-42,67)).segment((-46,59)).arc((-78,29),(-88,-13)).close().assemble().finalize().extrude(-18).union(w0.workplane(offset=60/2).moveTo(-11,-5).cylinder(60,84)).union(w0.workplane(offset=102/2).moveTo(-11,-5).cylinder(102,58)).union(w1.workplane(offset=-94/2).moveTo(-5,-5).cylinder(94,76))