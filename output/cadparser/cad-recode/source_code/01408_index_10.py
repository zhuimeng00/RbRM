import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,81,0))
w1=cq.Workplane('XY',origin=(0,0,-13))
r=w0.sketch().push([(0,0)]).circle(11).push([(0,0.5)]).rect(12,11,mode='s').finalize().extrude(-161).union(w0.workplane(offset=7/2).cylinder(7,25)).union(w1.sketch().push([(0,-14)]).circle(55).circle(39,mode='s').finalize().extrude(14))