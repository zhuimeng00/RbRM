import cadquery as cq
w0=cq.Workplane('YZ',origin=(-7,0,0))
w1=cq.Workplane('ZX',origin=(0,-11,0))
r=w0.sketch().segment((92,-1),(97,-1)).segment((97,4)).arc((95,3),(92,4)).close().assemble().finalize().extrude(-19).union(w0.workplane(offset=27/2).moveTo(-11,0).cylinder(27,45)).union(w1.workplane(offset=108/2).moveTo(0,-22).cylinder(108,8))