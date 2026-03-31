import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,27,0))
w1=cq.Workplane('XY',origin=(0,0,-68))
r=w0.sketch().segment((-25,-18),(25,-18)).segment((25,18)).segment((-25,18)).segment((-25,1)).arc((-23,-1),(-25,-3)).close().assemble().finalize().extrude(2).union(w1.sketch().push([(-0.5,-1.5)]).rect(103,103).push([(-6,36)]).circle(24,mode='s').finalize().extrude(137))