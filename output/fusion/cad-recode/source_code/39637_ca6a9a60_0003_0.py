import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,4,0))
r=w0.sketch().segment((-66,-75),(65,-75)).segment((65,74)).segment((-66,74)).segment((-66,48)).arc((-63,46),(-66,44)).close().assemble().push([(-1,-1)]).circle(21,mode='s').finalize().extrude(-8)