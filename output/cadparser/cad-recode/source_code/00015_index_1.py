import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,30,0))
r=w0.sketch().segment((-96,1),(-50,-79)).segment((-50,-83)).segment((50,-83)).segment((50,-75)).segment((96,-1)).segment((50,79)).segment((50,83)).segment((-50,83)).segment((-50,75)).close().assemble().finalize().extrude(-60)