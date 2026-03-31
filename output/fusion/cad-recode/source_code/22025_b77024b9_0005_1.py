import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,65))
r=w0.sketch().segment((-65,-39),(66,-39)).segment((66,-21)).segment((39,-21)).segment((39,14)).segment((66,14)).segment((66,40)).segment((-65,40)).segment((-65,14)).segment((-39,14)).segment((-39,-21)).segment((-65,-21)).close().assemble().finalize().extrude(-131)