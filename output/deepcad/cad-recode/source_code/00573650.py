import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,3))
r=w0.sketch().segment((-70,-52),(54,-51)).segment((54,-76)).segment((66,-76)).segment((65,-51)).segment((66,-51)).segment((66,54)).segment((-70,54)).close().assemble().finalize().extrude(-6)