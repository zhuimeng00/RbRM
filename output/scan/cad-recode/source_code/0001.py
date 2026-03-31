import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,64))
w1=cq.Workplane('XY',origin=(0,0,19))
r=w0.sketch().segment((-72,17),(59,-29)).segment((64,-14)).segment((-66,31)).close().assemble().finalize().extrude(-138).union(w1.sketch().push([(-9,2)]).circle(20).circle(15,mode='s').finalize().extrude(-43))