import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,50))
r=w0.sketch().push([(-1,0)]).rect(162,60).push([(-51.5,-0.5)]).rect(41,39,mode='s').push([(-1,0)]).rect(40,40,mode='s').push([(54.5,-0.5)]).rect(29,41,mode='s').finalize().extrude(-100)