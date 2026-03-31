import cadquery as cq
w0=cq.Workplane('YZ',origin=(97,0,0))
r=w0.sketch().rect(46,12).push([(2,3)]).circle(1,mode='s').finalize().extrude(-194)