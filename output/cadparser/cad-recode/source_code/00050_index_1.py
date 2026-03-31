import cadquery as cq
w0=cq.Workplane('YZ',origin=(17,0,0))
w1=cq.Workplane('YZ',origin=(40,0,0))
r=w0.sketch().circle(91).circle(20,mode='s').finalize().extrude(-54).union(w0.sketch().circle(92).circle(21,mode='s').finalize().extrude(20)).union(w1.sketch().circle(56).circle(21,mode='s').finalize().extrude(-80))