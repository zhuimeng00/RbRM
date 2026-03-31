import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,-69))
r=w0.workplane(offset=138/2).box(138,48,138)