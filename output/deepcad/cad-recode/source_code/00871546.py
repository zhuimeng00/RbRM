import cadquery as cq
w0=cq.Workplane('XY',origin=(0,0,10))
r=w0.sketch().arc((-1,-99),(1,-98),(3,-100)).arc((12,98),(-1,-99)).assemble().circle(52,mode='s').finalize().extrude(-20)