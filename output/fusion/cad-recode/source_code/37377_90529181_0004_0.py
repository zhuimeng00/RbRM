import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,-62,0))
r=w0.sketch().arc((-63,46),(58,-51),(-56,55)).segment((-56,46)).close().assemble().push([(-1,1)]).circle(31,mode='s').finalize().extrude(124)