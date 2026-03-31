import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,23,0))
r=w0.sketch().segment((-100,11),(-96,10)).arc((-41,-88),(70,-67)).segment((74,-68)).segment((76,-59)).arc((88,-40),(95,-18)).segment((100,-11)).segment((96,-10)).arc((41,88),(-70,67)).segment((-74,68)).segment((-76,59)).arc((-88,40),(-95,18)).close().assemble().finalize().extrude(-46)