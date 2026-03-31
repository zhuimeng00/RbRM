import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,-15,0))
r=w0.sketch().arc((-21,30),(-23,-3),(-1,-27)).segment((-1,-99)).segment((1,-99)).segment((1,-28)).arc((23,-10),(16,17)).arc((1,99),(-21,30)).assemble().finalize().extrude(24).union(w0.sketch().circle(99).circle(36,mode='s').finalize().extrude(29))