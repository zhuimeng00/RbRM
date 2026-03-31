import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,29,0))
r=w0.workplane(offset=-77/2).moveTo(-21,13.5).box(12,141,77).union(w0.sketch().push([(17.5,-18.5)]).rect(89,77).push([(38,-32)]).circle(12,mode='s').finalize().extrude(-26)).union(w0.sketch().segment((-23,84),(-19,84)).arc((-21,85),(-23,84)).assemble().finalize().extrude(19))