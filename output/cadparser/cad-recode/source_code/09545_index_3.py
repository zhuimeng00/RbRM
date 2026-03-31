import cadquery as cq
w0=cq.Workplane('ZX',origin=(0,38,0))
w1=cq.Workplane('YZ',origin=(-44,0,0))
r=w0.sketch().segment((-50,-44),(50,-44)).segment((50,44)).segment((-50,44)).segment((-50,39)).arc((-81,0),(-50,-39)).close().assemble().finalize().extrude(6).union(w1.sketch().segment((-42,-81),(-36,-81)).segment((-36,46)).segment((-34,46)).segment((-34,47)).segment((-31,47)).segment((-31,46)).segment((38,46)).segment((38,-46)).segment((36,-46)).segment((36,-47)).segment((38,-47)).segment((38,-53)).segment((44,-53)).segment((44,53)).segment((-42,53)).close().assemble().finalize().extrude(88))