import numpy as np,itertools
from scipy.spatial import cKDTree
def rots():
  out=[]
  for perm in itertools.permutations(range(3)):
    for s in itertools.product([1,-1],repeat=3):
      R=np.zeros((3,3))
      for i,(p,sg) in enumerate(zip(perm,s)): R[i,p]=sg
      if np.linalg.det(R)>0: out.append(R)
  return out
def sample(v,n=8000):
  v=v.reshape(-1,3);return v[np.random.default_rng(0).choice(len(v),min(n,len(v)),replace=False)]
def align(src,dst,scale=1.0,iters=30,try_rots=True):
  """find R,t s.t. src@R.T*scale+t ~ dst. returns (R,t,err)"""
  A=sample(src)*scale;B=sample(dst);tb=cKDTree(B);best=None
  for R in (rots() if try_rots else [np.eye(3)]):
    P=A@R.T;t=(B.min(0)+B.max(0))/2-(P.min(0)+P.max(0))/2
    d=tb.query(P+t)[0].mean()
    if best is None or d<best[0]: best=(d,R,t)
  d,R,t=best
  for _ in range(iters):  # ICP refine
    P=A@R.T+t;dd,ii=tb.query(P);m=dd<np.percentile(dd,90)
    X=A[m];Y=B[ii[m]];cx,cy=X.mean(0),Y.mean(0)
    U,S,Vt=np.linalg.svd((X-cx).T@(Y-cy));D=np.diag([1,1,np.sign(np.linalg.det(Vt.T@U.T))])
    R=Vt.T@D@U.T;t=cy-cx@R.T
  P=A@R.T+t;d=tb.query(P)[0]
  return R,t,float(np.median(d)),float(d.mean())
