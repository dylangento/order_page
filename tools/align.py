import sys,itertools,numpy as np,xml.etree.ElementTree as ET
sys.path.insert(0,'.');sys.path.insert(0,'lib')
from pack import stl
from glb import read_glb
from scipy.spatial import cKDTree
M='gd/static/models/luna/'
r=ET.parse(M+'luna-615.urdf').getroot()
mesh={l.get('name'):l.find('visual/geometry/mesh').get('filename') for l in r.findall('link') if l.find('visual/geometry/mesh') is not None}
rots=[]
for perm in itertools.permutations(range(3)):
  for s in itertools.product([1,-1],repeat=3):
    R=np.zeros((3,3))
    for i,(p,sg) in enumerate(zip(perm,s)): R[i,p]=sg
    if np.linalg.det(R)>0: rots.append(R)
out={}
for side in 'LR':
  for i in range(1,8):
    hv=stl(f'src/lite/Arm_{side}{i}_Link.STL').reshape(-1,3);hv=hv[::max(1,len(hv)//6000)]
    g,_,_=read_glb(M+mesh[f'Link{i}_{side}']);gv=g.reshape(-1,3);gv=gv[::max(1,len(gv)//6000)]
    t=cKDTree(gv);best=None
    for R in rots:
      d=t.query(hv@R.T)[0].mean()
      if best is None or d<best[0]: best=(d,R)
    out[f'{side}{i}']=best[1].tolist();print(side,i,round(best[0]*1000,2),'mm',best[1].astype(int).tolist() if not np.allclose(best[1],np.eye(3)) else 'I')
import json;json.dump(out,open('src/lite/align.json','w'))
