import json,struct,numpy as np
CT={5120:np.int8,5121:np.uint8,5122:np.int16,5123:np.uint16,5125:np.uint32,5126:np.float32}
NC={'SCALAR':1,'VEC2':2,'VEC3':3,'VEC4':4}
def read_glb(path):
    b=open(path,'rb').read()
    jl=struct.unpack('<I',b[12:16])[0]; js=json.loads(b[20:20+jl]); bin_=b[20+jl+8:]
    def acc(i):
        a=js['accessors'][i]; bv=js['bufferViews'][a['bufferView']]
        off=bv.get('byteOffset',0)+a.get('byteOffset',0); n=a['count']; c=NC[a['type']]; dt=CT[a['componentType']]
        stride=bv.get('byteStride')
        arr=np.frombuffer(bin_,dtype=dt,count=n*c if not stride else None,offset=off) if not stride else None
        if stride:
            raw=np.frombuffer(bin_[off:off+stride*n],dtype=np.uint8).reshape(n,stride)[:,:c*np.dtype(dt).itemsize]
            arr=raw.copy().view(dt)
        arr=arr.reshape(n,c).astype(np.float64)
        if a.get('normalized'):
            arr/= {np.int8:127,np.uint8:255,np.int16:32767,np.uint16:65535}[dt]
        return arr
    tris=[];cols=[]
    for m in js['meshes']:
        for p in m['primitives']:
            P=acc(p['attributes']['POSITION'])
            I=acc(p['indices']).astype(int).reshape(-1,3) if 'indices' in p else np.arange(len(P)).reshape(-1,3)
            col=[0.8,0.8,0.8]
            if 'material' in p:
                mat=js['materials'][p['material']]; col=mat.get('pbrMetallicRoughness',{}).get('baseColorFactor',[.8,.8,.8,1])[:3]
            tris.append(P[I]);cols.append(np.tile(col,(len(I),1)))
    # node transforms ignored (assume identity)
    return np.concatenate(tris),np.concatenate(cols),js

def _quat(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def read_glb_scene(path):
    """triangles with node transforms applied; returns tris, linear rgb per tri, metallic per tri, roughness per tri"""
    b = open(path, 'rb').read()
    jl = struct.unpack('<I', b[12:16])[0]; js = json.loads(b[20:20 + jl]); bin_ = b[20 + jl + 8:]
    def acc(i):
        a = js['accessors'][i]; bv = js['bufferViews'][a['bufferView']]
        off = bv.get('byteOffset', 0) + a.get('byteOffset', 0); n = a['count']; c = NC[a['type']]; dt = CT[a['componentType']]
        st = bv.get('byteStride'); isz = np.dtype(dt).itemsize
        if st and st != c * isz:
            raw = np.frombuffer(bin_[off:off + st * n], dtype=np.uint8).reshape(n, st)[:, :c * isz]; arr = raw.copy().view(dt)
        else:
            arr = np.frombuffer(bin_, dtype=dt, count=n * c, offset=off)
        return arr.reshape(n, c).astype(np.float64)
    out_t, out_c, out_m, out_r = [], [], [], []
    def local(n):
        M = np.eye(4)
        if 'matrix' in n: return np.array(n['matrix']).reshape(4, 4).T
        S = np.diag(n.get('scale', [1, 1, 1])); R = _quat(n.get('rotation', [0, 0, 0, 1]))
        M[:3, :3] = R @ S; M[:3, 3] = n.get('translation', [0, 0, 0]); return M
    def visit(i, P):
        n = js['nodes'][i]; M = P @ local(n)
        if 'mesh' in n:
            for p in js['meshes'][n['mesh']]['primitives']:
                V = acc(p['attributes']['POSITION']); I = acc(p['indices']).astype(int).reshape(-1, 3) if 'indices' in p else np.arange(len(V)).reshape(-1, 3)
                V = V @ M[:3, :3].T + M[:3, 3]
                mat = js['materials'][p['material']] if 'material' in p else {}
                pb = mat.get('pbrMetallicRoughness', {})
                out_t.append(V[I]); out_c.append(np.tile(pb.get('baseColorFactor', [.8, .8, .8, 1])[:3], (len(I), 1)))
                out_m.append(np.full(len(I), pb.get('metallicFactor', 1.0))); out_r.append(np.full(len(I), pb.get('roughnessFactor', 1.0)))
        for c in n.get('children', []): visit(c, M)
    sc = js['scenes'][js.get('scene', 0)]
    for i in sc['nodes']: visit(i, np.eye(4))
    return np.concatenate(out_t), np.concatenate(out_c), np.concatenate(out_m), np.concatenate(out_r)
