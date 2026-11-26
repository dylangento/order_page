import numpy as np, struct, json, glob, math, sys, xml.etree.ElementTree as ET
S = '/home/claude/src/stand/Marvin M6-S-CCS-696-V4.0_Base_and_Stand_Asm urdf/'
FT = '/home/claude/src/ft/Marvin_Stand_M6_2026.4.8/'

def stl(p):
    b = open(p, 'rb').read(); n = struct.unpack('<I', b[80:84])[0]
    return np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))['v'].reshape(-1, 3, 3).astype(np.float64)

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]); Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]); Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def T(xyz=(0, 0, 0), R=np.eye(3)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = xyz; return M

def rotz(q): return T(R=rpy(0, 0, q))

def urdf_fk(path, q):
    r = ET.parse(path).getroot()
    joints = {}
    for j in r.findall('joint'):
        o = j.find('origin'); xyz = [float(v) for v in o.get('xyz').split()]; rp = [float(v) for v in o.get('rpy').split()]
        joints[j.find('child').get('link')] = (j.find('parent').get('link'), T(xyz, rpy(*rp)), j.get('name'), j.get('type'))
    W = {}
    def w(l):
        if l in W: return W[l]
        if l not in joints: W[l] = np.eye(4); return W[l]
        p, M, n, t = joints[l]
        W[l] = w(p) @ M @ (rotz(q.get(n, 0)) if t != 'fixed' else np.eye(4)); return W[l]
    for l in [x.get('name') for x in r.findall('link')]: w(l)
    return W

def apply(M, tris): return tris @ M[:3, :3].T + M[:3, 3]

# ---------- decimation + normals ----------
def cluster(tris, cols, cell):
    v = tris.reshape(-1, 3); c = np.repeat(cols, 3, 0)
    key = np.floor(v / cell).astype(np.int64)
    ck = np.round(c * 31).astype(np.int64)
    k = key[:, 0] * 73856093 ^ key[:, 1] * 19349663 ^ key[:, 2] * 83492791 ^ (ck[:, 0] * 1024 + ck[:, 1] * 32 + ck[:, 2]) * 2654435761
    uk, inv = np.unique(k, return_inverse=True)
    cnt = np.bincount(inv); pos = np.stack([np.bincount(inv, v[:, i]) for i in range(3)], 1) / cnt[:, None]
    col = np.stack([np.bincount(inv, c[:, i]) for i in range(c.shape[1])], 1) / cnt[:, None]
    f = inv.reshape(-1, 3)
    ok = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = f[ok]
    # drop duplicate faces
    fs = np.sort(f, 1); _, ui = np.unique(fs[:, 0] * 1e12 + fs[:, 1] * 1e6 + fs[:, 2], return_index=True)
    f = f[np.sort(ui)]
    return pos, col, f

def normals_crease(pos, col, f, crease_deg=40):
    fn = np.cross(pos[f[:, 1]] - pos[f[:, 0]], pos[f[:, 2]] - pos[f[:, 0]])
    area = np.linalg.norm(fn, axis=1); fnu = fn / (area[:, None] + 1e-20)
    nv = len(pos); cosT = math.cos(math.radians(crease_deg))
    # accumulate smooth normals per vertex
    acc = np.zeros((nv, 3))
    # angle-weighted vertex normals (robust against long sliver triangles in CAD exports)
    for i in range(3):
        e1 = pos[f[:, (i + 1) % 3]] - pos[f[:, i]]; e2 = pos[f[:, (i + 2) % 3]] - pos[f[:, i]]
        cosang = np.sum(e1 * e2, 1) / (np.linalg.norm(e1, axis=1) * np.linalg.norm(e2, axis=1) + 1e-20)
        np.add.at(acc, f[:, i], fnu * np.arccos(np.clip(cosang, -1, 1))[:, None])
    vn = acc / (np.linalg.norm(acc, axis=1, keepdims=True) + 1e-20)
    # corners where face normal deviates from vertex normal beyond crease -> split (use face normal)
    P = []; N = []; C = []
    corner_v = f.reshape(-1); corner_f = np.repeat(np.arange(len(f)), 3)
    smooth = np.sum(vn[corner_v] * fnu[corner_f], 1) > cosT
    # build unique (vertex, smooth) or (vertex, face) keys
    key = np.where(smooth, corner_v, nv + corner_f * 3 + np.tile([0, 1, 2], len(f)))
    uk, inv = np.unique(key, return_inverse=True)
    first = np.zeros(len(uk), int); first[inv[::-1]] = np.arange(len(inv))[::-1]
    nrm = np.where(smooth[first][:, None], vn[corner_v[first]], fnu[corner_f[first]])
    return pos[corner_v[first]], col[corner_v[first]], nrm, inv.reshape(-1, 3)

# ---------- scene ----------
def build(qL, qR, out):
    W = urdf_fk(glob.glob(S + 'urdf/*.urdf')[0], {**qL, **qR})
    parts = []
    def add(name, tris, color, cell, mat='fixed', cols=None, crease=40):
        if cols is None: cols = np.tile(np.array(color, float), (len(tris), 1))
        parts.append((name, tris, cols, cell, mat, crease))
    add('base', apply(W['Link_Base'], stl(S + 'meshes/Link_Base.STL')), (0.62, 0.64, 0.66), 0.004)
    add('frame', apply(W['Link_Stand'], stl(S + 'meshes/Link_Stand.STL')), (0.72, 0.74, 0.76), 0.0015)
    for s in 'LR':
        arm = [apply(W[f'Base_{s}'], stl(S + f'meshes/Base_{s}.STL'))] + [apply(W[f'Link{i}_{s}'], stl(S + f'meshes/Link{i}_{s}.STL')) for i in range(1, 8)]
        add('arm' + s, np.concatenate(arm), (1, 1, 1), 0.0016, 'shell', crease=50)
        # F/T sensor: attach like Pro URDF Arm_X8_Joint: Link7 * T(0,-0.095,0) rpy(0,1.5708,0)
        M = W[f'Link7_{s}'] @ T((0, -0.095, 0), rpy(0, 1.5708, 0))
        add('ft' + s, apply(M, stl(FT + f'meshes/Arm_{s}8_Link.STL')), (0.55, 0.57, 0.6), 0.0013)
    # control box (STEP, mm, Y-up) -> metres, Z-up, on floor beside the base
    d = np.load('/home/claude/cb.npz'); cb = d['tris'].astype(np.float64) / 1000; cc = d['rgb'].astype(np.float64)
    import os
    cr_ = np.round(cc, 3)
    encl = np.all(np.abs(cr_ - 0.741) < .002, 1); P = cb[encl].reshape(-1, 3); elo, ehi = P.min(0) - 0.003, P.max(0) + 0.003
    cen = cb.mean(1); outside = ((cen < elo) | (cen > ehi)).any(1)
    green = (cr_[:, 1] > 0.85) & (cr_[:, 0] < 0.1)
    keep = ~(outside & green); cb = cb[keep]; cc = cc[keep]
    rec = json.loads(os.environ.get('CBMAP', '{"0.741,0.741,0.741":[0.16,0.165,0.175],"0.863,0.902,0.929":[0.30,0.31,0.33],"0.882,0.882,0.882":[0.16,0.165,0.175],"1.0,1.0,1.0":[0.16,0.165,0.175]}'))
    for k, v in rec.items():
        kk = np.array([float(x) for x in k.split(',')])
        m = np.all(np.abs(np.round(cc, 3) - kk) < 0.002, 1); cc[m] = v
    cb = cb @ rpy(math.pi / 2, 0, 0).T
    lo, hi = cb.reshape(-1, 3).min(0), cb.reshape(-1, 3).max(0)
    cb = cb - [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]]
    cb = cb @ rpy(0, 0, math.radians(-25)).T + [0.62, -0.18, 0.0]
    add('ctrl', cb, None, 0.0011, 'fixed', cols=cc, crease=35)

    blobs = []; meta = []; off = 0
    for name, tris, cols, cell, mat, crease in parts:
        pos, col, f = cluster(tris, cols, cell)
        P, C, N, F = normals_crease(pos, col, f, crease)
        lo, hi = P.min(0), P.max(0); ext = np.maximum(hi - lo, 1e-9)
        qp = np.round((P - lo) / ext * 65535).astype(np.uint16)
        qn = np.round(N * 127).astype(np.int8)
        qc = np.round(np.clip(C, 0, 1) * 255).astype(np.uint8)
        idx32 = len(P) > 65535
        ib = F.astype(np.uint32 if idx32 else np.uint16).tobytes()
        vb = np.concatenate([qp.view(np.uint8).reshape(-1, 6), qn.view(np.uint8).reshape(-1, 3), qc, np.zeros((len(P), 0), np.uint8)], 1)
        # pad vertex stride to 12 bytes (6 pos + 3 nrm + 3 col)
        vbytes = vb.tobytes()
        def pad(b):
            return b + b'\0' * ((-len(b)) % 4)
        vb_ = pad(vbytes); ib_ = pad(ib)
        meta.append(dict(name=name, mat=mat, vcount=len(P), icount=int(F.size), voff=off, ioff=off + len(vb_), idx32=idx32,
                         lo=[round(x, 6) for x in lo], ext=[round(x, 6) for x in ext]))
        blobs += [vb_, ib_]; off += len(vb_) + len(ib_)
        print(name, len(tris), '->', len(F), 'tris', len(P), 'verts', round((len(vb_) + len(ib_)) / 1024), 'KB')
    open(out + '.bin', 'wb').write(b''.join(blobs))
    json.dump(dict(parts=meta, stride=12), open(out + '.json', 'w'))

if __name__ == '__main__':
    d = math.radians
    L = [85.5, -69.8, -76.8, -94.3, 0, 0, 0]; R = [-85.5, -69.8, 76.8, -94.3, 0, 0, 0]
    build({f'Joint{i+1}_L': d(v) for i, v in enumerate(L)}, {f'Joint{i+1}_R': d(v) for i, v in enumerate(R)}, sys.argv[1])
