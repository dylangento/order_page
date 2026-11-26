import sys, json, math, glob, numpy as np, xml.etree.ElementTree as ET
sys.path.insert(0, '/home/claude'); sys.path.insert(0, '/home/claude/lib')
from pack import stl, rpy, T, rotz, apply, cluster, normals_crease
from glb import read_glb, read_glb_scene
from meshops import taubin, split_plane, weld, text_quads, rect_tris
import gzip
from scipy.optimize import minimize

OUT = '/home/claude/web/m/'
S696 = '/home/claude/src/stand/Marvin M6-S-CCS-696-V4.0_Base_and_Stand_Asm urdf/meshes/'
LITE = '/home/claude/src/lite/'
GD = '/home/claude/gd/static/models/'
FR = '/home/claude/src/frame/marvin_frame_base_drawing260811/'
ALIGN = json.load(open(LITE + 'align.json'))

# material ids
SHELL, ALU, BLACK, PAINT, STEEL, CHROME, PLASTIC, GENERIC = range(8)

def srgb(h):
    h = h.lstrip('#'); return [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]

# ---------------- packing ----------------
def pack(parts, out, extra=None):
    blobs = []; meta = []; off = 0
    for name, tris, cols, mats, cell, crease in parts:
        cm = np.concatenate([cols, mats[:, None] / 7.0], 1)  # keep mat id through clustering
        pos, col, f = cluster(tris, cm, cell)
        P, C, N, F = normals_crease(pos, col, f, crease)
        lo, hi = P.min(0), P.max(0); ext = np.maximum(hi - lo, 1e-9)
        qp = np.round((P - lo) / ext * 65535).astype(np.uint16).view(np.uint8).reshape(-1, 6)
        qn = np.round(np.clip(N, -1, 1) * 127).astype(np.int8).view(np.uint8).reshape(-1, 3)
        qm = np.round(C[:, 3] * 7).astype(np.uint8)[:, None]
        qc = np.round(np.clip(C[:, :3], 0, 1) * 255).astype(np.uint8)
        vb = np.concatenate([qp, qn, qm, qc, np.zeros((len(P), 3), np.uint8)], 1).tobytes()
        idx32 = len(P) > 65535
        ib = F.astype(np.uint32 if idx32 else np.uint16).tobytes()
        pad = lambda b: b + b'\0' * ((-len(b)) % 4)
        vb, ib = pad(vb), pad(ib)
        meta.append(dict(name=name, vcount=len(P), icount=int(F.size), voff=off, ioff=off + len(vb), idx32=idx32,
                         lo=[round(float(x), 6) for x in lo], ext=[round(float(x), 6) for x in ext]))
        blobs += [vb, ib]; off += len(vb) + len(ib)
        print(f'  {name}: {len(tris)} -> {len(F)} tris, {round((len(vb) + len(ib)) / 1024)} KB')
    open(out + '.bin', 'wb').write(gzip.compress(b''.join(blobs), 9))
    m = dict(stride=16, parts=meta); m.update(extra or {})
    json.dump(m, open(out + '.json', 'w'))
    print(out, round(off / 1024), 'KB')

def solid(tris, color, mat):
    return np.tile(color, (len(tris), 1)).astype(float), np.full(len(tris), mat)

# ---------------- world frame from base_m6 ----------------
# base_m6 coordinates -> world (floor z=0, column centred at origin, robot faces +x)
W0 = T(R=rpy(0, 0, math.pi)) @ T((0.1723, 1.2629, 0.4531))
J0 = {'L': T((-0.17452969301375, -1.3098383288393, 0.749883376907328), rpy(-1.5707963267949, 0, 3.09467805449802)),
      'R': T((-0.17012, -1.2159, 0.74988), rpy(1.5708, 0, 3.0947))}

# ---------------- arm models ----------------
ARMS = {
    'm696': dict(fam='m6', d1=0.1745, up=0.287, fo=0.314, tcp=0.095),
    'm809': dict(fam='m6', d1=0.1745, up=0.400, fo=0.314, tcp=0.095),
    'm920': dict(fam='m6', d1=0.1745, up=0.400, fo=0.425, tcp=0.095),
    'l615': dict(fam='lite', d1=0.1586, up=0.264, fo=0.264, tcp=0.087),
    'l680': dict(fam='lite', d1=0.1586, up=0.329, fo=0.264, tcp=0.087),
    'l750': dict(fam='lite', d1=0.1586, up=0.400, fo=0.264, tcp=0.087),
}

def chain(a, q):
    # joint frames per the Marvin URDF convention (same for M6S and Lite)
    J = [T((0, 0, a['d1'])), T(R=rpy(1.5708, 0, 0)), T((0, a['up'], 0), rpy(-1.5708, 0, 0)),
         T((0.018, 0, 0), rpy(-1.5708, 0, math.pi)), T((0.018, -a['fo'], 0), rpy(-1.5708, 0, math.pi)),
         T(R=rpy(1.5708, -1.5708, 0)), T(R=rpy(1.5708, -1.5708, 0))]
    M = [np.eye(4)]; cur = np.eye(4)
    for i in range(7):
        cur = cur @ J[i] @ rotz(q[i]); M.append(cur)
    tcp = cur @ T((0, -a['tcp'], 0), rpy(1.5708, -1.5708, 0))
    return M, tcp  # M[0]=base, M[i]=link i

def stretch(v, ax, cut, d):
    v = v.copy(); m = v[..., ax] < cut; v[..., ax] = np.where(m, v[..., ax] - d, v[..., ax]); return v

def link_mesh(a, side, i):
    """triangles in link frame + per-tri material + colour"""
    if a['fam'] == 'm6':
        t = stl(S696 + (f'Base_{side}.STL' if i == 0 else f'Link{i}_{side}.STL'))
        base = ARMS['m696']
    else:
        if i == 0:
            t = stl(S696 + f'Base_{side}.STL') * [0.91, 0.91, 0.965]   # Lite base ~ scaled M6S base (only a coarse Lite base mesh exists)
        else:
            t = stl(LITE + f'Arm_{side}{i}_Link.STL') @ np.array(ALIGN[f'{side}{i}']).T
        base = ARMS['l615']
    if i == 3 and a['up'] != base['up']: t = stretch(t, 2, -0.12 if a['fam'] == 'm6' else -0.11, a['up'] - base['up'])
    if i == 5 and a['fo'] != base['fo']: t = stretch(t, 2, -0.105, a['fo'] - base['fo'])
    if i == 7:
        lim = -0.077 if a['fam'] == 'm6' else -0.068
        t = split_plane(t, 1, lim)
    extra = None
    if i == 5 and a['fam'] == 'lite':
        # 'ROBOT ARM' badge: black raised oval with white lettering
        z0, z1, rr = -0.0352, -0.0002, 0.0064
        def in_stad(p):
            zc = np.clip(p[..., 2], z0, z1); return np.hypot(p[..., 0], p[..., 2] - zc) < rr
        badge = np.zeros(len(t), bool); txt = []
        for sg in (1, -1):
            el = np.max(np.stack([np.linalg.norm(t[:, k] - t[:, (k + 1) % 3], axis=1) for k in range(3)], 1), 1)
            m = in_stad(t).all(1) & (sg * t[:, :, 1].min(1) > 0.0455) & (el < 0.0045)
            if m.sum() > 50:
                badge |= m; ytop = np.percentile(sg * t[m][:, :, 1], 95)
                rects = text_quads('ROBOT ARM', 0.027, 0.0048)
                txt.append(rect_tris(rects, np.array([0, sg * ytop, (z0 + z1) / 2]), np.array([0, 0, 1.0]), np.array([sg * 1.0, 0, 0]), np.array([0, sg * 1.0, 0]), 0.00025))
        extra = (badge, np.concatenate(txt) if txt else None)
    c = t.mean(1); mat = np.full(len(t), SHELL)
    if i == 0: mat[:] = ALU
    if i == 7: mat[c[:, 1] < lim] = ALU
    col = np.ones((len(t), 3))
    col[mat == ALU] = srgb('#cdd0d3')
    if extra is not None:
        badge, txt = extra
        mat[badge] = BLACK; col[badge] = srgb('#141517')
        if txt is not None:
            t = np.concatenate([t, txt]); mat = np.concatenate([mat, np.full(len(txt), GENERIC)]); col = np.concatenate([col, np.tile(srgb('#eef0f1'), (len(txt), 1))])
    return t, mat, col

def pose_for(a, side):
    sg = 1 if side == 'L' else -1
    Wb = W0 @ J0[side]
    def pts(q):
        M, tcp = chain(a, list(q) + [0, 0, 0])
        return [(Wb @ M[k])[:3, 3] for k in (2, 4, 6)], Wb @ tcp
    S = pts([0] * 4)[0][0]
    up_d = np.array([0.12, sg * 0.18, -1]); up_d /= np.linalg.norm(up_d)
    fo_d = np.array([1, -sg * 0.08, -0.12]); fo_d /= np.linalg.norm(fo_d)
    E_t = S + up_d * a['up']; W_t = E_t + fo_d * a['fo']; T_t = W_t + fo_d * a['tcp']
    def cost(q):
        (s, e, w), tc = pts(q)
        return 10 * np.sum((e - E_t) ** 2) + 10 * np.sum((w - W_t) ** 2) + np.sum((tc[:3, 3] - T_t) ** 2) + 1e-3 * np.sum(np.array(q) ** 2)
    best = None
    for seed in [[0, 1, 0, -1], [0, -1, 0, -1], [1.5, -1, -1, -1.5], [-1.5, -1, 1, -1.5], [1.5, 1, 1, -1.5], [-1.5, 1, -1, -1.5]]:
        r = minimize(cost, seed, method='Powell', bounds=[(-3.1, 3.1), (-2.09, 2.09), (-3.1, 3.1), (-2.53, 1.04)])
        if best is None or r.fun < best.fun: best = r
    q = list(best.x) + [0, 0, 0]
    # roll the wrist so the tool's camera side faces up
    return q, best.fun


_ENV = None
def env_mesh():
    """column/base + dual-arm frame in world (obstacles for the external cable)"""
    global _ENV
    if _ENV is None:
        b = stl(FR + 'base/base_urdf/base_m6/meshes/base_link.STL'); b = apply(W0, b[b.mean(1)[:, 2] < 0.6075])
        t = stl(FR + 'torso frame/torso_frame_urdf/Dual_arm_mount-m6/meshes/base_link.STL')
        Mt = J0['L'] @ np.linalg.inv(T((0.0470005033993383, 0, 0.142), rpy(-1.5707963267949, 0, -1.5707963267949)))
        _ENV = np.concatenate([b, apply(W0 @ Mt, t)])
    return _ENV

def arm_route(a, side, M, Wb, arm_tris):
    from route import SDF, raycast, relax, inside_count
    m6 = a['fam'] == 'm6'; sg = 1.0 if side == 'L' else -1.0
    o = np.array([0, sg, 0]); up = np.array([0, 0, 1.0]); fwd = np.array([1.0, 0, 0])
    L7 = Wb @ M[7]
    TT = L7 @ T((0, -a['tcp'], 0), rpy(1.5708, -1.5708, 0))           # tool frame (z along the tool)
    tool = lambda p: (TT @ np.r_[p, 1])[:3]
    obst = np.concatenate([arm_tris, env_mesh()])
    sdf = SDF(obst)
    CL = 0.0062                                                         # cable centre above surface at a clip
    # clips on the flat end faces of J2 (back face), J4 and J6 (outer faces), centred on the joint axis
    clips = []
    for k, pref in ((2, -fwd), (4, o), (6, o)):
        A = Wb @ M[k]; ax = A[:3, 2] * np.sign(A[:3, 2] @ pref); C = A[:3, 3]
        h, n = raycast(arm_tris, C + ax * 0.4, -ax)
        clips.append((h + n * CL, n))
    # far end of the cable: it leaves the bottom of the J1 base cylinder (Link0) and hangs down to a blue FAKRA plug
    B0 = Wb @ M[0]; down = np.array([0, 0, -1.0]); dl = np.linalg.inv(B0[:3, :3]) @ down
    zb = 0.045 if m6 else 0.042; Rb = (0.0565 if m6 else 0.0514)
    P0in = (B0 @ np.r_[dl * (Rb - 0.008) + [0, 0, zb], 1])[:3]
    P0 = (B0 @ np.r_[dl * (Rb + 0.0042) + [0, 0, zb], 1])[:3]
    mpB = P0 + down * 0.15 + fwd * 0.025 + o * 0.01; axB = -down          # connector: mating face down, cable leaves upwards
    bootB = mpB + axB * 0.042; preB = bootB + axB * 0.014
    tail, _ = relax([P0, P0 + down * 0.02, (P0 + preB) / 2 + fwd * 0.012, preB, bootB], [True, True, False, True, True], sdf, clear=0.0042, step=0.004, iters=200, hug=0.0)
    start = P0
    # internal routing: the cable leaves the arm at the centre of the base disc (where the arm meets the support frame),
    # runs inside the frame and drops out of the frame's front window to a blue FAKRA plug
    yi = sg * 0.016
    Ci = (B0 @ np.r_[0, 0, 0.02, 1])[:3]; C0 = (B0 @ np.r_[0, 0, -0.004, 1])[:3]
    k_in = [Ci, C0, np.array([0.0, sg * 0.03, B0[2, 3] - 0.005]), np.array([0.025, yi, B0[2, 3] - 0.04]),
            np.array([0.052, yi, 1.150]), np.array([0.072, yi, 1.125])]
    mpI = np.array([0.082, yi, 0.975]); axI = up; bootI = mpI + axI * 0.042; preI = bootI + axI * 0.014
    itail, isd = relax(k_in + [np.array([0.08, yi, 1.07]), preI, bootI], [True, True, True, False, True, True, False, True, True], sdf, clear=0.0035, step=0.004, iters=200, hug=0.0)
    print('   int tail', side, 'neg idx', np.where(isd < 0.002)[0].tolist(), 'of', len(isd))
    # end of the fixed route: on the Link7 surface on the camera side (tool +x), just behind the flange
    eh, en = raycast(arm_tris, tool(np.array([0.15, 0.0, -0.05])), TT[:3, :3] @ np.array([-1.0, 0, 0]))
    E = eh + en * 0.0042
    knots = [start]; fixed = [True]
    anchors = [start] + [c[0] for c in clips] + [E]
    for i, (p, n) in enumerate(clips):
        prv, nxt = anchors[i], anchors[i + 2]
        t_ = nxt - prv; t_ = t_ - n * (t_ @ n); t_ /= np.linalg.norm(t_)
        knots += [p - t_ * 0.009, p, p + t_ * 0.009]; fixed += [True, True, True]
        clips[i] = (p, t_, n)
    knots.append(E); fixed.append(True)
    X, sd = relax(knots, fixed, sdf, clear=0.0042)
    bad = inside_count(arm_tris, X[::3])
    print('   route', side, 'pts', len(X), 'min sd', round(float(sd.min()), 4), 'inside', int(bad.sum()))
    # flange ports (Link7 frame): GMSL exit hole on the +-x side, 8-pin connector on the -+z side
    yc = -0.0859 if m6 else -0.0775; R = 0.0395 if m6 else 0.0345; Rp = 0.039 if m6 else 0.034
    hole = (L7 @ np.r_[sg * R, yc, 0, 1])[:3]; hdir = L7[:3, :3] @ np.array([sg, 0, 0.0])
    pin = (L7 @ np.r_[0, yc, -sg * Rp, 1])[:3]; pdir = L7[:3, :3] @ np.array([0, 0, -sg])
    TI = np.linalg.inv(TT)
    loc = lambda p: (TI @ np.r_[p, 1])[:3]; locd = lambda d: TI[:3, :3] @ d
    r5 = lambda v: [round(float(x), 5) for x in v]
    return dict(ext=[r5(p) for p in X[::2]] + ([r5(X[-1])] if (len(X) - 1) % 2 else []),
                clips=[[r5(p), r5(t_), r5(n)] for p, t_, n in clips],
                holeT=r5(loc(hole)), holeDirT=r5(locd(hdir)), pinT=r5(loc(pin)), pinDirT=r5(locd(pdir)),
                endT=r5(loc(E)), out=r5(o),
                tail=[r5(p) for p in tail[::2]] + ([r5(tail[-1])] if (len(tail) - 1) % 2 else []), tailConn=[r5(mpB), r5(axB), r5(fwd)],
                itail=[r5(p) for p in itail[::2]] + ([r5(itail[-1])] if (len(itail) - 1) % 2 else []), itailConn=[r5(mpI), r5(axI), r5(fwd)])

def build_arm(aid):
    a = ARMS[aid]; parts = []; extra = {}
    for side in 'LR':
        q, err = pose_for(a, side)
        M, tcp = chain(a, q); Wb = W0 @ J0[side]
        tris = []; mats = []; cols = []
        for i in range(8):
            t, m, c = link_mesh(a, side, i)
            tris.append(apply(Wb @ M[i], t)); mats.append(m); cols.append(c)
        parts.append(('arm' + side, np.concatenate(tris), np.concatenate(cols), np.concatenate(mats), 0.00002, 32))
        extra['tcp' + side] = [round(float(x), 6) for x in (Wb @ tcp).T.reshape(-1)]  # column-major
        # ---- cable routing data (GMSL2 external route relaxed around the real meshes; flange ports) ----
        extra['route' + side] = arm_route(a, side, M, Wb, np.concatenate(tris))
        print(aid, side, 'pose err', round(err, 5), np.round(np.degrees(q[:4]), 1))
    pack(parts, OUT + 'arm-' + aid, extra)

# ---------------- stand, frame, controller ----------------
def build_common():
    parts = []
    # base (base_m6 minus the top bracket)
    b = stl(FR + 'base/base_urdf/base_m6/meshes/base_link.STL')
    c = b.mean(1); b = b[c[:, 2] < 0.6075]
    b = apply(W0, b)
    e = np.stack([np.linalg.norm(b[:, i] - b[:, (i + 1) % 3], axis=1) for i in range(3)], 1)
    ar = np.linalg.norm(np.cross(b[:, 1] - b[:, 0], b[:, 2] - b[:, 0]), axis=1)
    b = b[~(((ar / (e.max(1) + 1e-12) < 2e-4) & (e.max(1) > 0.05)) | ((e.max(1) > 0.3) & (b[:, :, 2].max(1) > 0.105) & (np.ptp(b[:, :, 2], 1) < 0.03)))]   # drop long sliver triangles
    c = b.mean(1); r = np.hypot(c[:, 0], c[:, 1])
    mat = np.full(len(b), PAINT); col = np.tile(srgb('#5e6164'), (len(b), 1))
    col_m = (r < 0.075) & (c[:, 2] > 0.105) & (c[:, 2] < 1.0)
    col[col_m] = srgb('#55585b')
    top = (r < 0.075) & (c[:, 2] >= 1.0); mat[top] = ALU; col[top] = srgb('#d0d3d5')
    knob = (r > 0.4) & (c[:, 2] > 0.115); mat[knob] = PLASTIC; col[knob] = srgb('#141517')
    low = c[:, 2] < 0.055
    pad_ = low & (r > 0.44); mat[pad_] = CHROME; col[pad_] = srgb('#d6d8da')
    cast = low & (r <= 0.44); mat[cast] = PLASTIC; col[cast] = srgb('#2a2b2d')
    parts.append(('base', b, col, mat, 0.00002, 30))
    # dual-arm mount (torso frame)
    t = stl(FR + 'torso frame/torso_frame_urdf/Dual_arm_mount-m6/meshes/base_link.STL')
    Mt = J0['L'] @ np.linalg.inv(T((0.0470005033993383, 0, 0.142), rpy(-1.5707963267949, 0, -1.5707963267949)))
    t = apply(W0 @ Mt, t)
    parts.append(('frame', t, *solid(t, srgb('#cfd2d5'), ALU), 0.00002, 30))
    # controller (converted STEP), recoloured black
    d = np.load('/home/claude/cb2.npz'); cb = d['tris'].astype(float) / 1000; cc = np.round(d['rgb'].astype(float), 3)
    encl = np.all(np.abs(cc - 0.741) < .002, 1); P = cb[encl].reshape(-1, 3); elo, ehi = P.min(0) - 0.003, P.max(0) + 0.003
    cen = cb.mean(1); keep = ~((((cen < elo) | (cen > ehi)).any(1)) & (cc[:, 1] > 0.85) & (cc[:, 0] < 0.1))
    # unplug: drop the cable heads / terminal plug sticking out of both port panels
    keep &= ~((cen[:, 2] < 0.0709) | (cen[:, 2] > 0.2004))
    cb, cc = cb[keep], cc[keep]
    col = cc.copy(); mat = np.full(len(cb), GENERIC)
    shell_ = np.zeros(len(cb), bool)
    for k in ([0.741] * 3, [0.882] * 3, [1.0] * 3, [0.686, 0.545, 0.545]):
        shell_ |= np.all(np.abs(cc - k) < .002, 1)
    cover = np.all(np.abs(cc - [0.863, 0.902, 0.929]) < .002, 1)
    col[shell_] = [0.075, 0.077, 0.082]; mat[shell_] = PLASTIC
    col[cover] = [0.13, 0.135, 0.14]; mat[cover] = PLASTIC
    gold = (cc[:, 0] > 0.9) & (cc[:, 1] > 0.75) & (cc[:, 2] < 0.6); mat[gold] = CHROME
    cb = cb @ rpy(math.pi / 2, 0, 0).T
    lo, hi = cb.reshape(-1, 3).min(0), cb.reshape(-1, 3).max(0)
    cb = cb - [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]]
    cb = cb @ rpy(0, 0, math.radians(-15)).T + [0.66, -0.36, 0.0]
    parts.append(('ctrl', cb, col, mat, 0.0007, 35))
    # controller-module accessories (colored meshes), laid out on the floor around the box for showcase
    kit = [('power-adapter', 0.92, -0.20, 10), ('emergency-stop', 0.92, -0.42, -5), ('communication-extension-leads', 0.92, -0.64, 0),
           ('red-black-extension-leads', 0.70, -0.64, 5), ('green-terminal-power-lead', 0.48, -0.62, -10), ('wrist-8pin-breakout-pair', 0.46, -0.42, 0)]
    for name, x, y, yaw in kit:
        t, c, m, r = read_glb_scene(f'/home/claude/src/cm/{name}.glb')
        t = t[..., [0, 2, 1]] * [1, -1, 1]          # glTF Y-up -> Z-up
        lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
        t = (t - [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]]) @ rpy(0, 0, math.radians(yaw)).T + [x, y, 0]
        parts.append(('kit_' + name, t, np.clip(c, 0, 1) ** (1 / 2.2), np.where(m > 0.5, ALU, GENERIC), 0.00002, 40))
    # power supply (Mean Well RSP-2000 STEP)
    d = np.load('/home/claude/psu.npz'); pt = d['tris'].astype(float) / 1000; pc = np.round(d['rgb'].astype(float), 3)
    pm = np.full(len(pt), GENERIC); pcol = pc.copy()
    case = np.all(np.abs(pc - [0.882, 0.949, 1.0]) < .003, 1) | np.all(np.abs(pc - [0.796, 0.737, 0.404]) < .003, 1); pm[case] = STEEL; pcol[case] = srgb('#a4a8ac')
    grey = np.all(np.abs(pc - 0.6) < .003, 1); pm[grey] = STEEL; pcol[grey] = srgb('#a9acb0')
    lo, hi = pt.reshape(-1, 3).min(0), pt.reshape(-1, 3).max(0)
    pt = pt - [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]]
    pt = pt @ rpy(0, 0, math.radians(20)).T + [0.70, 0.36, 0.0]
    parts.append(('psu', pt, pcol, pm, 0.0008, 35))
    pack(parts, OUT + 'common')

# ---------------- end effectors (TCP-local, z = tool axis) ----------------
def revolve(profile, seg=96):
    """profile: list of (r, z) going around; returns tris of the surface of revolution"""
    th = np.linspace(0, 2 * math.pi, seg + 1)
    pr = np.array(profile, float); tris = []
    for k in range(len(pr) - 1):
        (r0, z0), (r1, z1) = pr[k], pr[k + 1]
        for j in range(seg):
            a0, a1 = th[j], th[j + 1]
            p00 = [r0 * math.cos(a0), r0 * math.sin(a0), z0]; p01 = [r0 * math.cos(a1), r0 * math.sin(a1), z0]
            p10 = [r1 * math.cos(a0), r1 * math.sin(a0), z1]; p11 = [r1 * math.cos(a1), r1 * math.sin(a1), z1]
            tris += [[p00, p10, p11], [p00, p11, p01]]
    return np.array(tris)

def cyl(r, z0, z1, x=0, y=0, seg=24, axis='z'):
    t = revolve([(0, z0), (r, z0), (r, z1), (0, z1)], seg)
    if axis == 'x': t = t[..., [2, 1, 0]] * [1, 1, 1]
    return t + [x, y, 0]

def bolt_ring(n, R, rad, z0, z1, phase=0):
    return np.concatenate([cyl(rad, z0, z1, R * math.cos(phase + 2 * math.pi * k / n), R * math.sin(phase + 2 * math.pi * k / n), 16) for k in range(n)])

def ft_lb63():
    H = 0.033; R = 0.0315
    body = revolve([(0.0, 0), (R, 0), (R, 0.006), (R - 0.0006, 0.0062), (R - 0.0006, 0.0085), (R, 0.0087), (R, H - 0.0008), (R - 0.0008, H), (0.0165, H), (0.0165, H - 0.0035), (0.0, H - 0.0035)])
    ring = revolve([(R + 0.0002, 0.0062), (R + 0.0002, 0.0085)])
    bolts = bolt_ring(6, 0.0085, 0.0028, H - 0.0035, H - 0.0005, math.pi / 6)
    holes = bolt_ring(8, 0.0235, 0.0024, H - 0.0004, H + 0.0001, 0.2)
    conn = cyl(0.004, 0, 0.012, 0, 0, 20) @ rpy(0, math.pi / 2, 0).T + [R - 0.001, 0, 0.018]
    parts = [(body, ALU, '#c8cbce'), (ring, BLACK, '#151617'), (bolts, BLACK, '#2a2b2d'), (holes, BLACK, '#0c0c0d'), (conn, CHROME, '#c0c3c6')]
    return parts

def ft_kwr75b():
    H = 0.0315; R = 0.0375
    top = revolve([(0.016, H - 0.003), (0.016, H), (R - 0.001, H), (R, H - 0.001), (R, 0.019)])
    mid = revolve([(R, 0.019), (R - 0.0004, 0.018), (R - 0.0004, 0.0165), (R, 0.0165)])
    bot = revolve([(R, 0.0165), (R, 0.001), (R - 0.001, 0), (0, 0)])
    bore = revolve([(0.016, H - 0.003), (0.0, H - 0.003)])
    holes = bolt_ring(6, 0.027, 0.0022, H - 0.0004, H + 0.0001, 0)
    box = np.array(box_tris([R - 0.002, -0.007, 0.004], [R + 0.004, 0.007, 0.015]))
    conn = cyl(0.0042, 0, 0.012, 0, 0, 20) @ rpy(0, math.pi / 2, 0).T + [R + 0.004, 0, 0.0095]
    return [(top, ALU, '#d0d3d5'), (mid, BLACK, '#121314'), (bot, PLASTIC, '#1d1e20'), (bore, BLACK, '#080809'), (holes, BLACK, '#0c0c0d'), (box, PLASTIC, '#141516'), (conn, CHROME, '#c9ccce')]

def box_tris(lo, hi):
    x0, y0, z0 = lo; x1, y1, z1 = hi
    v = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]]
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return [[v[a], v[b], v[c]] for a, b, c in f]

def glbmesh(urdf, link):
    r = ET.parse(GD + urdf).getroot()
    fn = [l.find('visual/geometry/mesh').get('filename') for l in r.findall('link') if l.get('name') == link][0]
    t, _, _ = read_glb(GD + urdf.split('/')[0] + '/' + fn); return t

def joint_T(urdf, child):
    r = ET.parse(GD + urdf).getroot()
    j = [j for j in r.findall('joint') if j.find('child').get('link') == child][0]
    o = j.find('origin'); return T([float(x) for x in o.get('xyz').split()], rpy(*[float(x) for x in o.get('rpy').split()]))

def tag_dark(t, base_hex, mat, dark_hex='#141517'):
    return t, np.tile(srgb(base_hex), (len(t), 1)), np.full(len(t), mat)

def build_ee():
    parts = []
    def add(name, items, cell=0.0004, crease=35):
        tris = np.concatenate([i[0] for i in items]); mats = np.concatenate([np.full(len(i[0]), i[1]) for i in items])
        cols = np.concatenate([np.tile(srgb(i[2]), (len(i[0]), 1)) for i in items])
        parts.append((name, tris, cols, mats, cell, crease))
    add('ft_lb63', ft_lb63(), 0.0002, 30)
    add('ft_kwr', ft_kwr75b(), 0.0002, 30)
    U = 'skye/skye-696-d405ga-gripper.urdf'; V = 'skye/skye-696-shw5g.urdf'
    AL = np.load('/home/claude/src/ee_align.npy', allow_pickle=True).item()
    FTM = '/home/claude/src/ft/Marvin_Stand_M6_2026.4.8/meshes/'
    hr = lambda f, k: stl(f) @ AL[k][0].T + AL[k][1]
    # D405 kit: only the docs-site mesh exists (no high-res source)
    add('cam_d405', [(glbmesh(U, 'Arm_L8_Link'), PLASTIC, '#202124')], 0.00002, 45)
    # SHW5G kit: high-res bracket + camera, placed with the Pro-m6-696 URDF (Link8 at TCP, camera on Link8)
    cam = apply(T((0.06, 0, 0.057822), rpy(-1.5707, -0.00012967, 1.3615)), stl('/home/claude/src/Link_Left_wrist.STL'))
    add('cam_shw5g', [(stl('/home/claude/src/Link8_L.STL'), PLASTIC, '#202124'), (cam, PLASTIC, '#0f1011')], 0.00002, 35)
    # KernelMind Omni-Gripper (high-res, aligned to the Skye mounting)
    g = hr(FTM + 'Arm_L9_Link.STL', 'L9')
    f1 = apply(joint_T(U, 'Arm_L10_Link'), hr(FTM + 'Arm_L10_Link.STL', 'L10'))
    f2 = apply(joint_T(U, 'Arm_L11_Link'), hr(FTM + 'Arm_L11_Link.STL', 'L11'))
    add('grip', [(g, PLASTIC, '#1c1d1f'), (f1, ALU, '#c4c7ca'), (f2, ALU, '#c4c7ca')], 0.00002, 35)
    # GMSL2 FAKRA (blue) cable-end connector, local frame: mating face at origin, +z along the cable
    t, c, m, r = read_glb_scene('/home/claude/src/gmsl.glb')
    cen = t.mean(1); sel = (cen[:, 0] < -0.128)          # one connector end (x from -0.172 to -0.13)
    t, c, m = t[sel], c[sel], m[sel]
    t = t - [-0.1703, 0.009, 0.059]                       # socket opening -> origin
    t = t[..., [1, 2, 0]]                                 # local x (towards cable) -> z
    parts.append(('fakra', t, np.clip(c, 0, 1) ** (1 / 2.2), np.where(m > 0.5, ALU, GENERIC), 0.00002, 40))
    # cable clip: band around the cable + small foot, local z = cable direction, +x = towards the arm surface
    band = revolve([(0.0042, -0.004), (0.0042, 0.004), (0.0034, 0.004), (0.0034, -0.004), (0.0042, -0.004)], 40)
    foot = np.array(box_tris([0.0032, -0.006, -0.006], [0.0062, 0.006, 0.006]))
    parts.append(('clip', np.concatenate([band, foot]), np.tile(srgb('#16181a'), (len(band) + len(foot), 1)), np.full(len(band) + len(foot), PLASTIC), 0.00002, 40))
    # straight 8-pin plug (knurled silver body, cable straight out of the back): origin = mating face, +z = away from the flange
    body = revolve([(0.0, 0.0), (0.006, 0.0), (0.006, 0.0025), (0.0063, 0.0025), (0.0063, 0.0037), (0.0056, 0.0041), (0.0056, 0.0049), (0.0063, 0.0049), (0.0063, 0.0061), (0.0056, 0.0065), (0.0056, 0.0073), (0.0063, 0.0073), (0.0063, 0.0085), (0.0056, 0.0089), (0.0056, 0.0097), (0.0063, 0.0097), (0.0063, 0.0109), (0.0056, 0.0113), (0.0056, 0.0121), (0.0063, 0.0121), (0.0063, 0.0133), (0.0056, 0.0137), (0.0056, 0.0145), (0.0063, 0.0145), (0.0063, 0.0157), (0.0056, 0.0161), (0.0056, 0.0169), (0.0052, 0.0174), (0.0034, 0.0199), (0.0, 0.0199)], 48)
    parts.append(('pin8', body, np.tile(srgb('#c3c6c9'), (len(body), 1)), np.full(len(body), ALU), 0.00002, 30))
    # plug for the gripper's side port: origin = port face, +z = out of the port, x = along the tool axis
    plug = np.array(box_tris([-0.0092, -0.0052, -0.004], [0.0092, 0.0052, 0.010]))
    tail = revolve([(0.0, 0.009), (0.0042, 0.009), (0.0030, 0.016), (0.0, 0.016)], 32)
    parts.append(('gport', np.concatenate([plug, tail]), np.tile(srgb('#18191b'), (len(plug) + len(tail), 1)), np.full(len(plug) + len(tail), PLASTIC), 0.00002, 40))
    pack(parts, OUT + 'ee')

if __name__ == '__main__':
    import os; os.makedirs(OUT, exist_ok=True)
    what = sys.argv[1:] or ['common', 'ee'] + ['arm:' + k for k in ARMS]
    for w in what:
        if w == 'common': build_common()
        elif w == 'ee': build_ee()
        elif w.startswith('arm:'): build_arm(w[4:])
