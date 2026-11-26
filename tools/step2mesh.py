# Minimal STEP (AP214 B-rep) tessellator: planes, cylinders, cones, spheres, tori.
# Output: npz with tris (N,3,3) in mm and per-triangle rgb.
import re, sys, math, numpy as np
from scipy.interpolate import BSpline

def parse(path):
    s = open(path, errors='ignore').read()
    s = s[s.index('DATA;'):]
    ents = {}
    for m in re.finditer(r'#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|ENDSEC)', s, re.S):
        ents[int(m.group(1))] = m.group(2).replace('\n', '').replace('\r', '')
    return ents

tok_re = re.compile(r"\s*(#\d+|'(?:[^']|'')*'|\.[A-Z_0-9]+\.|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|\$|\*|\(|\)|,|[A-Z_][A-Z_0-9]*)")

def parse_args(t):
    toks = tok_re.findall(t)
    pos = 0
    def val():
        nonlocal pos
        tk = toks[pos]; pos += 1
        if tk == '(':
            out = []
            while toks[pos] != ')':
                if toks[pos] == ',': pos += 1; continue
                out.append(val())
            pos += 1
            return out
        if tk.startswith('#'): return ('ref', int(tk[1:]))
        if tk.startswith("'"): return tk[1:-1]
        if tk.startswith('.'): return tk
        if tk in ('$', '*'): return None
        if re.match(r'[A-Z_]', tk):  # typed value e.g. LENGTH_MEASURE(1.)
            if pos < len(toks) and toks[pos] == '(':
                return (tk, val())
            return tk
        return float(tk)
    return val()

class Step:
    def __init__(self, path):
        self.raw = parse(path); self.cache = {}
    def get(self, i):
        if i in self.cache: return self.cache[i]
        t = self.raw[i]
        m = re.match(r'([A-Z_0-9]+)\s*(\(.*\))\s*$', t, re.S)
        if m: r = (m.group(1), parse_args(m.group(2)))
        else: r = ('COMPLEX', t)
        self.cache[i] = r; return r
    def R(self, v): return self.get(v[1])

def vec(st, ref):
    k, a = st.R(ref)
    if k == 'CARTESIAN_POINT': return np.array(a[1], float)
    if k == 'DIRECTION': d = np.array(a[1], float); return d / np.linalg.norm(d)
    if k == 'VECTOR': return vec(st, a[1]) * a[2]
    raise ValueError(k)

def axis(st, ref):
    k, a = st.R(ref)
    o = vec(st, a[1])
    z = vec(st, a[2]) if a[2] else np.array([0, 0, 1.])
    x = vec(st, a[3]) if len(a) > 3 and a[3] else (np.array([1, 0, 0.]) if abs(z[0]) < .9 else np.array([0, 1, 0.]))
    x = x - z * (x @ z); x /= np.linalg.norm(x); y = np.cross(z, x)
    return o, x, y, z

SEG = math.radians(12)

def sample_edge(st, ec_ref):
    k, a = st.R(ec_ref)  # EDGE_CURVE(name, v1, v2, curve, same_sense)
    p1 = vec(st, st.R(a[1])[1][1]); p2 = vec(st, st.R(a[2])[1][1])
    cref = a[3]; same = a[4] == '.T.'
    ck, ca = st.get(cref[1])
    # unwrap SURFACE_CURVE / SEAM_CURVE
    while ck in ('SURFACE_CURVE', 'SEAM_CURVE'):
        ck, ca = st.R(ca[1])
    if ck == 'LINE':
        pts = [p1, p2]
    elif ck == 'CIRCLE' or ck == 'ELLIPSE':
        o, x, y, z = axis(st, ca[1]); r1 = ca[2]; r2 = ca[3] if ck == 'ELLIPSE' else r1
        def ang(p):
            d = p - o; return math.atan2((d @ y) / r2, (d @ x) / r1)
        t1, t2 = ang(p1), ang(p2)
        if same:
            if t2 <= t1 + 1e-9: t2 += 2 * math.pi
        else:
            if t2 >= t1 - 1e-9: t2 -= 2 * math.pi
        n = max(2, int(abs(t2 - t1) / SEG) + 1)
        ts = np.linspace(t1, t2, n + 1)
        pts = [o + x * r1 * math.cos(t) + y * r2 * math.sin(t) for t in ts]
        pts[0] = p1; pts[-1] = p2
    elif ck == 'B_SPLINE_CURVE_WITH_KNOTS':
        deg = int(ca[1]); cp = np.array([vec(st, c) for c in ca[2]])
        mult = ca[6]; kn = ca[7]
        knots = np.repeat(kn, [int(m) for m in mult])
        try:
            bs = BSpline(knots, cp, deg)
            ts = np.linspace(knots[deg], knots[-deg - 1], max(8, len(cp) * 3))
            pts = list(bs(ts))
        except Exception:
            pts = list(cp)
        if np.linalg.norm(pts[0] - p1) > np.linalg.norm(pts[-1] - p1): pts = pts[::-1]
        pts[0] = p1; pts[-1] = p2
    else:
        # complex / rational splines etc: straight segment fallback
        pts = [p1, p2]
    return pts

def loop_points(st, loop_ref):
    k, a = st.R(loop_ref)
    if k == 'VERTEX_LOOP': return None
    out = []
    for oe in a[1]:
        ok, oa = st.R(oe)  # ORIENTED_EDGE(name,*,*,edge,orient)
        pts = sample_edge(st, oa[3])
        if oa[4] != '.T.': pts = pts[::-1]
        out.extend(pts[:-1])
    return np.array(out)

# ---------------- triangulation (earcut port, simplified) ----------------
def area2(p): return np.sum(p[:-1, 0] * p[1:, 1] - p[1:, 0] * p[:-1, 1]) + p[-1, 0] * p[0, 1] - p[0, 0] * p[-1, 1]

def earcut(outer, holes):
    """outer: (n,2) ccw; holes: list of (m,2) cw. returns triangles as index triples into concatenated array"""
    pts = [outer] + holes
    allp = np.concatenate(pts)
    idx = list(range(len(outer)))
    off = len(outer)
    hs = []
    for h in holes:
        hs.append(list(range(off, off + len(h)))); off += len(h)
    # bridge holes (sorted by max x)
    hs.sort(key=lambda h: -allp[h, 0].max())
    pending = [list(h) for h in hs]
    for h in hs:
        pending.pop(0)
        hi = max(h, key=lambda i: allp[i, 0]); hp = allp[hi]
        extra = [e for ph in pending for e in zip(ph, ph[1:] + ph[:1])]
        cand = sorted(range(len(idx)), key=lambda j: np.sum((allp[idx[j]] - hp) ** 2))
        best = None
        for j in cand[:200]:
            q = allp[idx[j]]
            if not seg_cross(allp, idx, hp, q, extra):
                best = j; break
        if best is None: best = cand[0]
        k = h.index(hi); hr = h[k:] + h[:k]
        idx = idx[:best + 1] + hr + [hi, idx[best]] + idx[best + 1:]
    return ear_clip(allp, idx), allp

def seg_cross(P, idx, a, b, extra=()):
    n = len(idx)
    I0 = np.array([idx[i] for i in range(n)] + [e[0] for e in extra]); I1 = np.array([idx[(i + 1) % n] for i in range(n)] + [e[1] for e in extra])
    C = P[I0]; D = P[I1]
    skip = (C == b).all(1) | (D == b).all(1) | (C == a).all(1) | (D == a).all(1)
    def crv(o, p, q): return (p[..., 0] - o[..., 0]) * (q[..., 1] - o[..., 1]) - (p[..., 1] - o[..., 1]) * (q[..., 0] - o[..., 0])
    A = np.broadcast_to(a, C.shape); B = np.broadcast_to(b, C.shape)
    x = (crv(A, B, C) * crv(A, B, D) < 0) & (crv(C, D, A) * crv(C, D, B) < 0)
    return bool((x & ~skip).any())

def cr(o, a, b): return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
def inter(a, b, c, d):
    return (cr(a, b, c) * cr(a, b, d) < 0) and (cr(c, d, a) * cr(c, d, b) < 0)

def ear_clip(P, idx):
    tris = []
    V = list(idx)
    guard = 0
    while len(V) > 3 and guard < 10 * len(idx) + 100:
        guard += 1
        n = len(V); found = False
        for i in range(n):
            a, b, c = V[i - 1], V[i], V[(i + 1) % n]
            if cr(P[a], P[b], P[c]) <= 1e-12: continue
            ok = True
            pa, pb, pc = P[a], P[b], P[c]
            for j in V:
                if j in (a, b, c): continue
                p = P[j]
                if (p == pa).all() or (p == pb).all() or (p == pc).all(): continue
                if cr(pa, pb, p) >= 0 and cr(pb, pc, p) >= 0 and cr(pc, pa, p) >= 0: ok = False; break
            if ok:
                tris.append((a, b, c)); V.pop(i); found = True; break
        if not found:  # degenerate: force clip
            tris.append((V[-1], V[0], V[1])); V.pop(0)
    if len(V) == 3: tris.append(tuple(V))
    return tris

def densify(loop, step):
    out = []
    n = len(loop)
    for i in range(n):
        a, b = loop[i], loop[(i + 1) % n]; L = np.linalg.norm(b - a); k = max(1, int(np.ceil(L / step)))
        for j in range(k): out.append(a + (b - a) * j / k)
    return np.array(out)

def delaunay_fill(allp, O, H):
    from scipy.spatial import Delaunay
    from matplotlib.path import Path
    ext = np.ptp(O, 0).max(); step = max(ext / 400, 1e-3)
    loops = [densify(O, step)] + [densify(h, step) for h in H]
    pts = np.concatenate(loops)
    # interior grid helps keep triangles well-shaped
    lo, hi = O.min(0), O.max(0); g = max(ext / 60, step * 2)
    gx, gy = np.meshgrid(np.arange(lo[0] + g / 2, hi[0], g), np.arange(lo[1] + g / 2, hi[1], g)); G = np.stack([gx.ravel(), gy.ravel()], 1)
    inside = Path(O).contains_points(G)
    for h in H: inside &= ~Path(h).contains_points(G, radius=-step)
    # keep grid points away from boundaries
    from scipy.spatial import cKDTree
    if inside.any():
        G = G[inside]; d_, _ = cKDTree(pts).query(G); G = G[d_ > g * 0.4]; pts = np.concatenate([pts, G])
    allp.resize(0, refcheck=False) if False else None
    try: d = Delaunay(pts)
    except Exception: return []
    t = d.simplices; c = pts[t].mean(1)
    ins = Path(O).contains_points(c)
    for h in H: ins &= ~Path(h).contains_points(c)
    out = []
    for a, b, c_ in t[ins]:
        out.append((a, b, c_) if cr(pts[a], pts[b], pts[c_]) > 0 else (a, c_, b))
    return out, pts

def _unused(allp, O, H):
    from scipy.spatial import Delaunay
    from matplotlib.path import Path
    try: d = Delaunay(allp)
    except Exception: return []
    t = d.simplices; c = allp[t].mean(1)
    ins = Path(O).contains_points(c)
    for h in H: ins &= ~Path(h).contains_points(c)
    out = []
    for a, b, c_ in t[ins]:
        out.append((a, b, c_) if cr(allp[a], allp[b], allp[c_]) > 0 else (a, c_, b))
    return out

# --------------- surfaces ----------------
class Surf:
    def __init__(self, st, ref):
        k, a = st.R(ref); self.k = k
        if k == 'COMPLEX' or k not in ('PLANE', 'CYLINDRICAL_SURFACE', 'CONICAL_SURFACE', 'SPHERICAL_SURFACE', 'TOROIDAL_SURFACE'):
            self.k = None; return
        self.o, self.x, self.y, self.z = axis(st, a[1])
        if k == 'CYLINDRICAL_SURFACE': self.r = a[2]
        if k == 'CONICAL_SURFACE': self.r = a[2]; self.ta = math.tan(a[3]) if abs(a[3]) < 3.2 else math.tan(math.radians(a[3]))
        if k == 'SPHERICAL_SURFACE': self.r = a[2]
        if k == 'TOROIDAL_SURFACE': self.R_ = a[2]; self.r = a[3]
    def to_uv(self, p):
        d = p - self.o
        lx, ly, lz = d @ self.x, d @ self.y, d @ self.z
        if self.k == 'PLANE': return np.stack([lx, ly], -1)
        th = np.arctan2(ly, lx)
        if self.k in ('CYLINDRICAL_SURFACE', 'CONICAL_SURFACE'): return np.stack([th, lz], -1)
        if self.k == 'SPHERICAL_SURFACE': return np.stack([th, np.arctan2(lz, np.hypot(lx, ly))], -1)
        if self.k == 'TOROIDAL_SURFACE': return np.stack([th, np.arctan2(lz, np.hypot(lx, ly) - self.R_)], -1)
    def from_uv(self, uv):
        u, v = uv[:, 0], uv[:, 1]
        if self.k == 'PLANE': return self.o + np.outer(u, self.x) + np.outer(v, self.y)
        if self.k == 'CYLINDRICAL_SURFACE': rr = np.full_like(u, self.r); zz = v
        elif self.k == 'CONICAL_SURFACE': rr = self.r + v * self.ta; zz = v
        elif self.k == 'SPHERICAL_SURFACE': rr = self.r * np.cos(v); zz = self.r * np.sin(v)
        elif self.k == 'TOROIDAL_SURFACE': rr = self.R_ + self.r * np.cos(v); zz = self.r * np.sin(v)
        return self.o + np.outer(rr * np.cos(u), self.x) + np.outer(rr * np.sin(u), self.y) + np.outer(zz, self.z)
    def scale(self):  # metric scale of u,v for subdivision checks (radians vs mm)
        return SEG

def unwrap(uv):
    u = np.unwrap(uv[:, 0]); return np.stack([u, uv[:, 1]], -1)

def subdiv(uv, tris, surf, maxd):
    # split triangles whose u or v span exceeds maxd (angular dims only)
    out = []
    uv = list(uv)
    stack = list(tris)
    ang_v = surf.k in ('SPHERICAL_SURFACE', 'TOROIDAL_SURFACE')
    cache = {}
    def mid(i, j):
        key = (min(i, j), max(i, j))
        if key not in cache:
            uv.append((np.asarray(uv[i]) + np.asarray(uv[j])) / 2); cache[key] = len(uv) - 1
        return cache[key]
    guard = 0
    while stack and guard < 200000:
        guard += 1
        a, b, c = stack.pop()
        A, B, C = np.asarray(uv[a]), np.asarray(uv[b]), np.asarray(uv[c])
        du = max(abs(A[0] - B[0]), abs(B[0] - C[0]), abs(A[0] - C[0]))
        dv = max(abs(A[1] - B[1]), abs(B[1] - C[1]), abs(A[1] - C[1])) if ang_v else 0
        if du > maxd or dv > maxd:
            # bisect the edge with largest angular span
            def sp(P, Q): return max(abs(P[0] - Q[0]), abs(P[1] - Q[1]) if ang_v else 0)
            e = max([(sp(A, B), 0), (sp(B, C), 1), (sp(C, A), 2)])[1]
            if e == 0: m = mid(a, b); stack += [(a, m, c), (m, b, c)]
            elif e == 1: m = mid(b, c); stack += [(a, b, m), (a, m, c)]
            else: m = mid(c, a); stack += [(a, b, m), (m, b, c)]
        else:
            out.append((a, b, c))
    return np.array(uv), out

def face_tris(st, face_ref):
    k, a = st.R(face_ref)  # ADVANCED_FACE(name,(bounds),surface,same_sense)
    surf = Surf(st, a[2])
    if surf.k is None: return None
    same = a[3] == '.T.'
    loops = []
    for b in a[1]:
        bk, ba = st.R(b)
        lp = loop_points(st, ba[1])
        if lp is None or len(lp) < 2: continue
        if ba[2] != '.T.': lp = lp[::-1]
        loops.append((bk == 'FACE_OUTER_BOUND', lp))
    if not loops: return None
    curved = surf.k != 'PLANE'
    uvl = []
    for outer, lp in loops:
        uv = surf.to_uv(lp)
        if curved: uv = unwrap(uv)
        uvl.append((outer, lp, uv))
    if curved:
        closed = [abs(uv[-1, 0] + (uv[1, 0] - uv[0, 0]) * 0 - uv[0, 0]) for _, _, uv in uvl]
        full = [abs((uv[:, 0].max() - uv[:, 0].min())) > 2 * math.pi - 0.3 and abs(np.unwrap(np.r_[uv[:, 0], uv[0, 0]])[-1] - uv[0, 0]) > 1 for _, _, uv in uvl]
        if any(full):
            fl = [uv for (o, l, uv), f in zip(uvl, full) if f]
            if len(fl) >= 2:
                v1, v2 = fl[0][:, 1].mean(), fl[1][:, 1].mean()
                n = 30
                us = np.linspace(0, 2 * math.pi, n + 1)
                grid = np.array([[u, v1] for u in us] + [[u, v2] for u in us])
                tris = []
                for i in range(n):
                    tris += [(i, i + 1, n + 1 + i), (i + 1, n + 2 + i, n + 1 + i)]
                P = surf.from_uv(grid); T = P[np.array(tris)]
                return orient(T, surf, same)
            return None  # caps etc: skip
        # align hole loops near outer's u range
        ou = [uv for o, l, uv in uvl if o] or [uvl[0][2]]
        c0 = ou[0][:, 0].mean()
        new = []
        for o, l, uv in uvl:
            sh = round((c0 - uv[:, 0].mean()) / (2 * math.pi)) * 2 * math.pi
            new.append((o, l, uv + [sh, 0]))
        uvl = new
    # scale u for curved surfaces so earcut sees roughly metric space
    sc = np.array([1., 1.])
    if surf.k in ('CYLINDRICAL_SURFACE', 'CONICAL_SURFACE'): sc = np.array([max(surf.r, 1e-3), 1.])
    if surf.k == 'SPHERICAL_SURFACE': sc = np.array([surf.r, surf.r])
    if surf.k == 'TOROIDAL_SURFACE': sc = np.array([surf.R_, surf.r])
    outer = [uv for o, l, uv in uvl if o]
    holes = [uv for o, l, uv in uvl if not o]
    if not outer:
        outer = [max(holes, key=lambda u: abs(area2(u * sc)))]; holes = [h for h in holes if h is not outer[0]]
    O = outer[0]
    if area2(O * sc) < 0: O = O[::-1]
    H = [h[::-1] if area2(h * sc) > 0 else h for h in holes]
    if len(O) < 3: return None
    tris, allp = earcut(O * sc, [h * sc for h in H])
    exp = abs(area2(O * sc)) / 2 - sum(abs(area2(h * sc)) / 2 for h in H)
    got = sum(abs(cr(allp[a_], allp[b_], allp[c_])) / 2 for a_, b_, c_ in tris)
    if exp > 0 and (abs(got - exp) > 0.01 * exp + 1e-6 or len(H) > 6):
        r_ = delaunay_fill(allp, O * sc, [h * sc for h in H])
        if r_: tris, allp = r_
    uv = allp / sc
    if curved:
        uv, tris = subdiv(uv, tris, surf, math.radians(25))
    if not tris: return None
    P = surf.from_uv(uv)
    T = P[np.array(tris)]
    return orient(T, surf, same)

def orient(T, surf, same):
    # make winding agree with surface normal (outward if same_sense)
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    c = T.mean(1)
    if surf.k == 'PLANE': sn = np.tile(surf.z, (len(T), 1))
    else:
        d = c - surf.o; ax = surf.z
        radial = d - np.outer(d @ ax, ax)
        if surf.k == 'SPHERICAL_SURFACE': sn = d
        elif surf.k == 'TOROIDAL_SURFACE':
            rn = radial / (np.linalg.norm(radial, axis=1, keepdims=True) + 1e-12)
            sn = d - rn * surf.R_
        else: sn = radial
    if not same: sn = -sn
    flip = np.sum(n * sn, 1) < 0
    T[flip] = T[flip][:, [0, 2, 1]]
    return T

def color_map(st):
    face_col = {}; solid_col = {}
    for i, t in st.raw.items():
        if not t.startswith('STYLED_ITEM'): continue
        k, a = st.get(i)
        try:
            rgb = find_rgb(st, a[1])
        except Exception:
            rgb = None
        if rgb is None: continue
        tgt = a[2][1]; tk = st.get(tgt)[0]
        if tk == 'ADVANCED_FACE': face_col[tgt] = rgb
        else: solid_col[tgt] = rgb
    return face_col, solid_col

def find_rgb(st, node, depth=0):
    if depth > 60: return None
    if isinstance(node, list):
        for x in node:
            r = find_rgb(st, x, depth + 1)
            if r is not None: return r
        return None
    if isinstance(node, tuple) and node[0] == 'ref':
        k, a = st.R(node)
        if k == 'COLOUR_RGB': return tuple(a[1:4])
        if k == 'DRAUGHTING_PRE_DEFINED_COLOUR':
            return {'white': (1, 1, 1), 'black': (0, 0, 0), 'red': (1, 0, 0), 'green': (0, 1, 0), 'blue': (0, 0, 1), 'yellow': (1, 1, 0)}.get(a[0], (.7, .7, .7))
        return find_rgb(st, a, depth + 1)
    return None

def main(path, out):
    st = Step(path)
    fc, sc = color_map(st)
    allT = []; allC = []; allS = []; skipped = 0; total = 0
    for i, t in st.raw.items():
        if not t.startswith('MANIFOLD_SOLID_BREP') and not t.startswith('BREP_WITH_VOIDS'): continue
        k, a = st.get(i)
        shell = st.R(a[1])
        scol = sc.get(i, (.6, .6, .6))
        for f in shell[1][1]:
            total += 1
            try:
                T = face_tris(st, f)
            except Exception as e:
                T = None
            if T is None or len(T) == 0: skipped += 1; continue
            try:
                fk, fa = st.get(f[1])
                bp = np.concatenate([loop_points(st, st.R(b)[1][1]) for b in fa[1] if st.R(st.R(b)[1][1])[0] == 'EDGE_LOOP'])
                s_ = Surf(st, fa[2]); pad = (getattr(s_, 'r', 0) or 0) * 1.05 + 0.5
                P = T.reshape(-1, 3)
                if (P < bp.min(0) - pad).any() or (P > bp.max(0) + pad).any(): skipped += 1; continue
            except Exception:
                pass
            allT.append(T); allC.append(np.tile(fc.get(f[1], scol), (len(T), 1))); allS.append(np.full(len(T), i))
    T = np.concatenate(allT); C = np.concatenate(allC)
    print('faces', total, 'skipped', skipped, 'tris', len(T), 'bbox', T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0))
    print('colors', np.unique(np.round(C, 3), axis=0, return_counts=True))
    np.savez_compressed(out, tris=T.astype(np.float32), rgb=C.astype(np.float32), solid=np.concatenate(allS))

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])

# ---------------- assembly support ----------------
def _ax_mat(st, ref):
    o, x, y, z = axis(st, ref); M = np.eye(4); M[:3, 0] = x; M[:3, 1] = y; M[:3, 2] = z; M[:3, 3] = o; return M

def main_asm(path, out):
    st = Step(path)
    fc, sc = color_map(st)
    reps = {}
    for i, t in st.raw.items():
        m = re.match(r'([A-Z_]*SHAPE_REPRESENTATION)\s*\(', t)
        if m and 'RELATIONSHIP' not in t[:60]:
            k, a = st.get(i); reps[i] = [x[1] for x in a[1] if isinstance(x, tuple)]
    edges = []; same = []
    for i, t in st.raw.items():
        if t.startswith('(') and 'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION' in t:
            r = re.search(r"REPRESENTATION_RELATIONSHIP\s*\(\s*'[^']*'\s*,\s*'[^']*'\s*,\s*#(\d+)\s*,\s*#(\d+)", t)
            tr = re.search(r'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)', t)
            k, a = st.get(int(tr.group(1)))
            M = _ax_mat(st, a[3]) @ np.linalg.inv(_ax_mat(st, a[2]))
            edges.append((int(r.group(2)), int(r.group(1)), M))  # parent, child
        elif t.startswith('SHAPE_REPRESENTATION_RELATIONSHIP'):
            k, a = st.get(i); same.append((a[2][1], a[3][1]))
    children = {}
    for p, c, M in edges: children.setdefault(p, []).append((c, M))
    sib = {}
    for a, b in same: sib.setdefault(a, []).append(b); sib.setdefault(b, []).append(a)
    childset = {c for _, c, _ in edges}
    roots = [r for r in reps if r not in childset and any(r == p for p, _, _ in edges)] or [r for r in reps if r not in childset]
    cache = {}
    def solid_tris(sid):
        if sid in cache: return cache[sid]
        k, a = st.get(sid); shell = st.R(a[1]); scol = sc.get(sid, (.6, .6, .6)); T_ = []; C_ = []
        for f in shell[1][1]:
            try: T = face_tris(st, f)
            except Exception: T = None
            if T is None or len(T) == 0: continue
            T_.append(T); C_.append(np.tile(fc.get(f[1], scol), (len(T), 1)))
        cache[sid] = (np.concatenate(T_), np.concatenate(C_)) if T_ else None
        return cache[sid]
    allT = []; allC = []
    def visit(rep, M, seen):
        if rep in seen: return
        seen = seen | {rep}
        for it in reps.get(rep, []):
            if st.get(it)[0] in ('MANIFOLD_SOLID_BREP', 'BREP_WITH_VOIDS'):
                r = solid_tris(it)
                if r: allT.append(r[0] @ M[:3, :3].T + M[:3, 3]); allC.append(r[1])
        for s2 in sib.get(rep, []): visit(s2, M, seen)
        for c, Mc in children.get(rep, []): visit(c, M @ Mc, seen)
    for r in roots: visit(r, np.eye(4), frozenset())
    T = np.concatenate(allT); C = np.concatenate(allC)
    print('roots', roots, 'tris', len(T), 'bbox', T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0))
    np.savez_compressed(out, tris=T.astype(np.float32), rgb=C.astype(np.float32))
