import numpy as np
from scipy.spatial import cKDTree

def surface_samples(tris, spacing=0.0015, seed=0):
    """points + outward normals sampled on the triangle soup"""
    rng = np.random.default_rng(seed)
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    n = np.cross(b - a, c - a); ar = np.linalg.norm(n, axis=1) / 2
    ok = ar > 1e-12; a, b, c, n, ar = a[ok], b[ok], c[ok], n[ok], ar[ok]
    n = n / (2 * ar[:, None])
    P = [(a + b + c) / 3]; N = [n]
    k = np.floor(ar / spacing ** 2).astype(int)
    idx = np.repeat(np.arange(len(a)), k)
    if len(idx):
        u = rng.random(len(idx)); v = rng.random(len(idx)); f = u + v > 1; u[f] = 1 - u[f]; v[f] = 1 - v[f]
        P.append(a[idx] + (b[idx] - a[idx]) * u[:, None] + (c[idx] - a[idx]) * v[:, None]); N.append(n[idx])
    return np.concatenate(P), np.concatenate(N)

class SDF:
    def __init__(self, tris):
        self.P, self.N = surface_samples(tris); self.tree = cKDTree(self.P); self.tris = tris
    def query(self, X):
        d, i = self.tree.query(X, k=6)
        q = self.P[i]; n = self.N[i]
        s = np.einsum('ijk,ijk->ij', X[:, None] - q, n)        # signed distance to each sample plane
        # take the nearest sample but trust its sign only if the neighbours agree (robust at edges)
        sd = np.where(np.median(s, 1) < 0, -d[:, 0], d[:, 0])
        return sd, n[:, 0], q[:, 0]

def raycast(tris, o, d):
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2 = b - a, c - a; p = np.cross(d, e2); det = e1 @ np.zeros(3) if False else np.einsum('ij,ij->i', e1, p)
    ok = np.abs(det) > 1e-14; inv = np.where(ok, 1 / np.where(ok, det, 1), 0)
    s = o - a; u = np.einsum('ij,ij->i', s, p) * inv
    qv = np.cross(s, e1); v = (qv @ d) * inv; t = np.einsum('ij,ij->i', e2, qv) * inv
    hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6)
    if not hit.any(): return None, None
    j = np.argmin(np.where(hit, t, np.inf)); n = np.cross(e1[j], e2[j]); n /= np.linalg.norm(n)
    if n @ d > 0: n = -n
    return o + d * t[j], n

def inside_count(tris, X, dirs=((0.577, 0.577, 0.577), (-0.6, 0.48, 0.64), (0.3, -0.9, 0.316))):
    """majority-vote ray parity; returns bool inside per point"""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]; e1, e2 = b - a, c - a
    res = []
    for x in X:
        votes = 0
        for d in dirs:
            d = np.array(d) / np.linalg.norm(d); p = np.cross(d, e2); det = np.einsum('ij,ij->i', e1, p)
            ok = np.abs(det) > 1e-14; inv = np.where(ok, 1 / np.where(ok, det, 1), 0)
            s = x - a; u = np.einsum('ij,ij->i', s, p) * inv; qv = np.cross(s, e1); v = (qv @ d) * inv; t = np.einsum('ij,ij->i', e2, qv) * inv
            votes += int(((ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-7)).sum()) % 2)
        res.append(votes >= 2)
    return np.array(res)

def resample(pts, step):
    pts = np.asarray(pts, float); seg = np.linalg.norm(np.diff(pts, axis=0), axis=1); s = np.r_[0, np.cumsum(seg)]
    n = max(2, int(np.ceil(s[-1] / step)) + 1); u = np.linspace(0, s[-1], n)
    return np.stack([np.interp(u, s, pts[:, j]) for j in range(3)], 1)

def relax(knots, fixed_k, sdf, clear, step=0.004, iters=600, lam=0.5, hug=0.25, reach=0.05):
    """knots: list of control points; fixed_k: bool per knot (kept exactly). Returns dense path that wraps
    tautly around the obstacle surface with at least `clear` clearance."""
    segs = []; fixed = []
    for i in range(len(knots) - 1):
        s = resample([knots[i], knots[i + 1]], step)
        if i: s = s[1:]
        segs.append(s); f = np.zeros(len(s), bool)
        if i == 0: f[0] = fixed_k[0]
        f[-1] = fixed_k[i + 1]; fixed.append(f)
    X = np.concatenate(segs); F = np.concatenate(fixed); F[0] = True; F[-1] = True
    free = ~F
    for it in range(iters):
        L = 0.5 * (X[:-2] + X[2:]) - X[1:-1]
        X[1:-1][free[1:-1]] += lam * L[free[1:-1]]
        sd, n, q = sdf.query(X)
        push = (sd < clear) & free
        X[push] += n[push] * (clear - sd[push])[:, None]
        near = (sd > clear) & (sd < reach) & free          # drape onto nearby surfaces like a tied-down cable
        X[near] -= n[near] * (hug * (sd[near] - clear))[:, None]
        if it % 50 == 49:   # keep spacing even
            keep = X.copy(); Fi = np.where(F)[0]; parts = []
            for a_, b_ in zip(Fi[:-1], Fi[1:]):
                seg = resample(keep[a_:b_ + 1], step); parts.append(seg if not parts else seg[1:])
            nf = [np.r_[True, np.zeros(len(p_) - 2, bool), True] if k == 0 else np.r_[np.zeros(len(p_) - 1, bool), True] for k, p_ in enumerate(parts)]
            X = np.concatenate(parts); F = np.concatenate(nf); free = ~F
    sd, _, _ = sdf.query(X)
    return X, sd
