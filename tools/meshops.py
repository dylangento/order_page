import numpy as np
def weld(tris, tol=1e-6):
    v = tris.reshape(-1, 3); k = np.round(v / tol).astype(np.int64)
    _, idx, inv = np.unique(k, axis=0, return_index=True, return_inverse=True)
    return v[idx].copy(), inv.reshape(-1, 3)
def taubin(tris, iters=10, lam=0.5, mu=-0.53, tol=2e-5):
    V, F = weld(tris, tol)
    e = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]); e = np.concatenate([e, e[:, ::-1]])
    e = np.unique(e, axis=0); n = len(V)
    deg = np.bincount(e[:, 0], minlength=n).astype(float)[:, None]
    # boundary vertices stay fixed
    ue = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), 1)
    u, c = np.unique(ue, axis=0, return_counts=True); bnd = np.zeros(n, bool); bnd[u[c == 1].ravel()] = True
    for _ in range(iters):
        for f in (lam, mu):
            s = np.zeros_like(V); np.add.at(s, e[:, 0], V[e[:, 1]])
            d = s / np.maximum(deg, 1) - V; d[bnd] = 0; V = V + f * d
    return V[F]
def split_plane(tris, axis, val):
    """split triangles crossing plane coord[axis]=val so per-triangle classification has a clean edge"""
    s = tris[:, :, axis] - val
    cross = (s.min(1) < -1e-9) & (s.max(1) > 1e-9)
    keep = tris[~cross]; out = [keep]
    for t, d in zip(tris[cross], s[cross]):
        pos = d > 0
        lone = np.where(pos)[0] if pos.sum() == 1 else np.where(~pos)[0]
        i = lone[0]; j, k = (i + 1) % 3, (i + 2) % 3
        a, b, c = t[i], t[j], t[k]
        pj = a + (b - a) * (d[i] / (d[i] - d[j])); pk = a + (c - a) * (d[i] / (d[i] - d[k]))
        out.append(np.array([[a, pj, pk], [pj, b, c], [pj, c, pk]]))
    return np.concatenate(out)

def text_quads(text, length, height, px=0.00016, font='DejaVuSans-Bold.ttf'):
    """rasterise text, return list of rectangles (u0,u1,v0,v1) in metres, centred at 0; u = reading dir, v = up"""
    from PIL import Image, ImageDraw, ImageFont
    f = ImageFont.truetype(font, 200)
    l, t, r, b = f.getbbox(text)
    im = Image.new('L', (r - l + 4, b - t + 4), 0); ImageDraw.Draw(im).text((2 - l, 2 - t), text, fill=255, font=f)
    W, H = im.size; sc = min(length / W, height / H)
    nw, nh = max(1, int(W * sc / px)), max(1, int(H * sc / px))
    a = np.array(im.resize((nw, nh), Image.LANCZOS)) > 110
    rects = []
    for j in range(nh):
        row = a[nh - 1 - j]; i = 0
        while i < nw:
            if row[i]:
                k = i
                while k < nw and row[k]: k += 1
                rects.append(((i - nw / 2) * px, (k - nw / 2) * px, (j - nh / 2) * px, (j + 1 - nh / 2) * px)); i = k
            else: i += 1
    return rects

def rect_tris(rects, origin, U, V, N, h):
    out = []
    for u0, u1, v0, v1 in rects:
        P = [origin + U * u + V * v for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]
        top = [p + N * h for p in P]
        out += [[top[0], top[1], top[2]], [top[0], top[2], top[3]]]
        for a_, b_ in ((0, 1), (1, 2), (2, 3), (3, 0)):
            out += [[P[a_], P[b_], top[b_]], [P[a_], top[b_], top[a_]]]
    return np.array(out)
