// Marvin 3D configurator viewer — dependency-free WebGL2 renderer.
// Physically based shading (GGX), studio environment lighting, shadow mapping, ACES tone mapping.
// World: Z-up, metres. Assets: m/common, m/ee, m/arm-<id> (.json + .bin).

const MAT = [ // metal, roughness, clearcoat
  [0.0, 0.32, 1.0], // 0 shell paint (tinted)
  [1.0, 0.24, 0.0], // 1 anodised aluminium
  [0.0, 0.42, 0.3], // 2 black anodised / seal
  [0.0, 0.55, 0.2], // 3 painted steel
  [1.0, 0.46, 0.0], // 4 brushed steel
  [1.0, 0.14, 0.0], // 5 chrome
  [0.0, 0.50, 0.0], // 6 black plastic
  [0.0, 0.50, 0.0], // 7 generic
];

const VS = `#version 300 es
layout(location=0) in vec3 aPos; layout(location=1) in vec3 aNrm; layout(location=2) in float aMat; layout(location=3) in vec3 aCol;
uniform vec3 uLo, uExt; uniform mat4 uVP, uM, uLightVP;
out vec3 vN, vW, vC; out vec4 vL; flat out int vMat;
void main(){ vec4 w = uM * vec4(uLo + aPos * uExt, 1.0); vW = w.xyz; vN = mat3(uM) * aNrm; vC = aCol; vMat = int(aMat*255.0+0.5); vL = uLightVP * w; gl_Position = uVP * w; }`;

const FS = `#version 300 es
precision highp float;
in vec3 vN, vW, vC; in vec4 vL; flat in int vMat;
uniform vec3 uEye, uTint; uniform float uGhost, uAlpha, uTintMetal, uTintRough, uOpq;
uniform vec3 uMatA[8]; uniform highp sampler2DShadow uShadow; uniform vec3 uKey;
out vec4 frag;
const float PI = 3.14159265;
vec3 toLin(vec3 c){ return pow(c, vec3(2.2)); }
// studio environment: soft top light, two side softboxes, warm floor bounce
vec3 env(vec3 r, float rough){
  float w = mix(0.02, 0.6, rough);
  vec3 c = mix(vec3(0.42,0.43,0.44), vec3(0.97,0.98,0.99), smoothstep(-0.45, 0.8, r.z));
  c = mix(c, vec3(0.55,0.56,0.54), smoothstep(0.0, -0.7, r.z)*0.5);
  float b1 = smoothstep(1.0 - 0.06 - w, 1.0 - 0.06*(1.0-rough), dot(r, normalize(vec3(0.55,-0.6,0.58))));
  float b2 = smoothstep(1.0 - 0.10 - w, 1.0 - 0.10*(1.0-rough), dot(r, normalize(vec3(-0.75,0.35,0.45))));
  float b3 = smoothstep(1.0 - 0.15 - w, 1.0 - 0.15*(1.0-rough), dot(r, normalize(vec3(-0.2,-0.2,1.0))));
  return c + vec3(3.2)*b1*(1.0-0.6*rough) + vec3(1.6)*b2*(1.0-0.6*rough) + vec3(1.2)*b3*(1.0-0.5*rough);
}
float D_GGX(float nh, float a){ float a2=a*a; float d=nh*nh*(a2-1.0)+1.0; return a2/(PI*d*d); }
float V_SmithJ(float nv, float nl, float a){ float k=a*0.5; return 0.25/((nv*(1.0-k)+k)*(nl*(1.0-k)+k)); }
vec3 F_Sch(vec3 f0, float vh){ return f0 + (1.0-f0)*pow(1.0-vh, 5.0); }
vec3 aces(vec3 x){ return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14), 0.0, 1.0); }
float shadow(){
  vec3 p = vL.xyz / vL.w * 0.5 + 0.5;
  if(p.x<0.0||p.x>1.0||p.y<0.0||p.y>1.0) return 1.0;
  float s = 0.0; vec2 t = 1.0/vec2(textureSize(uShadow,0));
  for(int x=-2;x<=2;x++) for(int y=-2;y<=2;y++) s += texture(uShadow, vec3(p.xy + vec2(x,y)*t*1.2, p.z - 0.0008));
  return s/25.0;
}
void main(){
  vec3 n = normalize(vN); vec3 v = normalize(uEye - vW);
  if(!gl_FrontFacing) n = -n;
  if(uGhost > 0.5){ float rim = 1.0 - abs(dot(n,v)); vec3 gc = mix(vec3(0.42,0.52,0.49), vec3(0.08,0.40,0.31), rim); float ga = uAlpha*(0.25+0.75*rim);
    // premultiplied output; on a transparent canvas fade as a soft see-through outline (same in every browser)
    frag = uOpq > 0.5 ? vec4(gc*ga, ga) : vec4(gc*ga*ga*ga, ga*ga); return; }
  vec3 m = uMatA[vMat];
  float metal = m.x, rough = m.y, coat = m.z;
  vec3 base = toLin(vC);
  if(vMat == 0){ base = toLin(uTint); metal = uTintMetal; rough = uTintRough; }
  vec3 f0 = mix(vec3(0.04), base, metal); vec3 diff = base*(1.0-metal);
  float nv = max(dot(n,v), 1e-3);
  vec3 L = normalize(uKey); vec3 h = normalize(L+v); float nl = max(dot(n,L),0.0), nh = max(dot(n,h),0.0), vh = max(dot(v,h),0.0);
  float a = max(rough*rough, 0.002);
  vec3 F = F_Sch(f0, vh);
  float sh = shadow();
  vec3 key = vec3(2.6) * nl * sh;
  vec3 col = key * (diff/PI*(1.0-F)*PI*0.55 + F*D_GGX(nh,a)*V_SmithJ(nv,nl,a));
  // fill + ambient from environment
  vec3 r = reflect(-v, n);
  vec3 Fe = f0 + (max(vec3(1.0-rough), f0) - f0)*pow(1.0-nv, 5.0);
  vec3 irr = env(n, 1.0)*0.55;
  float ao = mix(0.75, 1.0, sh);
  col += diff * irr * ao + Fe * env(r, rough) * mix(0.42, 0.7, metal) * mix(0.6, 1.0, sh);
  if(coat > 0.0){ float fc = 0.04 + 0.96*pow(1.0-nv,5.0); col += coat*fc*env(r, 0.08)*0.32; col += coat*0.6*D_GGX(nh,0.03)*V_SmithJ(nv,nl,0.03)*nl*sh*0.04; }
  col = aces(col*0.8);
  frag = vec4(pow(col, vec3(1.0/2.2)), uAlpha);
}`;

const SVS = `#version 300 es
layout(location=0) in vec3 aPos; uniform vec3 uLo, uExt; uniform mat4 uM, uLightVP;
void main(){ gl_Position = uLightVP * uM * vec4(uLo + aPos*uExt, 1.0); }`;
const SFS = `#version 300 es
precision mediump float; out vec4 f; void main(){ f = vec4(1.0); }`;
const GVS = `#version 300 es
layout(location=0) in vec2 aP; uniform mat4 uVP, uLightVP; uniform float uR; out vec4 vL; out vec2 vP;
void main(){ vP=aP; vec4 w=vec4(aP*uR,0.0,1.0); vL=uLightVP*w; gl_Position=uVP*w; }`;
const GFS = `#version 300 es
precision highp float; in vec4 vL; in vec2 vP; uniform highp sampler2DShadow uShadow; uniform vec3 uBg; uniform float uOpq; out vec4 f;
void main(){
  vec3 p = vL.xyz/vL.w*0.5+0.5; float s=0.0; vec2 t=1.0/vec2(textureSize(uShadow,0));
  for(int x=-3;x<=3;x++) for(int y=-3;y<=3;y++) s += texture(uShadow, vec3(p.xy+vec2(x,y)*t*1.6, p.z-0.001));
  s/=49.0; float fade = smoothstep(1.0, 0.25, length(vP));
  float a = (1.0-s)*0.55*fade + 0.10*fade*fade; vec3 c = vec3(0.02,0.04,0.035);
  f = uOpq > 0.5 ? vec4(c*a, a) : vec4(c*a*a*a, a*a);
}`;

const m4 = {
  mul(a, b) { const o = new Float32Array(16); for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) { let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + j] * b[i * 4 + k]; o[i * 4 + j] = s; } return o; },
  persp(fov, asp, n, f) { const t = 1 / Math.tan(fov / 2), o = new Float32Array(16); o[0] = t / asp; o[5] = t; o[10] = (f + n) / (n - f); o[11] = -1; o[14] = 2 * f * n / (n - f); return o; },
  ortho(l, r, b, t, n, f) { const o = new Float32Array(16); o[0] = 2 / (r - l); o[5] = 2 / (t - b); o[10] = -2 / (f - n); o[12] = -(r + l) / (r - l); o[13] = -(t + b) / (t - b); o[14] = -(f + n) / (f - n); o[15] = 1; return o; },
  look(e, c, u) { const z = norm(sub(e, c)), x = norm(cross(u, z)), y = cross(z, x); return new Float32Array([x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -dot(x, e), -dot(y, e), -dot(z, e), 1]); },
  I: new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]),
  xf(m, p) { return [m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12], m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13], m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]]; }
};
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]], add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]], scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2], cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => scl(a, 1 / Math.hypot(...a));
const lerp = (a, b, t) => a + (b - a) * t;
const CTRL_SET = ['ctrl', 'kit_power-adapter', 'kit_emergency-stop', 'kit_communication-extension-leads', 'kit_red-black-extension-leads', 'kit_green-terminal-power-lead', 'kit_wrist-8pin-breakout-pair'];
const zoff = z => { const m = new Float32Array(m4.I); m[14] = z; return m; };

// model binaries: <name>.bin, or <name>.b64.txt (base64 of the same bytes) where a host cannot serve .bin files
async function loadBin(p) {
  try { const r = await fetch(p + '.bin'); if (r.ok) return await r.arrayBuffer(); } catch (e) { }
  const s = (await (await fetch(p + '.b64.txt')).text()).trim(), n = s.length, b = atob(s), u = new Uint8Array(b.length);
  for (let i = 0; i < b.length; i++) u[i] = b.charCodeAt(i);
  return u.buffer;
}
export async function createViewer(canvas, base = 'm/', extra = []) {
  const gl = canvas.getContext('webgl2', { antialias: true, preserveDrawingBuffer: true, alpha: true, premultipliedAlpha: true });
  if (!gl) throw new Error('WebGL2 unavailable');
  const prog = mk(gl, VS, FS), sprog = mk(gl, SVS, SFS), gprog = mk(gl, GVS, GFS);
  const U = {}, u = (p, n) => (U[p.k + n] ??= gl.getUniformLocation(p, n)); prog.k = 'p'; sprog.k = 's'; gprog.k = 'g';
  // shadow map
  const SM = 2048, stex = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, stex); gl.texStorage2D(gl.TEXTURE_2D, 1, gl.DEPTH_COMPONENT24, SM, SM);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_COMPARE_MODE, gl.COMPARE_REF_TO_TEXTURE); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  const sfb = gl.createFramebuffer(); gl.bindFramebuffer(gl.FRAMEBUFFER, sfb); gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.TEXTURE_2D, stex, 0); gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  const quad = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, quad); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
  const qvao = gl.createVertexArray(); gl.bindVertexArray(qvao); gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0); gl.bindVertexArray(null);

  const parts = {}; // name -> gpu mesh
  async function load(name) {
    const [meta, buf] = await Promise.all([fetch(base + name + '.json').then(r => r.json()), loadBin(base + name).then(gunzip)]);
    for (const p of meta.parts) {
      const vao = gl.createVertexArray(); gl.bindVertexArray(vao);
      const vb = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, vb); gl.bufferData(gl.ARRAY_BUFFER, new Uint8Array(buf, p.voff, p.vcount * meta.stride), gl.STATIC_DRAW);
      gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, 3, gl.UNSIGNED_SHORT, true, 16, 0);
      gl.enableVertexAttribArray(1); gl.vertexAttribPointer(1, 3, gl.BYTE, true, 16, 6);
      gl.enableVertexAttribArray(2); gl.vertexAttribPointer(2, 1, gl.UNSIGNED_BYTE, true, 16, 9);
      gl.enableVertexAttribArray(3); gl.vertexAttribPointer(3, 3, gl.UNSIGNED_BYTE, true, 16, 10);
      const ib = gl.createBuffer(); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib);
      gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, p.idx32 ? new Uint32Array(buf, p.ioff, p.icount) : new Uint16Array(buf, p.ioff, p.icount), gl.STATIC_DRAW);
      gl.bindVertexArray(null);
      parts[(meta.prefix || '') + p.name] = { ...p, vao, hi: p.lo.map((v, i) => v + p.ext[i]) };
    }
    return meta;
  }
  const arms = {}; // id -> meta (tcpL, tcpR)
  await Promise.all([load('common'), load('ee'), ...extra.map(load)]);

  const state = { arm: null, tint: '#eeede9', tintMetal: 0, tintRough: 0.3, ghosts: true,
    show: { frame: false, base: false, ctrl: false, psu: false, kit: true }, ft: { L: null, R: null }, cam: null, gmsl: null, grip: { L: false, R: false } };
  const cam = { yaw: -0.55, pitch: 0.22, dist: 3, target: [0, 0, 0.9] };
  let anim = null, dirty = true;
  const hover = { name: null, lift: {}, info: {}, tip: null, miss: 0 };
  function hoverM(n) { const h = hover.lift[n] || 0; return h ? zoff(h) : m4.I; }
  const tip = document.createElement('div'); tip.className = 'v3d-tip'; tip.hidden = true; canvas.parentElement.appendChild(tip);
  // picking target
  const pfb = gl.createFramebuffer(), ptex = gl.createTexture(), pdep = gl.createRenderbuffer(); let pw = 0, ph = 0;
  const pprog = mk(gl, SVS.replace('uLightVP * uM', 'uLightVP * uM'), `#version 300 es
precision mediump float; uniform vec4 uId; out vec4 f; void main(){ f = uId; }`); pprog.k = 'q';

  // --------- scene list: [partName, matrix, ghost] ---------
  function items(includeGhosts = state.ghosts) {
    const L = [];
    const a = arms[state.arm];
    for (const k of ['frame', 'base', 'ctrl', 'psu']) {
      const ns = k === 'ctrl' ? CTRL_SET : [k];
      for (const n of ns) { if (n.startsWith('kit_') && !state.show.kit) continue; if (state.show[k]) L.push([n, hoverM(n), false]); else if (includeGhosts) L.push([n, m4.I, true]); }
    }
    if (!a) return L;
    for (const s of ['L', 'R']) {
      L.push([state.arm + ':arm' + s, m4.I, false]);
      const tcp = a['tcp' + s]; let z = 0;
      const ft = state.ft[s];
      if (ft) { L.push([ft === 'kwr' ? 'ft_kwr' : 'ft_lb63', tcp, false]); z += ft === 'kwr' ? 0.0315 : 0.033; }
      if (state.cam) { L.push([state.cam === 'd405' ? 'cam_d405' : 'cam_shw5g', m4.mul(tcp, zoff(z)), false]); }
      if (state.gmsl && a['route' + s]) {
        const c = cable(s, a, tcp, z);
        if (c) { L.push([c.name, m4.I, false], ['fakra', c.conn, false], ['fakra', c.conn2, false]); if (c.base) L.push([c.base, m4.I, false]); for (const M of c.clips) L.push(['clip', M, false]); }
      }
      if (state.grip[s]) { const zg = z + (state.cam ? 0 : -0.006); L.push(['grip', m4.mul(tcp, zoff(zg)), false]);
        const pl = parts.pin8 && pinLead(s, a, tcp, zg); if (pl) L.push([pl.name, m4.I, false], ['pin8', pl.pin, false], ['gport', pl.port, false]); }
      else if (includeGhosts) L.push(['grip', m4.mul(tcp, zoff(z + (state.cam ? 0 : -0.006))), true]);
    }
    return L;
  }
  // ---------- GMSL2 cable (built at runtime from routing waypoints) ----------
  const basis = (o, z, xh) => { z = norm(z); let x = sub(xh, scl(z, dot(xh, z))); if (Math.hypot(...x) < 1e-6) x = Math.abs(z[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0], x = sub(x, scl(z, dot(x, z))); x = norm(x); const y = cross(z, x); return new Float32Array([...x, 0, ...y, 0, ...z, 0, ...o, 1]); };
  const xfd = (m, d) => [m[0] * d[0] + m[4] * d[1] + m[8] * d[2], m[1] * d[0] + m[5] * d[1] + m[9] * d[2], m[2] * d[0] + m[6] * d[1] + m[10] * d[2]];
  const SHW = (() => { // camera frame on the SHW5G bracket (Pro-m6-696 URDF): T(0.06,0,0.0578) rpy(-1.5707,-0.00013,1.3615)
    const [r, p, y] = [-1.5707, -0.00012967, 1.3615], cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
    const R = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]];
    const ay = [R[0][1], R[1][1], R[2][1]]; return { p: add([0.06, 0, 0.057822], scl(ay, 0.036)), a: ay };
  })();
  const cableCache = {};
  // tool-local helpers: points are given in the tool frame (z along the tool), W maps to world
  const cyl = p => [Math.hypot(p[0], p[1]), Math.atan2(p[1], p[0]), p[2]];
  const fromCyl = (r, t, z) => [r * Math.cos(t), r * Math.sin(t), z];
  function arc(A, B, step = 0.005) { // wrap around the tool axis from A to B (linear in radius, angle and height)
    const [ra, ta, za] = cyl(A), [rb, tb0, zb] = cyl(B); let tb = tb0;
    while (tb - ta > Math.PI) tb -= 2 * Math.PI; while (ta - tb > Math.PI) tb += 2 * Math.PI;
    const len = Math.abs(tb - ta) * Math.max(ra, rb) + Math.abs(zb - za) + Math.abs(rb - ra), n = Math.max(2, Math.ceil(len / step)), out = [];
    for (let i = 1; i < n; i++) { const u = i / n; out.push(fromCyl(ra + (rb - ra) * u, ta + (tb - ta) * u, za + (zb - za) * u)); }
    return out;
  }
  function chaikin(P, it = 2) { // round off corners, keeping both ends
    if (P.length < 3) return P;
    for (let k = 0; k < it; k++) { const Q = [P[0]]; for (let i = 0; i < P.length - 1; i++) { const a = P[i], b = P[i + 1]; if (i) Q.push(add(scl(a, 0.75), scl(b, 0.25))); if (i < P.length - 2) Q.push(add(scl(a, 0.25), scl(b, 0.75))); } Q.push(P[P.length - 1]); P = Q; }
    return P;
  }
  function cable(s, a, tcp, z) {
    const rt = a['route' + s]; if (!rt || !rt.holeT || state.cam === 'd405') return null;   // D405GA does not use the GMSL2 cable
    const W = p => m4.xf(tcp, p), Wd = d => norm(xfd(tcp, d));
    const h = rt.holeT, d = rt.holeDirT, E = rt.endT;
    let mpL, axL, loc;                         // connector pose + cable points in tool coordinates
    if (state.cam === 'shw5g') { mpL = add(SHW.p, [0, 0, z]); axL = SHW.a; }
    else if (state.gmsl === 'int') { mpL = add(h, scl(d, 0.07)); axL = scl(d, -1); }
    else { mpL = add(E, [0, 0, 0.06]); axL = [0, 0, -1]; }
    const boot = add(mpL, scl(axL, 0.042)), pre = add(boot, scl(axL, 0.012));
    if (state.gmsl === 'int') {
      const h0 = sub(h, scl(d, 0.006)), h1 = add(h, scl(d, 0.005));
      if (state.cam === 'shw5g') {      // hug the silver ring round to the camera side, then up into the FAKRA boot
        loc = [h0, h1, ...arc(h1, pre), pre, boot];
      } else loc = [h0, h1, boot];
    } else loc = [...(state.cam === 'shw5g' ? [pre] : []), boot];
    const pts = [...(state.gmsl === 'ext' ? [...rt.tail.slice().reverse(), ...rt.ext.slice(1)] : []), ...chaikin(loc).map(W)];
    const key = [state.arm, s, state.gmsl, state.cam, z].join('|'), name = 'cable' + s;
    if (cableCache[name] !== key) { cableCache[name] = key; if (parts[name]) gl.deleteVertexArray(parts[name].vao); parts[name] = tube(pts, 0.0024); }
    const conn = basis(W(mpL), Wd(axL), Wd([0, 0, 1]));
    const clips = state.gmsl === 'ext' ? rt.clips.map(([p, t, o]) => basis(p, t, scl(o, -1))) : [];
    // far end: leaves the bottom of the J1 base and ends in a blue FAKRA plug (internal: separate piece out of the base)
    let base = null;
    if (state.gmsl === 'int') { base = 'bcable' + s; const bk = [state.arm, s].join('|');
      if (cableCache[base] !== bk) { cableCache[base] = bk; if (parts[base]) gl.deleteVertexArray(parts[base].vao); parts[base] = tube(rt.itail, 0.0024); } }
    const [mpB, axB, xB] = state.gmsl === 'int' ? rt.itailConn : rt.tailConn;
    return { name, conn, clips, base, conn2: basis(mpB, axB, xB) };
  }
  // 8-pin lead: flange 8-pin connector -> connector port on the side of the gripper
  function pinLead(s, a, tcp, zg) {
    const rt = a['route' + s]; if (!rt || !rt.pinT) return null;
    const W = p => m4.xf(tcp, p), Wd = d => norm(xfd(tcp, d));
    const p = rt.pinT, d = rt.pinDirT, sp = s === 'L' ? 1 : -1;
    const port = [0, sp * 0.033, zg + 0.0245], pd = [0, sp, 0];
    // right-angle plug: the lead leaves tangentially, towards the gripper port
    const [, tp] = cyl(p), [, tq] = cyl(port); let dt = tq - tp; while (dt > Math.PI) dt -= 2 * Math.PI; while (dt < -Math.PI) dt += 2 * Math.PI;
    const tx = norm(cross([0, 0, Math.sign(dt) || 1], d));
    // straight plug: the lead leaves along the plug axis, then bends round towards the gripper port
    const t0 = add(p, scl(d, 0.0195)), t1 = add(add(p, scl(d, 0.029)), scl(tx, 0.008)), t2 = add(add(p, scl(d, 0.027)), scl(tx, 0.024));
    const [r2] = cyl(t2), A3 = fromCyl(Math.max(0.05, Math.min(r2, 0.058)), tq, t2[2]), q1 = add(port, scl(pd, 0.03)), q0 = add(port, scl(pd, 0.016));
    const loc = [t0, t1, t2, ...arc(t2, A3), A3, q1, q0];
    const key = [state.arm, s, zg].join('|'), name = 'pcable' + s;
    if (cableCache[name] !== key) { cableCache[name] = key; if (parts[name]) gl.deleteVertexArray(parts[name].vao); parts[name] = tube(chaikin(loc).map(W), 0.0022); }
    return { name, pin: basis(W(p), Wd(d), Wd(tx)), port: basis(W(port), Wd(pd), Wd([1, 0, 0])) };
  }
  function tube(ctrl, rad, sides = 12) {
    // Catmull-Rom through control points, parallel-transport frames
    const P = [ctrl[0], ...ctrl, ctrl[ctrl.length - 1]], pts = [];
    for (let i = 1; i < P.length - 2; i++) {
      const L = Math.hypot(...sub(P[i + 1], P[i])), n = Math.max(2, Math.ceil(L / 0.004));
      for (let k = 0; k < n; k++) { const t = k / n, t2 = t * t, t3 = t2 * t;
        pts.push([0, 1, 2].map(j => 0.5 * ((2 * P[i][j]) + (-P[i - 1][j] + P[i + 1][j]) * t + (2 * P[i - 1][j] - 5 * P[i][j] + 4 * P[i + 1][j] - P[i + 2][j]) * t2 + (-P[i - 1][j] + 3 * P[i][j] - 3 * P[i + 1][j] + P[i + 2][j]) * t3))); }
    }
    pts.push(ctrl[ctrl.length - 1]);
    const V = [], N = [], I = []; let tPrev = norm(sub(pts[1], pts[0])), nrm = norm(cross(tPrev, Math.abs(tPrev[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0]));
    for (let i = 0; i < pts.length; i++) {
      const t = norm(sub(pts[Math.min(i + 1, pts.length - 1)], pts[Math.max(i - 1, 0)]));
      nrm = norm(sub(nrm, scl(t, dot(nrm, t)))); const bn = cross(t, nrm);
      for (let k = 0; k < sides; k++) { const an = 2 * Math.PI * k / sides, d = add(scl(nrm, Math.cos(an)), scl(bn, Math.sin(an))); V.push(add(pts[i], scl(d, rad))); N.push(d); }
      if (i) for (let k = 0; k < sides; k++) { const a0 = (i - 1) * sides + k, a1 = (i - 1) * sides + (k + 1) % sides, b0 = a0 + sides, b1 = a1 + sides; I.push(a0, a1, b1, a0, b1, b0); }
    }
    let lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9]; for (const v of V) for (let j = 0; j < 3; j++) { lo[j] = Math.min(lo[j], v[j]); hi[j] = Math.max(hi[j], v[j]); }
    const ext = hi.map((h, j) => Math.max(h - lo[j], 1e-6)), buf = new ArrayBuffer(V.length * 16), dv = new DataView(buf);
    V.forEach((v, i) => { const o = i * 16; for (let j = 0; j < 3; j++) dv.setUint16(o + 2 * j, Math.round((v[j] - lo[j]) / ext[j] * 65535), true);
      for (let j = 0; j < 3; j++) dv.setInt8(o + 6 + j, Math.round(N[i][j] * 127)); dv.setUint8(o + 9, 6); dv.setUint8(o + 10, 30); dv.setUint8(o + 11, 34); dv.setUint8(o + 12, 38); });
    const vao = gl.createVertexArray(); gl.bindVertexArray(vao);
    const vb = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, vb); gl.bufferData(gl.ARRAY_BUFFER, buf, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, 3, gl.UNSIGNED_SHORT, true, 16, 0);
    gl.enableVertexAttribArray(1); gl.vertexAttribPointer(1, 3, gl.BYTE, true, 16, 6);
    gl.enableVertexAttribArray(2); gl.vertexAttribPointer(2, 1, gl.UNSIGNED_BYTE, true, 16, 9);
    gl.enableVertexAttribArray(3); gl.vertexAttribPointer(3, 3, gl.UNSIGNED_BYTE, true, 16, 10);
    const ib = gl.createBuffer(); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib); gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint32Array(I), gl.STATIC_DRAW);
    gl.bindVertexArray(null);
    return { name: 'cable', vao, icount: I.length, idx32: true, lo, ext, hi };
  }
  function bounds(list) {
    let lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
    for (const [n, M] of list) {
      const p = parts[n]; if (!p) continue;
      for (let c = 0; c < 8; c++) { const q = m4.xf(M, [c & 1 ? p.hi[0] : p.lo[0], c & 2 ? p.hi[1] : p.lo[1], c & 4 ? p.hi[2] : p.lo[2]]); for (let i = 0; i < 3; i++) { lo[i] = Math.min(lo[i], q[i]); hi[i] = Math.max(hi[i], q[i]); } }
    }
    return { c: scl(add(lo, hi), 0.5), r: Math.hypot(...sub(hi, lo)) / 2, lo, hi };
  }
  function viewFor(list, yaw = cam.yaw, pitch = cam.pitch, asp = canvas.width / Math.max(1, canvas.height)) {
    const b = bounds(list); const f = Math.min(1, asp) ; return { yaw, pitch, dist: Math.max(0.3, b.r / Math.tan(0.38) / Math.max(0.75, f) * 0.98), target: b.c };
  }
  const eyeOf = c => { const cp = Math.cos(c.pitch); return add(c.target, scl([Math.cos(c.yaw) * cp, Math.sin(c.yaw) * cp, Math.sin(c.pitch)], c.dist)); };

  function resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2), w = Math.round(canvas.clientWidth * dpr), h = Math.round(canvas.clientHeight * dpr);
    if (w && h && (canvas.width !== w || canvas.height !== h)) { canvas.width = w; canvas.height = h; dirty = true; }
  }

  const KEY = norm([0.45, -0.35, 0.82]);
  function lightVP(list) {
    const b = bounds(list.length ? list : [['base', m4.I]]); const r = Math.max(b.r, 0.6) * 1.15;
    const e = add(b.c, scl(KEY, 4));
    return m4.mul(m4.ortho(-r, r, -r, r, 0.5, 8), m4.look(e, b.c, [0, 0, 1]));
  }
  function drawMesh(p, prg) { gl.bindVertexArray(p.vao); gl.uniform3fv(u(prg, 'uLo'), p.lo); gl.uniform3fv(u(prg, 'uExt'), p.ext); gl.drawElements(gl.TRIANGLES, p.icount, p.idx32 ? gl.UNSIGNED_INT : gl.UNSIGNED_SHORT, 0); }

  function draw(c = cam, w = canvas.width, h = canvas.height, opts = {}) {
    const all = opts.list || items(!opts.noGhosts);
    const solid = all.filter(x => !x[2]);
    const LVP = lightVP(solid);
    // shadow pass
    gl.bindFramebuffer(gl.FRAMEBUFFER, sfb); gl.viewport(0, 0, SM, SM); gl.clear(gl.DEPTH_BUFFER_BIT); gl.enable(gl.DEPTH_TEST); gl.depthMask(true); gl.disable(gl.BLEND);
    gl.colorMask(false, false, false, false); gl.useProgram(sprog); gl.uniformMatrix4fv(u(sprog, 'uLightVP'), false, LVP);
    gl.enable(gl.POLYGON_OFFSET_FILL); gl.polygonOffset(2, 4);
    for (const [n, M] of solid) { const p = parts[n]; if (!p) continue; gl.uniformMatrix4fv(u(sprog, 'uM'), false, M); drawMesh(p, sprog); }
    gl.disable(gl.POLYGON_OFFSET_FILL); gl.colorMask(true, true, true, true); gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    // main
    gl.viewport(0, 0, w, h); const bg = opts.bg || [0, 0, 0]; gl.clearColor(bg[0], bg[1], bg[2], opts.bg ? 1 : 0); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    const eye = eyeOf(c), near = Math.max(0.02, c.dist * 0.04);
    const VP = m4.mul(m4.persp(0.76, w / h, near, c.dist * 6 + 6), m4.look(eye, c.target, [0, 0, 1]));
    gl.activeTexture(gl.TEXTURE0); gl.bindTexture(gl.TEXTURE_2D, stex);
    // ground shadow
    if (!opts.noGround) {
      gl.useProgram(gprog); gl.enable(gl.BLEND); gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA); gl.depthMask(false); gl.uniform1f(u(gprog, 'uOpq'), opts.bg ? 1 : 0);
      gl.uniformMatrix4fv(u(gprog, 'uVP'), false, VP); gl.uniformMatrix4fv(u(gprog, 'uLightVP'), false, LVP); gl.uniform1f(u(gprog, 'uR'), 1.6); gl.uniform1i(u(gprog, 'uShadow'), 0);
      gl.bindVertexArray(qvao); gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4); gl.depthMask(true); gl.disable(gl.BLEND);
    }
    gl.useProgram(prog);
    gl.uniformMatrix4fv(u(prog, 'uVP'), false, VP); gl.uniformMatrix4fv(u(prog, 'uLightVP'), false, LVP); gl.uniform3fv(u(prog, 'uEye'), eye);
    gl.uniform3fv(u(prog, 'uKey'), KEY); gl.uniform1i(u(prog, 'uShadow'), 0);
    gl.uniform3fv(u(prog, 'uMatA'), MAT.flat()); gl.uniform3fv(u(prog, 'uTint'), hex(state.tint)); gl.uniform1f(u(prog, 'uTintMetal'), state.tintMetal); gl.uniform1f(u(prog, 'uTintRough'), state.tintRough);
    gl.uniform1f(u(prog, 'uGhost'), 0); gl.uniform1f(u(prog, 'uAlpha'), 1);
    for (const [n, M] of solid) { const p = parts[n]; if (!p) continue; gl.uniformMatrix4fv(u(prog, 'uM'), false, M); drawMesh(p, prog); }
    const ghosts = all.filter(x => x[2]);
    if (ghosts.length) {
      gl.enable(gl.BLEND); gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA); gl.depthMask(false); gl.uniform1f(u(prog, 'uOpq'), opts.bg ? 1 : 0); gl.uniform1f(u(prog, 'uGhost'), 1); gl.uniform1f(u(prog, 'uAlpha'), 0.22);
      for (const [n, M] of ghosts) { const p = parts[n]; if (!p) continue; gl.uniformMatrix4fv(u(prog, 'uM'), false, M); drawMesh(p, prog); }
      gl.depthMask(true); gl.disable(gl.BLEND);
    }
    gl.bindVertexArray(null);
  }

  function pick(px, py) {
    const names = Object.keys(hover.info).filter(n => state.show.ctrl || !CTRL_SET.includes(n)).filter(n => state.show.kit || !n.startsWith('kit_')).filter(n => parts[n] && (CTRL_SET.includes(n) ? state.show.ctrl : state.show[n]));
    if (!names.length) return null;
    const w = Math.max(1, canvas.width >> 1), h = Math.max(1, canvas.height >> 1);
    if (w !== pw || h !== ph) {
      pw = w; ph = h; gl.bindTexture(gl.TEXTURE_2D, ptex); gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
      gl.bindRenderbuffer(gl.RENDERBUFFER, pdep); gl.renderbufferStorage(gl.RENDERBUFFER, gl.DEPTH_COMPONENT24, w, h);
      gl.bindFramebuffer(gl.FRAMEBUFFER, pfb); gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, ptex, 0); gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.RENDERBUFFER, pdep);
    }
    gl.bindFramebuffer(gl.FRAMEBUFFER, pfb); gl.viewport(0, 0, w, h); gl.clearColor(0, 0, 0, 0); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST); gl.disable(gl.BLEND); gl.useProgram(pprog);
    const eye = eyeOf(cam), near = Math.max(0.02, cam.dist * 0.04);
    const VP = m4.mul(m4.persp(0.76, canvas.width / canvas.height, near, cam.dist * 6 + 6), m4.look(eye, cam.target, [0, 0, 1]));
    gl.uniformMatrix4fv(u(pprog, 'uLightVP'), false, VP);
    const all = items(false); let id = 0; const map = {};
    for (const [n, M0] of all) { const p = parts[n]; if (!p) continue; id++; map[id] = n; let M = M0;
      if (hover.info[n]) { const c = p.lo.map((v, i) => v + p.ext[i] / 2), k = 1.18; M = new Float32Array([k, 0, 0, 0, 0, k, 0, 0, 0, 0, k, 0, c[0] * (1 - k), c[1] * (1 - k), c[2] * (1 - k), 1]); } gl.uniform4f(u(pprog, 'uId'), (id & 255) / 255, ((id >> 8) & 255) / 255, 0, 1); gl.uniformMatrix4fv(u(pprog, 'uM'), false, M); drawMesh(p, pprog); }
    const px_ = new Uint8Array(4); gl.readPixels(Math.floor(px * w), Math.floor((1 - py) * h), 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, px_);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null); gl.bindVertexArray(null); dirty = true;
    const hit = map[px_[0] + (px_[1] << 8)];
    return hit && hover.info[hit] ? hit : null;
  }
  let lastPick = 0, pendingPick = null;
  canvas.addEventListener('pointermove', e => {
    if (drag) { setHover(null); return; }
    pendingPick = e;
  });
  canvas.addEventListener('pointerleave', () => { pendingPick = null; setHover(null); });
  function setHover(n, e) {
    if (n !== hover.name) { hover.name = n; dirty = true; canvas.style.cursor = n ? 'pointer' : ''; }
    if (!n) { tip.hidden = true; return; }
    const info = hover.info[n]; tip.innerHTML = '<b>' + info.title + '</b>' + info.desc; tip.hidden = false;
    const r = canvas.getBoundingClientRect(), pr = canvas.parentElement.getBoundingClientRect();
    let x = e.clientX - pr.left + 14, y = e.clientY - pr.top + 14;
    if (x + 260 > pr.width) x = e.clientX - pr.left - 274; if (y + tip.offsetHeight > pr.height) y = e.clientY - pr.top - tip.offsetHeight - 10;
    tip.style.left = x + 'px'; tip.style.top = y + 'px';
  }
  function frame(t) {
    resize();
    if (pendingPick && t - lastPick > 50) { const e = pendingPick; pendingPick = null; lastPick = t; const r = canvas.getBoundingClientRect(); let h = pick((e.clientX - r.left) / r.width, (e.clientY - r.top) / r.height); if (!h && hover.name && ++hover.miss < 3) h = hover.name; else if (h) hover.miss = 0; setHover(h, e); }
    for (const n of Object.keys(hover.info)) { const tgt = n === hover.name ? 0.025 : 0, cur = hover.lift[n] || 0; if (Math.abs(tgt - cur) > 1e-4) { hover.lift[n] = cur + (tgt - cur) * 0.25; dirty = true; } else if (cur !== tgt) { hover.lift[n] = tgt; dirty = true; } }
    if (anim) {
      const k = Math.min(1, (t - anim.t0) / anim.dur), e = k < 0.5 ? 2 * k * k : 1 - Math.pow(-2 * k + 2, 2) / 2;
      cam.yaw = lerp(anim.a.yaw, anim.b.yaw, e); cam.pitch = lerp(anim.a.pitch, anim.b.pitch, e); cam.dist = lerp(anim.a.dist, anim.b.dist, e);
      cam.target = anim.a.target.map((v, i) => lerp(v, anim.b.target[i], e)); dirty = true; if (k >= 1) anim = null;
    }
    if (dirty) { draw(); dirty = false; }
    requestAnimationFrame(frame);
  }

  // controls
  let drag = null; const pts = new Map();
  canvas.addEventListener('pointerdown', e => { canvas.setPointerCapture(e.pointerId); pts.set(e.pointerId, [e.clientX, e.clientY]); drag = { pan: e.button === 2 || e.shiftKey, d0: null }; anim = null; });
  canvas.addEventListener('pointermove', e => {
    if (!drag || !pts.has(e.pointerId)) return;
    const prev = pts.get(e.pointerId); pts.set(e.pointerId, [e.clientX, e.clientY]);
    if (pts.size === 2) { const [a, b] = [...pts.values()], d = Math.hypot(a[0] - b[0], a[1] - b[1]); if (drag.d0) cam.dist = Math.min(8, Math.max(0.15, cam.dist * drag.d0 / d)); drag.d0 = d; dirty = true; return; }
    const dx = e.clientX - prev[0], dy = e.clientY - prev[1];
    if (drag.pan) { const right = [-Math.sin(cam.yaw), Math.cos(cam.yaw), 0], s = cam.dist * 0.0016; cam.target = add(cam.target, add(scl(right, -dx * s), [0, 0, dy * s])); }
    else { cam.yaw -= dx * 0.008; cam.pitch = Math.max(-0.15, Math.min(1.45, cam.pitch + dy * 0.006)); }
    dirty = true;
  });
  const up = e => { pts.delete(e.pointerId); if (!pts.size) drag = null; };
  canvas.addEventListener('pointerup', up); canvas.addEventListener('pointercancel', up);
  canvas.addEventListener('contextmenu', e => e.preventDefault());
  canvas.addEventListener('wheel', e => { e.preventDefault(); anim = null; cam.dist = Math.min(8, Math.max(0.15, cam.dist * Math.exp(e.deltaY * 0.0012))); dirty = true; }, { passive: false });
  canvas.addEventListener('dblclick', () => api.focus('all'));

  const groups = {
    all: () => items(false),
    arms: () => [[state.arm + ':armL', m4.I], [state.arm + ':armR', m4.I], ...items(false).filter(x => /^(ft_|cam_|grip)/.test(x[0]))],
    armL: () => [[state.arm + ':armL', m4.I]], armR: () => [[state.arm + ':armR', m4.I]],
    frame: () => [['frame', m4.I]], base: () => [['base', m4.I]], ctrl: () => CTRL_SET.filter(n => state.show.kit || !n.startsWith('kit_')).map(n => [n, m4.I]), psu: () => [['psu', m4.I]],
    ee: () => items(false).filter(x => /^(ft_|cam_|grip|fakra|pin8|gport|pcable)/.test(x[0])),
    cable: () => items(false).filter(x => /^(cable|bcable|fakra|clip)/.test(x[0])),
  };
  const api = {
    async setArm(id) {
      if (!arms[id]) { const meta = await load('arm-' + id).then(m => m); for (const p of meta.parts) { parts[id + ':' + p.name] = parts[p.name]; delete parts[p.name]; } meta.tcpL = new Float32Array(meta.tcpL); meta.tcpR = new Float32Array(meta.tcpR); arms[id] = meta; }
      const first = !state.arm; state.arm = id; dirty = true; if (first) api.focus('all', 0);
    },
    setHoverInfo(info) { hover.info = info; },
    set(s) { for (const k in s) state[k] = (typeof s[k] === 'object' && s[k] && !Array.isArray(s[k])) ? { ...s[k] } : s[k]; dirty = true; },
    focus(g, ms = 700) {
      let list = typeof g === 'string' ? (groups[g] ? groups[g]() : [[g, m4.I]]) : g;
      if (!list.length) list = groups.all();
      const single = g !== 'all' && g !== 'arms';
      const b = viewFor(list, g === 'ctrl' || g === 'psu' ? 0.2 : g === 'ee' ? -0.35 : cam.yaw, single ? 0.4 : 0.2);
      if (g === 'arms') b.dist *= 1.12;
      anim = { t0: performance.now(), dur: ms, a: { ...cam, target: [...cam.target] }, b }; dirty = true;
      if (!ms) { Object.assign(cam, b); anim = null; }
    },
    look(target, dist, yaw, pitch) { Object.assign(cam, { target, dist, yaw, pitch }); anim = null; dirty = true; },
    snapshot(w = 1200, h = 800, bg = [1, 1, 1]) {
      const list = items(false); const c = viewFor(list, -0.6, 0.2, w / h);
      const ow = canvas.width, oh = canvas.height; canvas.width = w; canvas.height = h;
      draw(c, w, h, { bg, noGhosts: true }); const url = canvas.toDataURL('image/jpeg', 0.92);
      canvas.width = ow; canvas.height = oh; dirty = true; return url;
    },
    thumb(names, w = 320, h = 240, yaw = -0.6, pitch = 0.35) {
      const list = names.map(n => [n, m4.I, false]); const c = viewFor(list, yaw, pitch, w / h); c.dist *= 0.9;
      const ow = canvas.width, oh = canvas.height; canvas.width = w; canvas.height = h;
      draw(c, w, h, { list, noGround: true }); const url = canvas.toDataURL('image/png');
      canvas.width = ow; canvas.height = oh; dirty = true; return url;
    }
  };
  requestAnimationFrame(frame);
  return api;
}

async function gunzip(buf) {
  const u = new Uint8Array(buf);
  if (u[0] !== 0x1f || u[1] !== 0x8b) return buf;
  return new Response(new Blob([u]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
}
function hex(h) { h = h.replace('#', ''); return [0, 2, 4].map(i => parseInt(h.slice(i, i + 2), 16) / 255); }
function mk(gl, vs, fs) {
  const p = gl.createProgram();
  for (const [t, s] of [[gl.VERTEX_SHADER, vs], [gl.FRAGMENT_SHADER, fs]]) {
    const sh = gl.createShader(t); gl.shaderSource(sh, s); gl.compileShader(sh);
    if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(sh));
    gl.attachShader(p, sh);
  }
  gl.linkProgram(p); if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
  return p;
}
