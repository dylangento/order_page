import {createViewer} from './viewer.js';
import {EN} from './i18n.js';
// ---------- language ----------
let lang=(()=>{try{const q=new URLSearchParams(location.search).get('lang');if(q)return q==='en'?'en':'zh';const s=localStorage.getItem('gento-lang');if(s==='en'||s==='zh')return s}catch(e){}return /^zh/i.test(navigator.language||'zh')?'zh':'en'})();
const t=s=>lang==='en'&&EN[s]!==undefined?EN[s]:s;
const tf=(s,...a)=>t(s).replace(/\{(\d)\}/g,(_,i)=>a[i]);
const body=document.getElementById('body'),spec=document.getElementById('spec'),steps=document.getElementById('steps');
const C=await fetch('catalog.json').then(r=>r.json());
const D=Object.fromEntries(C.dims.map((d,i)=>[d.key,i]));
const REC=['controller','power','frame'];
const vals=C.dims.map(d=>REC.includes(d.key)?d.values[1]:d.values[0]);
const V=k=>vals[D[k]],setV=(k,v)=>{vals[D[k]]=v};
const customer={机构:'',联系人:'',邮箱:'',交付地区:'',套数:'1',其他需求:''};
let step=0,ack=false,revision=1,exported=false;const ref='MV-'+crypto.randomUUID().slice(0,8).toUpperCase();
const NAMES=['机械臂','外观与走线','末端与相机','配件','配置单'];
const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const model=()=>C.models.find(m=>m.name===V('model'));
const famOf=()=>C.families.find(f=>f.models.includes(V('model')));
const color=()=>C.colors.find(c=>c.name===V('color'));
const off=k=>V(k)===C.dims[D[k]].values[0];
const sensorOn=()=>!off('sensor');
const locked=()=>V('gmsl')==='GMSL2 内走线'&&!ack;
const colon=()=>lang==='en'?': ':'：';
function code(){let n=0;for(const d of C.dims){let i=d.values.indexOf(V(d.key));if(d.key==='sensorModel'&&!sensorOn())i=0;n=n*d.values.length+i}return 'MV-D2-'+String(n+1).padStart(6,'0')}
function changed(){if(exported){revision++;exported=false}}

// static page text (index.html elements carrying data-i18n)
function staticText(){
 document.documentElement.lang=lang==='en'?'en':'zh-CN';
 document.title=t('Marvin 配置工作室')+' · Gento Robotics';
 document.querySelectorAll('[data-i18n]').forEach(el=>{el.textContent=t(el.dataset.i18n)});
 document.querySelectorAll('[data-i18n-aria]').forEach(el=>el.setAttribute('aria-label',t(el.dataset.i18nAria)));
 document.querySelectorAll('[data-lang]').forEach(b=>b.setAttribute('aria-pressed',b.dataset.lang===lang));
}

// ---------- 3D ----------
let viewer=null,thumbs={};
const sides=k=>k==='gripper'?{L:!off(k),R:!off(k)}:({L:V(k)==='仅左臂'||V(k)==='左右双臂',R:V(k)==='仅右臂'||V(k)==='左右双臂'});
function state3d(){const s=sides('sensor'),m=V('sensorModel').includes('KWR')?'kwr':'lb63',c=color();
 return {tint:c.hex,tintMetal:c.metal,tintRough:c.rough,show:{frame:!off('frame'),base:!off('base'),ctrl:!off('controller'),psu:!off('power'),kit:step>=3},ft:{L:s.L?m:null,R:s.R?m:null},cam:off('camera')?null:V('camera')==='D405GA'?'d405':'shw5g',gmsl:off('gmsl')?null:V('gmsl')==='GMSL2 内走线'?'int':'ext',grip:sides('gripper')}}
let armReq=null;
async function sync3d(){
 if(!viewer)return;viewer.set(state3d());
 const id=model().id;if(armReq!==id){armReq=id;document.getElementById('loading').hidden=false;await viewer.setArm(id);if(armReq===id){document.getElementById('loading').hidden=true;if(step===0)viewer.focus('arms')}}
}
const CHIPS=[['全景','all'],['左臂','armL'],['右臂','armR'],['末端','ee'],['支撑架','frame'],['底座','base'],['控制器','ctrl'],['电源','psu']];
let chipOn=-1;
function chipsHTML(){const el=document.getElementById('chips');if(!el)return;el.innerHTML=CHIPS.map(([x],k)=>'<button data-chip="'+k+'"'+(chipOn===k?' class="on"':'')+(viewer?'':' disabled')+'>'+t(x)+'</button>').join('')}
const hoverInfo=()=>Object.fromEntries(Object.entries(C.hover).map(([k,[a,d]])=>[k,{title:esc(t(a)),desc:'<span>'+esc(t(d))+'</span>'}]));
staticText();chipsHTML();
createViewer(document.getElementById('v3d')).then(async v=>{
 viewer=v;viewer.setHoverInfo(hoverInfo());await sync3d();viewer.focus(step===0?'arms':'all',0);
 chipsHTML();
 const T=(k,n,y,p)=>thumbs[k]=v.thumb(n,264,208,y,p);
 T('frame',['frame'],-0.6,0.35);T('base',['base'],-0.6,0.28);T('ft_lb63',['ft_lb63'],-0.6,0.6);T('ft_kwr',['ft_kwr'],-0.6,0.6);
 T('cam_shw5g',['cam_shw5g'],-1.1,0.5);T('cam_d405',['cam_d405'],-1.1,0.5);T('grip',['grip'],-1.0,0.4);T('psu',['psu'],-0.5,0.55);
 if(step===2||step===3)view();
}).catch(e=>{console.warn(e);document.getElementById('loading').remove();document.getElementById('v3d').outerHTML='<img class="fallback" src="marvin.webp" alt="'+t('Marvin 双臂产品参考图')+'">';document.getElementById('chips').remove();const h=document.getElementById('hint');h.dataset.i18n='当前浏览器不支持 WebGL2，无法显示 3D 预览，图为系列参考图。';h.textContent=t(h.dataset.i18n)});


// ---------- UI pieces ----------
const seg=(k,labels)=>'<div class="seg">'+C.dims[D[k]].values.map((v,j)=>'<button data-k="'+k+'" data-j="'+j+'" aria-pressed="'+(V(k)===v)+'">'+esc(t(labels?labels[j]:v))+'</button>').join('')+'</div>';
function card(k,title,note,thumb,focus,extra=''){
 const sel=k&&!off(k);
 const pic=thumb?'<img src="'+thumb.src+'" alt="">'+(thumb.cap?'<em>'+t(thumb.cap)+'</em>':''):'<small>'+t(viewer?'暂无 3D 模型':'加载中…')+'</small>';
 return '<div class="acc'+(sel?' sel':'')+'"><div class="pic">'+pic+'</div><div><h4><span>'+title+'</span>'+(focus?'<button data-focus="'+focus+'" '+(viewer?'':'disabled')+'>'+t('在 3D 中查看')+'</button>':'<small>'+t('暂无 3D 模型')+'</small>')+'</h4><p>'+t(note)+'</p>'+(k?seg(k):'')+extra+'</div></div>';
}
const rec=()=>' <span class="rec">'+t('官方推荐')+'</span>';
const miss=k=>off(k)?'<p class="warn"><b>'+t(C.missing[k][0])+colon()+'</b>'+t(C.missing[k][1])+'</p>':'';
const missing=()=>REC.filter(off);
const th=(key,cap='3D 模型')=>thumbs[key]?{src:thumbs[key],cap}:null;
const help=(id,text)=>'<button class="q" data-help="'+id+'" aria-expanded="false" aria-label="'+t('说明')+'">?</button><div class="tip" id="tip-'+id+'" hidden>'+text+'</div>';
const GMSL_HELP_ZH='<b>什么是 GMSL2 相机线？</b>GMSL2（Gigabit Multimedia Serial Link 2）是一种高速串行相机链路：一根同轴线即可同时传输高清视频、控制信号和供电，前向带宽约 6 Gbps，线长可达约 15 m，常用于把腕部或头部相机接到 Jetson Orin / Thor 等主机。<br><br><b>外走线</b>：线缆沿手臂外侧固定，更换维护方便，推荐。<br><b>内走线</b>：线缆穿过关节内部，外观更整洁，但长期反复弯折可能磨损线缆，导致掉帧或信号中断，更换也更费时。';
function sensorTable(){const s=C.sensors,sel=s.cols.indexOf(V('sensorModel'));
 return '<table class="cmp"><thead><tr><th>'+t('厂家规格')+'</th>'+s.cols.map((c,i)=>'<th'+(i===sel?' class="on"':'')+'>'+c+'</th>').join('')+'</tr></thead><tbody>'+s.rows.map(r=>'<tr><td>'+t(r[0])+'</td><td'+(sel===0?' class="on"':'')+'>'+r[1]+'</td><td'+(sel===1?' class="on"':'')+'>'+r[2]+'</td></tr>').join('')+'</tbody></table><p class="fine">'+t(s.foot)+' '+t('资料：')+s.docs.map(d=>'<a href="'+d[1]+'" target="_blank" rel="noopener">'+t(d[0])+'</a>').join(' · ')+'</p>'}
const CUST_KEYS=Object.keys(customer);

// rows: [label (zh key, translated on display), value (already in the current language)]
function rows(){
 const m=model(),s=sides('sensor'),n=(s.L&&s.R)?2:1;
 const r=[['配置单号',ref+' / V'+revision],['配置编号',code()],['日期',new Date().toLocaleDateString(lang==='en'?'en-US':'zh-CN')],...Object.entries(customer).filter(([k])=>k!=='其他需求').map(([k,v])=>[k,v||t('未填写')]),
  ['机械臂型号',tf('左右臂均为 {0}',m.name)+' · '+[m.reach,m.payload,m.weight].join(' / ')],['壳体颜色',tf('{0}（亮光）',t(V('color')))],
  ...(V('color')!=='天机白'?[['交期提示',t('非天机白颜色可能延长交期，以销售确认为准')]]:[]),
  ['GMSL2 相机线',t(V('gmsl'))],...(V('gmsl')==='GMSL2 内走线'?[['内走线风险',t(ack?'客户已确认线缆弯折、信号与维护风险':'待确认')]]:[]),
  ['腕部相机套件',off('camera')?t('不选配'):tf('{0}（左右臂各 1 套）',V('camera'))],...(V('camera')==='SHW5G'?[['相机接入提示',t('需自备 GMSL2 解串板读取视频流（Marvin 双臂不含 Orin 与解串板）')]]:[]),...(V('camera')==='D405GA'&&!off('gmsl')?[['相机线提示',t('D405GA 不使用 GMSL2 相机线，线缆方案待销售确认')]]:[]),
  ['末端六维力传感器',sensorOn()?V('sensorModel')+' · '+t(V('sensor'))+tf('（{0} 个）',n):t('不选配')],
  ['KernelMind Omni-Gripper',off('gripper')?t('不选配'):t('左右双臂各 1 个（共 2 个）')],...(off('gripper')?[]:[['8Pin 接口方向',t('默认按当前方向出厂（可调整为双臂 8Pin 均朝上）')]]),
  ...['controller','power','frame','base'].map(k=>[C.dims[D[k]].label,(lang==='en'?({'选配 1 套':'1 set','选配 1 件':'1 unit'}[V(k)]||t(V(k))):V(k))+(k==='controller'&&!off(k)?t('（预装控制 SDK 固件，含以太网线）'):'')]),
  ...(off('base')?[]:[['海外交付提示',t('移动底座体积大、重量重，中国大陆以外交付运费可能大幅增加；可考虑含底座及一体化控制箱的 Marvin Pro')]]),
  ...missing().map(k=>['未选配提示',t(C.dims[D[k]].label)+colon()+t(C.missing[k][2])]),
  ['其他需求',customer.其他需求||t('无')],...(customer.其他需求?[['定制状态',t('待研发评估')]]:[])];
 return r;
}
function view(){
 sync3d();
 const m=model(),names=NAMES.map(t);
 spec.innerHTML=[[m.reach,'工作半径'],[m.payload,'额定负载'],[m.weight,'单臂重量']].map(([v,l])=>'<div><strong>'+v+'</strong><small>'+t(l)+'</small></div>').join('');
 steps.innerHTML=names.map((v,i)=>'<button class="'+(step===i?'on':'')+'" data-step="'+i+'"'+(i>1&&locked()?' disabled title="'+t('请先确认内走线风险')+'"':'')+'><i>'+(i+1)+'</i>'+v+'</button>').join('');
 let h='<h2>'+t(['选择机械臂','外观与相机走线','末端执行与腕部相机','选择配件','预览配置单'][step])+'</h2>';
 if(step===0){const fam=famOf();
  h+='<p class="quiet">'+t('先选择系列，再选择臂长。双臂系统左右臂为同一型号。')+'</p><h3>'+t('系列')+'</h3><div class="grid">'+C.families.map(f=>'<button class="choice fam" data-fam="'+f.id+'" aria-pressed="'+(fam.id===f.id)+'"><b>'+t(f.name)+(f.tag?' <span class="rec">'+t(f.tag)+'</span>':'')+'</b><ul>'+f.points.map(p=>'<li>'+t(p)+'</li>').join('')+'</ul><small>'+t(f.use)+'</small></button>').join('')+'</div>';
  h+='<h3>'+tf('臂长（{0}）',t(fam.name))+'</h3><div class="grid g3">'+fam.models.map(nm=>{const x=C.models.find(q=>q.name===nm),j=C.models.indexOf(x);return '<button class="choice" data-k="model" data-j="'+j+'" aria-pressed="'+(V('model')===nm)+'"><b>'+x.reach+'</b><small>'+nm+'<br>'+tf('负载 {0} · 单臂 {1}',x.payload,x.weight)+'</small></button>'}).join('')+'</div>';}
 if(step===1){
  h+='<h3>'+t('壳体颜色')+'</h3><div class="grid">'+C.colors.map((c,j)=>'<button class="choice" data-k="color" data-j="'+j+'" aria-pressed="'+(V('color')===c.name)+'"><b><i class="swatch" style="background:'+c.swatch+'"></i>'+t(c.name)+'</b><small>'+t(j?'可能延长交期':'标准色 · 默认')+'</small></button>').join('')+'</div>';
  h+='<p class="quiet">'+t('壳体统一为亮光表面。双臂共用颜色。')+'</p>'+(V('color')!=='天机白'?'<p class="warn">'+t('非天机白颜色需按排产安排喷涂，交期可能延长，具体以销售确认为准。')+'</p>':'');
  h+='<h3 class="hq">'+t('GMSL2 相机线')+help('gmsl',t(GMSL_HELP_ZH))+'</h3><div class="grid g3">'+C.dims[D.gmsl].values.map((v,j)=>'<button class="choice" data-k="gmsl" data-j="'+j+'" aria-pressed="'+(V('gmsl')===v)+'"><b>'+t(v)+'</b><small>'+t(['无相机线需求','推荐 · 维护方便','需确认风险'][j])+'</small></button>').join('')+'</div>';
  h+='<p class="quiet">'+t('GMSL2 相机线用于 SHW5G 腕部相机；D405GA 不使用 GMSL2 相机线。')+'</p>';
  if(V('gmsl')==='GMSL2 内走线')h+='<label class="pair warn strong"><input id="ack" type="checkbox" '+(ack?'checked':'')+'><span><b>'+t('必须确认后才能继续：')+'</b>'+t('我已了解并同意，GMSL2 内走线随关节长期反复弯折，可能导致线缆磨损、偶发掉帧或信号中断，且后续更换更费时。')+'</span></label>';
 }
 if(step===2){
  h+='<p class="quiet">'+t('末端配件会实时显示在 3D 视图中，未选的夹爪以浅色轮廓显示。安装顺序为法兰 → 力传感器 → 相机支架 → 夹爪。')+'</p>';
  const camKey=V('camera')==='D405GA'?'cam_d405':'cam_shw5g';
  const cd=C.cameras[V('camera')];
  h+=card('camera',t('腕部相机套件（双臂）'),C.notes.camera,th(camKey),'ee',cd?'<div class="sub"><b>'+t(cd.name)+'</b><table class="cmp spec1"><tbody>'+cd.rows.map(r=>'<tr><td>'+t(r[0])+'</td><td>'+t(r[1])+'</td></tr>').join('')+'</tbody></table><p class="fine">'+t(cd.note)+' <a href="'+cd.link[1]+'" target="_blank" rel="noopener">'+t(cd.link[0])+'</a></p>'+(V('camera')==='SHW5G'?'<p class="warn">'+t(C.gmslDecoder)+'</p>'+(off('gmsl')?'<p class="warn">'+t('SHW5G 通过 GMSL2 传输图像，请在「外观与走线」中选择 GMSL2 相机线（外走线或内走线）。')+'</p>':''):(off('gmsl')?'':'<p class="warn">'+t(C.d405Gmsl)+'</p>'))+'</div>':'');
  const ftKey=V('sensorModel').includes('KWR')?'ft_kwr':'ft_lb63';
  h+=card('sensor',t('末端六维力传感器'),C.notes.sensor,th(ftKey,'3D 示意'),'ee',sensorOn()?'<div class="sub"><b>'+t('型号')+'</b>'+seg('sensorModel')+sensorTable()+'</div>':'');
  h+=card('gripper',t('KernelMind Omni-Gripper 夹爪（双臂）'),C.notes.gripper,th('grip'),'ee',off('gripper')?'':'<p class="note8">'+t(C.pinOrient)+'</p>');
 }
 if(step===3){
  h+='<p class="quiet">'+t('选中的配件会实时出现在 3D 视图中，未选配件以浅色轮廓显示。')+'</p>';
  h+=card('controller',t('控制器模块')+rec(),C.notes.controller,{src:'acc-controller.jpg',cap:'实物图'},'ctrl',miss('controller'));
  h+=card('power',t('电源模块')+rec(),C.notes.power,th('psu')||{src:'acc-power.jpg',cap:'实物图'},'psu',miss('power'));
  h+=card('frame',t('机器人支撑架')+rec(),C.notes.frame,th('frame'),'frame',miss('frame'));
  h+=card('base',t('机器人底座'),C.notes.base,th('base'),'base',off('base')?'':'<p class="warn"><b>'+t(C.baseShip[0])+colon()+'</b>'+t(C.baseShip[1])+'<br>'+t(C.baseShip[2])+'</p>');
 }
 if(step===4)h+=(viewer?'<h3>'+t('配置效果图')+'</h3><img class="shot" id="shot" alt="'+t('当前配置的 3D 效果图')+'">':'')+(missing().length?'<div class="warn"><b>'+t('以下官方推荐配件未选配：')+'</b><ul class="misslist">'+missing().map(k=>'<li><b>'+t(C.dims[D[k]].label)+'</b>'+colon()+t(C.missing[k][1])+'</li>').join('')+'</ul><button data-step="3">'+t('返回配件选择')+'</button></div>':'')+'<details open><summary>'+t('询价信息')+'</summary>'+Object.entries(customer).map(([k,v])=>'<label class="field">'+t(k)+(k==='其他需求'?'<textarea data-c="'+k+'" rows="3">'+esc(v)+'</textarea>':'<input data-c="'+k+'" type="'+(k==='套数'?'number':k==='邮箱'?'email':'text')+'" '+(k==='套数'?'min="1" step="1" required':'')+' value="'+esc(v)+'">')+'</label>').join('')+'</details><h3>'+t('配置明细')+'</h3><div id="summary"></div>';
 const next=step<4?'<button class="next" data-step="'+(step+1)+'"'+(step===1&&locked()?' disabled':'')+'>'+t('下一步')+' · '+names[step+1]+'</button>':'<button class="next" id="pdf">'+t('下载配置单 PDF')+'</button><button id="mail">'+t('通过邮件询价')+'</button>';
 body.innerHTML=h+'<div class="actions">'+(step?'<button data-step="'+(step-1)+'">'+t('上一步')+'</button>':'')+next+'</div>'+(step===1&&locked()?'<p class="block">'+t('请先勾选上方的内走线风险确认，才能继续下一步。')+'</p>':'')+(step===4?'<p class="quiet">'+t('邮件询价会打开邮件草稿。请附上下载的 PDF 和定制资料，再发送给 Gento。')+' '+t('邮箱')+colon()+'<a href="mailto:dylanli@gentorobotics.ai">dylanli@gentorobotics.ai</a></p>':'');
 if(step===4){summaryView();if(viewer)setTimeout(()=>{const s=document.getElementById('shot');if(s)s.src=viewer.snapshot(1200,800)},400)}
}
function summaryView(){document.getElementById('summary').innerHTML=rows().filter(([k])=>k!=='配置单号'&&k!=='配置编号').map(([k,v])=>'<div class="line"><span>'+t(k)+'</span><b>'+esc(v)+'</b></div>').join('')}
function valid(){if(locked()){step=1;view();document.getElementById('ack').focus();return false}return [...document.querySelectorAll('input[data-c]')].every(x=>x.reportValidity())}
function setLang(l){if(l===lang)return;lang=l;try{localStorage.setItem('gento-lang',l)}catch(e){}staticText();chipsHTML();if(viewer)viewer.setHoverInfo(hoverInfo());view()}
const FOCUS_OF={gmsl:'cable',camera:'ee',sensor:'ee',sensorModel:'ee',gripper:'ee',controller:'ctrl',power:'psu',frame:'frame',base:'base'};
document.addEventListener('click',e=>{
 const lb=e.target.closest('[data-lang]');if(lb){setLang(lb.dataset.lang);return}
 const hb=e.target.closest('[data-help]');if(hb){const x=document.getElementById('tip-'+hb.dataset.help);x.hidden=!x.hidden;hb.setAttribute('aria-expanded',!x.hidden);return}
 const b=e.target.closest('button');if(!b||b.disabled)return;
 if(b.dataset.chip!==undefined){chipOn=+b.dataset.chip;viewer&&viewer.focus(CHIPS[chipOn][1]);document.querySelectorAll('[data-chip]').forEach(x=>x.classList.toggle('on',x===b));return}
 if(b.dataset.focus){if(viewer){viewer.focus(b.dataset.focus);document.getElementById('view').scrollIntoView({behavior:'smooth',block:'nearest'})}return}
 if(b.dataset.step!==undefined){const s=+b.dataset.step;if(s>1&&locked()){step=1;view();return}step=s;view();if(viewer&&step===4)viewer.focus('all');if(viewer&&step===0)viewer.focus('arms');if(viewer&&step===3)viewer.focus('all');return}
 if(b.dataset.fam){const f=C.families.find(x=>x.id===b.dataset.fam);if(!f.models.includes(V('model'))){changed();setV('model',f.models[0]);}view();if(viewer)setTimeout(()=>viewer.focus('arms'),400);return}
 if(b.dataset.k){changed();const k=b.dataset.k;setV(k,C.dims[D[k]].values[+b.dataset.j]);if(k==='gmsl')ack=false;view();
  const f=FOCUS_OF[k];if(viewer&&f&&!off(k==='sensorModel'?'sensor':k))setTimeout(()=>viewer.focus(f),60);if(k==='model'&&viewer)setTimeout(()=>viewer.focus('arms'),400);return}
 if(b.id==='pdf'&&valid())downloadPdf();
 else if(b.id==='mail'&&valid())downloadPdf().then(()=>{location.href='mailto:dylanli@gentorobotics.ai?subject='+encodeURIComponent(t('Marvin 配置询价')+' '+ref+' V'+revision)+'&body='+encodeURIComponent(rows().map(([k,v])=>t(k)+(lang==='en'?': ':'　')+v).join('\n'))});
});
document.addEventListener('change',e=>{const x=e.target;if(x.id==='ack'){changed();ack=x.checked;view()}});
document.addEventListener('input',e=>{if(e.target.dataset.c){changed();customer[e.target.dataset.c]=e.target.value;summaryView()}});
const PDF_FONT=getComputedStyle(document.body).fontFamily||'sans-serif';
// save a generated file: inside a claude.ai artifact use its downloads capability, elsewhere a normal browser download
async function saveFile(name,blob){
 if(window.claude&&typeof window.claude.use==='function'){try{const dl=await window.claude.use('downloads');if(dl){await dl.save({filename:name,data:blob});return}}catch(e){if(e&&e.code==='declined')return;console.warn(e)}}
 const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=name;document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),20000);
}
async function downloadPdf(){
 let shot=null;if(viewer){shot=new Image();shot.src=viewer.snapshot(1120,700);await shot.decode().catch(()=>{shot=null})}
 const pages=[];let c,ctx,y;
 function page(){c=document.createElement('canvas');c.width=1240;c.height=1754;ctx=c.getContext('2d');ctx.fillStyle='white';ctx.fillRect(0,0,c.width,c.height);ctx.fillStyle='#0B0A0A';ctx.font='600 40px '+PDF_FONT;ctx.fillText('Gento Robotics  /  '+t('Marvin 配置单'),60,90);ctx.fillStyle='#00C896';ctx.fillRect(60,108,64,4);y=165;pages.push(c)}
 page();
 if(shot){ctx.drawImage(shot,60,125,1120,700);ctx.fillStyle='#5E6461';ctx.font='20px '+PDF_FONT;ctx.fillText(t('3D 效果图（颜色与材质为示意，以实物为准）'),60,855);y=915}
 for(const [k0,v] of rows()){const k=t(k0);if(y>1570)page();let fs=23;ctx.font='600 '+fs+'px '+PDF_FONT;while(ctx.measureText(k).width>295&&fs>15){fs--;ctx.font='600 '+fs+'px '+PDF_FONT}ctx.fillStyle='#5E6461';ctx.fillText(k,60,y);ctx.font='24px '+PDF_FONT;let line='';const toks=(String(v).match(/[\u2e80-\u9fff\u3000-\u303f\uff00-\uffef]|[^\s\u2e80-\u9fff\u3000-\u303f\uff00-\uffef]+\s*|\s+/g)||[]).concat(['\n']);for(const tk of toks){if(tk==='\n'||(line&&ctx.measureText(line+tk).width>800)){ctx.fillStyle='#0B0A0A';ctx.font='24px '+PDF_FONT;ctx.fillText(line.trimEnd(),375,y);y+=36;if(y>1630)page();ctx.font='24px '+PDF_FONT;line=''}if(tk!=='\n')line+=(line?tk:tk.trimStart())}ctx.fillStyle='#e4e6e5';ctx.fillRect(60,y-19,1120,1);y+=19}
 const enc=new TextEncoder(),chunks=[],offs=[0];let len=0;function add(s){let a=typeof s==='string'?enc.encode(s):s;chunks.push(a);len+=a.length}function obj(n,s){offs[n]=len;add(n+' 0 obj\n'+s)}add('%PDF-1.4\n');obj(1,'<< /Type /Catalog /Pages 2 0 R >>\nendobj\n');obj(2,'<< /Type /Pages /Count '+pages.length+' /Kids ['+pages.map((_,i)=>(3+i*3)+' 0 R').join(' ')+'] >>\nendobj\n');
 pages.forEach((cv,i)=>{let n=3+i*3;obj(n,'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 595 842] /Resources << /XObject << /Art '+(n+1)+' 0 R >> >> /Contents '+(n+2)+' 0 R >>\nendobj\n');let bytes=Uint8Array.from(atob(cv.toDataURL('image/jpeg',.94).split(',')[1]),v=>v.charCodeAt(0));obj(n+1,'<< /Type /XObject /Subtype /Image /Width 1240 /Height 1754 /ColorSpace /DeviceRGB /BitsPerComponent 8 /Filter /DCTDecode /Length '+bytes.length+' >>\nstream\n');add(bytes);add('\nendstream\nendobj\n');let draw='q 595 0 0 842 0 0 cm /Art Do Q\n';obj(n+2,'<< /Length '+draw.length+' >>\nstream\n'+draw+'endstream\nendobj\n')});let start=len;add('xref\n0 '+offs.length+'\n0000000000 65535 f \n'+offs.slice(1).map(n=>String(n).padStart(10,'0')+' 00000 n \n').join('')+'trailer\n<< /Size '+offs.length+' /Root 1 0 R >>\nstartxref\n'+start+'\n%%EOF');await saveFile(ref+'-V'+revision+'.pdf',new Blob(chunks,{type:'application/pdf'}));exported=true
}
view();
