#pragma once
// Self-contained heatmap viewer served at /thermal. Polls /thermal.json and
// writes settings with POST /thermal/set?key=..&value=.. (same settings as the
// ESPHome number/select/switch entities, so everything stays in sync).

namespace esphome::mlx90640 {

static const char HEATMAP_PAGE[] = R"HTML(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Thermal camera</title>
<style>
:root{color-scheme:dark}
*{box-sizing:border-box}
body{margin:0;padding:12px;font:14px system-ui,sans-serif;background:#111;color:#eee;display:flex;flex-direction:column;align-items:center}
.col{width:100%;max-width:640px}
h1{font-size:18px;margin:4px 0 10px}
#wrap{position:relative}
#view{width:100%;aspect-ratio:4/3;display:block;border-radius:6px;background:#000;cursor:crosshair}
#tip{position:absolute;pointer-events:none;background:#000c;padding:2px 6px;border-radius:4px;font-size:12px;display:none}
#bar{display:flex;align-items:center;gap:8px;margin-top:8px}
#bar span{flex:none;white-space:nowrap}
/* width:0 + min-width:0 stop the 256x1 canvas's intrinsic aspect ratio from forcing a huge width */
#legend{flex:1 1 0;width:0;min-width:0;height:12px;border-radius:3px}
#stats{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-top:10px}
.s{background:#1d1d1d;border-radius:6px;padding:6px 8px}.s span{color:#999;font-size:12px}.s b{display:block;font-size:18px}
fieldset{border:1px solid #333;border-radius:6px;margin:12px 0 0;padding:8px 10px;display:flex;flex-wrap:wrap;gap:10px 16px;align-items:center}
legend{color:#999;font-size:12px;padding:0 4px}
input[type=number]{width:64px}
#st{color:#999;font-size:12px;margin-top:10px}
a{color:#8cf}
</style></head><body>
<h1 class="col">Thermal camera <small style="font-weight:normal"><a href="/">device page</a></small></h1>
<div id="wrap" class="col"><canvas id="view" width="640" height="480"></canvas><div id="tip"></div></div>
<div id="bar" class="col"><span id="lmin">-</span><canvas id="legend" width="256" height="1"></canvas><span id="lmax">-</span></div>
<div id="stats" class="col">
<div class="s"><span>Min</span><b id="vmin">-</b></div>
<div class="s"><span>Max</span><b id="vmax">-</b></div>
<div class="s"><span>Centre</span><b id="vctr">-</b></div>
<div class="s"><span>Sensor die</span><b id="vta">-</b></div>
</div>
<fieldset class="col"><legend>View</legend>
<label>Palette <select data-key="palette"><option value="0">Iron</option><option value="1">Rainbow</option><option value="2">Grey</option></select></label>
<label><input type="checkbox" data-key="auto_range"> Auto range</label>
<label>Min <input type="number" data-key="range_min" step="0.5"></label>
<label>Max <input type="number" data-key="range_max" step="0.5"></label>
<label><input type="checkbox" data-key="mirror"> Mirror</label>
<label><input type="checkbox" data-key="rotate_180"> Rotate 180&deg;</label>
<label><input type="checkbox" data-key="freeze"> Freeze</label>
<label><input type="checkbox" id="smooth" checked> Smooth (this browser)</label>
</fieldset>
<fieldset class="col"><legend>Sensor</legend>
<label>Refresh <select data-key="refresh_rate"><option value="0">0.5 Hz</option><option value="1">1 Hz</option><option value="2">2 Hz</option><option value="3">4 Hz</option><option value="4">8 Hz</option><option value="5">16 Hz</option><option value="6">32 Hz</option><option value="7">64 Hz</option></select></label>
<label>ADC <select data-key="resolution"><option value="0">16-bit</option><option value="1">17-bit</option><option value="2">18-bit</option><option value="3">19-bit</option></select></label>
<label>Pattern <select data-key="pattern"><option value="0">Chess</option><option value="1">Interleaved</option></select></label>
<label>Emissivity <input type="number" data-key="emissivity" min="0.1" max="1" step="0.01"></label>
<label>Reflected offset &deg;C <input type="number" data-key="reflected_offset" min="-20" max="40" step="0.5"></label>
<label>Smoothing <input type="number" data-key="smoothing" min="0" max="0.95" step="0.05"></label>
<label><input type="checkbox" data-key="interpolate_bad_pixels"> Fix bad pixels</label>
<label>HA publish s <input type="number" data-key="publish_interval" min="1" max="3600" step="1"></label>
</fieldset>
<fieldset class="col" id="camfs" style="display:none"><legend>Camera (Home Assistant)</legend>
<label><input type="checkbox" data-key="camera_overlay"> Overlay</label>
<label>JPEG quality <input type="number" data-key="jpeg_quality" min="10" max="100" step="1"></label>
<a href="/thermal.jpg" target="_blank">Snapshot (JPEG)</a>
</fieldset>
<div id="st" class="col">Connecting&hellip;</div>
<script>
const W=32,H=24,$=id=>document.getElementById(id);
const view=$('view'),ctx=view.getContext('2d');
const off=document.createElement('canvas');off.width=W;off.height=H;
const octx=off.getContext('2d'),img=octx.createImageData(W,H);
const PAL=[
 [[0,0,0],[30,0,90],[120,0,150],[200,30,100],[240,90,0],[255,170,0],[255,240,80],[255,255,255]],
 [[0,0,130],[0,60,255],[0,200,255],[0,255,120],[200,255,0],[255,160,0],[255,0,0]],
 [[0,0,0],[255,255,255]]];
let lut=[],lutPal=-1,last=null,hover=null,fails=0;
const fmt=v=>(v==null||!isFinite(v))?'-':v.toFixed(1)+'°C';
const cfg=k=>last&&last.cfg?last.cfg[k]:undefined;

function buildLut(p){
 if(p===lutPal)return;lutPal=p;const st=PAL[p]||PAL[0];lut=[];
 for(let i=0;i<256;i++){const f=i/255*(st.length-1),k=Math.min(Math.floor(f),st.length-2),t=f-k;
  lut.push(st[k].map((c,j)=>Math.round(c+(st[k+1][j]-c)*t)));}
 const lc=$('legend').getContext('2d'),ld=lc.createImageData(256,1);
 for(let i=0;i<256;i++)ld.data.set([lut[i][0],lut[i][1],lut[i][2],255],i*4);
 lc.putImageData(ld,0,0);}

function draw(){
 if(!last||!last.t)return;const t=last.t;let lo,hi;
 buildLut(cfg('palette')|0);
 if(cfg('auto_range')){lo=Infinity;hi=-Infinity;for(const v of t)if(v!==null){if(v<lo)lo=v;if(v>hi)hi=v;}
  if(!isFinite(lo))return;hi=Math.max(hi,lo+2);}
 else{lo=+cfg('range_min');hi=+cfg('range_max');if(!(hi>lo))hi=lo+1;}
 let hx=0,hy=0,hv=-Infinity;
 for(let y=0;y<H;y++)for(let x=0;x<W;x++){
  const v=t[y*W+x],i=(y*W+x)*4;
  if(v===null){img.data[i]=img.data[i+1]=img.data[i+2]=60;img.data[i+3]=255;continue;}
  if(v>hv){hv=v;hx=x;hy=y;}
  const c=lut[Math.max(0,Math.min(255,Math.round((v-lo)/(hi-lo)*255)))];
  img.data[i]=c[0];img.data[i+1]=c[1];img.data[i+2]=c[2];img.data[i+3]=255;}
 octx.putImageData(img,0,0);
 ctx.imageSmoothingEnabled=$('smooth').checked;ctx.imageSmoothingQuality='high';
 ctx.drawImage(off,0,0,view.width,view.height);
 const sx=view.width/W,sy=view.height/H,cx=(hx+.5)*sx,cy=(hy+.5)*sy;
 ctx.lineWidth=2;ctx.strokeStyle='#fff';ctx.beginPath();
 ctx.moveTo(cx-12,cy);ctx.lineTo(cx+12,cy);ctx.moveTo(cx,cy-12);ctx.lineTo(cx,cy+12);ctx.stroke();
 ctx.lineWidth=1;ctx.strokeStyle='#fff8';ctx.strokeRect(view.width/2-sx,view.height/2-sy,2*sx,2*sy);
 $('lmin').textContent=fmt(lo);$('lmax').textContent=fmt(hi);
 showTip();}

function showTip(){
 const tip=$('tip');if(!hover||!last||!last.t){tip.style.display='none';return;}
 tip.textContent=fmt(last.t[hover.y*W+hover.x]);
 tip.style.left=(hover.px+12)+'px';tip.style.top=(hover.py+12)+'px';tip.style.display='block';}

view.addEventListener('mousemove',e=>{
 const r=view.getBoundingClientRect(),x=Math.floor((e.clientX-r.left)/r.width*W),y=Math.floor((e.clientY-r.top)/r.height*H);
 hover=(x>=0&&y>=0&&x<W&&y<H)?{x,y,px:e.clientX-r.left,py:e.clientY-r.top}:null;showTip();});
view.addEventListener('mouseleave',()=>{hover=null;showTip();});

// Device-backed controls
const ctl=[...document.querySelectorAll('[data-key]')];
function syncControls(){
 for(const el of ctl){
  const v=cfg(el.dataset.key);if(v===undefined)continue;
  if(el===document.activeElement||Date.now()-(el._t||0)<3000)continue;  // don't fight the user
  if(el.type==='checkbox')el.checked=v>0.5;else if(el.value!==String(v))el.value=String(v);}}
ctl.forEach(el=>el.addEventListener('change',async()=>{
 el._t=Date.now();
 const v=el.type==='checkbox'?(el.checked?1:0):el.value;
 if(last&&last.cfg)last.cfg[el.dataset.key]=+v;draw();  // optimistic
 try{const r=await fetch('/thermal/set?key='+encodeURIComponent(el.dataset.key)+'&value='+encodeURIComponent(v),{method:'POST'});
  if(!r.ok)throw new Error('HTTP '+r.status);}
 catch(e){$('st').textContent='Could not change '+el.dataset.key+' ('+e.message+')';}}));
$('smooth').checked=localStorage.getItem('thermal_smooth')!=='0';
$('smooth').addEventListener('change',()=>{localStorage.setItem('thermal_smooth',$('smooth').checked?'1':'0');draw();});

async function poll(){
 const t0=performance.now();
 try{
  const r=await fetch('/thermal.json',{cache:'no-store'});
  if(!r.ok)throw new Error('HTTP '+r.status);
  last=await r.json();fails=0;syncControls();
  $('camfs').style.display=last.camera?'':'none';
  $('vmin').textContent=fmt(last.min);$('vmax').textContent=fmt(last.max);$('vta').textContent=fmt(last.ta);
  if(last.t){
   const c=[11*W+15,11*W+16,12*W+15,12*W+16].map(i=>last.t[i]).filter(v=>v!==null);
   $('vctr').textContent=c.length?fmt(c.reduce((a,b)=>a+b,0)/c.length):'-';}
  $('st').textContent=!last.ready?'Sensor not responding - check wiring (retrying)':
   !last.t?'Waiting for first frame…':
   (cfg('freeze')?'Frozen · ':'')+'Sensor '+last.fps.toFixed(1)+' fps · frame '+last.frame+
   ' · '+last.vdd.toFixed(2)+' V'+(last.i2c_errors?' · I2C errors '+last.i2c_errors:'');
  draw();
 }catch(e){fails++;$('st').textContent='Device unreachable ('+e.message+')';}
 const period=fails?2000:Math.max(150,1000/Math.max(last&&last.fps||2,0.5));
 setTimeout(poll,Math.max(0,period-(performance.now()-t0)));}
poll();
</script></body></html>)HTML";

}  // namespace esphome::mlx90640
