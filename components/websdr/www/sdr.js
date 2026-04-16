/* SPDX-License-Identifier: GPL-2.0-or-later */
/* SDR Console V3 replica - ESP32-P4 WebSDR */
(function(){'use strict';
var $=document.getElementById.bind(document),DC=document.createElement.bind(document);
function on(el,ev,fn,opt){el.addEventListener(ev,fn,opt);}
function qsa(s){return document.querySelectorAll(s);}
var TC='textContent',MR=Math.round,MN=Math.min,MX=Math.max,MA=Math.abs,
MS=Math.sqrt,PI=parseInt,MC=Math.ceil,MP=Math.PI;
function clamp(v,a,b){return v<a?a:v>b?b:v;}
function gbc(el){return el.getBoundingClientRect();}

/* ---- State ---- */
var ws,cFreq=100e6,sRate=1024000,fftSz=1024,dbMin=-40,dbMax=40;
var reconDelay=2000;
var gain=0,mode='NFM',vol=.5,sql=0,audOn=false;
var zLvl=1,zCtr=.5,wfSpd=50,wfLast=0;
var tunedFreq=cFreq,filterBW=12500;
var isDrag=false,dragX=0,dragF=0,dragMoved=false;
var isResize=false,resEdge=null,tuneOff=0;
var aCtx=null,wkReady=false,wkNode=null,gNode=null,aQ=[],iqRate=0,pI=0,pQ=0;
var modeBW={AM:10000,NFM:12500,WBFM:150000,USB:3000,LSB:3000,CW:500};
var modeList=['AM','NFM','WBFM','USB','LSB','CW'];
var vfoA={f:100e6,m:'NFM',b:12500},vfoB={f:145e6,m:'NFM',b:12500},vfoC={f:433.5e6,m:'NFM',b:12500},vfoD={f:28.4e6,m:'USB',b:2700},actVfo='A';
var spCv=$('spectrum'),wfCv=$('waterfall');
var spCtx=spCv.getContext('2d'),wfCtx=null,wfGL=null;
var pkBuf=null,avgBuf=null,avgA=.15,pkD=.5;
var curX=-1,curY=-1,fftF32=null;
var glTex=null,glCmap=null,glProg=null,glRow=0,glTH=512;
var glVao=null,glUO=null,glUH=null;
var cmN='sdrc0',cmD=new Uint8Array(1024),cmaps={},wfID=null;

/* S-meter elements */
var smFill=$('smeter-fill'),smPkMark=$('smeter-peak-mark'),smVal=$('smeter-val');
var smDbm=$('smeter-dbm');
var smPct=0,smTgt=0,smPkPct=0,smPkT=0;

var settings={cmap:'sdrc0',fftAvg:50,pkDecay:5,agcSpd:'medium',deemph:'75us',limiter:true,limDrive:2.0,limCeil:0.35};
var mems=[],bStack={},rhDrag=false,rhY0=0,rhH0=0;
var eDot=$('status-led'),eTxt=$('status-txt'),eRD=$('rate-display');
var eGS=$('gain-slider'),eGV=$('gain-val'),eVS=$('vol-slider'),eVV=$('vol-val');
var eSS=$('sql-slider'),eSV=$('sql-val'),eAB=$('audio-btn');
var eRS=$('rate-select'),eFS=$('fft-select');
var eNS=$('range-slider'),eNV=$('range-val'),eLS=$('ref-slider'),eLV=$('ref-val');
var eZV=$('zoom-val'),eWS=$('wf-speed'),eWV=$('wf-speed-val');
var eTK=$('tune-slider'),eTL=$('tune-lbl'),eBW=$('bw-val'),eCR=$('cursor-readout');
var eRit=$('rit-val');
var pndF=null,fTmr=null;

/* ---- SDR Console waterfall palettes ---- */
function mkCm(fn){var d=new Uint8Array(1024);for(var i=0;i<256;i++){var t=i/255,c=fn(t);d[i*4]=c[0];d[i*4+1]=c[1];d[i*4+2]=c[2];d[i*4+3]=255;}return d;}
function buildCm(){
/* SDR Console V3 palette: dark blue (noise) -> blue -> light blue -> white (strong) */
cmaps.sdrc0=mkCm(function(t){
  if(t<0.10){var p=t/0.10;return[0,0,MR(10+p*50)];}
  if(t<0.30){var p=(t-0.10)/0.20;return[0,MR(p*40),MR(60+p*100)];}
  if(t<0.55){var p=(t-0.30)/0.25;return[MR(p*40),MR(40+p*120),MR(160+p*60)];}
  if(t<0.80){var p=(t-0.55)/0.25;return[MR(40+p*140),MR(160+p*80),MR(220+p*35)];}
  var p=(t-0.80)/0.20;return[MR(180+p*75),MR(240+p*15),255];
});
cmaps.jet=mkCm(function(t){if(t<.25)return[0,t*4*255|0,255];if(t<.5)return[0,255,(2-t*4)*255|0];if(t<.75)return[(t*4-2)*255|0,255,0];return[255,(4-t*4)*255|0,0];});
cmaps.iron=mkCm(function(t){if(t<.33)return[t*3*128|0,0,t*3*40|0];if(t<.66)return[128+(t*3-1)*127|0,(t*3-1)*128|0,40-(t*3-1)*40|0];return[255,128+(t*3-2)*127|0,(t*3-2)*255|0];});
cmaps.viridis=mkCm(function(t){return[68+t*187*(t<.5?t*2:1)|0,MN(255,2+t*252)|0,MX(0,80+t*100-t*t*180)|0];});
cmaps.grayscale=mkCm(function(t){var v=t*255|0;return[v,v,v];});
setCm(settings.cmap);
}
function setCm(n){cmN=n;cmD=cmaps[n]||cmaps.sdrc0;if(wfGL&&glCmap){wfGL.bindTexture(wfGL.TEXTURE_2D,glCmap);wfGL.texImage2D(wfGL.TEXTURE_2D,0,wfGL.RGBA,256,1,0,wfGL.RGBA,wfGL.UNSIGNED_BYTE,cmD);}wfID=null;}

/* ---- Helpers ---- */
function fF(h){var hz=MR(MA(h)),s=String(hz).padStart(9,'0');return s.substr(0,3)+'.'+s.substr(3,3)+'.'+s.substr(6,3);}
function vB(n){var f=1/zLvl,h=f/2,s=zCtr-h,e=zCtr+h;if(s<0){e-=s;s=0;}if(e>1){s-=e-1;e=1;}if(s<0)s=0;return{s:s*n|0,e:MN(MC(e*n),n)};}
function gVF(){var n=fftSz,v=vB(n),sf=cFreq-sRate/2;return{s:sf+v.s/n*sRate,e:sf+v.e/n*sRate,bw:(v.e-v.s)/n*sRate};}
function f2x(f,W,vf){return(f-vf.s)/vf.bw*W;}
function hpp(W){var v=vB(fftSz);return sRate*(v.e-v.s)/fftSz/W;}
function bwT(){return(filterBW>=1e3?(filterBW/1e3).toFixed(1)+'k':filterBW)+'Hz';}
function mP(){return{b:filterBW,o:tunedFreq-cFreq};}

/* ---- WebSocket ---- */
function tx(c,p){if(!ws||ws.readyState!==1)return;var m={cmd:c};if(p)for(var k in p)m[k]=p[k];ws.send(JSON.stringify(m));}
function subIQ(){if(audOn){var pm=mP();tx('subscribe_iq',{offset:pm.o,bw:pm.b});}}
function tDb(){tx('db_range',{min:PI(eLV.value)-PI(eNS.value),max:PI(eLV.value)});}
function wsc(){
var h=location.hostname||'192.168.1.232',p=location.port||'8080';
var proto=location.protocol==='https:'?'wss://':'ws://';
ws=new WebSocket(proto+h+':'+p+'/ws');ws.binaryType='arraybuffer';
ws.onopen=function(){eDot.className='connected';eTxt[TC]='Connected';reconDelay=2000;};
ws.onclose=function(){eDot.className='';eTxt[TC]='Disconnected';reconDelay=Math.min(reconDelay*2,30000);setTimeout(wsc,reconDelay);};
ws.onerror=function(){ws.close();};
ws.onmessage=function(v){typeof v.data==='string'?onTxt(v.data):onBin(v.data);};}
function onTxt(t){var m;try{m=JSON.parse(t);}catch(e){return;}
if(m.type==='info'){cFreq=m.freq||cFreq;sRate=m.rate||sRate;gain=m.gain||0;fftSz=m.fft_size||fftSz;if(m.db_min!=null)dbMin=m.db_min;if(m.db_max!=null)dbMax=m.db_max;tunedFreq=cFreq;tuneOff=0;syncC();uD();}
else if(m.type==='freq'){cFreq=m.value;tunedFreq=cFreq+tuneOff;uD(false);}
else if(m.type==='config'){fftSz=m.fft_size||fftSz;sRate=m.sample_rate||sRate;if(m.db_min!=null)dbMin=m.db_min;if(m.db_max!=null)dbMax=m.db_max;rstBuf();syncC();uD();}
else if(m.type==='iq_start'){iqRate=m.rate||0;sendCfg();}
else if(m.type==='iq_stop'){iqRate=0;}
else if(m.type==='error'){console.warn('Server error:',m.msg);if(eTxt)eTxt[TC]='Err: '+(m.msg||'').slice(0,30);setTimeout(function(){if(eTxt&&eTxt[TC].startsWith('Err:'))eTxt[TC]='Connected';},3000);}}
function onBin(buf){var d=new Uint8Array(buf);if(d.length<2)return;var t=d[0],p=d.subarray(1);
if(t===1){procFFT(p);var now=performance.now();if(now-wfLast>=wfSpd){dWf(p);wfLast=now;}uSm(p);}
else if(t===2&&audOn){if(aQ.length>100)aQ.splice(0,aQ.length-50);aQ.push(p.slice());}}
function rstBuf(){pkBuf=avgBuf=fftF32=wfID=null;glRow=0;}
function syncC(){if(eRS)eRS.value=''+sRate;if(eFS)eFS.value=''+fftSz;var r=dbMax-dbMin;if(eNS){eNS.value=r;if(eNV)eNV[TC]=r|0;}if(eLS){eLV.value=dbMax;if(eLV)eLV[TC]=dbMax|0;}}

/* ---- Frequency display ---- */
function buildFD(){
var fd=$('freq-display'),hz=MR(tunedFreq);
var s=String(hz).padStart(10,'0');
var digits=fd.querySelectorAll('.fd');
for(var i=0;i<digits.length&&i<s.length;i++)digits[i][TC]=s[i];
/* Update VFO-B display */
var fb=$('freq-display-b');
if(fb){
  var bv=actVfo==='A'?vfoB:vfoA;
  var bs=String(MR(bv.f)).padStart(10,'0');
  var bSpan=fb.querySelector('.fd-b');
  if(bSpan)bSpan[TC]=bs.substr(0,3)+'.'+bs.substr(3,3)+'.'+bs.substr(6,3);
}
/* Update mode labels */
var am=$('vfo-a-mode'),bm=$('vfo-b-mode');
if(am)am[TC]=actVfo==='A'?mode:(vfoB?vfoB.m:'NFM');
if(bm)bm[TC]=actVfo==='A'?(vfoB?vfoB.m:'NFM'):mode;
/* Update dBm readout */
var dbmEl=$('vfo-a-dbm');
if(dbmEl&&smDbm)dbmEl[TC]=smDbm[TC];
}
function initFD(){
var fd=$('freq-display');
if(!fd)return;
fd.querySelectorAll('.fd').forEach(function(d){
on(d,'wheel',function(e){e.preventDefault();var step=PI(d.dataset.step)||1;
cFreq=clamp(MR(cFreq+(e.deltaY<0?1:-1)*step),24e6,1766e6);tunedFreq=cFreq;tuneOff=0;tx('freq',{value:cFreq});uD();},{passive:false});
on(d,'mouseenter',function(){d.classList.add('active');});
on(d,'mouseleave',function(){d.classList.remove('active');});});}

function uD(){
buildFD();
if(eRD)eRD[TC]=(sRate/1e3|0)+' kSPS';
if(eBW)eBW[TC]='BW: '+bwT();
if(eRit)eRit[TC]='RIT: '+(tuneOff>=0?'+':'')+tuneOff+' Hz';
if(eTK){eTK.value=0;}
if(eTL)eTL[TC]='\xb10';
}

/* ---- S-meter (horizontal bar) ---- */
function dBmFFT(f){var c=f.length/2|0,s=0,n=0;for(var i=c-20;i<=c+20;i++)if(i>=0&&i<f.length){s+=f[i];n++;}return dbMin+(s/(n||1)/255)*(dbMax-dbMin);}
function dBm2pct(d){
if(d<=-127)return 0;
if(d<=-73)return(d+127)/54*60;
if(d<=-13)return 60+(d+73)/60*40;
return 100;
}
function dBm2s(d){if(d<=-121)return'S0';if(d<=-73){var u=MR((d+127)/6);return'S'+clamp(u,1,9);}return'S9+'+MX(0,MR(d+73));}
function uSm(f){
var dbm=dBmFFT(f);
smTgt=dBm2pct(dbm);
if(smVal)smVal[TC]=dBm2s(dbm);
if(smDbm)smDbm[TC]=MR(dbm)+' dBm';
if(smTgt>smPkPct){smPkPct=smTgt;smPkT=performance.now();}
}
function tickSm(dt){
smPct+=(smTgt-smPct)*MN(1,dt*12);
if(smFill)smFill.style.width=clamp(smPct,0,100).toFixed(1)+'%';
var el=(performance.now()-smPkT)/1e3;
if(el>1.5)smPkPct=MX(smPct,smPkPct-30*dt);
if(smPkMark){
  if(smPkPct>1){
    smPkMark.style.left=clamp(smPkPct,0,100).toFixed(1)+'%';
    smPkMark.style.opacity='0.7';
  }else{
    smPkMark.style.opacity='0';
  }
}
}

/* ---- FFT spectrum drawing ---- */
function procFFT(raw){
var n=raw.length,dr=dbMax-dbMin;
if(!fftF32||fftF32.length!==n){fftF32=new Float32Array(n);avgBuf=new Float32Array(n);pkBuf=new Float32Array(n);}
for(var i=0;i<n;i++){var dB=dbMin+raw[i]/255*dr;fftF32[i]=dB;avgBuf[i]=avgBuf[i]*(1-avgA)+dB*avgA;
if(dB>pkBuf[i])pkBuf[i]=dB;else pkBuf[i]-=pkD;if(pkBuf[i]<dbMin)pkBuf[i]=dbMin;}
dSp();}

function dSp(){
var w=spCv.width,h=spCv.height;if(!w||!h||!fftF32)return;
var n=fftF32.length,v=vB(n),vl=v.e-v.s,xs=w/vl,dr=dbMax-dbMin;
function d2y(d){return h-(d-dbMin)/dr*h;}

/* SDR Console V3 dark background */
spCtx.fillStyle='#000000';
spCtx.fillRect(0,0,w,h);

/* Horizontal grid lines at each dBm step */
spCtx.lineWidth=0.5;
spCtx.font='9px monospace';
spCtx.textAlign='right';
var st=10;if(dr>100)st=20;
for(var d=MC(dbMin/st)*st;d<=dbMax;d+=st){
  var y=d2y(d);
  spCtx.strokeStyle='rgba(30,40,80,0.5)';
  spCtx.beginPath();spCtx.moveTo(0,y);spCtx.lineTo(w,y);spCtx.stroke();
  /* dBm labels on LEFT side */
  spCtx.fillStyle='#8899BB';
  spCtx.fillText(d+' dBm',52,y-2);
}

/* Frequency grid */
var vf=gVF(),fs=vf.bw,fst=1e3;
if(fs>1e8)fst=1e7;else if(fs>1e7)fst=1e6;else if(fs>1e6)fst=1e5;else if(fs>5e5)fst=5e4;else if(fs>1e5)fst=1e4;
spCtx.textAlign='center';
for(var f=MC(vf.s/fst)*fst;f<=vf.e;f+=fst){
  var x=f2x(f,w,vf);
  spCtx.strokeStyle='rgba(30,40,80,0.5)';
  spCtx.beginPath();spCtx.moveTo(x,0);spCtx.lineTo(x,h);spCtx.stroke();
  spCtx.fillStyle='#8899BB';
  spCtx.fillText(fF(f),x,h-2);
}

/* Average trace */
spCtx.strokeStyle='rgba(100,100,255,0.3)';spCtx.lineWidth=1;spCtx.beginPath();
for(var i=0;i<vl;i++){var xi=i*xs,yi=d2y(avgBuf[v.s+i]);i?spCtx.lineTo(xi,yi):spCtx.moveTo(xi,yi);}spCtx.stroke();

/* Peak trace */
spCtx.strokeStyle='#FFAA00';spCtx.lineWidth=0.8;spCtx.beginPath();
for(var i=0;i<vl;i++){var xi=i*xs,yi=d2y(pkBuf[v.s+i]);i?spCtx.lineTo(xi,yi):spCtx.moveTo(xi,yi);}spCtx.stroke();

/* Main FFT trace - white/light blue (SDR Console V3 dark theme) */
spCtx.strokeStyle='#C0D0FF';spCtx.lineWidth=1.5;spCtx.beginPath();
for(var i=0;i<vl;i++){var xi=i*xs,yi=d2y(fftF32[v.s+i]);i?spCtx.lineTo(xi,yi):spCtx.moveTo(xi,yi);}spCtx.stroke();

/* Fill under FFT trace - blue glacier */
var gr=spCtx.createLinearGradient(0,0,0,h);
gr.addColorStop(0,'rgba(40,80,180,0.4)');
gr.addColorStop(1,'rgba(20,40,100,0.1)');
spCtx.fillStyle=gr;spCtx.lineTo((vl-1)*xs,h);spCtx.lineTo(0,h);spCtx.closePath();spCtx.fill();

/* Filter overlay - BW trapezoid (SDR Console characteristic) */
dFO(spCv,spCtx,w,h);

/* Squelch line */
if(sql>0){var sy=d2y(dbMin+sql/255*dr);spCtx.strokeStyle='rgba(255,100,100,0.5)';spCtx.lineWidth=1;spCtx.setLineDash([4,4]);spCtx.beginPath();spCtx.moveTo(0,sy);spCtx.lineTo(w,sy);spCtx.stroke();spCtx.setLineDash([]);}

/* Cursor readout */
if(curX>=0&&curX<=w&&curY>=0&&curY<=h&&eCR)eCR[TC]=fF(MR(vf.s+curX/w*vf.bw))+' / '+(dbMax-curY/h*dr|0)+' dBm';
}

/* Filter overlay - BW trapezoid style (SDR Console) */
function dFO(cv,ctx,w,h){
var vf=gVF(),fl=f2x(tunedFreq-filterBW/2,w,vf),fr=f2x(tunedFreq+filterBW/2,w,vf),cx=f2x(tunedFreq,w,vf);
var bwPx=fr-fl;
/* Trapezoid shape: slight slope at edges */
var slope=MN(bwPx*0.08,6);

/* Filter passband fill - green for active VFO */
if(fr>fl){
  ctx.beginPath();
  ctx.moveTo(MX(0,fl-slope),0);
  ctx.lineTo(MX(0,fl),h);
  ctx.lineTo(MN(w,fr),h);
  ctx.lineTo(MN(w,fr+slope),0);
  ctx.closePath();
  ctx.fillStyle=audOn?'rgba(40,180,80,0.15)':'rgba(180,40,40,0.15)';
  ctx.fill();
}

/* Filter edges - green active, red inactive */
ctx.lineWidth=1.5;ctx.strokeStyle=audOn?'#44BB66':'#BB4444';
if(fl>=0&&fl<=w){ctx.beginPath();ctx.moveTo(fl-slope,0);ctx.lineTo(fl,h);ctx.stroke();}
if(fr>=0&&fr<=w){ctx.beginPath();ctx.moveTo(fr+slope,0);ctx.lineTo(fr,h);ctx.stroke();}

/* Center tuning line - red */
ctx.strokeStyle='#FF4444';ctx.lineWidth=1;
if(cx>=0&&cx<=w){ctx.beginPath();ctx.moveTo(cx,0);ctx.lineTo(cx,h);ctx.stroke();}

/* BW label */
ctx.font='9px monospace';ctx.fillStyle=audOn?'rgba(68,187,102,0.85)':'rgba(187,68,68,0.85)';ctx.textAlign='left';
ctx.fillText(bwT(),MN(w-52,MX(2,cx+3)),10);
}

/* ---- Waterfall (WebGL + Canvas fallback) ---- */
function initGL(){
try{var gl=wfCv.getContext('webgl2',{antialias:false,depth:false,stencil:false,alpha:false});
if(!gl)return false;wfGL=gl;
var vs=gl.createShader(gl.VERTEX_SHADER);
gl.shaderSource(vs,'#version 300 es\nin vec2 a;out vec2 uv;void main(){uv=a*.5+.5;gl_Position=vec4(a,0,1);}');gl.compileShader(vs);
var fs=gl.createShader(gl.FRAGMENT_SHADER);
gl.shaderSource(fs,'#version 300 es\nprecision mediump float;in vec2 uv;out vec4 o;uniform sampler2D u_d;uniform sampler2D u_c;uniform float u_o;uniform float u_h;void main(){float r=mod(uv.y*u_h+u_o,u_h)/u_h;o=texture(u_c,vec2(texture(u_d,vec2(uv.x,r)).r,0.5));}');
gl.compileShader(fs);
var pg=gl.createProgram();gl.attachShader(pg,vs);gl.attachShader(pg,fs);gl.linkProgram(pg);
if(!gl.getProgramParameter(pg,gl.LINK_STATUS)){wfGL=null;return false;}
glProg=pg;gl.useProgram(pg);
var buf=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buf);
gl.bufferData(gl.ARRAY_BUFFER,new Float32Array([-1,-1,1,-1,-1,1,1,1]),gl.STATIC_DRAW);
glVao=gl.createVertexArray();gl.bindVertexArray(glVao);
var al=gl.getAttribLocation(pg,'a');gl.enableVertexAttribArray(al);gl.vertexAttribPointer(al,2,gl.FLOAT,false,0,0);
glTex=gl.createTexture();gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,glTex);
gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.LINEAR);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.LINEAR);
gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
gl.texImage2D(gl.TEXTURE_2D,0,gl.R8,fftSz,glTH,0,gl.RED,gl.UNSIGNED_BYTE,null);
glCmap=gl.createTexture();gl.activeTexture(gl.TEXTURE1);gl.bindTexture(gl.TEXTURE_2D,glCmap);
gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.LINEAR);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.LINEAR);
gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);
gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,256,1,0,gl.RGBA,gl.UNSIGNED_BYTE,cmD);
gl.uniform1i(gl.getUniformLocation(pg,'u_d'),0);gl.uniform1i(gl.getUniformLocation(pg,'u_c'),1);
glUO=gl.getUniformLocation(pg,'u_o');glUH=gl.getUniformLocation(pg,'u_h');
gl.uniform1f(glUH,glTH);glRow=0;return true;}catch(e){return false;}}

function dWfGL(raw){
var gl=wfGL;if(!gl)return;gl.viewport(0,0,wfCv.width,wfCv.height);
gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,glTex);
gl.texSubImage2D(gl.TEXTURE_2D,0,0,glRow,raw.length,1,gl.RED,gl.UNSIGNED_BYTE,raw);
glRow=(glRow+1)%glTH;gl.useProgram(glProg);gl.uniform1f(glUO,glRow);
gl.bindVertexArray(glVao);gl.drawArrays(gl.TRIANGLE_STRIP,0,4);}

function dWfC(raw){
var w=wfCv.width,h=wfCv.height,wC=wfCtx;if(!w||!h||!wC)return;
var n=raw.length,v=vB(n),vl=v.e-v.s;
if(!wfID||wfID.width!==w||wfID.height!==h){wfID=wC.createImageData(w,h);var d=wfID.data;for(var i=3;i<d.length;i+=4)d[i]=255;}
var d=wfID.data,st=w*4;d.copyWithin(st,0,(h-1)*st);
for(var x=0;x<w;x++){var bn=v.s+(x*vl/w|0);if(bn>=v.e)bn=v.e-1;var ci=raw[bn]*4,ix=x*4;d[ix]=cmD[ci];d[ix+1]=cmD[ci+1];d[ix+2]=cmD[ci+2];}
wC.putImageData(wfID,0,0);}

function dWf(raw){if(wfGL)dWfGL(raw);else{dWfC(raw);if(wfCtx)dFO(wfCv,wfCtx,wfCv.width,wfCv.height);}}

/* ---- Audio: AudioWorklet + ScriptProcessor fallback ---- */
var DSP_CODE='function S(q,n){return(q[n]-127.5)/127.5;}'+
'function demod(q,m,st){var r=[],n,I,Q;'+
'if(m==="AM"){var dc=st.dc||0;for(n=0;n<q.length-1;n+=2){I=S(q,n);Q=S(q,n+1);var v=Math.sqrt(I*I+Q*Q);dc=dc*.999+v*.001;r.push(v-dc);}st.dc=dc;return r;}'+
'if(m==="USB"||m==="LSB"){var u=m==="USB";for(n=0;n<q.length-1;n+=2){I=S(q,n);Q=S(q,n+1);r.push(u?(I+Q)*.5:(I-Q)*.5);}return r;}'+
'if(m==="CW"){var ph=st.cp||0,bw=2*Math.PI*700/48e3;for(n=0;n<q.length-1;n+=2){I=S(q,n);Q=S(q,n+1);r.push((I*Math.cos(ph)+Q*Math.sin(ph))*.5);ph+=bw;}st.cp=ph;return r;}'+
'var pI=st.pI||0,pQ=st.pQ||0;for(n=0;n<q.length-3;n+=2){I=S(q,n);Q=S(q,n+1);r.push(Math.atan2(Q*pI-I*pQ,I*pI+Q*pQ)/Math.PI);pI=I;pQ=Q;}st.pI=pI;st.pQ=pQ;return r;}'+
'function deemph(s,st,tau,fs){if(!tau)return s;var w=Math.tan(1/(tau*2*fs)),b=w/(1+w),a=(w-1)/(w+1),x1=st.dx||0,y1=st.dy||0;'+
'for(var i=0;i<s.length;i++){var x=s[i],y=b*x+b*x1-a*y1;x1=x;y1=y;s[i]=y;}st.dx=x1;st.dy=y1;return s;}'+
'function agc(s,st,sp){if(sp==="off")return s;var f=48e3,at,dc,hm;'+
'if(sp==="fast"){at=Math.exp(-1/(.002*f));dc=Math.exp(-1/(.1*f));hm=.1*f;}'+
'else if(sp==="slow"){at=Math.exp(-1/(.025*f));dc=Math.exp(-1/(.5*f));hm=.5*f;}'+
'else{at=Math.exp(-1/(.005*f));dc=Math.exp(-1/(.2*f));hm=.3*f;}'+
'var pk=st.pk||.001,g=st.ag||1,hc=st.hc||0;'+
'for(var i=0;i<s.length;i++){var a=Math.abs(s[i]);if(a>pk){pk=at*pk+(1-at)*a;hc=hm;}else if(hc>0)hc--;else pk=dc*pk+(1-dc)*a;'+
'var gg=pk>.0001?.5/pk:1;if(gg>100)gg=100;g+=(gg-g)*.01;s[i]*=g;}st.pk=pk;st.ag=g;st.hc=hc;return s;}'+
'function resamp(s,sr,rB){var R=sr/48e3;'+
'if(R>1.01){for(var k=0,p=0;p<s.length;k++,p+=R)rB.push(s[p|0]);}'+
'else if(R<.99){for(var k=0;k<s.length/R;k++){var f=k*R,l=f|0;rB.push(l+1<s.length?s[l]+(s[l+1]-s[l])*(f-l):s[l]);}}'+
'else{for(var k=0;k<s.length;k++)rB.push(s[k]);}}'+
'function nb(samples,level){'+
'if(level<=0)return samples;'+
'var threshold=3.0+(255-level)*0.1;'+
'var len=samples.length,blockSize=32;'+
'for(var b=0;b<len;b+=blockSize){'+
'var end=Math.min(b+blockSize,len),sum=0;'+
'for(var i=b;i<end;i++)sum+=samples[i]*samples[i];'+
'var rms=Math.sqrt(sum/(end-b))+1e-10,thr=rms*threshold;'+
'for(var i=b;i<end;i++){if(Math.abs(samples[i])>thr){'+
'var prev=(i>0)?samples[i-1]:0,next=(i<len-1)?samples[i+1]:0;'+
'samples[i]=(prev+next)*0.5;}}}return samples;}'+
'var nrBuf=new Float32Array(256),nrPos=0,nrNoiseFloor=new Float32Array(129),nrAlpha=0.98;'+
'function nr(samples,level){'+
'if(level<=0)return samples;'+
'var subtraction=1.0+level/64.0;'+
'var out=new Float32Array(samples.length),oi=0;'+
'for(var s=0;s<samples.length;s++){'+
'nrBuf[nrPos++]=samples[s];'+
'if(nrPos>=256){nrPos=0;'+
'var windowed=new Float32Array(256);'+
'for(var i=0;i<256;i++)windowed[i]=nrBuf[i]*(0.5-0.5*Math.cos(2*Math.PI*i/256));'+
'var blockPwr=0;'+
'for(var i=0;i<256;i++)blockPwr+=windowed[i]*windowed[i];'+
'blockPwr/=256;'+
'var noiseEst=nrNoiseFloor[0];'+
'nrNoiseFloor[0]=nrAlpha*nrNoiseFloor[0]+(1-nrAlpha)*blockPwr;'+
'var gain=1.0;'+
'if(blockPwr<noiseEst*subtraction*subtraction){'+
'gain=Math.max(0.01,blockPwr/(noiseEst*subtraction*subtraction+1e-10));'+
'gain=Math.sqrt(gain);}'+
'for(var i=0;i<256;i++){if(oi+i<out.length)out[oi+i]=samples[oi+i]*gain;}'+
'oi+=256;}}'+
'for(;oi<samples.length;oi++)out[oi]=samples[oi];'+
'return out;}'+
'var notchFreq=0,notchW0=0,notchW1=0;'+
'function notch(samples,enabled,autoMode){'+
'if(!enabled)return samples;'+
'var Q=30,r=1.0-Math.PI/Q/(48000/2);'+
'if(r<0.8)r=0.8;'+
'if(autoMode){'+
'var crossings=0;'+
'for(var i=1;i<samples.length;i++){if((samples[i]>=0)!==(samples[i-1]>=0))crossings++;}'+
'var estFreq=crossings/2.0/samples.length*48000;'+
'if(notchFreq===0)notchFreq=estFreq;'+
'else notchFreq=notchFreq*0.95+estFreq*0.05;}'+
'if(notchFreq<=0||notchFreq>=24000)return samples;'+
'var w0=2*Math.PI*notchFreq/48000,cosw=Math.cos(w0);'+
'var a1=-2*r*cosw,a2=r*r,b0=1,b1=-2*cosw,b2=1;'+
'var norm=(1+a1+a2)/(b0+b1+b2);b0*=norm;b1*=norm;b2*=norm;'+
'for(var i=0;i<samples.length;i++){'+
'var x=samples[i],y=b0*x+notchW0;'+
'notchW0=b1*x-a1*y+notchW1;'+
'notchW1=b2*x-a2*y;'+
'samples[i]=y;}'+
'return samples;}';

var WK_CODE=DSP_CODE+
'var iqB=[],st={},C={mode:"NFM",ir:48e3,ag:"medium",vol:.5,sql:0,de:75e-6,lim:true,limD:2.0,limC:0.35,nbEn:false,nbLvl:0,nrEn:false,nrLvl:0,notchEn:false,notchAuto:false},mCnt=0;'+
'var TM={"off":0,"50us":50e-6,"75us":75e-6};'+
'class P extends AudioWorkletProcessor{constructor(){super();this.rB=[];'+
'this.port.onmessage=function(e){var d=e.data;if(d.type==="iq")iqB.push(d.data);'+
'else if(d.type==="config"){for(var k in d)if(k!=="type")C[k]=k==="deemph"?TM[d[k]]||75e-6:d[k];}};}'+
'process(i,o){var out=o[0][0];if(!out)return true;var rB=this.rB;'+
'while(iqB.length>0&&rB.length<out.length*2){var raw=iqB.shift(),s=demod(raw,C.mode,st);'+
'if(C.mode==="NFM"||C.mode==="WBFM")s=deemph(s,st,C.de,48e3);'+
'if(C.nbEn)s=nb(s,C.nbLvl);'+
'if(C.nrEn)s=nr(s,C.nrLvl);'+
'if(C.notchEn)s=notch(s,true,C.notchAuto);'+
'if(!Array.isArray(s)){var tmp=[];for(var _i=0;_i<s.length;_i++)tmp.push(s[_i]);s=tmp;}'+
's=agc(s,st,C.ag);resamp(s,C.ir||48e3,rB);}'+
'var pw=0,pk=0,ch=Math.min(rB.length,out.length);for(var i=0;i<ch;i++)pw+=rB[i]*rB[i];pw/=ch||1;'+
'var mt=C.sql>0&&pw<C.sql/255*.01;'+
'for(var p=0;p<ch;p++){var v=mt?0:rB[p]*C.vol;'+
'if(C.lim&&C.mode==="WBFM"){v=Math.tanh(v*C.limD)*C.limC;}'+
'out[p]=v;var av=Math.abs(v);if(av>pk)pk=av;}'+
'this.rB=ch<rB.length?rB.slice(ch):[];for(;p<out.length;p++)out[p]=0;'+
'if(++mCnt%20===0)this.port.postMessage({pk:pk,buf:rB.length});'+
'return true;}}'+
'registerProcessor("sdr-processor",P);';

var spCfg={mode:'NFM',ir:48e3,ag:'medium',vol:.5,sql:0,deemph:'75us',lim:true,limD:2.0,limC:0.35};
function sendCfg(){var c={type:'config',mode:mode,ir:iqRate,ag:settings.agcSpd,vol:vol,sql:sql,deemph:settings.deemph,lim:settings.limiter,limD:settings.limDrive,limC:settings.limCeil};
spCfg.mode=mode;spCfg.ir=iqRate;spCfg.ag=settings.agcSpd;spCfg.vol=vol;spCfg.sql=sql;spCfg.deemph=settings.deemph;spCfg.lim=settings.limiter;spCfg.limD=settings.limDrive;spCfg.limC=settings.limCeil;
if(wkNode&&wkNode.port)wkNode.port.postMessage(c);}
function pumpIQ(){if(!wkReady||!wkNode)return;while(aQ.length)wkNode.port.postMessage({type:'iq',data:aQ.shift()});}

function startAud(){
if(aCtx)return;aCtx=new(window.AudioContext||window.webkitAudioContext)({sampleRate:48000});aCtx.resume();
gNode=aCtx.createGain();gNode.gain.value=1;gNode.connect(aCtx.destination);
if(typeof AudioWorkletNode!=='undefined'&&aCtx.audioWorklet){
var blob=new Blob([WK_CODE],{type:'application/javascript'}),url=URL.createObjectURL(blob);
aCtx.audioWorklet.addModule(url).then(function(){URL.revokeObjectURL(url);
wkNode=new AudioWorkletNode(aCtx,'sdr-processor');wkNode.connect(gNode);wkReady=true;
wkNode.port.onmessage=function(e){var d=e.data;if(d.pk!==undefined){var pct=MN(100,d.pk/0.35*100);var el=$('audio-meter-fill');if(el)el.style.width=pct.toFixed(1)+'%';}};
audOn=true;subIQ();sendCfg();eAB[TC]='Stop Audio';eAB.classList.add('active');
var enBtn=$('vfo-a-enable');if(enBtn)enBtn.classList.add('active');
}).catch(function(){startSP();});}else startSP();}

function startSP(){
var sp=aCtx.createScriptProcessor(4096,1,1),rBuf=[],st={pI:0,pQ:0,deY:0,sIdx:0};
sp.onaudioprocess=function(e){
var o=e.outputBuffer.getChannelData(0),C=spCfg,ir=C.ir||150000;
/* Demod queued IQ frames */
while(aQ.length>0&&rBuf.length<o.length*2){
  var raw=aQ.shift(),n=raw.length/2;if(n<2)continue;
  if(C.mode==='WBFM'){
    /* FM discriminator + de-emphasis + decimation (fm_player approach) */
    var dec=MR(ir/48e3);if(dec<1)dec=1;
    for(var k=0;k<n;k++){
      var ci=(raw[k*2]-127.5)/127.5,cq=(raw[k*2+1]-127.5)/127.5;
      var re=ci*st.pI+cq*st.pQ,im=cq*st.pI-ci*st.pQ;
      var s=Math.atan2(im,re)/MP;
      st.pI=ci;st.pQ=cq;
      /* 75us de-emphasis IIR */
      st.deY=0.217*s+0.783*st.deY;
      s=st.deY;
      st.sIdx++;
      if(st.sIdx%dec===0)rBuf.push(s);
    }
  }else{
    /* Other modes: use existing DSP chain */
    var r=[],pI=st.pI,pQ=st.pQ;
    for(var k=0;k<raw.length-1;k+=2){
      var I=(raw[k]-127.5)/127.5,Q=(raw[k+1]-127.5)/127.5;
      if(C.mode==='AM'){var v=MS(I*I+Q*Q);var dc=st.dc||0;dc=dc*.999+v*.001;r.push(v-dc);st.dc=dc;}
      else if(C.mode==='USB')r.push((I+Q)*.5);
      else if(C.mode==='LSB')r.push((I-Q)*.5);
      else if(C.mode==='CW'){var ph=st.cp||0,bw=2*MP*700/48e3;r.push((I*Math.cos(ph)+Q*Math.sin(ph))*.5);ph+=bw;st.cp=ph;}
      else{/* NFM */r.push(Math.atan2(Q*pI-I*pQ,I*pI+Q*pQ)/MP);pI=I;pQ=Q;}
    }
    st.pI=pI;st.pQ=pQ;
    /* De-emphasis for NFM */
    if(C.mode==='NFM'){
      var tau=C.deemph==='50us'?50e-6:C.deemph==='off'?0:75e-6;
      if(tau){var w=Math.tan(1/(tau*2*48e3)),b=w/(1+w),a=(w-1)/(w+1),x1=st.dx||0,y1=st.dy||0;
      for(var i=0;i<r.length;i++){var x=r[i],y=b*x+b*x1-a*y1;x1=x;y1=y;r[i]=y;}st.dx=x1;st.dy=y1;}
    }
    /* Resample to 48kHz */
    var R=ir/48e3;
    if(R>1.01){for(var j=0,pp=0;pp<r.length;j++,pp+=R)rBuf.push(r[pp|0]);}
    else{for(var j=0;j<r.length;j++)rBuf.push(r[j]);}
  }
}
/* Output with limiter + metering */
var pk=0,ch=MN(rBuf.length,o.length);
for(var p=0;p<ch;p++){
  var v=rBuf[p]*C.vol;
  if(C.lim&&C.mode==='WBFM')v=Math.tanh(v*C.limD)*C.limC;
  o[p]=v;var av=MA(v);if(av>pk)pk=av;
}
rBuf=ch<rBuf.length?rBuf.slice(ch):[];for(;p<o.length;p++)o[p]=0;
var el=$('audio-meter-fill');if(el)el.style.width=MN(100,pk/0.35*100).toFixed(1)+'%';};
sp.connect(gNode);wkNode=sp;audOn=true;subIQ();sendCfg();eAB[TC]='Stop Audio';eAB.classList.add('active');
var enBtn=$('vfo-a-enable');if(enBtn)enBtn.classList.add('active');}

function stopAud(){
audOn=false;if(ws&&ws.readyState===1)tx('unsubscribe_iq',{});
if(wkNode){try{wkNode.disconnect();}catch(e){}wkNode=null;wkReady=false;}
if(gNode){try{gNode.disconnect();}catch(e){}gNode=null;}
if(aCtx){try{aCtx.close();}catch(e){}aCtx=null;}aQ=[];iqRate=0;
if(eAB){eAB[TC]='Start Audio';eAB.classList.remove('active');}
var enBtn=$('vfo-a-enable');if(enBtn)enBtn.classList.remove('active');}

/* ---- Mouse/touch interaction ---- */
function nrE(x,W){var vf=gVF(),fl=f2x(tunedFreq-filterBW/2,W,vf),fr=f2x(tunedFreq+filterBW/2,W,vf);if(MA(x-fl)<7)return'l';if(MA(x-fr)<7)return'r';return null;}
function gcx(e){return e.touches&&e.touches.length?e.touches[0].clientX:e.clientX;}
function gcy(e){return e.touches&&e.touches.length?e.touches[0].clientY:e.clientY;}
function commitF(){if(pndF!==null){tx('freq',{value:pndF});pndF=null;}}

function onDn(e){e.preventDefault();var t=e.currentTarget,r=gbc(t),x=gcx(e)-r.left,edge=nrE(x,r.width);
if(edge){isResize=true;resEdge=edge;t.style.cursor='ew-resize';return;}isDrag=true;dragMoved=false;dragX=x;dragF=cFreq;t.style.cursor='grabbing';}

function onMv(e){e.preventDefault();var t=e.currentTarget,r=gbc(t),x=gcx(e)-r.left;curX=x;curY=gcy(e)-r.top;
if(!isDrag&&!isResize){t.style.cursor=nrE(x,r.width)?'ew-resize':'crosshair';return;}
if(isDrag){var dx=x-dragX;if(MA(dx)>3)dragMoved=true;var nf=clamp(MR(dragF-dx*hpp(r.width)),24e6,1766e6);
cFreq=nf;tunedFreq=nf;tuneOff=0;pndF=nf;if(fTmr)clearTimeout(fTmr);fTmr=setTimeout(commitF,120);buildFD();}
if(isResize){var vf=gVF(),fHz=vf.s+x/r.width*vf.bw;filterBW=clamp(MR(MA(fHz-tunedFreq)*2),200,sRate*.9);if(eBW)eBW[TC]='BW: '+bwT();}}

function onUp(e){var t=e.currentTarget,r=gbc(t),x=(e.changedTouches?e.changedTouches[0].clientX:gcx(e))-r.left;
if(isDrag&&!dragMoved){var v=vB(fftSz);tunedFreq=MR(cFreq-sRate/2+(v.s+x/r.width*(v.e-v.s))/fftSz*sRate);tuneOff=0;subIQ();buildFD();}
if(isDrag){commitF();isDrag=false;dragMoved=false;}if(isResize){isResize=false;resEdge=null;subIQ();sendCfg();}t.style.cursor='crosshair';}

function onLv(e){if(isDrag){commitF();isDrag=false;dragMoved=false;}if(isResize){isResize=false;resEdge=null;}curX=-1;curY=-1;if(eCR)eCR[TC]='';e.currentTarget.style.cursor='crosshair';}

function onWhl(e){e.preventDefault();
if(e.ctrlKey){filterBW=clamp(MR(filterBW*(e.deltaY<0?1.1:.9)),200,sRate*.9);if(eBW)eBW[TC]='BW: '+bwT();subIQ();sendCfg();return;}
var r=gbc(e.currentTarget),v=vB(fftSz),vl=v.e-v.s;zCtr=(v.s+(e.clientX-r.left)/r.width*vl)/fftSz;
if(e.deltaY<0&&zLvl<16)zLvl*=2;else if(e.deltaY>0&&zLvl>1)zLvl/=2;if(eZV)eZV[TC]=zLvl+'x';rstBuf();}

var pd0=0,pz0=1;
function onTS2(e){if(e.touches.length===2){var dx=e.touches[0].clientX-e.touches[1].clientX,dy=e.touches[0].clientY-e.touches[1].clientY;pd0=MS(dx*dx+dy*dy);pz0=zLvl;e.preventDefault();}}
function onTM2(e){if(e.touches.length===2){var dx=e.touches[0].clientX-e.touches[1].clientX,dy=e.touches[0].clientY-e.touches[1].clientY;zLvl=clamp(MR(pz0*MS(dx*dx+dy*dy)/pd0),1,16);if(eZV)eZV[TC]=zLvl+'x';rstBuf();e.preventDefault();}}

[spCv,wfCv].forEach(function(cv){
on(cv,'mousedown',onDn);on(cv,'mousemove',onMv);on(cv,'mouseup',onUp);on(cv,'mouseleave',onLv);
on(cv,'wheel',onWhl,{passive:false});
on(cv,'touchstart',function(e){e.touches.length>=2?onTS2(e):onDn(e);},{passive:false});
on(cv,'touchmove',function(e){e.touches.length>=2?onTM2(e):onMv(e);},{passive:false});
on(cv,'touchend',onUp,{passive:false});});

/* Resize handle */
var rh=$('resize-handle');
if(rh){on(rh,'mousedown',function(e){rhDrag=true;rhY0=e.clientY;rhH0=gbc($('spectrum-wrap')).height;e.preventDefault();});
on(document,'mousemove',function(e){if(!rhDrag)return;$('spectrum-wrap').style.flexBasis=clamp(rhH0+e.clientY-rhY0,60,window.innerHeight*.7)+'px';rsz();});
on(document,'mouseup',function(){rhDrag=false;});
on(rh,'dblclick',function(){$('spectrum-wrap').style.flexBasis='40%';rsz();});}

/* ---- Ribbon tabs ---- */
function initRibbon(){
qsa('.ribbon-tab').forEach(function(tab){
  on(tab,'click',function(){
    qsa('.ribbon-tab').forEach(function(t){t.classList.remove('active');});
    tab.classList.add('active');
    qsa('.ribbon-panel').forEach(function(p){p.classList.remove('active');});
    var panel=document.querySelector('.ribbon-panel[data-panel="'+tab.dataset.tab+'"]');
    if(panel)panel.classList.add('active');
  });
});
}

/* ---- DSP Sidebar sections ---- */
function initDSP(){
qsa('.dsp-section-header').forEach(function(hdr){
  on(hdr,'click',function(){
    var section=hdr.parentElement;
    section.classList.toggle('open');
    var chev=hdr.querySelector('.dsp-chevron');
    if(chev)chev.innerHTML=section.classList.contains('open')?'&#9660;':'&#9654;';
  });
});

/* AGC speed in sidebar */
var agcSel=$('dsp-agc-speed');
if(agcSel){on(agcSel,'change',function(){settings.agcSpd=this.value;sendCfg();saveSet();});}

/* Squelch enable checkbox */
var sqlEn=$('dsp-sql-enable');
if(sqlEn){on(sqlEn,'change',function(){
  if(!this.checked){sql=0;if(eSS)eSS.value=0;if(eSV)eSV[TC]='0';sendCfg();}
  else{sql=128;if(eSS)eSS.value=128;if(eSV)eSV[TC]='128';sendCfg();}
});}

/* Squelch auto */
var sqlAuto=$('dsp-sql-auto');
if(sqlAuto){on(sqlAuto,'click',function(){sql=128;if(eSS)eSS.value=128;if(eSV)eSV[TC]='128';var en=$('dsp-sql-enable');if(en)en.checked=true;sendCfg();});}

/* Noise Blanker enable */
var nbEn=$('dsp-nb-enable'),nbLvl=$('dsp-nb-level');
if(nbEn&&nbLvl){
  on(nbEn,'change',function(){
    nbLvl.disabled=!this.checked;
    if(wkNode&&wkNode.port)wkNode.port.postMessage({type:'config',nbEn:this.checked,nbLvl:PI(nbLvl.value)||128});
  });
  on(nbLvl,'input',function(){
    if(wkNode&&wkNode.port)wkNode.port.postMessage({type:'config',nbLvl:PI(this.value)});
  });
}

/* Noise Reduction enable */
var nrEn=$('dsp-nr-enable'),nrLvl=$('dsp-nr-level');
if(nrEn&&nrLvl){
  on(nrEn,'change',function(){
    nrLvl.disabled=!this.checked;
    if(wkNode&&wkNode.port)wkNode.port.postMessage({type:'config',nrEn:this.checked,nrLvl:PI(nrLvl.value)||128});
  });
  on(nrLvl,'input',function(){
    if(wkNode&&wkNode.port)wkNode.port.postMessage({type:'config',nrLvl:PI(this.value)});
  });
}

/* Notch enable */
var notchEn=$('dsp-notch-enable');
if(notchEn){on(notchEn,'change',function(){
  if(wkNode&&wkNode.port)wkNode.port.postMessage({type:'config',notchEn:this.checked});
});}
var notchAuto=$('dsp-notch-auto');
if(notchAuto){on(notchAuto,'click',function(){
  if(notchEn)notchEn.checked=true;
  if(wkNode&&wkNode.port)wkNode.port.postMessage({type:'config',notchEn:true,notchAuto:true});
});}

/* Filter width buttons */
qsa('.filter-btn').forEach(function(btn){
  on(btn,'click',function(){
    filterBW=PI(btn.dataset.bw);
    qsa('.filter-btn').forEach(function(b){b.classList.toggle('active',b.dataset.bw===btn.dataset.bw);});
    if(eBW)eBW[TC]='BW: '+bwT();
    subIQ();sendCfg();
  });
});
}

/* ---- Band buttons ---- */
function initBands(){
try{var s=localStorage.getItem('sdr_bs');if(s)bStack=JSON.parse(s);}catch(e){}
qsa('.band-panel').forEach(function(panel){
  var bd=panel.dataset.bands;if(!bd)return;
  bd.split(',').forEach(function(entry){
    var parts=entry.split(':');
    var btn=DC('button');btn.className='band-btn';
    btn[TC]=parts[0];btn.dataset.freq=parts[1];btn.dataset.mode=parts[2];btn.title=parts[0];
    panel.appendChild(btn);
  });
});
/* Also populate favourites tab */
var favPanel=$('fav-bands');
if(favPanel){
  qsa('#band-panel-hf .band-btn, #band-panel-vhf .band-btn, #band-panel-bc .band-btn').forEach(function(btn){
    var fb=DC('button');fb.className='ribbon-btn';
    fb[TC]=btn[TC];fb.dataset.freq=btn.dataset.freq;fb.dataset.mode=btn.dataset.mode;
    on(fb,'click',function(){
      cFreq=PI(fb.dataset.freq);tunedFreq=cFreq;setM(fb.dataset.mode);filterBW=modeBW[mode]||12500;
      tuneOff=0;tx('freq',{value:cFreq});subIQ();uD();
    });
    favPanel.appendChild(fb);
  });
}
qsa('.band-tab').forEach(function(tab){on(tab,'click',function(){
  /* Find nearest band-tabs container */
  var container=tab.parentElement;
  container.querySelectorAll('.band-tab').forEach(function(t){t.classList.remove('active');t.setAttribute('aria-selected','false');});
  tab.classList.add('active');tab.setAttribute('aria-selected','true');
  /* Find sibling panels */
  var parent=container.parentElement;
  parent.querySelectorAll('.band-panel').forEach(function(p){p.hidden=true;});
  var pn=parent.querySelector('#band-panel-'+tab.dataset.tab);
  if(pn)pn.hidden=false;
});});
qsa('.band-btn').forEach(function(b){on(b,'click',function(){var nm=b.title||b[TC],st=bStack[nm];
if(st){cFreq=st.f;tunedFreq=st.f;setM(st.m);filterBW=st.b;}else{cFreq=PI(b.dataset.freq);tunedFreq=cFreq;setM(b.dataset.mode);filterBW=modeBW[mode]||12500;}
tuneOff=0;tx('freq',{value:cFreq});subIQ();uD();});});}

var preWbfmRate=null;
function setM(m){if(!m)return;
/* Auto sample rate for WBFM: switch to 300kHz, restore on exit */
if(m==='WBFM'&&mode!=='WBFM'){preWbfmRate=sRate;tx('sample_rate',{value:300000});if(eRS)eRS.value='250000';}
else if(m!=='WBFM'&&mode==='WBFM'&&preWbfmRate){tx('sample_rate',{value:preWbfmRate});if(eRS)eRS.value=''+preWbfmRate;preWbfmRate=null;}
mode=m;filterBW=modeBW[m]||12500;
qsa('.mode-btn').forEach(function(x){x.classList.toggle('active',x.dataset.mode===m);});
/* Update filter btn highlights */
qsa('.filter-btn').forEach(function(b){b.classList.toggle('active',PI(b.dataset.bw)===filterBW);});
if(eBW)eBW[TC]='BW: '+bwT();sendCfg();}

/* ---- VFO switching ---- */
function initVFO(){
/* VFO selection buttons in ribbon */
qsa('.vfo-sel-btn').forEach(function(b){on(b,'click',function(){
  if(b.dataset.vfo==='A'||b.dataset.vfo==='B'){
    var cv=actVfo==='A'?vfoA:vfoB;cv.f=tunedFreq;cv.m=mode;cv.b=filterBW;
    actVfo=b.dataset.vfo;
    qsa('.vfo-sel-btn').forEach(function(x){x.classList.toggle('active',x.dataset.vfo===actVfo);});
    var va=$('vfo-a-block'),vb=$('vfo-b-block');
    if(va)va.classList.toggle('active',actVfo==='A');
    if(vb)vb.classList.toggle('active',actVfo==='B');
    var nv=actVfo==='A'?vfoA:vfoB;cFreq=nv.f;tunedFreq=nv.f;tuneOff=0;setM(nv.m);filterBW=nv.b;tx('freq',{value:cFreq});subIQ();uD();
  }
});});

/* VFO enable button */
var enBtn=$('vfo-a-enable');
if(enBtn){on(enBtn,'click',function(){audOn?stopAud():startAud();});}
}

function initMode(){qsa('.mode-btn').forEach(function(b){on(b,'click',function(){setM(b.dataset.mode);subIQ();uD();});});}

/* ---- Hover overlay controls ---- */
function initHoverControls(){
var hiUp=$('hc-high-up'),hiDn=$('hc-high-dn'),hiAuto=$('hc-high-auto');
var loUp=$('hc-low-up'),loDn=$('hc-low-dn');
var zmIn=$('hc-zoom-in'),zmOut=$('hc-zoom-out'),zmRst=$('hc-zoom-reset');

if(hiUp)on(hiUp,'click',function(){if(eLS){eLS.value=PI(eLS.value)+5;if(eLV)eLV[TC]=eLS.value;tDb();}});
if(hiDn)on(hiDn,'click',function(){if(eLS){eLS.value=PI(eLS.value)-5;if(eLV)eLV[TC]=eLS.value;tDb();}});
if(hiAuto)on(hiAuto,'click',function(){
  /* Auto-scale reference level based on peak FFT */
  if(fftF32){var pk=-999;for(var i=0;i<fftF32.length;i++)if(fftF32[i]>pk)pk=fftF32[i];
  var nRef=MR(pk+10);if(eLS){eLS.value=nRef;if(eLV)eLV[TC]=nRef;tDb();}}
});
if(loUp)on(loUp,'click',function(){if(eNS){eNS.value=PI(eNS.value)-5;if(eNV)eNV[TC]=eNS.value;tDb();}});
if(loDn)on(loDn,'click',function(){if(eNS){eNS.value=PI(eNS.value)+5;if(eNV)eNV[TC]=eNS.value;tDb();}});
if(zmIn)on(zmIn,'click',function(){if(zLvl<16){zLvl*=2;if(eZV)eZV[TC]=zLvl+'x';rstBuf();}});
if(zmOut)on(zmOut,'click',function(){if(zLvl>1){zLvl/=2;if(eZV)eZV[TC]=zLvl+'x';rstBuf();}});
if(zmRst)on(zmRst,'click',function(){zLvl=1;zCtr=.5;if(eZV)eZV[TC]='1x';rstBuf();});
}

/* ---- Bottom tabs ---- */
function initBottomTabs(){
qsa('.bottom-tab').forEach(function(tab){
  on(tab,'click',function(){
    qsa('.bottom-tab').forEach(function(t){t.classList.remove('active');});
    tab.classList.add('active');
    qsa('.bottom-pane').forEach(function(p){p.classList.remove('active');});
    var pane=document.querySelector('.bottom-pane[data-bpane="'+tab.dataset.btab+'"]');
    if(pane)pane.classList.add('active');
  });
});

/* Span buttons */
qsa('.span-btn[data-zoom]').forEach(function(btn){
  on(btn,'click',function(){
    zLvl=PI(btn.dataset.zoom);zCtr=.5;
    if(eZV)eZV[TC]=zLvl+'x';
    qsa('.span-btn[data-zoom]').forEach(function(b){b.classList.toggle('active',b===btn);});
    rstBuf();
  });
});

var pkClr=$('peak-clear');
if(pkClr)on(pkClr,'click',function(){pkBuf=null;});
}

/* ---- Settings (stored in localStorage) ---- */
function initSet(){
try{var s=localStorage.getItem('sdr_set');if(s){var p=JSON.parse(s);for(var k in p)settings[k]=p[k];}}catch(e){}
try{var m=localStorage.getItem('sdr_mem');if(m)mems=JSON.parse(m);}catch(e){}
applySet();

/* Ribbon display tab controls */
var cmSel=$('set-colormap');
if(cmSel)on(cmSel,'change',function(){settings.cmap=this.value;setCm(this.value);saveSet();});

var avgSl=$('set-fft-avg');
if(avgSl)on(avgSl,'input',function(){settings.fftAvg=PI(this.value);avgA=PI(this.value)/100;var lbl=$('set-fft-avg-val');if(lbl)lbl[TC]=avgA.toFixed(2);saveSet();});

var pkSl=$('set-peak-decay');
if(pkSl)on(pkSl,'input',function(){settings.pkDecay=PI(this.value);pkD=PI(this.value)/10;var lbl=$('set-peak-decay-val');if(lbl)lbl[TC]=pkD.toFixed(1);saveSet();});

/* Ribbon DSP tab controls */
var agcSel=$('set-agc-speed');
if(agcSel)on(agcSel,'change',function(){settings.agcSpd=this.value;
  var dspAgc=$('dsp-agc-speed');if(dspAgc)dspAgc.value=this.value;
  sendCfg();saveSet();});

var deemSel=$('set-deemph');
if(deemSel)on(deemSel,'change',function(){settings.deemph=this.value;sendCfg();saveSet();});

/* Limiter controls */
var limCb=$('set-limiter');
if(limCb)on(limCb,'change',function(){settings.limiter=this.checked;sendCfg();saveSet();});
var limDrv=$('set-lim-drive');
if(limDrv)on(limDrv,'input',function(){settings.limDrive=PI(this.value)/10;var lbl=$('set-lim-drive-val');if(lbl)lbl[TC]=settings.limDrive.toFixed(1);sendCfg();saveSet();});
var limCl=$('set-lim-ceil');
if(limCl)on(limCl,'input',function(){settings.limCeil=PI(this.value)/100;var lbl=$('set-lim-ceil-val');if(lbl)lbl[TC]=settings.limCeil.toFixed(2);sendCfg();saveSet();});

/* Memory add */
var memAdd=$('mem-add-btn');
if(memAdd)on(memAdd,'click',function(){var n=prompt('Channel name:');if(!n)return;mems.push({n:n,f:tunedFreq,m:mode,b:filterBW});saveSet();popMem();});
}

function popMem(){
var l=$('mem-channel-list');if(!l)return;l.innerHTML='';
mems.forEach(function(m,i){
  var b=DC('button');b.className='ribbon-btn';
  b[TC]=m.n+' '+fF(m.f)+' '+m.m;
  on(b,'click',function(){cFreq=m.f;tunedFreq=m.f;tuneOff=0;setM(m.m);filterBW=m.b;tx('freq',{value:cFreq});subIQ();uD();});
  l.appendChild(b);
  var d=DC('button');d.className='ribbon-btn';d[TC]='\u2715';d.title='Delete';
  on(d,'click',function(){mems.splice(i,1);saveSet();popMem();});
  l.appendChild(d);
});}

function saveSet(){try{localStorage.setItem('sdr_set',JSON.stringify(settings));}catch(e){}try{localStorage.setItem('sdr_mem',JSON.stringify(mems));}catch(e){}}

function applySet(){
if(settings.cmap!==cmN)setCm(settings.cmap);
avgA=settings.fftAvg/100;pkD=settings.pkDecay/10;
/* Sync DSP sidebar AGC */
var dspAgc=$('dsp-agc-speed');if(dspAgc)dspAgc.value=settings.agcSpd;
var ribAgc=$('set-agc-speed');if(ribAgc)ribAgc.value=settings.agcSpd;
/* Sync limiter controls */
if(settings.limiter===undefined)settings.limiter=true;
if(settings.limDrive===undefined)settings.limDrive=2.0;
if(settings.limCeil===undefined)settings.limCeil=0.35;
var lCb=$('set-limiter');if(lCb)lCb.checked=settings.limiter;
var lDr=$('set-lim-drive');if(lDr){lDr.value=MR(settings.limDrive*10);var lbl=$('set-lim-drive-val');if(lbl)lbl[TC]=settings.limDrive.toFixed(1);}
var lCl=$('set-lim-ceil');if(lCl){lCl.value=MR(settings.limCeil*100);var lbl=$('set-lim-ceil-val');if(lbl)lbl[TC]=settings.limCeil.toFixed(2);}
}

/* ---- Controls ---- */
function initCtrl(){
if(eAB)on(eAB,'click',function(){audOn?stopAud():startAud();});
if(eRS)on(eRS,'change',function(){tx('sample_rate',{value:PI(this.value)});});
if(eFS)on(eFS,'change',function(){tx('fft_size',{value:PI(this.value)});rstBuf();});

function dbI(){
  if(this===eNS&&eNV)eNV[TC]=this.value;
  if(this===eLS&&eLV)eLV[TC]=this.value;
  tDb();
}
if(eNS)on(eNS,'input',dbI);
if(eLS)on(eLS,'input',dbI);

if(eGS)on(eGS,'input',function(){gain=PI(this.value);if(eGV)eGV[TC]=gain?((gain/10).toFixed(1)+' dB'):'Auto';tx('gain',{value:gain});});
if(eVS)on(eVS,'input',function(){vol=PI(this.value)/100;if(eVV)eVV[TC]=(vol*100|0);if(gNode)gNode.gain.value=vol;sendCfg();});
if(eSS)on(eSS,'input',function(){sql=PI(this.value);if(eSV)eSV[TC]=sql;var en=$('dsp-sql-enable');if(en)en.checked=sql>0;sendCfg();});
if(eWS)on(eWS,'input',function(){wfSpd=110-PI(this.value);if(eWV)eWV[TC]=this.value;});

if(eTK)on(eTK,'input',function(){tuneOff=PI(this.value);if(eTL)eTL[TC]=(tuneOff>=0?'+':'')+tuneOff;tunedFreq=MR(cFreq+tuneOff);buildFD();if(eBW)eBW[TC]='BW: '+bwT();if(eRit)eRit[TC]='RIT: '+(tuneOff>=0?'+':'')+tuneOff+' Hz';subIQ();});

/* Lock button in ribbon */
var lockBtn=$('btn-lock');
if(lockBtn)on(lockBtn,'click',function(){lockBtn.classList.toggle('active');});
var fastBtn=$('btn-fast-tune');
if(fastBtn)on(fastBtn,'click',function(){fastBtn.classList.toggle('active');});
}

/* ---- Keyboard shortcuts ---- */
function initKbd(){on(document,'keydown',function(e){
var tag=e.target.tagName;if(tag==='INPUT'||tag==='SELECT'||tag==='TEXTAREA')return;
var step=modeBW[mode]||1e3;
if(window._sdrFastTune&&window._sdrFastTune())step*=10; /* Fast tune = 10x step */
function tune(d){cFreq=clamp(cFreq+d,24e6,1766e6);tunedFreq=cFreq;tuneOff=0;tx('freq',{value:cFreq});uD();}
switch(e.key){
case'ArrowUp':e.preventDefault();tune(step);break;case'ArrowDown':e.preventDefault();tune(-step);break;
case'ArrowRight':e.preventDefault();tune(step*10);break;case'ArrowLeft':e.preventDefault();tune(-step*10);break;
case'PageUp':e.preventDefault();tune(step*100);break;case'PageDown':e.preventDefault();tune(-step*100);break;
case'+':case'=':if(zLvl<16){zLvl*=2;if(eZV)eZV[TC]=zLvl+'x';rstBuf();}break;
case'-':if(zLvl>1){zLvl/=2;if(eZV)eZV[TC]=zLvl+'x';rstBuf();}break;
case'm':case'M':var ci=modeList.indexOf(mode);setM(modeList[(ci+1)%modeList.length]);subIQ();uD();break;
case' ':e.preventDefault();audOn?stopAud():startAud();break;
case'f':case'F':if(!document.fullscreenElement)document.documentElement.requestFullscreen().catch(function(){});else document.exitFullscreen();break;
case'p':case'P':pkBuf=null;break;}
if(e.key>='1'&&e.key<='9'){var btns=document.querySelectorAll('.band-panel:not([hidden]) .band-btn'),idx=PI(e.key)-1;if(idx<btns.length)btns[idx].click();}});}

/* ---- Clock ---- */
function tickClock(){
var ct=$('clock-time');
if(ct){var now=new Date();ct[TC]=String(now.getHours()).padStart(2,'0')+':'+String(now.getMinutes()).padStart(2,'0')+':'+String(now.getSeconds()).padStart(2,'0');}
}

/* ---- Resize ---- */
function rsz(){
var sw=$('spectrum-wrap'),ww=$('waterfall-wrap');
if(sw&&spCv){spCv.width=sw.clientWidth;spCv.height=sw.clientHeight;}
if(ww&&wfCv){wfCv.width=ww.clientWidth;wfCv.height=ww.clientHeight;}
rstBuf();}
on(window,'resize',rsz);

/* ---- Main loop ---- */
var lastT=0,clockT=0;
function loop(ts){var dt=(ts-lastT)/1e3;if(dt>.05)dt=.05;lastT=ts;tickSm(dt);pumpIQ();
if(ts-clockT>1000){tickClock();clockT=ts;}
requestAnimationFrame(loop);}

/* ---- Init ---- */
buildCm();rsz();
if(!initGL())wfCtx=wfCv.getContext('2d');
initRibbon();initDSP();initFD();initVFO();initMode();initBands();initHoverControls();initBottomTabs();initCtrl();initSet();initKbd();tickClock();popMem();uD();wsc();
requestAnimationFrame(loop);
})();
