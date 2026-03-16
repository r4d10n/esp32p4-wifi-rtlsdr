/* SPDX-License-Identifier: GPL-2.0-or-later */
(function(){'use strict';
var $=document.getElementById.bind(document),DC=document.createElement.bind(document);
function on(el,ev,fn,opt){el.addEventListener(ev,fn,opt)}
function qsa(s){return document.querySelectorAll(s)}
var TC='textContent',MR=Math.round,MN=Math.min,MX=Math.max,MA=Math.abs,
MS=Math.sqrt,PI=parseInt,MC=Math.ceil,MP=Math.PI;
function clamp(v,a,b){return v<a?a:v>b?b:v;}
function gbc(el){return el.getBoundingClientRect()}
var ws,cFreq=100e6,sRate=1024000,fftSz=1024,dbMin=-40,dbMax=40;
var gain=0,mode='NFM',vol=.5,sql=0,audOn=false;
var zLvl=1,zCtr=.5,wfSpd=50,wfLast=0;
var tunedFreq=cFreq,filterBW=12500;
var isDrag=false,dragX=0,dragF=0,dragMoved=false;
var isResize=false,resEdge=null,tuneOff=0;
var aCtx=null,wkReady=false,wkNode=null,gNode=null,aQ=[],iqRate=0,pI=0,pQ=0;
var modeBW={AM:10000,NFM:12500,WBFM:150000,USB:3000,LSB:3000,CW:500};
var modeList=['AM','NFM','WBFM','USB','LSB','CW'];
var vfoA={f:100e6,m:'NFM',b:12500},vfoB={f:145e6,m:'NFM',b:12500},actVfo='A';
var spCv=$('spectrum'),wfCv=$('waterfall');
var spCtx=spCv.getContext('2d'),wfCtx=null,wfGL=null;
var pkBuf=null,avgBuf=null,avgA=.15,pkD=.5;
var curX=-1,curY=-1,fftF32=null;
var glTex=null,glCmap=null,glProg=null,glRow=0,glTH=512;
var glVao=null,glUO=null,glUH=null;
var cmN='jet',cmD=new Uint8Array(1024),cmaps={},wfID=null;
var smAng=-70,smTgt=-70,smVel=0,smPkA=-70,smPkT=0;
var smNdl=$('smeter-needle'),smPkL=$('smeter-peak'),smVal=$('smeter-val');
var settings={theme:'phosphor',freqFont:'dseg7',smStyle:'analog',cmap:'jet',
fftAvg:50,pkDecay:5,agcSpd:'medium',deemph:'75us',tabHf:1,tabVhf:1,tabBc:1};
var mems=[],bStack={},rhDrag=false,rhY0=0,rhH0=0;
var eDot=$('status-led'),eTxt=$('status-txt'),eRD=$('rate-display');
var eGS=$('gain-slider'),eGV=$('gain-val'),eVS=$('vol-slider'),eVV=$('vol-val');
var eSS=$('sql-slider'),eSV=$('sql-val'),eAB=$('audio-btn');
var eRS=$('rate-select'),eFS=$('fft-select');
var eNS=$('range-slider'),eNV=$('range-val'),eLS=$('ref-slider'),eLV=$('ref-val');
var eZV=$('zoom-val'),eWS=$('wf-speed'),eWV=$('wf-speed-val');
var eTK=$('tune-slider'),eTL=$('tune-lbl'),eBW=$('bw-val'),eCR=$('cursor-readout');
var pndF=null,fTmr=null;
function mkCm(fn){var d=new Uint8Array(1024);for(var i=0;i<256;i++){var t=i/255,c=fn(t);d[i*4]=c[0];d[i*4+1]=c[1];d[i*4+2]=c[2];d[i*4+3]=255;}return d;}
function buildCm(){
cmaps.jet=mkCm(function(t){if(t<.25)return[0,t*4*255|0,255];if(t<.5)return[0,255,(2-t*4)*255|0];if(t<.75)return[(t*4-2)*255|0,255,0];return[255,(4-t*4)*255|0,0];});
cmaps.iron=mkCm(function(t){if(t<.33)return[t*3*128|0,0,t*3*40|0];if(t<.66)return[128+(t*3-1)*127|0,(t*3-1)*128|0,40-(t*3-1)*40|0];return[255,128+(t*3-2)*127|0,(t*3-2)*255|0];});
cmaps.viridis=mkCm(function(t){return[68+t*187*(t<.5?t*2:1)|0,MN(255,2+t*252)|0,MX(0,80+t*100-t*t*180)|0];});
cmaps.phosphor=mkCm(function(t){return[0,t*255|0,t*40|0]});
cmaps.grayscale=mkCm(function(t){var v=t*255|0;return[v,v,v]});
setCm(settings.cmap);
}
function setCm(n){cmN=n;cmD=cmaps[n]||cmaps.jet;if(wfGL&&glCmap){wfGL.bindTexture(wfGL.TEXTURE_2D,glCmap);wfGL.texImage2D(wfGL.TEXTURE_2D,0,wfGL.RGBA,256,1,0,wfGL.RGBA,wfGL.UNSIGNED_BYTE,cmD);}wfID=null;}
function fF(h){if(h>=1e9)return(h/1e9).toFixed(6)+' GHz';if(h>=1e6)return(h/1e6).toFixed(3)+' MHz';if(h>=1e3)return(h/1e3).toFixed(1)+' kHz';return h+' Hz';}
function vB(n){var f=1/zLvl,h=f/2,s=zCtr-h,e=zCtr+h;if(s<0){e-=s;s=0;}if(e>1){s-=e-1;e=1;}if(s<0)s=0;return{s:s*n|0,e:MN(MC(e*n),n)};}
function gVF(){var n=fftSz,v=vB(n),sf=cFreq-sRate/2;return{s:sf+v.s/n*sRate,e:sf+v.e/n*sRate,bw:(v.e-v.s)/n*sRate};}
function f2x(f,W,vf){return(f-vf.s)/vf.bw*W;}
function hpp(W){var v=vB(fftSz);return sRate*(v.e-v.s)/fftSz/W;}
function bwT(){return(filterBW>=1e3?(filterBW/1e3).toFixed(1)+'k':filterBW)+'Hz';}
function mP(){return{b:filterBW,o:tunedFreq-cFreq};}
function tx(c,p){if(!ws||ws.readyState!==1)return;var m={cmd:c};if(p)for(var k in p)m[k]=p[k];ws.send(JSON.stringify(m));}
function subIQ(){if(audOn){var pm=mP();tx('subscribe_iq',{offset:pm.o,bw:pm.b});}}
function tDb(){tx('db_range',{min:PI(eLV.value)-PI(eNS.value),max:PI(eLV.value)});}
function wsc(){
var h=location.hostname||'192.168.1.232',p=location.port||'8080';
ws=new WebSocket('ws://'+h+':'+p+'/ws');ws.binaryType='arraybuffer';
ws.onopen=function(){eDot.className='connected';eTxt[TC]='Connected';};
ws.onclose=function(){eDot.className='';eTxt[TC]='Disconnected';setTimeout(wsc,2000);};
ws.onerror=function(){ws.close();};
ws.onmessage=function(v){typeof v.data==='string'?onTxt(v.data):onBin(v.data);};}
function onTxt(t){var m;try{m=JSON.parse(t);}catch(e){return;}
if(m.type==='info'){cFreq=m.freq||cFreq;sRate=m.rate||sRate;gain=m.gain||0;fftSz=m.fft_size||fftSz;if(m.db_min!=null)dbMin=m.db_min;if(m.db_max!=null)dbMax=m.db_max;tunedFreq=cFreq;tuneOff=0;syncC();uD();}
else if(m.type==='freq'){cFreq=m.value;tunedFreq=cFreq;tuneOff=0;uD();}
else if(m.type==='config'){fftSz=m.fft_size||fftSz;sRate=m.sample_rate||sRate;if(m.db_min!=null)dbMin=m.db_min;if(m.db_max!=null)dbMax=m.db_max;rstBuf();syncC();uD();}
else if(m.type==='iq_start'){iqRate=m.rate||0;}}
function onBin(buf){var d=new Uint8Array(buf);if(d.length<2)return;var t=d[0],p=d.subarray(1);
if(t===1){procFFT(p);var now=performance.now();if(now-wfLast>=wfSpd){dWf(p);wfLast=now;}uSm(p);}
else if(t===2&&audOn)aQ.push(p.slice());}
function rstBuf(){pkBuf=avgBuf=fftF32=wfID=null;glRow=0;}
function syncC(){eRS.value=''+sRate;eFS.value=''+fftSz;var r=dbMax-dbMin;eNS.value=r;eNV[TC]=r|0;eLV.value=dbMax;eLV[TC]=dbMax|0;}
function buildFD(){
var fd=$('freq-display'),hz=MR(tunedFreq),digits=fd.querySelectorAll('.freq-digit');
var s=String(hz).padStart(10,'0');
for(var i=0;i<digits.length&&i<s.length;i++)digits[i][TC]=s[i];
var u=fd.querySelector('.freq-unit');if(u)u[TC]=hz>=1e9?'GHz':'MHz';}
function initFD(){
$('freq-display').querySelectorAll('.freq-digit').forEach(function(d){
on(d,'wheel',function(e){e.preventDefault();var step=PI(d.dataset.step)||1;
cFreq=clamp(MR(cFreq+(e.deltaY<0?1:-1)*step),24e6,1766e6);tunedFreq=cFreq;tuneOff=0;tx('freq',{value:cFreq});uD();},{passive:false});
on(d,'mouseenter',function(){d.classList.add('active')});
on(d,'mouseleave',function(){d.classList.remove('active')});});}
function uD(){buildFD();eRD[TC]=(sRate/1e3|0)+' kSPS';eBW[TC]=bwT();eTK.value=0;eTL[TC]='\xb10';}
function dBmFFT(f){var c=f.length/2|0,s=0,n=0;for(var i=c-20;i<=c+20;i++)if(i>=0&&i<f.length){s+=f[i];n++;}return dbMin+(s/(n||1)/255)*(dbMax-dbMin);}
function dBm2a(d){if(d<=-121)return-70;if(d<=-73)return-70+(d+121)/48*120;if(d<=-13)return 50+(d+73)/60*20;return 70;}
function dBm2s(d){if(d<=-121)return'S0';if(d<=-73){var u=MR((d+127)/6);return'S'+clamp(u,1,9);}return'S9+'+MX(0,MR(d+73));}
function uSm(f){smTgt=dBm2a(dBmFFT(f));if(smVal)smVal[TC]=dBm2s(dBmFFT(f));if(smTgt>smPkA){smPkA=smTgt;smPkT=performance.now();}}
function tickSm(dt){
var F=-180*(smAng-smTgt)-14*smVel;smVel+=F*dt;smAng=clamp(smAng+smVel*dt,-72,72);
if(smNdl)smNdl.setAttribute('transform','rotate('+smAng.toFixed(1)+', 80, 46)');
var el=(performance.now()-smPkT)/1e3;if(el>1.5)smPkA=MX(smAng,smPkA-30*dt);
if(smPkL){var sa=smPkL.setAttribute.bind(smPkL);if(smPkA>-69){var r=(smPkA-90)*MP/180,co=Math.cos(r),si=Math.sin(r);
sa('x1',(80+co*28).toFixed(1));sa('y1',(46+si*28).toFixed(1));
sa('x2',(80+co*14).toFixed(1));sa('y2',(46+si*14).toFixed(1));
sa('opacity','0.35');}else sa('opacity','0');}}
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
spCtx.fillStyle='#0a0a0a';spCtx.fillRect(0,0,w,h);
spCtx.strokeStyle='#1a1a1a';spCtx.lineWidth=.5;spCtx.font='9px monospace';
spCtx.textAlign='left';spCtx.fillStyle='#445544';
var st=10;if(dr>100)st=20;
for(var d=MC(dbMin/st)*st;d<=dbMax;d+=st){var y=d2y(d);spCtx.beginPath();spCtx.moveTo(0,y);spCtx.lineTo(w,y);spCtx.stroke();spCtx.fillText(d+' dB',2,y-2);}
var vf=gVF(),fs=vf.bw,fst=1e3;
if(fs>1e8)fst=1e7;else if(fs>1e7)fst=1e6;else if(fs>1e6)fst=1e5;else if(fs>5e5)fst=5e4;else if(fs>1e5)fst=1e4;
spCtx.textAlign='center';spCtx.fillStyle='#446644';
for(var f=MC(vf.s/fst)*fst;f<=vf.e;f+=fst){var x=f2x(f,w,vf);spCtx.beginPath();spCtx.moveTo(x,0);spCtx.lineTo(x,h);spCtx.stroke();spCtx.fillText(fF(f),x,h-2);}
spCtx.strokeStyle='rgba(100,180,100,0.3)';spCtx.lineWidth=1;spCtx.beginPath();
for(var i=0;i<vl;i++){var x=i*xs,y=d2y(avgBuf[v.s+i]);i?spCtx.lineTo(x,y):spCtx.moveTo(x,y);}spCtx.stroke();
spCtx.strokeStyle='rgba(255,60,60,0.4)';spCtx.beginPath();
for(var i=0;i<vl;i++){var x=i*xs,y=d2y(pkBuf[v.s+i]);i?spCtx.lineTo(x,y):spCtx.moveTo(x,y);}spCtx.stroke();
spCtx.strokeStyle='#FFB000';spCtx.lineWidth=1.5;spCtx.beginPath();
for(var i=0;i<vl;i++){var x=i*xs,y=d2y(fftF32[v.s+i]);i?spCtx.lineTo(x,y):spCtx.moveTo(x,y);}spCtx.stroke();
var gr=spCtx.createLinearGradient(0,0,0,h);gr.addColorStop(0,'rgba(255,176,0,0.25)');gr.addColorStop(1,'rgba(255,176,0,0.02)');
spCtx.fillStyle=gr;spCtx.lineTo((vl-1)*xs,h);spCtx.lineTo(0,h);spCtx.closePath();spCtx.fill();
dFO(spCv,spCtx,w,h);
if(sql>0){var sy=d2y(dbMin+sql/255*dr);spCtx.strokeStyle='rgba(255,100,100,0.5)';spCtx.lineWidth=1;spCtx.setLineDash([4,4]);spCtx.beginPath();spCtx.moveTo(0,sy);spCtx.lineTo(w,sy);spCtx.stroke();spCtx.setLineDash([]);}
if(curX>=0&&curX<=w&&curY>=0&&curY<=h&&eCR)eCR[TC]=fF(MR(vf.s+curX/w*vf.bw))+' / '+(dbMax-curY/h*dr|0)+' dB';}
function dFO(cv,ctx,w,h){
var vf=gVF(),fl=f2x(tunedFreq-filterBW/2,w,vf),fr=f2x(tunedFreq+filterBW/2,w,vf),cx=f2x(tunedFreq,w,vf);
if(fr>fl){ctx.fillStyle='rgba(255,176,0,0.08)';ctx.fillRect(MX(0,fl),0,MN(w,fr)-MX(0,fl),h);}
ctx.lineWidth=2;ctx.strokeStyle='rgba(255,200,0,0.6)';
if(fl>=0&&fl<=w){ctx.beginPath();ctx.moveTo(fl,0);ctx.lineTo(fl,h);ctx.stroke();}
if(fr>=0&&fr<=w){ctx.beginPath();ctx.moveTo(fr,0);ctx.lineTo(fr,h);ctx.stroke();}
ctx.strokeStyle='rgba(255,60,60,0.7)';ctx.lineWidth=1;
if(cx>=0&&cx<=w){ctx.beginPath();ctx.moveTo(cx,0);ctx.lineTo(cx,h);ctx.stroke();}
ctx.font='9px monospace';ctx.fillStyle='rgba(255,200,0,0.85)';ctx.textAlign='left';
ctx.fillText(bwT(),MN(w-52,MX(2,cx+3)),10);}
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
'else{for(var k=0;k<s.length;k++)rB.push(s[k]);}}';
var WK_CODE=DSP_CODE+
'var iqB=[],st={},C={mode:"NFM",ir:48e3,ag:"medium",vol:.5,sql:0,de:75e-6};'+
'var TM={"off":0,"50us":50e-6,"75us":75e-6};'+
'class P extends AudioWorkletProcessor{constructor(){super();this.rB=[];'+
'this.port.onmessage=function(e){var d=e.data;if(d.type==="iq")iqB.push(d.data);'+
'else if(d.type==="config"){for(var k in d)if(k!=="type")C[k]=k==="deemph"?TM[d[k]]||75e-6:d[k];}};}'+
'process(i,o){var out=o[0][0];if(!out)return true;var rB=this.rB;'+
'while(iqB.length>0&&rB.length<out.length*2){var raw=iqB.shift(),s=demod(raw,C.mode,st);'+
'if(C.mode==="NFM"||C.mode==="WBFM")s=deemph(s,st,C.de,48e3);'+
's=agc(s,st,C.ag);resamp(s,C.ir||48e3,rB);}'+
'var pw=0,ch=Math.min(rB.length,out.length);for(var i=0;i<ch;i++)pw+=rB[i]*rB[i];pw/=ch||1;'+
'var mt=C.sql>0&&pw<C.sql/255*.01;'+
'for(var p=0;p<ch;p++)out[p]=mt?0:rB[p]*C.vol;'+
'this.rB=ch<rB.length?rB.slice(ch):[];for(;p<out.length;p++)out[p]=0;return true;}}'+
'registerProcessor("sdr-processor",P);';
function sendCfg(){if(!wkNode)return;wkNode.port.postMessage({type:'config',mode:mode,ir:iqRate,ag:settings.agcSpd,vol:vol,sql:sql,deemph:settings.deemph});}
function pumpIQ(){if(!wkReady||!wkNode)return;while(aQ.length)wkNode.port.postMessage({type:'iq',data:aQ.shift()});}
function startAud(){
if(aCtx)return;aCtx=new(window.AudioContext||window.webkitAudioContext)({sampleRate:48000});aCtx.resume();
gNode=aCtx.createGain();gNode.gain.value=1;gNode.connect(aCtx.destination);
if(typeof AudioWorkletNode!=='undefined'){
var blob=new Blob([WK_CODE],{type:'application/javascript'}),url=URL.createObjectURL(blob);
aCtx.audioWorklet.addModule(url).then(function(){URL.revokeObjectURL(url);
wkNode=new AudioWorkletNode(aCtx,'sdr-processor');wkNode.connect(gNode);wkReady=true;
audOn=true;subIQ();sendCfg();eAB[TC]='Stop';eAB.classList.add('active');
}).catch(function(){startSP();});}else startSP();}
function startSP(){
var g=aCtx.createGain();g.gain.value=1;g.connect(aCtx.destination);
var sp=aCtx.createScriptProcessor(4096,1,1),rBuf=[],st={};
var env={};eval('(function(){'+DSP_CODE+'env.S=S;env.demod=demod;env.deemph=deemph;env.agc=agc;env.resamp=resamp;})()');
sp.onaudioprocess=function(e){
var o=e.outputBuffer.getChannelData(0),p=0;
while(aQ.length>0&&rBuf.length<o.length*2){var raw=aQ.shift(),s=env.demod(raw,mode,st);
if(mode==='NFM'||mode==='WBFM'){var tau=settings.deemph==='50us'?50e-6:settings.deemph==='off'?0:75e-6;s=env.deemph(s,st,tau,48e3);}
s=env.agc(s,st,settings.agcSpd);env.resamp(s,iqRate||48e3,rBuf);}
var pw=0,ch=MN(rBuf.length,o.length);for(var i=0;i<ch;i++)pw+=rBuf[i]*rBuf[i];pw/=ch||1;
var mute=sql>0&&pw<sql/255*.01;for(;p<ch;p++)o[p]=mute?0:rBuf[p]*vol;
rBuf=ch<rBuf.length?rBuf.slice(ch):[];while(p<o.length)o[p++]=0;};
sp.connect(g);sp._g=g;audOn=true;subIQ();eAB[TC]='Stop';eAB.classList.add('active');}
function stopAud(){
audOn=false;if(ws&&ws.readyState===1)tx('unsubscribe_iq',{});
if(wkNode){wkNode.disconnect();wkNode=null;wkReady=false;}
if(gNode){gNode.disconnect();gNode=null;}
if(aCtx){aCtx.close();aCtx=null;}aQ=[];iqRate=0;
eAB[TC]='Audio';eAB.classList.remove('active');}
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
if(isResize){var vf=gVF(),fHz=vf.s+x/r.width*vf.bw;filterBW=clamp(MR(MA(fHz-tunedFreq)*2),200,sRate*.9);eBW[TC]=bwT();}}
function onUp(e){var t=e.currentTarget,r=gbc(t),x=(e.changedTouches?e.changedTouches[0].clientX:gcx(e))-r.left;
if(isDrag&&!dragMoved){var v=vB(fftSz);tunedFreq=MR(cFreq-sRate/2+(v.s+x/r.width*(v.e-v.s))/fftSz*sRate);tuneOff=0;subIQ();buildFD();}
if(isDrag){commitF();isDrag=false;dragMoved=false;}if(isResize){isResize=false;resEdge=null;subIQ();sendCfg();}t.style.cursor='crosshair';}
function onLv(e){if(isDrag){commitF();isDrag=false;dragMoved=false;}if(isResize){isResize=false;resEdge=null;}curX=-1;curY=-1;if(eCR)eCR[TC]='';e.currentTarget.style.cursor='crosshair';}
function onWhl(e){e.preventDefault();
if(e.ctrlKey){filterBW=clamp(MR(filterBW*(e.deltaY<0?1.1:.9)),200,sRate*.9);eBW[TC]=bwT();subIQ();sendCfg();return;}
var r=gbc(e.currentTarget),v=vB(fftSz),vl=v.e-v.s;zCtr=(v.s+(e.clientX-r.left)/r.width*vl)/fftSz;
if(e.deltaY<0&&zLvl<16)zLvl*=2;else if(e.deltaY>0&&zLvl>1)zLvl/=2;eZV[TC]=zLvl+'x';rstBuf();}
var pd0=0,pz0=1;
function onTS2(e){if(e.touches.length===2){var dx=e.touches[0].clientX-e.touches[1].clientX,dy=e.touches[0].clientY-e.touches[1].clientY;pd0=MS(dx*dx+dy*dy);pz0=zLvl;e.preventDefault();}}
function onTM2(e){if(e.touches.length===2){var dx=e.touches[0].clientX-e.touches[1].clientX,dy=e.touches[0].clientY-e.touches[1].clientY;zLvl=clamp(MR(pz0*MS(dx*dx+dy*dy)/pd0),1,16);eZV[TC]=zLvl+'x';rstBuf();e.preventDefault();}}
[spCv,wfCv].forEach(function(cv){
on(cv,'mousedown',onDn);on(cv,'mousemove',onMv);on(cv,'mouseup',onUp);on(cv,'mouseleave',onLv);
on(cv,'wheel',onWhl,{passive:false});
on(cv,'touchstart',function(e){e.touches.length>=2?onTS2(e):onDn(e);},{passive:false});
on(cv,'touchmove',function(e){e.touches.length>=2?onTM2(e):onMv(e);},{passive:false});
on(cv,'touchend',onUp,{passive:false});});
var rh=$('resize-handle');
if(rh){on(rh,'mousedown',function(e){rhDrag=true;rhY0=e.clientY;rhH0=gbc($('spectrum-wrap')).height;e.preventDefault();});
on(document,'mousemove',function(e){if(!rhDrag)return;$('spectrum-wrap').style.height=clamp(rhH0+e.clientY-rhY0,60,window.innerHeight*.7)+'px';rsz();});
on(document,'mouseup',function(){rhDrag=false;});
on(rh,'dblclick',function(){$('spectrum-wrap').style.height='22%';rsz();});}
function initBands(){
try{var s=localStorage.getItem('sdr_bs');if(s)bStack=JSON.parse(s);}catch(e){}
qsa('.band-tab').forEach(function(tab){on(tab,'click',function(){qsa('.band-tab').forEach(function(t){t.classList.remove('active');t.setAttribute('aria-selected','false');});
tab.classList.add('active');tab.setAttribute('aria-selected','true');qsa('.band-panel').forEach(function(p){p.hidden=true;});var pn=$('band-panel-'+tab.dataset.tab);if(pn)pn.hidden=false;});});
qsa('.band-btn').forEach(function(b){on(b,'click',function(){var nm=b.title||b[TC],st=bStack[nm];
if(st){cFreq=st.f;tunedFreq=st.f;setM(st.m);filterBW=st.b;}else{cFreq=PI(b.dataset.freq);tunedFreq=cFreq;setM(b.dataset.mode);filterBW=modeBW[mode]||12500;}
tuneOff=0;tx('freq',{value:cFreq});subIQ();uD();});});}
function setM(m){if(!m)return;mode=m;filterBW=modeBW[m]||12500;qsa('.mode-btn').forEach(function(x){x.classList.toggle('active',x.dataset.mode===m);});eBW[TC]=bwT();sendCfg();}
function initVFO(){qsa('.vfo-btn').forEach(function(b){on(b,'click',function(){
var cv=actVfo==='A'?vfoA:vfoB;cv.f=tunedFreq;cv.m=mode;cv.b=filterBW;
actVfo=b.dataset.vfo;qsa('.vfo-btn').forEach(function(x){x.classList.toggle('active',x.dataset.vfo===actVfo);});
var nv=actVfo==='A'?vfoA:vfoB;cFreq=nv.f;tunedFreq=nv.f;tuneOff=0;setM(nv.m);filterBW=nv.b;tx('freq',{value:cFreq});subIQ();uD();});});}
function initMode(){qsa('.mode-btn').forEach(function(b){on(b,'click',function(){setM(b.dataset.mode);subIQ();uD();});});}
function initSet(){
try{var s=localStorage.getItem('sdr_set');if(s){var p=JSON.parse(s);for(var k in p)settings[k]=p[k];}}catch(e){}
try{var m=localStorage.getItem('sdr_mem');if(m)mems=JSON.parse(m);}catch(e){}
applySet();
var dlg=$('settings-dialog');
on($('settings-btn'),'click',function(){if(dlg.open)dlg.close();else{popSet();dlg.showModal();}});
on($('settings-close'),'click',function(){dlg.close();});
on(dlg,'click',function(e){if(e.target===dlg)dlg.close();});
function sChg(id,k){on($(id),'change',function(){settings[k]=this.value;applySet();saveSet();});}
sChg('set-theme','theme');sChg('set-freq-font','freqFont');sChg('set-smeter-style','smStyle');
on($('set-colormap'),'change',function(){settings.cmap=this.value;setCm(this.value);saveSet();});
on($('set-fft-avg'),'input',function(){settings.fftAvg=PI(this.value);avgA=PI(this.value)/100;$('set-fft-avg-val')[TC]=avgA.toFixed(2);saveSet();});
on($('set-peak-decay'),'input',function(){settings.pkDecay=PI(this.value);pkD=PI(this.value)/10;$('set-peak-decay-val')[TC]=pkD.toFixed(1);saveSet();});
on($('set-agc-speed'),'change',function(){settings.agcSpd=this.value;sendCfg();saveSet();});
on($('set-deemph'),'change',function(){settings.deemph=this.value;sendCfg();saveSet();});
['hf','vhf','bc'].forEach(function(t){var cb=$('set-tab-'+t);if(cb)on(cb,'change',function(){settings['tab'+t.charAt(0).toUpperCase()+t.slice(1)]=this.checked?1:0;applySet();saveSet();});});
on($('mem-add-btn'),'click',function(){var n=prompt('Channel name:');if(!n)return;mems.push({n:n,f:tunedFreq,m:mode,b:filterBW});saveSet();popMem();});}
function popSet(){
var sv=function(id,v){var e=$(id);if(e)e.value=v;};
sv('set-theme',settings.theme);sv('set-freq-font',settings.freqFont);
sv('set-smeter-style',settings.smStyle);sv('set-colormap',settings.cmap);
sv('set-fft-avg',settings.fftAvg);$('set-fft-avg-val')[TC]=(settings.fftAvg/100).toFixed(2);
sv('set-peak-decay',settings.pkDecay);$('set-peak-decay-val')[TC]=(settings.pkDecay/10).toFixed(1);
sv('set-agc-speed',settings.agcSpd);sv('set-deemph',settings.deemph);
['hf','vhf','bc'].forEach(function(t){var c=$('set-tab-'+t);if(c)c.checked=!!settings['tab'+t.charAt(0).toUpperCase()+t.slice(1)];});
popMem();}
function popMem(){var l=$('mem-channel-list');if(!l)return;l.innerHTML='';
mems.forEach(function(m,i){var r=DC('div');r.className='mem-row';
var b=DC('button');b.className='ctrl-btn';b[TC]=m.n+' '+fF(m.f)+' '+m.m;
on(b,'click',function(){cFreq=m.f;tunedFreq=m.f;tuneOff=0;setM(m.m);filterBW=m.b;tx('freq',{value:cFreq});subIQ();uD();});
var d=DC('button');d.className='ctrl-btn';d[TC]='\u2715';on(d,'click',function(){mems.splice(i,1);saveSet();popMem();});
r.appendChild(b);r.appendChild(d);l.appendChild(r);});}
function saveSet(){try{localStorage.setItem('sdr_set',JSON.stringify(settings));}catch(e){}try{localStorage.setItem('sdr_mem',JSON.stringify(mems));}catch(e){}}
function applySet(){
document.body.setAttribute('data-theme',settings.theme);
var fd=$('freq-display');if(fd)fd.style.fontFamily=settings.freqFont==='dseg7'?"'DSEG7',monospace":'monospace';
var sw=$('smeter-wrap');if(sw)sw.setAttribute('data-style',settings.smStyle);
if(settings.cmap!==cmN)setCm(settings.cmap);
avgA=settings.fftAvg/100;pkD=settings.pkDecay/10;
var m={'hf':'tabHf','vhf':'tabVhf','bc':'tabBc'};
for(var k in m){var t=document.querySelector('[data-tab="'+k+'"]');if(t)t.style.display=settings[m[k]]?'':'none';}}
function initKbd(){on(document,'keydown',function(e){
var tag=e.target.tagName;if(tag==='INPUT'||tag==='SELECT'||tag==='TEXTAREA')return;
var dlg=$('settings-dialog');if(dlg&&dlg.open&&e.key==='Escape'){dlg.close();return;}
var step=modeBW[mode]||1e3;
function tune(d){cFreq=clamp(cFreq+d,24e6,1766e6);tunedFreq=cFreq;tuneOff=0;tx('freq',{value:cFreq});uD();}
switch(e.key){
case'ArrowUp':e.preventDefault();tune(step);break;case'ArrowDown':e.preventDefault();tune(-step);break;
case'ArrowRight':e.preventDefault();tune(step*10);break;case'ArrowLeft':e.preventDefault();tune(-step*10);break;
case'PageUp':e.preventDefault();tune(step*100);break;case'PageDown':e.preventDefault();tune(-step*100);break;
case'+':case'=':if(zLvl<16){zLvl*=2;eZV[TC]=zLvl+'x';rstBuf();}break;
case'-':if(zLvl>1){zLvl/=2;eZV[TC]=zLvl+'x';rstBuf();}break;
case'm':case'M':var ci=modeList.indexOf(mode);setM(modeList[(ci+1)%modeList.length]);subIQ();uD();break;
case' ':e.preventDefault();audOn?stopAud():startAud();break;
case'f':case'F':if(!dlg||!dlg.open){if(!document.fullscreenElement)document.documentElement.requestFullscreen().catch(function(){});else document.exitFullscreen();}break;
case's':case'S':if(dlg){if(dlg.open)dlg.close();else{popSet();dlg.showModal();}}break;
case'p':case'P':pkBuf=null;break;}
if(e.key>='1'&&e.key<='9'){var btns=document.querySelectorAll('.band-panel:not([hidden]) .band-btn'),idx=PI(e.key)-1;if(idx<btns.length)btns[idx].click();}});}
function rsz(){var sw=$('spectrum-wrap'),ww=$('waterfall-wrap');
if(sw&&spCv){spCv.width=sw.clientWidth;spCv.height=sw.clientHeight;}
if(ww&&wfCv){wfCv.width=ww.clientWidth;wfCv.height=ww.clientHeight;}rstBuf();}
on(window,'resize',rsz);
function initCtrl(){
on(eAB,'click',function(){audOn?stopAud():startAud();});
on(eRS,'change',function(){tx('sample_rate',{value:PI(this.value)});});
on(eFS,'change',function(){tx('fft_size',{value:PI(this.value)});rstBuf();});
function dbI(){this===eNS?eNV[TC]=this.value:eLV[TC]=this.value;tDb();}
on(eNS,'input',dbI);on(eLS,'input',dbI);
on(eGS,'input',function(){gain=PI(this.value);eGV[TC]=gain?((gain/10).toFixed(1)+' dB'):'Auto';tx('gain',{value:gain});});
on(eVS,'input',function(){vol=PI(this.value)/100;eVV[TC]=(vol*100|0)+'%';if(gNode)gNode.gain.value=vol;sendCfg();});
on(eSS,'input',function(){sql=PI(this.value);eSV[TC]=sql;sendCfg();});
on(eWS,'input',function(){wfSpd=110-PI(this.value);eWV[TC]=this.value;});
on($('zoom-in'),'click',function(){if(zLvl<16){zLvl*=2;eZV[TC]=zLvl+'x';rstBuf();}});
on($('zoom-out'),'click',function(){if(zLvl>1){zLvl/=2;eZV[TC]=zLvl+'x';rstBuf();}});
on($('zoom-reset'),'click',function(){zLvl=1;zCtr=.5;eZV[TC]='1x';rstBuf();});
on($('peak-clear'),'click',function(){pkBuf=null;});
on(eTK,'input',function(){tuneOff=PI(this.value);eTL[TC]=(tuneOff>=0?'+':'')+tuneOff;tunedFreq=MR(cFreq+tuneOff);buildFD();eBW[TC]=bwT();subIQ();});}
var lastT=0;
function loop(ts){var dt=(ts-lastT)/1e3;if(dt>.05)dt=.05;lastT=ts;tickSm(dt);pumpIQ();requestAnimationFrame(loop);}
buildCm();rsz();
if(!initGL())wfCtx=wfCv.getContext('2d');
initFD();initVFO();initMode();initBands();initCtrl();initSet();initKbd();uD();wsc();
requestAnimationFrame(loop);
})();
