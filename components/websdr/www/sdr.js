/* ESP32-P4 WebSDR - SPDX-License-Identifier: GPL-2.0-or-later */
(function(){'use strict';
var ws,cFreq=100e6,sRate=1024000,fftSz=1024,dbMin=-40,dbMax=40;
var gain=0,mode='WBFM',vol=0.5,sql=0,audOn=false;
var zLvl=1,zCtr=0.5,wfSpd=50,wfLast=0;
var tunedFreq=cFreq,filterBW=150000;
var isDrag=false,dragX=0,dragF=0,dragMoved=false;
var isResize=false,resEdge=null,tuneOff=0;
var $=document.getElementById.bind(document);
var DC=document.createElement.bind(document);
function on(el,ev,fn,opt){el.addEventListener(ev,fn,opt);}
function qsa(s){return document.querySelectorAll(s);}
function subIQ(){if(audOn){var pm=mP();tx('subscribe_iq',{offset:pm.o,bw:pm.b});}}
var TC='textContent',MR=Math.round,MN=Math.min,MX=Math.max,MA=Math.abs,PI=parseInt,MC=Math.ceil;
function gbc(el){return el.getBoundingClientRect();}
function cur(el,v){el.style.cursor=v;}
var eDot=$('status-dot'),eTxt=$('status-txt'),eRD=$('rate-display');
var eSm=$('smeter'),eSmF=$('smeter-fill');
var spCv=$('spectrum'),wfCv=$('waterfall');
var eGS=$('gain-slider'),eGV=$('gain-val'),eVS=$('vol-slider'),eVV=$('vol-val');
var eSS=$('sql-slider'),eSV=$('sql-val'),eAB=$('audio-btn');
var eRS=$('rate-select'),eFS=$('fft-select');
var eNS=$('range-slider'),eNV=$('range-val'),eLS=$('ref-slider'),eLV=$('ref-val');
var eZV=$('zoom-val'),eWS=$('wf-speed'),eWV=$('wf-speed-val');
var eTK=$('tune-slider'),eTL=$('tune-lbl'),eBW=$('bw-val');
var sC=spCv.getContext('2d'),wC=wfCv.getContext('2d'),wI=null,pkBuf=null;
var cm=new Array(256),aCtx=null,sN=null,aQ=[],pI=0,pQ=0;
var modeBW={NFM:12500,AM:10000,WBFM:150000,USB:3000,LSB:3000};
(function(){for(var i=0;i<256;i++){var t=i/255,r,g,b;
if(t<.25){r=0;g=t/.25*255|0;b=255;}
else if(t<.5){r=0;g=255;b=(.5-t)/.25*255|0;}
else if(t<.75){r=(t-.5)/.25*255|0;g=255;b=0;}
else{r=255;g=(1-t)/.25*255|0;b=0;}
cm[i]=[r,g,b];}})();
function rsz(){var sw=$('spectrum-wrap'),ww=$('waterfall-wrap');
spCv.width=sw.clientWidth;spCv.height=sw.clientHeight;
wfCv.width=ww.clientWidth;wfCv.height=ww.clientHeight;pkBuf=null;wI=null;}
on(window,'resize',rsz);
function fF(h){if(h>=1e9)return(h/1e9).toFixed(6)+' GHz';
if(h>=1e6)return(h/1e6).toFixed(3)+' MHz';
if(h>=1e3)return(h/1e3).toFixed(1)+' kHz';return h+' Hz';}
function vB(n){var f=1/zLvl,h=f/2,s=zCtr-h,e=zCtr+h;
if(s<0){e-=s;s=0;}if(e>1){s-=e-1;e=1;}if(s<0)s=0;
return{s:s*n|0,e:MN(MC(e*n),n)};}
function tx(c,p){if(!ws||ws.readyState!==1)return;
var m={cmd:c};if(p)for(var k in p)m[k]=p[k];ws.send(JSON.stringify(m));}
function tDb(){tx('db_range',{min:PI(eLV.value)-PI(eNS.value),max:PI(eLV.value)});}
function sC2(){eRS.value=''+sRate;eFS.value=''+fftSz;
var r=dbMax-dbMin;eNS.value=r;eNV[TC]=r|0;eLV.value=dbMax;eLV[TC]=dbMax|0;}
function mP(){return{b:filterBW,o:tunedFreq-cFreq};}
function bwTxt(){return(filterBW>=1000?(filterBW/1000).toFixed(1)+'k':filterBW)+'Hz BW';}
function buildFD(){
var fd=$('freq-display'),hz=MR(cFreq+tuneOff);
fd.innerHTML='';
var ghz=hz/1e9|0,mhz=(hz%1e9)/1e6|0,khz=(hz%1e6)/1e3|0,sub=hz%1e3;
function mk(v,step,pad){var el=DC('span');
el.className='fd-group';el.dataset.step=step;
el[TC]=pad?String(v).padStart(3,'0'):String(v);
on(el,'wheel',function(e){e.preventDefault();
var d=e.deltaY<0?+step:-step;tuneOff=0;
cFreq=MX(24e6,MN(1766e6,cFreq+d));
tunedFreq=cFreq;tx('freq',{value:cFreq});uD();},{passive:false});
return el;}
function sep(c){var s=DC('span');s.className='fd-sep';s[TC]=c;return s;}
if(ghz>0){fd.appendChild(mk(ghz,1e9,false));fd.appendChild(sep('.'));}
fd.appendChild(mk(mhz,1e6,ghz>0));fd.appendChild(sep('.'));
fd.appendChild(mk(khz,1e3,true));fd.appendChild(sep('.'));
fd.appendChild(mk(sub,1,true));
var u=DC('span');u.className='fd-unit';
u[TC]=hz>=1e9?'GHz':hz>=1e6?'MHz':hz>=1e3?'kHz':'Hz';fd.appendChild(u);}
function uD(){buildFD();eRD[TC]=(sRate/1e3|0)+' kSPS';eBW[TC]=bwTxt();eTK.value=0;eTL[TC]='\xb10';}
function wsc(){var h=location.hostname||'192.168.1.232',p=location.port||'8080';
ws=new WebSocket('ws://'+h+':'+p+'/ws');ws.binaryType='arraybuffer';
ws.onopen=function(){eDot.className='connected';eTxt[TC]='Connected';};
ws.onclose=function(){eDot.className='';eTxt[TC]='Disconnected';setTimeout(wsc,2000);};
ws.onerror=function(){ws.close();};
ws.onmessage=function(v){typeof v.data==='string'?oT(v.data):oB(v.data);};}
function oT(t){var m;try{m=JSON.parse(t);}catch(e){return;}
if(m.type==='info'){cFreq=m.freq||cFreq;sRate=m.rate||sRate;gain=m.gain||0;fftSz=m.fft_size||fftSz;
if(m.db_min!=null)dbMin=m.db_min;if(m.db_max!=null)dbMax=m.db_max;
tunedFreq=cFreq;tuneOff=0;sC2();uD();}
else if(m.type==='freq'){cFreq=m.value;tunedFreq=cFreq;tuneOff=0;uD();}
else if(m.type==='config'){fftSz=m.fft_size||fftSz;sRate=m.sample_rate||sRate;
if(m.db_min!=null)dbMin=m.db_min;if(m.db_max!=null)dbMax=m.db_max;
wI=null;pkBuf=null;sC2();uD();}}
function oB(buf){var d=new Uint8Array(buf);if(d.length<2)return;
var t=d[0],p=d.subarray(1);
if(t===1){dSp(p);var n=performance.now();if(n-wfLast>=wfSpd){dWf(p);wfLast=n;}uSm(p);}
else if(t===2&&audOn)aQ.push(p.slice());}
function gVF(){var n=fftSz,v=vB(n),sf=cFreq-sRate/2;
var vs=sf+(v.s/n)*sRate,ve=sf+(v.e/n)*sRate;
return{s:vs,e:ve,bw:ve-vs};}
function f2x(f,W,vf){return(f-vf.s)/vf.bw*W;}
function hpp(W){var v=vB(fftSz);return sRate*(v.e-v.s)/fftSz/W;}
function drawFO(cv,ctx){
var w=cv.width,h=cv.height;if(!w||!h)return;
var vf=gVF(),fl=f2x(tunedFreq-filterBW/2,w,vf),fr=f2x(tunedFreq+filterBW/2,w,vf),cx=f2x(tunedFreq,w,vf);
if(fr>fl){ctx.fillStyle='rgba(255,255,255,0.10)';
ctx.fillRect(MX(0,fl),0,MN(w,fr)-MX(0,fl),h);}
ctx.lineWidth=1.5;ctx.strokeStyle='rgba(255,220,0,0.75)';
if(fl>=0&&fl<=w){ctx.beginPath();ctx.moveTo(fl+.5,0);ctx.lineTo(fl+.5,h);ctx.stroke();}
if(fr>=0&&fr<=w){ctx.beginPath();ctx.moveTo(fr-.5,0);ctx.lineTo(fr-.5,h);ctx.stroke();}
ctx.strokeStyle='#f33';ctx.lineWidth=1;
if(cx>=0&&cx<=w){ctx.beginPath();ctx.moveTo(cx+.5,0);ctx.lineTo(cx+.5,h);ctx.stroke();}
ctx.font='9px monospace';ctx.fillStyle='rgba(255,220,0,0.85)';
ctx.fillText(bwTxt(),MN(w-52,MX(2,cx+3)),10);}
function dSp(f){var w=spCv.width,h=spCv.height,n=f.length,v=vB(n),vl=v.e-v.s,xs=w/vl;
sC.fillStyle='#0d0d0d';sC.fillRect(0,0,w,h);
sC.strokeStyle='#1e1e1e';sC.lineWidth=1;
for(var d=0;d<=100;d+=10){var y=h-d/100*h;sC.beginPath();sC.moveTo(0,y);sC.lineTo(w,y);sC.stroke();}
for(var i=0;i<=5;i++){sC.beginPath();sC.moveTo(i/5*w,0);sC.lineTo(i/5*w,h);sC.stroke();}
if(!pkBuf||pkBuf.length!==vl)pkBuf=new Uint8Array(vl);
for(var i=0;i<vl;i++)if(f[v.s+i]>pkBuf[i])pkBuf[i]=f[v.s+i];
sC.strokeStyle='rgba(255,60,60,0.4)';sC.lineWidth=1;sC.beginPath();
for(var i=0;i<vl;i++){var x=i*xs,y=h-pkBuf[i]/255*h;i?sC.lineTo(x,y):sC.moveTo(x,y);}sC.stroke();
sC.strokeStyle='#00ff41';sC.lineWidth=1.5;sC.beginPath();
for(var i=0;i<vl;i++){var x=i*xs,y=h-f[v.s+i]/255*h;i?sC.lineTo(x,y):sC.moveTo(x,y);}sC.stroke();
sC.fillStyle='#446644';sC.font='9px monospace';sC.textAlign='center';
var sf=cFreq-sRate/2;
for(var p=0;p<=5;p++){var bf=v.s/n+p/5*(vl/n);sC.fillText(fF(sf+bf*sRate),p/5*w,h-2);}
sC.textAlign='left';sC.fillStyle='#445544';var dr=dbMax-dbMin;
for(var d=0;d<=100;d+=20){var y=h-d/100*h;sC.fillText((dbMin+d/100*dr|0)+' dB',2,y-2);}
drawFO(spCv,sC);}
function dWf(f){var w=wfCv.width,h=wfCv.height,n=f.length,v=vB(n),vl=v.e-v.s;
if(!wI||wI.width!==w||wI.height!==h){wI=wC.createImageData(w,h);
var di=wI.data;for(var i=3;i<di.length;i+=4)di[i]=255;}
var di=wI.data,st=w*4;di.copyWithin(st,0,(h-1)*st);
for(var x=0;x<w;x++){var bn=v.s+(x*vl/w|0);if(bn>=v.e)bn=v.e-1;
var cl=cm[f[bn]],ix=x*4;di[ix]=cl[0];di[ix+1]=cl[1];di[ix+2]=cl[2];di[ix+3]=255;}
wC.putImageData(wI,0,0);drawFO(wfCv,wC);}
function uSm(f){var c=f.length/2|0,s=0;
for(var i=c-20;i<=c+20;i++)if(i>=0&&i<f.length)s+=f[i];
var a=s/41,txt=a<100?'S'+(a/20|0):a<170?'S9':'S9+'+(a-170|0);
eSm[TC]=txt;eSmF.style.width=MN(100,a/255*100)+'%';}
function S(q,n){return(q[n]-127.5)/127.5;}
function dm(q){var m=mode,r=[],n,i,j;
if(m==='AM'){var dc=0;for(n=0;n<q.length-1;n+=2){i=S(q,n);j=S(q,n+1);var v=Math.sqrt(i*i+j*j);dc=dc*.999+v*.001;r.push(v-dc);}return r;}
if(m==='USB'||m==='LSB'){var u=m==='USB';for(n=0;n<q.length-1;n+=2){i=S(q,n);j=S(q,n+1);r.push(u?(i+j)*.5:(i-j)*.5);}return r;}
for(n=0;n<q.length-3;n+=2){i=S(q,n);j=S(q,n+1);r.push(Math.atan2(j*pI-i*pQ,i*pI+j*pQ)/Math.PI);pI=i;pQ=j;}return r;}
function stA(){if(aCtx)return;
aCtx=new(window.AudioContext||window.webkitAudioContext)({sampleRate:48000});
var g=aCtx.createGain();g.gain.value=vol;g.connect(aCtx.destination);
sN=aCtx.createScriptProcessor(4096,1,1);
sN.onaudioprocess=function(e){var o=e.outputBuffer.getChannelData(0),p=0;
while(p<o.length&&aQ.length>0){var s=dm(aQ.shift()),pw=0,sl=s.length;
for(var i=0;i<sl;i++)pw+=s[i]*s[i];pw/=sl||1;
var mute=sql>0&&pw<sql/255*.01;
for(var i=0;i<sl&&p<o.length;i++,p++)o[p]=mute?0:s[i]*.8;}
while(p<o.length)o[p++]=0;};
sN.connect(g);sN._g=g;audOn=true;subIQ();
eAB[TC]='Stop';eAB.classList.add('active');}
function spA(){audOn=false;if(ws&&ws.readyState===1)tx('unsubscribe_iq',{});
if(sN){sN.disconnect();if(sN._g)sN._g.disconnect();sN=null;}
if(aCtx){aCtx.close();aCtx=null;}aQ=[];pI=pQ=0;
eAB[TC]='Audio';eAB.classList.remove('active');}
function nrEdge(x,W){var vf=gVF(),fl=f2x(tunedFreq-filterBW/2,W,vf),fr=f2x(tunedFreq+filterBW/2,W,vf);
if(MA(x-fl)<7)return'left';if(MA(x-fr)<7)return'right';return null;}
function gcx(e){return e.touches&&e.touches.length?e.touches[0].clientX:e.clientX;}
var pndF=null,fTmr=null;
function commitF(){if(pndF!==null){tx('freq',{value:pndF});pndF=null;}}
function onDn(e){e.preventDefault();
var t=e.target,r=gbc(t),x=gcx(e)-r.left,edge=nrEdge(x,r.width);
if(edge){isResize=true;resEdge=edge;cur(t,'ew-resize');return;}
isDrag=true;dragMoved=false;dragX=x;dragF=cFreq;cur(t,'grabbing');}
function onMv(e){e.preventDefault();
var t=e.target,r=gbc(t),x=gcx(e)-r.left;
if(!isDrag&&!isResize){cur(t,nrEdge(x,r.width)?'ew-resize':'crosshair');return;}
if(isDrag){var dx=x-dragX;if(MA(dx)>3)dragMoved=true;
var nf=MX(24e6,MN(1766e6,MR(dragF-dx*hpp(r.width))));
cFreq=nf;tunedFreq=nf;tuneOff=0;pndF=nf;
if(fTmr)clearTimeout(fTmr);fTmr=setTimeout(commitF,120);buildFD();}
if(isResize){var vf=gVF(),fHz=vf.s+x/r.width*vf.bw;
filterBW=MX(500,MN(sRate*.9,MR(MA(fHz-tunedFreq)*2)));
eBW[TC]=bwTxt();}}
function onUp(e){
var t=e.target,r=gbc(t);
var x=(e.changedTouches?e.changedTouches[0].clientX:gcx(e))-r.left;
if(isDrag&&!dragMoved){
var v=vB(fftSz);
tunedFreq=MR(cFreq-sRate/2+(v.s+x/r.width*(v.e-v.s))/fftSz*sRate);
tuneOff=0;subIQ();buildFD();}
if(isDrag){commitF();isDrag=false;dragMoved=false;}
if(isResize){isResize=false;resEdge=null;subIQ();}
cur(t,'crosshair');}
function onLv(e){if(isDrag){commitF();isDrag=false;dragMoved=false;}
if(isResize){isResize=false;resEdge=null;}cur(e.target,'crosshair');}
function whl(e){e.preventDefault();var r=gbc(e.target);
var v=vB(fftSz),vl=v.e-v.s;zCtr=(v.s+(e.clientX-r.left)/r.width*vl)/fftSz;
if(e.deltaY<0&&zLvl<8)zLvl*=2;else if(e.deltaY>0&&zLvl>1)zLvl/=2;
eZV[TC]=zLvl+'x';wI=null;pkBuf=null;}
[spCv,wfCv].forEach(function(cv){
on(cv,'mousedown',onDn);on(cv,'mousemove',onMv);on(cv,'mouseup',onUp);on(cv,'mouseleave',onLv);
on(cv,'wheel',whl,{passive:false});
on(cv,'touchstart',onDn,{passive:false});on(cv,'touchmove',onMv,{passive:false});on(cv,'touchend',onUp,{passive:false});});
on(eAB,'click',function(){audOn?spA():stA();});
qsa('.mode-btn').forEach(function(b){on(b,'click',function(){
qsa('.mode-btn').forEach(function(x){x.classList.remove('active');});
b.classList.add('active');mode=b.getAttribute('data-mode');
filterBW=modeBW[mode]||12500;eBW[TC]=bwTxt();subIQ();});});
on(eRS,'change',function(){tx('sample_rate',{value:PI(this.value)});});
on(eFS,'change',function(){tx('fft_size',{value:PI(this.value)});wI=null;pkBuf=null;});
function dbInp(){this===eNS?eNV[TC]=this.value:eLV[TC]=this.value;tDb();}
on(eNS,'input',dbInp);on(eLS,'input',dbInp);
on(eGS,'input',function(){gain=PI(this.value);
eGV[TC]=gain?((gain/10).toFixed(1)+' dB'):'Auto';tx('gain',{value:gain});});
on(eVS,'input',function(){vol=PI(this.value)/100;
eVV[TC]=(vol*100|0)+'%';if(sN&&sN._g)sN._g.gain.value=vol;});
on(eSS,'input',function(){sql=PI(this.value);eSV[TC]=sql;});
on(eWS,'input',function(){wfSpd=110-PI(this.value);eWV[TC]=this.value;});
on($('zoom-in'),'click',function(){if(zLvl<8){zLvl*=2;eZV[TC]=zLvl+'x';wI=null;pkBuf=null;}});
on($('zoom-out'),'click',function(){if(zLvl>1){zLvl/=2;eZV[TC]=zLvl+'x';wI=null;pkBuf=null;}});
on($('zoom-reset'),'click',function(){zLvl=1;zCtr=.5;eZV[TC]='1x';wI=null;pkBuf=null;});
on($('peak-clear'),'click',function(){pkBuf=null;});
on(eTK,'input',function(){tuneOff=PI(this.value);
eTL[TC]=(tuneOff>=0?'+':'')+tuneOff;
tunedFreq=MR(cFreq+tuneOff);buildFD();eBW[TC]=bwTxt();subIQ();});
rsz();uD();wsc();
})();
