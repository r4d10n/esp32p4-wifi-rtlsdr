/* ESP32-P4 WebSDR - SPDX-License-Identifier: GPL-2.0-or-later */
(function(){
'use strict';
var ws,cFreq=100e6,sRate=1024000,fftSz=1024,dbMin=-40,dbMax=40;
var gain=0,mode='WBFM',vol=0.5,sql=0,audOn=false;
var zLvl=1,zCtr=0.5,wfSpd=50,wfLast=0;
var $=document.getElementById.bind(document);
var eS=$('status'),eFD=$('freq-display'),eRD=$('rate-display'),eSm=$('smeter');
var spCv=$('spectrum'),wfCv=$('waterfall'),eFI=$('freq-input');
var eGS=$('gain-slider'),eGV=$('gain-val'),eVS=$('vol-slider'),eVV=$('vol-val');
var eSS=$('sql-slider'),eSV=$('sql-val'),eAB=$('audio-btn');
var eRS=$('rate-select'),eFS=$('fft-select');
var eNS=$('range-slider'),eNV=$('range-val'),eLS=$('ref-slider'),eLV=$('ref-val');
var eZI=$('zoom-in'),eZO=$('zoom-out'),eZR=$('zoom-reset'),eZV=$('zoom-val');
var eWS=$('wf-speed'),eWV=$('wf-speed-val');
var sC=spCv.getContext('2d'),wC=wfCv.getContext('2d'),wI=null;
var cm=new Array(256),aCtx=null,sN=null,aQ=[],pI=0,pQ=0;
(function(){for(var i=0;i<256;i++){var t=i/255,r,g,b;
if(t<.25){r=0;g=t/.25*255|0;b=255;}
else if(t<.5){r=0;g=255;b=(.5-t)/.25*255|0;}
else if(t<.75){r=(t-.5)/.25*255|0;g=255;b=0;}
else{r=255;g=(1-t)/.25*255|0;b=0;}
cm[i]=[r,g,b];}})();
function rsz(){var p=$('display'),w=p.clientWidth;
spCv.width=w;spCv.height=spCv.clientHeight;
wfCv.width=w;wfCv.height=wfCv.clientHeight;wI=null;}
window.addEventListener('resize',rsz);
function fF(h){if(h>=1e9)return(h/1e9).toFixed(6)+' GHz';
if(h>=1e6)return(h/1e6).toFixed(3)+' MHz';
if(h>=1e3)return(h/1e3).toFixed(1)+' kHz';return h+' Hz';}
function uD(){eFD.textContent=fF(cFreq);eRD.textContent=(sRate/1e3|0)+' kSPS';}
function vB(n){var f=1/zLvl,h=f/2,s=zCtr-h,e=zCtr+h;
if(s<0){e-=s;s=0;}if(e>1){s-=e-1;e=1;}if(s<0)s=0;
return{s:s*n|0,e:Math.min(Math.ceil(e*n),n)};}
function tx(c,p){if(!ws||ws.readyState!==1)return;
var m={cmd:c};if(p)for(var k in p)m[k]=p[k];ws.send(JSON.stringify(m));}
function tDb(){tx('db_range',{min:parseInt(eLV.value)-parseInt(eNS.value),max:parseInt(eLV.value)});}
function sC2(){eRS.value=''+sRate;eFS.value=''+fftSz;
var r=dbMax-dbMin;eNS.value=r;eNV.textContent=r|0;eLV.value=dbMax;eLV.textContent=dbMax|0;}
function wsc(){var h=location.hostname||'192.168.1.232',p=location.port||'8080';
ws=new WebSocket('ws://'+h+':'+p+'/ws');ws.binaryType='arraybuffer';
ws.onopen=function(){eS.textContent='Connected';eS.className='connected';};
ws.onclose=function(){eS.textContent='Disconnected';eS.className='disconnected';setTimeout(wsc,2000);};
ws.onerror=function(){ws.close();};
ws.onmessage=function(v){typeof v.data==='string'?oT(v.data):oB(v.data);};}
function oT(t){var m;try{m=JSON.parse(t);}catch(e){return;}
if(m.type==='info'){cFreq=m.freq||cFreq;sRate=m.rate||sRate;gain=m.gain||0;fftSz=m.fft_size||fftSz;
if(m.db_min!=null)dbMin=m.db_min;if(m.db_max!=null)dbMax=m.db_max;
eFI.value=(cFreq/1e6).toFixed(3);sC2();uD();}
else if(m.type==='freq'){cFreq=m.value;eFI.value=(cFreq/1e6).toFixed(3);uD();}
else if(m.type==='config'){fftSz=m.fft_size||fftSz;sRate=m.sample_rate||sRate;
if(m.db_min!=null)dbMin=m.db_min;if(m.db_max!=null)dbMax=m.db_max;wI=null;sC2();uD();}}
function oB(buf){var d=new Uint8Array(buf);if(d.length<2)return;
var t=d[0],p=d.subarray(1);
if(t===1){dSp(p);var n=performance.now();
if(n-wfLast>=wfSpd){dWf(p);wfLast=n;}uSm(p);}
else if(t===2&&audOn)aQ.push(p.slice());}
function dSp(f){var w=spCv.width,h=spCv.height,c=sC,n=f.length,v=vB(n),vl=v.e-v.s,xs=w/vl;
c.fillStyle='#0a0a0a';c.fillRect(0,0,w,h);
c.strokeStyle='#222';c.lineWidth=1;
for(var d=0;d<=100;d+=20){var y=h-(d/100)*h;c.beginPath();c.moveTo(0,y);c.lineTo(w,y);c.stroke();}
c.strokeStyle='#0f0';c.beginPath();
for(var i=0;i<vl;i++){var x=i*xs,y=h-(f[v.s+i]/255)*h;i?c.lineTo(x,y):c.moveTo(x,y);}c.stroke();
c.fillStyle='#888';c.font='10px monospace';c.textAlign='center';
var sf=cFreq-sRate/2;
for(var p=0;p<=4;p++){var fx=p/4*w,bf=v.s/n+p/4*(vl/n);c.fillText(fF(sf+bf*sRate),fx,h-2);}
c.textAlign='left';c.fillStyle='#666';var dr=dbMax-dbMin;
for(var d=0;d<=100;d+=20){var y=h-(d/100)*h;c.fillText((dbMin+d/100*dr|0)+' dB',2,y-2);}}
function dWf(f){var w=wfCv.width,h=wfCv.height,n=f.length,v=vB(n),vl=v.e-v.s;
if(!wI||wI.width!==w||wI.height!==h){wI=wC.createImageData(w,h);
var d=wI.data;for(var i=3;i<d.length;i+=4)d[i]=255;}
var d=wI.data,st=w*4;d.copyWithin(st,0,(h-1)*st);
for(var x=0;x<w;x++){var bn=v.s+(x*vl/w|0);if(bn>=v.e)bn=v.e-1;
var cl=cm[f[bn]],ix=x*4;d[ix]=cl[0];d[ix+1]=cl[1];d[ix+2]=cl[2];d[ix+3]=255;}
wC.putImageData(wI,0,0);}
function uSm(f){var c=f.length/2|0,s=0;
for(var i=c-20;i<=c+20;i++)if(i>=0&&i<f.length)s+=f[i];
var a=s/41;eSm.textContent=a<100?'S'+(a/20|0):a<170?'S9':'S9+'+(a-170|0);}
function mP(){var m=mode;return m==='NFM'?{b:12500,o:0}:m==='AM'?{b:10000,o:0}:
m==='WBFM'?{b:150000,o:0}:m==='USB'?{b:3000,o:1500}:
m==='LSB'?{b:3000,o:-1500}:{b:25000,o:0};}
function dmFM(q){var r=[];for(var n=0;n<q.length-3;n+=2){
var i=(q[n]-127.5)/127.5,j=(q[n+1]-127.5)/127.5;
r.push(Math.atan2(j*pI-i*pQ,i*pI+j*pQ)/Math.PI);pI=i;pQ=j;}return r;}
function dmAM(q){var r=[],dc=0;for(var n=0;n<q.length-1;n+=2){
var i=(q[n]-127.5)/127.5,j=(q[n+1]-127.5)/127.5,m=Math.sqrt(i*i+j*j);
dc=dc*.999+m*.001;r.push(m-dc);}return r;}
function dm(q){var m=mode;return m==='AM'?dmAM(q):m==='USB'||m==='LSB'?(function(){
var r=[],u=m==='USB';for(var n=0;n<q.length-1;n+=2){
var i=(q[n]-127.5)/127.5,j=(q[n+1]-127.5)/127.5;
r.push(u?(i+j)*.5:(i-j)*.5);}return r;})():dmFM(q);}
function stA(){if(aCtx)return;
aCtx=new(window.AudioContext||window.webkitAudioContext)({sampleRate:48000});
var g=aCtx.createGain();g.gain.value=vol;g.connect(aCtx.destination);
sN=aCtx.createScriptProcessor(4096,1,1);
sN.onaudioprocess=function(e){var o=e.outputBuffer.getChannelData(0),p=0;
while(p<o.length&&aQ.length>0){var s=dm(aQ.shift()),pw=0;
for(var i=0;i<s.length;i++)pw+=s[i]*s[i];pw/=(s.length||1);
if(sql>0&&pw<sql/255*.01){for(var i=0;i<s.length&&p<o.length;i++,p++)o[p]=0;}
else{for(var i=0;i<s.length&&p<o.length;i++,p++)o[p]=s[i]*.8;}}
while(p<o.length)o[p++]=0;};
sN.connect(g);sN._g=g;var p=mP();tx('subscribe_iq',{offset:p.o,bw:p.b});
audOn=true;eAB.textContent='Stop';eAB.classList.add('active');}
function spA(){audOn=false;if(ws&&ws.readyState===1)tx('unsubscribe_iq',{});
if(sN){sN.disconnect();if(sN._g)sN._g.disconnect();sN=null;}
if(aCtx){aCtx.close();aCtx=null;}aQ=[];pI=pQ=0;
eAB.textContent='Start Audio';eAB.classList.remove('active');}
function cvC(e){var r=e.target.getBoundingClientRect(),x=e.clientX-r.left;
var n=fftSz,v=vB(n),vl=v.e-v.s,bf=(v.s+x/r.width*vl)/n;
var nf=Math.round(cFreq-sRate/2+bf*sRate);eFI.value=(nf/1e6).toFixed(3);
if(audOn){var p=mP();tx('subscribe_iq',{offset:nf-cFreq,bw:p.b});}}
$('freq-set').addEventListener('click',function(){var m=parseFloat(eFI.value);
if(!isNaN(m))tx('freq',{value:Math.round(m*1e6)});});
eFI.addEventListener('keydown',function(e){if(e.key==='Enter')$('freq-set').click();});
document.querySelectorAll('.mode-btn').forEach(function(b){b.addEventListener('click',function(){
document.querySelectorAll('.mode-btn').forEach(function(x){x.classList.remove('active');});
b.classList.add('active');mode=b.getAttribute('data-mode');
if(audOn){var p=mP();tx('subscribe_iq',{offset:p.o,bw:p.b});}});});
eRS.addEventListener('change',function(){tx('sample_rate',{value:parseInt(this.value)});});
eFS.addEventListener('change',function(){tx('fft_size',{value:parseInt(this.value)});wI=null;});
eNS.addEventListener('input',function(){eNV.textContent=this.value;tDb();});
eLS.addEventListener('input',function(){eLV.textContent=this.value;tDb();});
eGS.addEventListener('input',function(){gain=parseInt(this.value);
eGV.textContent=gain?((gain/10).toFixed(1)+' dB'):'Auto';tx('gain',{value:gain});});
eVS.addEventListener('input',function(){vol=parseInt(this.value)/100;
eVV.textContent=(vol*100|0)+'%';if(sN&&sN._g)sN._g.gain.value=vol;});
eSS.addEventListener('input',function(){sql=parseInt(this.value);eSV.textContent=sql;});
eAB.addEventListener('click',function(){audOn?spA():stA();});
eZI.addEventListener('click',function(){if(zLvl<8){zLvl*=2;eZV.textContent=zLvl+'x';wI=null;}});
eZO.addEventListener('click',function(){if(zLvl>1){zLvl/=2;eZV.textContent=zLvl+'x';wI=null;}});
eZR.addEventListener('click',function(){zLvl=1;zCtr=.5;eZV.textContent='1x';wI=null;});
eWS.addEventListener('input',function(){wfSpd=110-parseInt(this.value);eWV.textContent=this.value;});
spCv.addEventListener('click',cvC);wfCv.addEventListener('click',cvC);
function whl(e){e.preventDefault();var r=e.target.getBoundingClientRect();
var n=fftSz,v=vB(n),vl=v.e-v.s;zCtr=(v.s+(e.clientX-r.left)/r.width*vl)/n;
if(e.deltaY<0&&zLvl<8)zLvl*=2;else if(e.deltaY>0&&zLvl>1)zLvl/=2;
eZV.textContent=zLvl+'x';wI=null;}
spCv.addEventListener('wheel',whl);wfCv.addEventListener('wheel',whl);
rsz();uD();wsc();
})();
