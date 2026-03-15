/* ESP32-P4 WebSDR Client - SPDX-License-Identifier: GPL-2.0-or-later */
(function(){
'use strict';
var ws,centerFreq=100e6,sampleRate=1024000,fftSize=1024,dbMin=-40,dbMax=40;
var currentGain=0,currentMode='WBFM',volume=0.5,squelch=0;
var audioRunning=false,iqSubscribed=false;
var zoomLevel=1,zoomCenter=0.5,wfSpeed=50,wfAccum=null,wfLastDraw=0;
var $=document.getElementById.bind(document);
var elStatus=$('status'),elFreqD=$('freq-display'),elRateD=$('rate-display');
var elSmeter=$('smeter'),specCv=$('spectrum'),wfCv=$('waterfall');
var elFreqIn=$('freq-input'),elGainS=$('gain-slider'),elGainV=$('gain-val');
var elVolS=$('vol-slider'),elVolV=$('vol-val'),elSqlS=$('sql-slider'),elSqlV=$('sql-val');
var elAudioBtn=$('audio-btn'),elRateS=$('rate-select'),elFftS=$('fft-select');
var elRngS=$('range-slider'),elRngV=$('range-val'),elRefS=$('ref-slider'),elRefV=$('ref-val');
var elZI=$('zoom-in'),elZO=$('zoom-out'),elZR=$('zoom-reset'),elZV=$('zoom-val');
var elWfS=$('wf-speed'),elWfV=$('wf-speed-val');
var sCtx=specCv.getContext('2d'),wCtx=wfCv.getContext('2d'),wfImg=null;
var cmap=new Array(256),lastFFT=null,peakHold=null;
var audioCtx=null,scriptNode=null,audioQueue=[],prevI=0,prevQ=0;
(function(){for(var i=0;i<256;i++){var t=i/255,r,g,b;
if(t<.25){r=0;g=Math.round(t/.25*255);b=255;}
else if(t<.5){r=0;g=255;b=Math.round((.5-t)/.25*255);}
else if(t<.75){r=Math.round((t-.5)/.25*255);g=255;b=0;}
else{r=255;g=Math.round((1-t)/.25*255);b=0;}
cmap[i]=[r,g,b];}})();

function resize(){var p=$('display'),w=p.clientWidth;
specCv.width=w;specCv.height=specCv.clientHeight;
wfCv.width=w;wfCv.height=wfCv.clientHeight;wfImg=null;}
window.addEventListener('resize',resize);

function fmtF(hz){if(hz>=1e9)return(hz/1e9).toFixed(6)+' GHz';
if(hz>=1e6)return(hz/1e6).toFixed(3)+' MHz';
if(hz>=1e3)return(hz/1e3).toFixed(1)+' kHz';return hz+' Hz';}

function updDisp(){elFreqD.textContent=fmtF(centerFreq);
elRateD.textContent=(sampleRate/1000).toFixed(0)+' kSPS';}

function visBins(len){var vf=1/zoomLevel,hv=vf/2,s=zoomCenter-hv,e=zoomCenter+hv;
if(s<0){e-=s;s=0;}if(e>1){s-=(e-1);e=1;}if(s<0)s=0;
var bs=Math.floor(s*len),be=Math.ceil(e*len);
if(be>len)be=len;return{s:bs,e:be};}

function sendCmd(cmd,p){if(!ws||ws.readyState!==1)return;
var m={cmd:cmd};if(p)for(var k in p)m[k]=p[k];ws.send(JSON.stringify(m));}

function sendDb(){var r=parseInt(elRngS.value),ref=parseInt(elRefS.value);
sendCmd('db_range',{min:ref-r,max:ref});}

function syncCtrl(){elRateS.value=String(sampleRate);elFftS.value=String(fftSize);
var r=dbMax-dbMin;elRngS.value=r;elRngV.textContent=Math.round(r);
elRefS.value=dbMax;elRefV.textContent=Math.round(dbMax);}

function wsConnect(){var h=location.hostname||'192.168.1.232';
var p=location.port||'8080';ws=new WebSocket('ws://'+h+':'+p+'/ws');
ws.binaryType='arraybuffer';
ws.onopen=function(){elStatus.textContent='Connected';elStatus.className='connected';};
ws.onclose=function(){elStatus.textContent='Disconnected';elStatus.className='disconnected';
iqSubscribed=false;setTimeout(wsConnect,2000);};
ws.onerror=function(){ws.close();};
ws.onmessage=function(ev){if(typeof ev.data==='string')onText(ev.data);else onBin(ev.data);};}

function onText(t){var m;try{m=JSON.parse(t);}catch(e){return;}
if(m.type==='info'){centerFreq=m.freq||centerFreq;sampleRate=m.rate||sampleRate;
currentGain=m.gain||0;fftSize=m.fft_size||fftSize;
if(m.db_min!=null)dbMin=m.db_min;if(m.db_max!=null)dbMax=m.db_max;
elFreqIn.value=(centerFreq/1e6).toFixed(3);peakHold=null;syncCtrl();updDisp();
}else if(m.type==='freq'){centerFreq=m.value;
elFreqIn.value=(centerFreq/1e6).toFixed(3);peakHold=null;updDisp();
}else if(m.type==='config'){fftSize=m.fft_size||fftSize;
sampleRate=m.sample_rate||sampleRate;
if(m.db_min!=null)dbMin=m.db_min;if(m.db_max!=null)dbMax=m.db_max;
peakHold=null;wfImg=null;syncCtrl();updDisp();
}else if(m.type==='iq_start')iqSubscribed=true;
else if(m.type==='iq_stop')iqSubscribed=false;}

function onBin(buf){var d=new Uint8Array(buf);if(d.length<2)return;
var ty=d[0],pl=d.subarray(1);
if(ty===1){lastFFT=pl;drawSpec(pl);
wfAccum=pl;var now=performance.now();
if(now-wfLastDraw>=wfSpeed){drawWF(wfAccum);wfLastDraw=now;wfAccum=null;}
updSm(pl);}
else if(ty===2&&audioRunning)audioQueue.push(pl.slice());}

function drawSpec(fft){var w=specCv.width,h=specCv.height,c=sCtx,len=fft.length;
var vr=visBins(len),vl=vr.e-vr.s,xs=w/vl;
c.fillStyle='#0a0a0a';c.fillRect(0,0,w,h);
c.strokeStyle='#222';c.lineWidth=1;
for(var db=0;db<=100;db+=20){var gy=h-(db/100)*h;
c.beginPath();c.moveTo(0,gy);c.lineTo(w,gy);c.stroke();}
c.strokeStyle='#00ff00';c.lineWidth=1;c.beginPath();
for(var i=0;i<vl;i++){var x=i*xs,y=h-(fft[vr.s+i]/255)*h;
if(i===0)c.moveTo(x,y);else c.lineTo(x,y);}c.stroke();
if(!peakHold||peakHold.length!==len)peakHold=new Uint8Array(len);
for(var i=0;i<len;i++){if(fft[i]>peakHold[i])peakHold[i]=fft[i];
else if(peakHold[i]>0)peakHold[i]--;}
c.strokeStyle='rgba(255,100,100,0.5)';c.beginPath();
for(var i=0;i<vl;i++){var x=i*xs,y=h-(peakHold[vr.s+i]/255)*h;
if(i===0)c.moveTo(x,y);else c.lineTo(x,y);}c.stroke();
c.fillStyle='#888';c.font='10px monospace';c.textAlign='center';
var sf=centerFreq-sampleRate/2;
for(var p=0;p<=4;p++){var fx=(p/4)*w,bf=vr.s/len+(p/4)*(vl/len);
c.fillText(fmtF(sf+bf*sampleRate),fx,h-2);}
c.textAlign='left';c.fillStyle='#666';var dr=dbMax-dbMin;
for(var db=0;db<=100;db+=20){var gy=h-(db/100)*h;
c.fillText((dbMin+(db/100)*dr).toFixed(0)+' dB',2,gy-2);}}

function drawWF(fft){var w=wfCv.width,h=wfCv.height,len=fft.length;
var vr=visBins(len),vl=vr.e-vr.s;
if(!wfImg||wfImg.width!==w||wfImg.height!==h){
wfImg=wCtx.createImageData(w,h);var d=wfImg.data;
for(var i=3;i<d.length;i+=4)d[i]=255;}
var d=wfImg.data,st=w*4;d.copyWithin(st,0,(h-1)*st);
for(var x=0;x<w;x++){var bn=vr.s+Math.floor(x*vl/w);
if(bn>=vr.e)bn=vr.e-1;var v=fft[bn],cl=cmap[v],ix=x*4;
d[ix]=cl[0];d[ix+1]=cl[1];d[ix+2]=cl[2];d[ix+3]=255;}
wCtx.putImageData(wfImg,0,0);}

function updSm(fft){var c=Math.floor(fft.length/2),sp=20,s=0;
for(var i=c-sp;i<=c+sp;i++)if(i>=0&&i<fft.length)s+=fft[i];
var a=s/(2*sp+1),sv;
if(a<80)sv='S0';else if(a<100)sv='S1';else if(a<115)sv='S3';
else if(a<130)sv='S5';else if(a<150)sv='S7';else if(a<170)sv='S9';
else if(a<200)sv='S9+20';else if(a<230)sv='S9+40';else sv='S9+60';
elSmeter.textContent=sv;}

function modeP(){switch(currentMode){
case'NFM':return{bw:12500,offset:0};case'AM':return{bw:10000,offset:0};
case'WBFM':return{bw:150000,offset:0};case'USB':return{bw:3000,offset:1500};
case'LSB':return{bw:3000,offset:-1500};default:return{bw:25000,offset:0};}}

function demodFM(iq){var r=[];for(var n=0;n<iq.length-3;n+=2){
var i0=(iq[n]-127.5)/127.5,q0=(iq[n+1]-127.5)/127.5;
var dot=i0*prevI+q0*prevQ,cross=q0*prevI-i0*prevQ;
r.push(Math.atan2(cross,dot)/Math.PI);prevI=i0;prevQ=q0;}return r;}

function demodAM(iq){var r=[],dc=0;for(var n=0;n<iq.length-1;n+=2){
var i=(iq[n]-127.5)/127.5,q=(iq[n+1]-127.5)/127.5;
var m=Math.sqrt(i*i+q*q);dc=dc*.999+m*.001;r.push(m-dc);}return r;}

function demodSSB(iq,u){var r=[];for(var n=0;n<iq.length-1;n+=2){
var i=(iq[n]-127.5)/127.5,q=(iq[n+1]-127.5)/127.5;
r.push(u?(i+q)*.5:(i-q)*.5);}return r;}

function demod(iq){switch(currentMode){
case'NFM':case'WBFM':return demodFM(iq);case'AM':return demodAM(iq);
case'USB':return demodSSB(iq,true);case'LSB':return demodSSB(iq,false);
default:return demodFM(iq);}}

function startAudio(){if(audioCtx)return;
audioCtx=new(window.AudioContext||window.webkitAudioContext)({sampleRate:48000});
var gn=audioCtx.createGain();gn.gain.value=volume;gn.connect(audioCtx.destination);
scriptNode=audioCtx.createScriptProcessor(4096,1,1);
scriptNode.onaudioprocess=function(e){var out=e.outputBuffer.getChannelData(0);
var pos=0;while(pos<out.length&&audioQueue.length>0){
var iq=audioQueue.shift(),sam=demod(iq),pw=0;
for(var s=0;s<sam.length;s++)pw+=sam[s]*sam[s];pw/=(sam.length||1);
if(squelch>0&&pw<(squelch/255)*.01){for(var s=0;s<sam.length&&pos<out.length;s++,pos++)out[pos]=0;}
else{for(var s=0;s<sam.length&&pos<out.length;s++,pos++)out[pos]=sam[s]*.8;}}
while(pos<out.length)out[pos++]=0;};
scriptNode.connect(gn);scriptNode._gn=gn;
var p=modeP();sendCmd('subscribe_iq',{offset:p.offset,bw:p.bw});
audioRunning=true;elAudioBtn.textContent='Stop Audio';elAudioBtn.classList.add('active');}

function stopAudio(){audioRunning=false;iqSubscribed=false;
if(ws&&ws.readyState===1)sendCmd('unsubscribe_iq',{});
if(scriptNode){scriptNode.disconnect();if(scriptNode._gn)scriptNode._gn.disconnect();scriptNode=null;}
if(audioCtx){audioCtx.close();audioCtx=null;}
audioQueue=[];prevI=0;prevQ=0;
elAudioBtn.textContent='Start Audio';elAudioBtn.classList.remove('active');}

function onCvClick(e){var r=e.target.getBoundingClientRect(),x=e.clientX-r.left,f=x/r.width;
var len=fftSize,vr=visBins(len),vl=vr.e-vr.s,bf=(vr.s+f*vl)/len;
var nf=Math.round(centerFreq-sampleRate/2+bf*sampleRate);
elFreqIn.value=(nf/1e6).toFixed(3);
if(audioRunning){var p=modeP();sendCmd('subscribe_iq',{offset:nf-centerFreq,bw:p.bw});}}

$('freq-set').addEventListener('click',function(){
var mhz=parseFloat(elFreqIn.value);if(isNaN(mhz))return;
sendCmd('freq',{value:Math.round(mhz*1e6)});});
elFreqIn.addEventListener('keydown',function(e){if(e.key==='Enter')$('freq-set').click();});

document.querySelectorAll('.mode-btn').forEach(function(btn){
btn.addEventListener('click',function(){
document.querySelectorAll('.mode-btn').forEach(function(b){b.classList.remove('active');});
btn.classList.add('active');currentMode=btn.getAttribute('data-mode');
if(audioRunning){var p=modeP();sendCmd('subscribe_iq',{offset:p.offset,bw:p.bw});}});});

elRateS.addEventListener('change',function(){sendCmd('sample_rate',{value:parseInt(this.value)});});
elFftS.addEventListener('change',function(){sendCmd('fft_size',{value:parseInt(this.value)});
peakHold=null;wfImg=null;});
elRngS.addEventListener('input',function(){elRngV.textContent=this.value;sendDb();});
elRefS.addEventListener('input',function(){elRefV.textContent=this.value;sendDb();});
elGainS.addEventListener('input',function(){currentGain=parseInt(this.value);
elGainV.textContent=currentGain===0?'Auto':(currentGain/10).toFixed(1)+' dB';
sendCmd('gain',{value:currentGain});});
elVolS.addEventListener('input',function(){volume=parseInt(this.value)/100;
elVolV.textContent=Math.round(volume*100)+'%';
if(scriptNode&&scriptNode._gn)scriptNode._gn.gain.value=volume;});
elSqlS.addEventListener('input',function(){squelch=parseInt(this.value);elSqlV.textContent=squelch;});
elAudioBtn.addEventListener('click',function(){if(audioRunning)stopAudio();else startAudio();});

elZI.addEventListener('click',function(){if(zoomLevel<8){zoomLevel*=2;elZV.textContent=zoomLevel+'x';wfImg=null;}});
elZO.addEventListener('click',function(){if(zoomLevel>1){zoomLevel/=2;elZV.textContent=zoomLevel+'x';wfImg=null;}});
elZR.addEventListener('click',function(){zoomLevel=1;zoomCenter=0.5;elZV.textContent='1x';wfImg=null;});
elWfS.addEventListener('input',function(){wfSpeed=110-parseInt(this.value);elWfV.textContent=this.value;});

specCv.addEventListener('click',onCvClick);
wfCv.addEventListener('click',onCvClick);

function onWheel(e){e.preventDefault();var r=e.target.getBoundingClientRect();
var x=e.clientX-r.left,f=x/r.width,len=fftSize,vr=visBins(len),vl=vr.e-vr.s;
zoomCenter=(vr.s+f*vl)/len;
if(e.deltaY<0&&zoomLevel<8)zoomLevel*=2;
else if(e.deltaY>0&&zoomLevel>1)zoomLevel/=2;
elZV.textContent=zoomLevel+'x';wfImg=null;}
specCv.addEventListener('wheel',onWheel);
wfCv.addEventListener('wheel',onWheel);

resize();updDisp();wsConnect();
})();
