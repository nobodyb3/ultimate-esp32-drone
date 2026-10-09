#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include "fc.h"

// Built-in config page. Connect phone/PC to the drone WiFi (ESP-DRONE_xxxxxx / 12345678)
// and open  http://192.168.43.42   (works together with the ESP-Drone app).

static WebServer webServer(80);

static const char WEB_PAGE[] PROGMEM = R"rawliteral(<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>ESPFLIGHT</title>
<style>
body{font-family:sans-serif;margin:8px;background:#111;color:#eee;font-size:14px}
h2,h3{margin:10px 0 4px}table{border-collapse:collapse}td,th{padding:2px 4px;text-align:center}
input{width:58px;background:#222;color:#eee;border:1px solid #555;padding:4px;border-radius:3px}
button{padding:9px 13px;margin:3px;background:#2a6;border:0;color:#fff;border-radius:5px;font-size:14px}
button.r{background:#c33}.w{background:#333;width:70px;height:10px;display:inline-block}
.b{height:10px;background:#2a6}#live{background:#1b1b1b;padding:8px;border-radius:6px;line-height:1.6}
</style></head><body>
<h2>ESPFLIGHT</h2>
<div id=live>connecting...</div>
<h3>PID &amp; rates</h3><div id=cfg></div>
<div><button onclick=apply()>Apply</button><button onclick=save()>Save to flash</button>
<button class=r onclick=defs()>Defaults</button><button onclick=acal()>Calibrate accel (keep flat)</button></div>
<h3>Motor test (PROPS OFF, disarmed only) - hold button</h3>
<div id=mt></div>
<script>
const $=id=>document.getElementById(id);
const j=async u=>(await fetch(u)).json();
const g=async u=>{try{await fetch(u)}catch(e){}};
const DIS={0:'NO_GYRO',2:'NO_RADIO',7:'THROTTLE_NOT_LOW',8:'TILTED',12:'CALIBRATING',25:'ARM_SWITCH_ON_FIRST'};
function flags(f){let s=[];for(const b in DIS)if(f&(1<<b))s.push(DIS[b]);return s.join(', ')||'none'}
async function tick(){
 try{
  const s=await j('/state');
  $('live').innerHTML=
   'State: <b>'+(s.armed?'ARMED':'disarmed')+'</b> | mode '+(s.angle?'ANGLE':'ACRO')+' | source <b>'+['none','ESP-NOW','ESP-Drone app'][s.src]+'</b> | gyro '+(s.gyro?'ok':'FAIL')+
   '<br>Arm blockers: '+(s.armed?'-':flags(s.dis))+
   '<br>Attitude: roll '+s.roll+' | pitch(nose down +) '+s.pitch+' | yaw '+s.yaw+
   '<br>App raw: roll '+s.crtp[0]+' pitch '+s.crtp[1]+' yaw '+s.crtp[2]+' thrust '+s.crtp[3]+
   '<br>RC: '+s.rc.join(' ')+
   '<br>Motors: '+s.m.map((v,i)=>'M'+(i+1)+' <span class=w><div class=b style="width:'+Math.round(v*100)+'%"></div></span>').join(' ')+
   '<br>loop '+s.cyc+'us | load '+s.load+'% | vbat '+s.vbat+'V | i2c err '+s.err;
 }catch(e){$('live').textContent='no connection'}
 setTimeout(tick,300);
}
async function loadCfg(){
 const c=await j('/cfg');
 let h='<table><tr><th></th><th>P</th><th>I</th><th>D</th><th>RC rate</th><th>Expo</th><th>Rate</th></tr>';
 ['Roll','Pitch','Yaw'].forEach((n,a)=>{
  h+='<tr><td>'+n+'</td>';
  ['p','i','d'].forEach((k,x)=>{h+='<td><input id='+k+a+' value='+c.pid[a][x]+'></td>'});
  h+='<td><input id=rc'+a+' value='+c.rc[a]+'></td><td><input id=ex'+a+' value='+c.ex[a]+'></td><td><input id=rt'+a+' value='+c.rt[a]+'></td></tr>';
 });
 h+='</table><br>Level strength <input id=lv value='+c.lv+'> Max angle <input id=la value='+c.la+'><br>TPA % <input id=tpa value='+c.tpa+'> TPA start pwm <input id=tpb value='+c.tpb+'>';
 h+='<h3>App (ESP-Drone) input scale <small>(negative = invert)</small></h3>Roll <input id=cs0 value='+c.cs[0]+'> Pitch <input id=cs1 value='+c.cs[1]+'> Yaw <input id=cs2 value='+c.cs[2]+'>';
 $('cfg').innerHTML=h;
}
async function apply(){
 const q=[...document.querySelectorAll('#cfg input')].map(e=>e.id+'='+encodeURIComponent(e.value)).join('&');
 await g('/set?'+q);alert('Applied. Press "Save to flash" to keep after reboot.');
}
async function save(){await g('/save');alert('Saved')}
async function defs(){if(confirm('Reset all settings?')){await g('/defaults');await loadCfg()}}
async function acal(){await g('/acccal');alert('Calibrating... keep the drone flat and still for 2 seconds')}
let iv=null;
function mdown(n){mup(n,true);iv=setInterval(()=>g('/motor?m='+n+'&v=1150'),200);g('/motor?m='+n+'&v=1150')}
function mup(n,quiet){if(iv){clearInterval(iv);iv=null}if(!quiet)g('/motor?m='+n+'&v=1000')}
let h='';
['M1 rear right','M2 front right','M3 rear left','M4 front left'].forEach((t,i)=>{
 const n=i+1;
 h+='<button onpointerdown="mdown('+n+')" onpointerup="mup('+n+')" onpointerleave="mup('+n+')" onpointercancel="mup('+n+')">'+t+'</button>';
});
$('mt').innerHTML=h;
loadCfg();tick();
</script></body></html>)rawliteral";

static void webSendJson(const char* s) {
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", s);
}

static void webState() {
  char b[720];
  snprintf(b, sizeof(b),
    "{\"armed\":%d,\"angle\":%d,\"gyro\":%d,\"src\":%d,\"dis\":%lu,"
    "\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.0f,"
    "\"m\":[%.2f,%.2f,%.2f,%.2f],\"rc\":[%u,%u,%u,%u,%u,%u,%u,%u],"
    "\"crtp\":[%.2f,%.2f,%.2f,%u],\"vbat\":%.2f,\"cyc\":%u,\"load\":%u,\"err\":%u}",
    fc.armed ? 1 : 0, fc.angleMode ? 1 : 0, fc.gyroOk ? 1 : 0, (int)fc.rcSource, (unsigned long)fc.armingDisable,
    fc.roll, fc.pitchDown, fc.yaw,
    fc.motor[0], fc.motor[1], fc.motor[2], fc.motor[3],
    (unsigned)fc.rc[0], (unsigned)fc.rc[1], (unsigned)fc.rc[2], (unsigned)fc.rc[3],
    (unsigned)fc.rc[4], (unsigned)fc.rc[5], (unsigned)fc.rc[6], (unsigned)fc.rc[7],
    (float)crtpSp.roll, (float)crtpSp.pitch, (float)crtpSp.yaw, (unsigned)crtpSp.thrust,
    fc.vbat, (unsigned)fc.cycleUs, (unsigned)fc.load, (unsigned)fc.i2cErrors);
  webSendJson(b);
}

static void webCfg() {
  char b[420];
  snprintf(b, sizeof(b),
    "{\"pid\":[[%u,%u,%u],[%u,%u,%u],[%u,%u,%u]],\"lv\":%u,\"la\":%u,"
    "\"rc\":[%u,%u,%u],\"ex\":[%u,%u,%u],\"rt\":[%u,%u,%u],\"tpa\":%u,\"tpb\":%u,"
    "\"cs\":[%.2f,%.2f,%.2f]}",
    cfg.pid[0][0], cfg.pid[0][1], cfg.pid[0][2],
    cfg.pid[1][0], cfg.pid[1][1], cfg.pid[1][2],
    cfg.pid[2][0], cfg.pid[2][1], cfg.pid[2][2],
    cfg.pid[3][0], cfg.levelAngle,
    cfg.rcRate[0], cfg.rcRate[1], cfg.rcRate[2],
    cfg.rcExpo[0], cfg.rcExpo[1], cfg.rcExpo[2],
    cfg.rate[0], cfg.rate[1], cfg.rate[2],
    cfg.tpaRate, (unsigned)cfg.tpaBreak,
    cfg.crtpScale[0], cfg.crtpScale[1], cfg.crtpScale[2]);
  webSendJson(b);
}

static long webInt(const char* key, long lo, long hi, long cur) {
  if (!webServer.hasArg(key)) return cur;
  long v = webServer.arg(key).toInt();
  return v < lo ? lo : (v > hi ? hi : v);
}

static void webSet() {
  char k[8];
  for (int a = 0; a < 3; a++) {
    for (int j = 0; j < 3; j++) {
      snprintf(k, sizeof(k), "%c%d", "pid"[j], a);
      cfg.pid[a][j] = (uint8_t)webInt(k, 0, 255, cfg.pid[a][j]);
    }
    snprintf(k, sizeof(k), "rc%d", a); cfg.rcRate[a] = (uint8_t)webInt(k, 0, 255, cfg.rcRate[a]);
    snprintf(k, sizeof(k), "ex%d", a); cfg.rcExpo[a] = (uint8_t)webInt(k, 0, 100, cfg.rcExpo[a]);
    snprintf(k, sizeof(k), "rt%d", a); cfg.rate[a]   = (uint8_t)webInt(k, 0, 99, cfg.rate[a]);
    snprintf(k, sizeof(k), "cs%d", a);
    if (webServer.hasArg(k)) {
      float v = webServer.arg(k).toFloat();
      if (v > 500.0f) v = 500.0f;
      if (v < -500.0f) v = -500.0f;
      cfg.crtpScale[a] = v;
    }
  }
  cfg.pid[3][0] = (uint8_t)webInt("lv", 0, 255, cfg.pid[3][0]);
  cfg.levelAngle = (uint8_t)webInt("la", 10, 80, cfg.levelAngle);
  cfg.tpaRate = (uint8_t)webInt("tpa", 0, 100, cfg.tpaRate);
  cfg.tpaBreak = (uint16_t)webInt("tpb", 1000, 2000, cfg.tpaBreak);
  webServer.send(200, "text/plain", "ok");
}

static void webMotor() {
  if (fc.armed) { webServer.send(403, "text/plain", "armed"); return; }
  int m = webServer.arg("m").toInt();
  int v = webServer.arg("v").toInt();
  if (m < 1 || m > 4) { webServer.send(400, "text/plain", "bad motor"); return; }
  if (v < 1000) v = 1000;
  if (v > 1500) v = 1500;                     // max 50 % for safety
  for (int i = 0; i < 4; i++) motorTestVal[i] = (i == m - 1) ? v : 1000;
  motorTestUntil = millis() + 600;
  webServer.send(200, "text/plain", "ok");
}

static void webBegin() {
  webServer.on("/", []() { webServer.send_P(200, "text/html", WEB_PAGE); });
  webServer.on("/state", webState);
  webServer.on("/cfg", webCfg);
  webServer.on("/set", webSet);
  webServer.on("/save", []() { if (!fc.armed) settingsSave(); webServer.send(200, "text/plain", "ok"); });
  webServer.on("/defaults", []() { if (!fc.armed) settingsDefaults(); webServer.send(200, "text/plain", "ok"); });
  webServer.on("/acccal", []() { if (!fc.armed) accCalRequested = true; webServer.send(200, "text/plain", "ok"); });
  webServer.on("/motor", webMotor);
  webServer.begin();
}

static void webPoll() {
  webServer.handleClient();
}
