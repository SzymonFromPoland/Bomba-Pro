#include "tuner.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

static AsyncWebServer server(80);

static float *_kp;
static float *_kd;
static float *_base_speed;
static VL53L1X_Result_t *_results;
static float *_error;
static float *_output;

static const char HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: sans-serif; padding: 20px; max-width: 500px; margin: auto; }
  label { display: flex; justify-content: space-between; margin-bottom: 4px; }
  input[type=range] { width: 100%; margin-bottom: 16px; }
  table { width: 100%; border-collapse: collapse; margin-top: 16px; }
  td, th { border: 1px solid #ccc; padding: 4px 8px; text-align: center; font-size: 13px; }
  th { background: #f0f0f0; }
  #error, #output { font-size: 18px; font-weight: bold; margin-top: 12px; }
</style>
</head>
<body>
<h2>Bomba-Pro Tuner</h2>

<label><span>Kp</span><span id="kpVal"></span></label>
<input type="range" id="kp" min="0" max="300" step="1" oninput="send('kp',this.value)">

<label><span>Kd</span><span id="kdVal"></span></label>
<input type="range" id="kd" min="0" max="100" step="0.5" oninput="send('kd',this.value)">

<label><span>Base speed</span><span id="bsVal"></span></label>
<input type="range" id="bs" min="0" max="100" step="1" oninput="send('bs',this.value)">

<table>
  <tr><th>#</th><th>Dist</th><th>Status</th></tr>
  <tr><td>0</td><td id="d0">-</td><td id="s0">-</td></tr>
  <tr><td>1</td><td id="d1">-</td><td id="s1">-</td></tr>
  <tr><td>2</td><td id="d2">-</td><td id="s2">-</td></tr>
  <tr><td>3</td><td id="d3">-</td><td id="s3">-</td></tr>
  <tr><td>4</td><td id="d4">-</td><td id="s4">-</td></tr>
  <tr><td>5</td><td id="d5">-</td><td id="s5">-</td></tr>
  <tr><td>6</td><td id="d6">-</td><td id="s6">-</td></tr>
</table>

<p>Error: <span id="error">-</span></p>
<p>Output: <span id="output">-</span></p>

<script>
function send(param, value) {
  document.getElementById(param+'Val').innerText = value;
  fetch('/set?'+param+'='+value);
}
fetch('/get').then(r=>r.json()).then(d=>{
  ['kp','kd','bs'].forEach(k=>{
    document.getElementById(k).value = d[k];
    document.getElementById(k+'Val').innerText = d[k];
  });
});
function updateData() {
  fetch('/data').then(r=>r.json()).then(d=>{
    for(let i=0;i<7;i++){
      document.getElementById('d'+i).innerText = d.s[i].d;
      document.getElementById('s'+i).innerText = d.s[i].st;
    }
    document.getElementById('error').innerText = d.err.toFixed(3);
    document.getElementById('output').innerText = d.out.toFixed(1);
  });
}
setInterval(updateData, 100);
</script>
</body>
</html>)HTML";

static void tunerTask(void *param)
{
    WiFi.softAP("Bomba-Pro", "12345678");

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/html", HTML);
    });

    server.on("/get", HTTP_GET, [](AsyncWebServerRequest *req) {
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"kp\":%.1f,\"kd\":%.1f,\"bs\":%.1f}", *_kp, *_kd, *_base_speed);
        req->send(200, "application/json", buf);
    });

    server.on("/set", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (req->hasParam("kp"))  *_kp        = req->getParam("kp")->value().toFloat();
        if (req->hasParam("kd"))  *_kd        = req->getParam("kd")->value().toFloat();
        if (req->hasParam("bs"))  *_base_speed = req->getParam("bs")->value().toFloat();
        req->send(200, "text/plain", "ok");
    });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *req) {
        char buf[256];
        int len = snprintf(buf, sizeof(buf),
            "{\"s\":["
            "{\"d\":%d,\"st\":%d},"
            "{\"d\":%d,\"st\":%d},"
            "{\"d\":%d,\"st\":%d},"
            "{\"d\":%d,\"st\":%d},"
            "{\"d\":%d,\"st\":%d},"
            "{\"d\":%d,\"st\":%d},"
            "{\"d\":%d,\"st\":%d}"
            "],\"err\":%.3f,\"out\":%.1f}",
            _results[0].Distance, _results[0].Status,
            _results[1].Distance, _results[1].Status,
            _results[2].Distance, _results[2].Status,
            _results[3].Distance, _results[3].Status,
            _results[4].Distance, _results[4].Status,
            _results[5].Distance, _results[5].Status,
            _results[6].Distance, _results[6].Status,
            *_error, *_output);
        req->send(200, "application/json", buf);
    });

    server.begin();
    vTaskDelete(NULL);
}

void startTuner(float *kp, float *kd, float *base_speed, VL53L1X_Result_t *results, float *error, float *output)
{
    _kp = kp;
    _kd = kd;
    _base_speed = base_speed;
    _results = results;
    _error = error;
    _output = output;
    xTaskCreatePinnedToCore(tunerTask, "tuner", 8192, NULL, 1, NULL, 0);
}
