#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include "ThermalModes.h"
#include "webs.h"

// Frame buffer and temperature range are defined in the main sketch
extern uint16_t* frameBuffer;
extern SemaphoreHandle_t frameMutex;
extern float frameMin;
extern float frameMax;

static WebServer webServer(80);
static bool serverStarted = false;
static int g_viewW = 0;
static int g_viewH = 0;

static void handleSnapshot();
static void handleStatus();
static void handleModeApi();

// HTML page served at "/"
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ThermalCam</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { margin:0; background:#000; color:#fff; font-family:sans-serif; }
    #container { display:flex; flex-direction:row; gap:16px; padding:8px; box-sizing:border-box; }
    #left { flex:0 0 auto; }
    #right { flex:1 1 auto; display:flex; flex-direction:column; gap:8px; }
    #stream { max-width:100%; height:auto; border:1px solid #444; background:#111; }
    button { padding:8px 12px; font-size:14px; background:#333; color:#fff; border:1px solid #666; cursor:pointer; }
    button:hover { background:#555; }
    #status { font-size:13px; color:#ccc; }
    @media (max-width: 700px) {
      #container { flex-direction:column; }
    }
  </style>
</head>
<body>
  <div id="container">
    <div id="left">
      <img id="stream" src="/snapshot" width="384" height="288" alt="Thermal stream">
    </div>
    <div id="right">
      <div>
        <button onclick="saveFrame()">Save frame</button>
        <button onclick="changeMode()">Change mode</button>
      </div>
      <div id="status">Status: loading...</div>
    </div>
  </div>
  <script>
    function saveFrame() {
      fetch('/snapshot')
        .then(r => {
          if (!r.ok) throw new Error('Snapshot failed');
          return r.blob();
        })
        .then(blob => {
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          a.href = url;
          a.download = 'thermal_frame.bmp';
          document.body.appendChild(a);
          a.click();
          a.remove();
          URL.revokeObjectURL(url);
        })
        .catch(err => {
          console.error(err);
          setStatus('Snapshot error');
        });
    }

    function changeMode() {
      fetch('/api/mode?next=1')
        .then(r => {
          if (!r.ok) throw new Error('Mode change failed');
          setStatus('Mode changed');
        })
        .catch(err => {
          console.error(err);
          setStatus('Mode change error');
        });
    }

    function refreshStatus() {
      fetch('/api/status')
        .then(r => r.ok ? r.json() : null)
        .then(data => {
          if (!data) {
            setStatus('No status');
            return;
          }
          setStatus('Mode: ' + (data.mode || '?') +
                    ' | Min: ' + data.min + '\u00b0C' +
                    ' | Max: ' + data.max + '\u00b0C');
        })
        .catch(() => setStatus('Status error'));
    }

    function setStatus(text) {
      var el = document.getElementById('status');
      if (el) el.textContent = 'Status: ' + text;
    }

    function refreshStream() {
      var img = document.getElementById('stream');
      if (!img) return;
      img.src = '/snapshot?ts=' + Date.now();
    }

    setInterval(refreshStatus, 3000);
    setInterval(refreshStream, 200);
    window.addEventListener('load', function() {
      refreshStatus();
      refreshStream();
    });
  </script>
</body>
</html>
)rawliteral";

void websBegin(int viewW, int viewH) {
  if (serverStarted) {
    return;
  }

  g_viewW = viewW;
  g_viewH = viewH;

  webServer.on("/", []() {
    webServer.send_P(200, "text/html", INDEX_HTML);
  });

  webServer.on("/snapshot", []() { handleSnapshot(); });
  webServer.on("/api/status", []() { handleStatus(); });
  webServer.on("/api/mode", []() { handleModeApi(); });

  webServer.begin();
  serverStarted = true;
  Serial.println("HTTP server started on port 80");
}

void websHandle() {
  if (!serverStarted) return;
  webServer.handleClient();
}

static void handleSnapshot() {
  if (!frameBuffer || g_viewW <= 0 || g_viewH <= 0) {
    webServer.send(500, "text/plain", "No frame buffer");
    return;
  }

  const int w = g_viewW;
  const int h = g_viewH;
  const int bytesPerPixel = 3; // 24-bit BMP
  const int rowSize = ((bytesPerPixel * w + 3) / 4) * 4; // 4-byte aligned
  const uint32_t dataSize = rowSize * h;
  const uint32_t fileSize = 54 + dataSize;

  uint8_t* bmp = (uint8_t*)ps_malloc(fileSize);
  if (!bmp) {
    webServer.send(500, "text/plain", "No memory for BMP");
    return;
  }

  if (xSemaphoreTake(frameMutex, portMAX_DELAY) != pdTRUE) {
    free(bmp);
    webServer.send(503, "text/plain", "Frame wait failed");
    return;
  }

  // BMP header
  bmp[0] = 'B'; bmp[1] = 'M';
  bmp[2] = (uint8_t)(fileSize & 0xFF);
  bmp[3] = (uint8_t)((fileSize >> 8) & 0xFF);
  bmp[4] = (uint8_t)((fileSize >> 16) & 0xFF);
  bmp[5] = (uint8_t)((fileSize >> 24) & 0xFF);
  bmp[6] = bmp[7] = bmp[8] = bmp[9] = 0;
  const uint32_t dataOffset = 54;
  bmp[10] = (uint8_t)(dataOffset & 0xFF);
  bmp[11] = (uint8_t)((dataOffset >> 8) & 0xFF);
  bmp[12] = (uint8_t)((dataOffset >> 16) & 0xFF);
  bmp[13] = (uint8_t)((dataOffset >> 24) & 0xFF);

  // DIB header (BITMAPINFOHEADER)
  const uint32_t dibSize = 40;
  bmp[14] = (uint8_t)(dibSize & 0xFF);
  bmp[15] = (uint8_t)((dibSize >> 8) & 0xFF);
  bmp[16] = (uint8_t)((dibSize >> 16) & 0xFF);
  bmp[17] = (uint8_t)((dibSize >> 24) & 0xFF);

  bmp[18] = (uint8_t)(w & 0xFF);
  bmp[19] = (uint8_t)((w >> 8) & 0xFF);
  bmp[20] = (uint8_t)((w >> 16) & 0xFF);
  bmp[21] = (uint8_t)((w >> 24) & 0xFF);

  bmp[22] = (uint8_t)(h & 0xFF);
  bmp[23] = (uint8_t)((h >> 8) & 0xFF);
  bmp[24] = (uint8_t)((h >> 16) & 0xFF);
  bmp[25] = (uint8_t)((h >> 24) & 0xFF);

  bmp[26] = 1; bmp[27] = 0;          // planes
  bmp[28] = 24; bmp[29] = 0;         // bits per pixel
  bmp[30] = 0; bmp[31] = bmp[32] = bmp[33] = 0;  // compression = BI_RGB

  bmp[34] = (uint8_t)(dataSize & 0xFF);
  bmp[35] = (uint8_t)((dataSize >> 8) & 0xFF);
  bmp[36] = (uint8_t)((dataSize >> 16) & 0xFF);
  bmp[37] = (uint8_t)((dataSize >> 24) & 0xFF);

  // pixels per meter (dummy)
  bmp[38] = bmp[39] = bmp[40] = bmp[41] = 0;
  bmp[42] = bmp[43] = bmp[44] = bmp[45] = 0;

  // colors used / important (0 = all)
  bmp[46] = bmp[47] = bmp[48] = bmp[49] = 0;
  bmp[50] = bmp[51] = bmp[52] = bmp[53] = 0;

  // Pixel data: bottom-up BGR24
  uint8_t* pixelBase = bmp + 54;
  for (int y = 0; y < h; ++y) {
    int srcY = h - 1 - y; // BMP bottom row first
    uint8_t* rowPtr = pixelBase + y * rowSize;

    for (int x = 0; x < w; ++x) {
      uint16_t pix = frameBuffer[srcY * w + x];
      uint8_t r = ((pix >> 11) & 0x1F) * 255 / 31;
      uint8_t g = ((pix >> 5) & 0x3F) * 255 / 63;
      uint8_t b = (pix & 0x1F) * 255 / 31;
      rowPtr[x * 3 + 0] = b;
      rowPtr[x * 3 + 1] = g;
      rowPtr[x * 3 + 2] = r;
    }
    for (int p = w * bytesPerPixel; p < rowSize; ++p) {
      rowPtr[p] = 0;
    }
  }

  xSemaphoreGive(frameMutex);

  WiFiClient client = webServer.client();
  if (!client) {
    free(bmp);
    return;
  }

  webServer.setContentLength(fileSize);
  webServer.send(200, "image/bmp", "");
  client.write(bmp, fileSize);
  client.flush();

  free(bmp);
}

static void handleStatus() {
  float minTemp, maxTemp;
  getTemperatureRange(minTemp, maxTemp, frameMin, frameMax);

  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"mode\":\"%s\",\"min\":%.1f,\"max\":%.1f}",
           getModeName(), minTemp, maxTemp);

  webServer.send(200, "application/json", buf);
}

static void handleModeApi() {
  if (webServer.hasArg("next") && webServer.arg("next") == "1") {
    ThermalMode mode = getCurrentMode();
    int next = ((int)mode + 1) % NUM_MODES;
    currentMode = (ThermalMode)next;
  }
  webServer.send(200, "text/plain", "OK");
}

