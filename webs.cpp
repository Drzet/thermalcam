#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include "driver/jpeg_encode.h"

#include "ThermalModes.h"
#include "webs.h"

// ESP32-S3 hardware JPEG encoder wrapper for RGB565
static jpeg_encoder_handle_t jpeg_encoder = nullptr;

static bool encodeRGB565ToJPEG(uint16_t* rgb565, int w, int h, int quality, 
                                uint8_t** out_jpg, size_t* out_size) {
  // Initialize encoder on first use
  if (!jpeg_encoder) {
    jpeg_encode_engine_cfg_t enc_config = {
      .intr_priority = 0,
      .timeout_ms = 1000
    };
    esp_err_t ret = jpeg_new_encoder_engine(&enc_config, &jpeg_encoder);
    if (ret != ESP_OK) {
      Serial.printf("JPEG encoder init failed: %d\n", ret);
      return false;
    }
    Serial.println("JPEG encoder initialized (HW accelerated)");
  }

  // Allocate output buffer (estimate 1/10 of input size for quality 80)
  size_t outbuf_size = (w * h * 2) / 5;  // conservative estimate
  uint8_t* outbuf = (uint8_t*)ps_malloc(outbuf_size);
  if (!outbuf) {
    return false;
  }

  // Configure encoding
  jpeg_encode_cfg_t enc_cfg = {
    .height = (uint32_t)h,
    .width = (uint32_t)w,
    .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
    .sub_sample = JPEG_DOWN_SAMPLING_YUV420,  // standard 4:2:0 chroma subsampling
    .image_quality = (uint32_t)quality
  };

  uint32_t encoded_size = 0;
  esp_err_t ret = jpeg_encoder_process(
    jpeg_encoder,
    &enc_cfg,
    (const uint8_t*)rgb565,
    w * h * 2,  // RGB565 is 2 bytes per pixel
    outbuf,
    outbuf_size,
    &encoded_size
  );

  if (ret != ESP_OK || encoded_size == 0) {
    free(outbuf);
    return false;
  }

  *out_jpg = outbuf;
  *out_size = encoded_size;
  return true;
}

// Frame buffer and temperature range are defined in the main sketch
extern uint16_t* readyBuffer;  // buffer ready for encoding (double-buffered)
extern SemaphoreHandle_t frameReadySem;
extern SemaphoreHandle_t frameFreeSem;
extern float frameMin;
extern float frameMax;

static WebServer webServer(80);
static bool serverStarted = false;
static int g_viewW = 0;
static int g_viewH = 0;

static void handleSnapshot();
static void handleStream();
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
      <img id="stream" src="/stream" width="384" height="288" alt="Thermal stream">
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
          a.download = 'thermal_frame.jpg';
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

    setInterval(refreshStatus, 3000);
    window.addEventListener('load', function() {
      refreshStatus();
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
  webServer.on("/stream", []() { handleStream(); });
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
  if (!readyBuffer || g_viewW <= 0 || g_viewH <= 0) {
    webServer.send(500, "text/plain", "No frame ready");
    return;
  }

  // Try to get access to the ready buffer (non-blocking)
  if (xSemaphoreTake(frameReadySem, pdMS_TO_TICKS(100)) != pdTRUE) {
    webServer.send(503, "text/plain", "Frame busy, retry");
    return;
  }

  const int w = g_viewW;
  const int h = g_viewH;

  // Encode RGB565 to JPEG
  uint8_t* jpgBuf = nullptr;
  size_t jpgSize = 0;
  bool ok = encodeRGB565ToJPEG(
    readyBuffer, w, h,
    80,  // quality 0-100
    &jpgBuf, &jpgSize
  );

  // Release buffer for core 1 to continue
  xSemaphoreGive(frameFreeSem);

  if (!ok || !jpgBuf || jpgSize == 0) {
    webServer.send(500, "text/plain", "JPEG encode failed");
    return;
  }

  WiFiClient client = webServer.client();
  if (!client) {
    free(jpgBuf);
    return;
  }

  webServer.setContentLength(jpgSize);
  webServer.send(200, "image/jpeg", "");

  // Chunked send with yields to keep system responsive
  const size_t chunkSize = 2048;
  size_t sent = 0;
  while (sent < jpgSize) {
    size_t toSend = (jpgSize - sent > chunkSize) ? chunkSize : (jpgSize - sent);
    client.write(jpgBuf + sent, toSend);
    sent += toSend;
    delay(0);  // yield to other tasks
  }
  client.flush();

  free(jpgBuf);
}

static void handleStream() {
  if (!readyBuffer || g_viewW <= 0 || g_viewH <= 0) {
    webServer.send(500, "text/plain", "No frame ready");
    return;
  }

  WiFiClient client = webServer.client();
  if (!client) {
    return;
  }

  const int w = g_viewW;
  const int h = g_viewH;

  // Send multipart MJPEG header
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "multipart/x-mixed-replace; boundary=frame", "");

  while (client.connected()) {
    // Wait for a new frame to be ready
    if (xSemaphoreTake(frameReadySem, pdMS_TO_TICKS(200)) != pdTRUE) {
      continue;  // timeout, retry
    }

    // Encode to JPEG
    uint8_t* jpgBuf = nullptr;
    size_t jpgSize = 0;
    bool ok = encodeRGB565ToJPEG(
      readyBuffer, w, h,
      80,  // quality
      &jpgBuf, &jpgSize
    );

    // Release buffer immediately for core 1
    xSemaphoreGive(frameFreeSem);

    if (!ok || !jpgBuf || jpgSize == 0) {
      delay(10);
      continue;
    }

    // Send multipart boundary + headers
    client.print("--frame\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %u\r\n\r\n", jpgSize);

    // Send JPEG data in chunks with yields
    const size_t chunkSize = 2048;
    size_t sent = 0;
    while (sent < jpgSize && client.connected()) {
      size_t toSend = (jpgSize - sent > chunkSize) ? chunkSize : (jpgSize - sent);
      client.write(jpgBuf + sent, toSend);
      sent += toSend;
      delay(0);
    }

    client.print("\r\n");
    client.flush();

    free(jpgBuf);

    // Small delay between frames
    delay(10);
  }
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

