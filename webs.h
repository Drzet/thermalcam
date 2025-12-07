#ifndef WEBS_H
#define WEBS_H

// Initialize and start the HTTP server once WiFi is up.
// viewW/viewH are the dimensions of the RGB565 frame buffer.
void websBegin(int viewW, int viewH);

// Pump the HTTP server; call from loop().
void websHandle();

#endif // WEBS_H

