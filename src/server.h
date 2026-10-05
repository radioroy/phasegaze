// server.h
// Mongoose HTTP static file server + WebSocket stream for the VRF visualizer.
// One background thread. Frames are handed over via double buffers so the
// DSP worker never blocks on the network.

#ifndef SERVER_H
#define SERVER_H

#include <stddef.h>

typedef struct {
    const char *listen_url;   /* e.g. "http://0.0.0.0:8002" */
    const char *web_root;     /* static files directory */

    /* Called from the server thread when a client sends a text message.
     * Return 1 if the state changed (state JSON is then re-broadcast). */
    int  (*on_control)(const char *msg, size_t len, void *user);

    /* Fill buf with the current state JSON. */
    void (*state_json)(char *buf, size_t cap, void *user);

    void *user;
} server_cfg_t;

int  server_start(const server_cfg_t *cfg);
void server_stop(void);

/* Queue a binary frame for broadcast (copies data).
 * slot 0 points, slot 1 spectrum, slot 2 one JPEG.
 * An unsent point frame keeps its hits and appends the next sweep of the
 * same plan; the packet goes out on the next poll that finds a client
 * keeping up. Spectrum and video stay latest-wins. */
#define SERVER_SLOT_POINTS   0
#define SERVER_SLOT_SPECTRUM 1
#define SERVER_SLOT_VIDEO    2

void server_publish(int slot, const void *data, size_t len);

/* Ask the server thread to push state JSON to everyone. */
void server_broadcast_state(void);

#endif /* SERVER_H */
