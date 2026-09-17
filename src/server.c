// server.c

#include "server.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "external/mongoose.h"

#define SLOTS      2
#define SLOT_CAP   (1u << 20)   /* 1 MB per slot, far above worst case */
#define STATE_CAP  4096

typedef struct {
    uint8_t *buf;
    size_t   len;
    int      ready;
} slot_t;

static struct {
    server_cfg_t     cfg;
    pthread_t        thread;
    volatile int     quit;
    pthread_mutex_t  mtx;
    slot_t           slots[SLOTS];
    volatile int     state_dirty;
} S;

static void broadcast_state_locked(struct mg_mgr *mgr)
{
    char json[STATE_CAP];
    json[0] = 0;
    S.cfg.state_json(json, sizeof(json), S.cfg.user);
    size_t n = strlen(json);
    if (!n) return;
    for (struct mg_connection *c = mgr->conns; c != NULL; c = c->next)
        if (c->is_websocket)
            mg_ws_send(c, json, n, WEBSOCKET_OP_TEXT);
}

static void ev_handler(struct mg_connection *c, int ev, void *ev_data)
{
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message *hm = (struct mg_http_message *)ev_data;
        if (mg_match(hm->uri, mg_str("/ws"), NULL)) {
            mg_ws_upgrade(c, hm, NULL);
        } else {
            struct mg_http_serve_opts opts = { .root_dir = S.cfg.web_root };
            mg_http_serve_dir(c, ev_data, &opts);
        }
    } else if (ev == MG_EV_WS_OPEN) {
        /* Greet the new client with the current state. */
        char json[STATE_CAP];
        json[0] = 0;
        S.cfg.state_json(json, sizeof(json), S.cfg.user);
        if (json[0]) mg_ws_send(c, json, strlen(json), WEBSOCKET_OP_TEXT);
    } else if (ev == MG_EV_WS_MSG) {
        struct mg_ws_message *wm = (struct mg_ws_message *)ev_data;
        if (S.cfg.on_control &&
            S.cfg.on_control(wm->data.buf, wm->data.len, S.cfg.user))
            S.state_dirty = 1;
    }
}

static void *server_thread(void *arg)
{
    (void)arg;
    struct mg_mgr mgr;
    mg_mgr_init(&mgr);
    if (mg_http_listen(&mgr, S.cfg.listen_url, ev_handler, NULL) == NULL) {
        fprintf(stderr, "[web] failed to listen on %s\n", S.cfg.listen_url);
        return NULL;
    }
    printf("[web] listening on %s (root %s)\n", S.cfg.listen_url, S.cfg.web_root);

    while (!S.quit) {
        mg_mgr_poll(&mgr, 5);

        pthread_mutex_lock(&S.mtx);
        for (struct mg_connection *c = mgr.conns; c != NULL; c = c->next) {
            if (!c->is_websocket) continue;
            /* Anti-bufferbloat: judge backlog before queueing this poll's
             * frames, so a big points frame can't starve the small
             * spectrum frame queued right behind it. */
            size_t backlog = c->send.len;
            for (int i = 0; i < SLOTS; ++i) {
                slot_t *s = &S.slots[i];
                if (!s->ready) continue;
                if (backlog > s->len) continue;  /* client is behind, drop */
                mg_ws_send(c, s->buf, s->len, WEBSOCKET_OP_BINARY);
            }
        }
        for (int i = 0; i < SLOTS; ++i) S.slots[i].ready = 0;
        int dirty = S.state_dirty;
        S.state_dirty = 0;
        pthread_mutex_unlock(&S.mtx);

        if (dirty) broadcast_state_locked(&mgr);
    }

    mg_mgr_free(&mgr);
    return NULL;
}

int server_start(const server_cfg_t *cfg)
{
    memset(&S, 0, sizeof(S));
    S.cfg = *cfg;
    pthread_mutex_init(&S.mtx, NULL);
    for (int i = 0; i < SLOTS; ++i) {
        S.slots[i].buf = malloc(SLOT_CAP);
        if (!S.slots[i].buf) return -1;
    }
    return pthread_create(&S.thread, NULL, server_thread, NULL);
}

void server_stop(void)
{
    S.quit = 1;
    pthread_join(S.thread, NULL);
    for (int i = 0; i < SLOTS; ++i) free(S.slots[i].buf);
    pthread_mutex_destroy(&S.mtx);
}

void server_publish(int slot, const void *data, size_t len)
{
    if (slot < 0 || slot >= SLOTS || len > SLOT_CAP) return;
    pthread_mutex_lock(&S.mtx);
    memcpy(S.slots[slot].buf, data, len);
    S.slots[slot].len = len;
    S.slots[slot].ready = 1;
    pthread_mutex_unlock(&S.mtx);
}

void server_broadcast_state(void)
{
    S.state_dirty = 1;
}
