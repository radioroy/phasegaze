// server.c

#include "server.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "external/mongoose.h"
#include "pg_stream.h"

#define SLOTS      2
#define POINT_SLOT 0
#define SLOT_CAP   (1u << 20)   /* 1 MB per slot, far above worst case */
#define STATE_CAP  4096
/* 16384 points is 256 KiB. One hop is ~0.44 ms and top-K is 512 points, so
 * a 5 ms poll holds ~6k points. Past this the oldest hits are dropped. */
#define MAX_COALESCED_POINTS 16384

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
        int nws = 0;
        int sent[SLOTS] = {0};
        for (struct mg_connection *c = mgr.conns; c != NULL; c = c->next) {
            if (!c->is_websocket) continue;
            nws++;
            /* Anti-bufferbloat: judge backlog before queueing this poll's
             * frames, so a big points frame can't starve the small
             * spectrum frame queued right behind it. */
            size_t backlog = c->send.len;
            for (int i = 0; i < SLOTS; ++i) {
                slot_t *s = &S.slots[i];
                if (!s->ready) continue;
                if (backlog > s->len) continue;  /* client is behind, drop */
                mg_ws_send(c, s->buf, s->len, WEBSOCKET_OP_BINARY);
                sent[i] = 1;
            }
        }
        /* Nobody listening: drop the pile. A client that took the packet
         * starts a fresh one. A client that is behind keeps the slot so
         * later sweeps append instead of replacing what it has not seen. */
        for (int i = 0; i < SLOTS; ++i)
            if (nws == 0 || sent[i]) S.slots[i].ready = 0;
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
    /* mongoose defaults to MG_LL_DEBUG, which logs every WS write to journald. */
    mg_log_set(MG_LL_ERROR);
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

static int point_frame_ok(const void *data, size_t len, const pg_hdr_t **out)
{
    if (len < sizeof(pg_hdr_t)) return 0;
    const pg_hdr_t *h = (const pg_hdr_t *)data;
    if (h->magic != PG_MAGIC || h->type != PG_FRAME_POINTS) return 0;
    size_t need = sizeof(pg_hdr_t) + (size_t)h->count * sizeof(pg_point_t);
    if (need != len) return 0;
    *out = h;
    return 1;
}

/* S.mtx held. Append this sweep to an unsent packet of the same plan.
 * lo_start/lo_end are the plan keep-band, so one 20 MHz channel, a few
 * separated Wi-Fi bands, and a wide sweep all fold together. A retune
 * changes that span and starts a new packet. */
static void slot_store_points(const void *data, size_t len)
{
    const pg_hdr_t *in = NULL;
    slot_t *s = &S.slots[POINT_SLOT];
    if (!point_frame_ok(data, len, &in)) {
        memcpy(s->buf, data, len);
        s->len = len;
        s->ready = 1;
        return;
    }

    pg_hdr_t *h = (pg_hdr_t *)s->buf;
    int same = s->ready &&
               s->len == sizeof(pg_hdr_t) + (size_t)h->count * sizeof(pg_point_t) &&
               h->magic == PG_MAGIC && h->type == PG_FRAME_POINTS &&
               h->lo_start == in->lo_start && h->lo_end == in->lo_end &&
               h->count <= MAX_COALESCED_POINTS;

    if (!same) {
        memcpy(s->buf, data, len);
        ((pg_hdr_t *)s->buf)->reserved = 1;
        s->len = len;
        s->ready = 1;
        return;
    }

    uint32_t have = h->count;
    uint32_t add = in->count;
    if (add > MAX_COALESCED_POINTS) {
        uint32_t skip = add - MAX_COALESCED_POINTS;
        const uint8_t *src = (const uint8_t *)data;
        memcpy(s->buf, data, sizeof(pg_hdr_t));
        memcpy(s->buf + sizeof(pg_hdr_t),
               src + sizeof(pg_hdr_t) + skip * sizeof(pg_point_t),
               MAX_COALESCED_POINTS * sizeof(pg_point_t));
        h = (pg_hdr_t *)s->buf;
        h->count = MAX_COALESCED_POINTS;
        h->reserved = 1;
        s->len = sizeof(pg_hdr_t) + MAX_COALESCED_POINTS * sizeof(pg_point_t);
        s->ready = 1;
        return;
    }

    if (have + add > MAX_COALESCED_POINTS) {
        uint32_t drop = have + add - MAX_COALESCED_POINTS;
        uint32_t keep = have - drop;
        memmove(s->buf + sizeof(pg_hdr_t),
                s->buf + sizeof(pg_hdr_t) + drop * sizeof(pg_point_t),
                keep * sizeof(pg_point_t));
        have = keep;
    }

    memcpy(s->buf + sizeof(pg_hdr_t) + have * sizeof(pg_point_t),
           (const uint8_t *)data + sizeof(pg_hdr_t),
           add * sizeof(pg_point_t));
    h->fps = in->fps;
    h->seq = in->seq;
    h->count = have + add;
    h->reserved += 1;
    s->len = sizeof(pg_hdr_t) + (size_t)h->count * sizeof(pg_point_t);
    s->ready = 1;
}

void server_publish(int slot, const void *data, size_t len)
{
    if (slot < 0 || slot >= SLOTS || len > SLOT_CAP || !data) return;
    pthread_mutex_lock(&S.mtx);
    if (slot == POINT_SLOT)
        slot_store_points(data, len);
    else {
        memcpy(S.slots[slot].buf, data, len);
        S.slots[slot].len = len;
        S.slots[slot].ready = 1;
    }
    pthread_mutex_unlock(&S.mtx);
}

void server_broadcast_state(void)
{
    S.state_dirty = 1;
}
