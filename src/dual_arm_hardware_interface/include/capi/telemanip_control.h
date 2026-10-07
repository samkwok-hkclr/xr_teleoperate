/*
 * telemanip_control.h — C ABI of the telemanip control side (shared layer).
 *
 * One `.so` speaks exactly one vendor ("piper", "realman", or "cuarm_2_2" —
 * ask tms_abi_vendor()). This header is the vendor-neutral half: the session,
 * the command staging / beat API, the state / status callbacks and the envelope
 * read models. Each vendor adds its own command builders and its own decoded
 * `JointState` in `telemanip_control_<vendor>.h`, which includes this file.
 *
 * Build / link:
 *   cc app.c -I <include dirs> -L <lib dir> -ltelemanip_capi_piper
 *   (or -ltelemanip_capi_realman / -ltelemanip_capi_cuarm_2_2; a static build
 *   can use the shipped .a with -lpthread -ldl -lm)
 *
 * ---------------------------------------------------------------------------
 * The contract, in one place
 * ---------------------------------------------------------------------------
 *
 * Errors
 *   Every function returning `int` returns a `TmsStatus` code: 0 (TMS_OK) or a
 *   negative error. `tms_last_error()` returns the message of the last failing
 *   call ON THIS THREAD (empty string when there is none). The pointer is owned
 *   by the library, is valid until the next failing call on the same thread,
 *   and is never NULL — do not free it.
 *
 *   A Rust panic inside the library never unwinds into your code: it becomes
 *   TMS_ERR_PANIC with a message. Treat the session as degraded after one.
 *
 * Out-parameters
 *   Every out-parameter is written (zeroed / NULLed) before the call can fail,
 *   so a caller that ignores the return code never reads uninitialised memory.
 *   A NULL out-parameter is TMS_ERR_INVALID_ARG.
 *
 * Strings and buffers
 *   `const char *` inputs must be NUL-terminated UTF-8. `(ptr, len)` inputs
 *   accept `ptr == NULL` only when `len == 0`. In every read model this header
 *   hands out, the same rule holds in reverse: a `(ptr, len)` pair is NULL
 *   exactly when its length is 0, and the `const char *` fields are
 *   NUL-terminated and never NULL.
 *
 * Ownership
 *   `tms_*_free` releases a read model: envelope, cell config, vendor joint
 *   state. It releases the whole thing — every pointer inside it points into
 *   buffers that belong to the read model, so do not free them separately and
 *   do not keep them after the free. `free(NULL)` is a no-op.
 *
 *   The `info` a TmsStreamCallback receives is the ONE exception: it is
 *   borrowed for the duration of the call and released when the callback
 *   returns. Do not free it, do not store the pointer, and copy out anything
 *   you want to keep.
 *
 * Threads
 *   A TmsSession may be used from several threads at once (a control loop on
 *   one, a monitor on another): every call is internally synchronised. The two
 *   exceptions are the ones an ABI cannot defend: `tms_session_destroy` must be
 *   the last call naming the handle, and it must not race with another call.
 *
 *   TmsStreamCallback runs on a background task of the session's own runtime,
 *   never on the thread that registered it, and it may still be running when
 *   you clear it. `user_data` must therefore stay valid (and be safe to touch
 *   from that thread) until the callback is cleared and you know it has
 *   returned — a mutex or an atomic flag in your own code is the usual way.
 *
 * Staging vs sending
 *   `tms_session_stage` and the vendor command helpers never send: the
 *   session's period beat is the ONLY sender (one envelope per `period_ms`,
 *   carrying the newest staged target of every device). Re-staging between two
 *   beats overwrites (latest wins); not re-staging holds the last target (pose
 *   hold). The period must equal the agent's — checked at open.
 */

#ifndef TELEMANIP_CONTROL_H
#define TELEMANIP_CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* C11 / C++ static assertion, or nothing on a compiler too old to have one. */
#if defined(__cplusplus)
#define TMS_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define TMS_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#else
#define TMS_STATIC_ASSERT(cond, msg)
#endif

/* ------------------------------------------------------------------------- */
/* Status codes                                                              */
/* ------------------------------------------------------------------------- */

/*
 * The whole error vocabulary. The codes are for branching, tms_last_error() is
 * for the log.
 */
typedef enum TmsStatus {
    TMS_OK = 0,
    /* A NULL pointer where one is required, or a value out of range. */
    TMS_ERR_INVALID_ARG = -1,
    /* A const char * argument is not valid UTF-8 (or has an interior NUL). */
    TMS_ERR_NOT_UTF8 = -2,
    /* The output buffer is too small; *out_len holds what is needed. */
    TMS_ERR_BUFFER_SMALL = -3,
    /* The configuration was rejected, or the device handshake failed. */
    TMS_ERR_CONFIG = -4,
    /* No agent liveliness token appeared within agent_wait_ms. */
    TMS_ERR_AGENT_TIMEOUT = -5,
    /* The vendor payload builder rejected the arguments (shape / range). */
    TMS_ERR_VENDOR_REJECT = -6,
    /* The transport failed (e.g. the CellConfig query timed out). */
    TMS_ERR_TRANSPORT = -7,
    /* A Rust panic was caught at the boundary. The session is degraded. */
    TMS_ERR_PANIC = -8
} TmsStatus;

TMS_STATIC_ASSERT(sizeof(TmsStatus) == sizeof(int32_t), "TmsStatus must be an int32_t");

/* Message of the last failing call on this thread; never NULL, empty when
 * there is none. Owned by the library; do not free. */
const char *tms_last_error(void);

/* Version of this C ABI (the crate version, e.g. "0.1.0"), and which vendor
 * this library speaks ("piper" / "realman" / "cuarm_2_2"). Both are literals
 * owned by the library, never NULL. */
const char *tms_version(void);
const char *tms_abi_vendor(void);

/* ------------------------------------------------------------------------- */
/* Configuration                                                             */
/* ------------------------------------------------------------------------- */

/*
 * CONTROL.md ControlConfig mirror. Fill it in on your own stack:
 *
 *     TmsControlConfig cfg;
 *     const char *devices[] = {"arm1"};
 *     tms_control_config_init(&cfg);
 *     cfg.cell_id = "cell1";
 *     cfg.device_ids = devices;
 *     cfg.device_count = 1;
 *     cfg.zenoh_config = "config/zenoh.json5";   // NULL -> zenoh defaults
 *
 * Every field is read by tms_session_create; the strings stay yours and only
 * have to live for that call.
 */
typedef struct TmsControlConfig {
    const char *cell_id;         /* required */
    const char *zenoh_config;    /* NULL -> zenoh defaults */
    const char *const *device_ids; /* required, device_count entries */
    size_t device_count;
    uint64_t period_ms;          /* 2..=50, must equal the agent's */
    size_t max_tick_bytes;       /* 256..=1200 */
    size_t max_tick_items;       /* 1..=16 */
    const char *sender_id;       /* NULL -> "ctrl-1" */
    uint64_t session_id;         /* 0 -> random at open */
    uint64_t agent_wait_ms;      /* 100..=300000 */
} TmsControlConfig;

TMS_STATIC_ASSERT(sizeof(TmsControlConfig) == 80, "TmsControlConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsControlConfig, cell_id) == 0, "TmsControlConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsControlConfig, zenoh_config) == 8, "TmsControlConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsControlConfig, device_ids) == 16, "TmsControlConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsControlConfig, device_count) == 24, "TmsControlConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsControlConfig, period_ms) == 32, "TmsControlConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsControlConfig, max_tick_bytes) == 40, "TmsControlConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsControlConfig, max_tick_items) == 48, "TmsControlConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsControlConfig, sender_id) == 56, "TmsControlConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsControlConfig, session_id) == 64, "TmsControlConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsControlConfig, agent_wait_ms) == 72, "TmsControlConfig layout");

/* Write the defaults (period_ms 10, limits 1024 / 16, sender_id "ctrl-1",
 * agent_wait_ms 30000) and NULL the rest. cell_id and device_ids are
 * deliberately left NULL: defaulting them would turn a forgotten field into a
 * silently wrong session. */
void tms_control_config_init(TmsControlConfig *cfg);

/* ------------------------------------------------------------------------- */
/* Envelope read models                                                      */
/* ------------------------------------------------------------------------- */

/* One accepted item of an envelope. `payload` is the vendor's bytes as they
 * arrived — decode them with the vendor's own codec. */
typedef struct TmsItemInfo {
    uint32_t vendor;         /* 0x0001 RealMan, 0x0002 Piper */
    uint32_t type;           /* message id within that vendor namespace */
    const char *device_id;
    const uint8_t *payload;  /* NULL when payload_len == 0 */
    size_t payload_len;
} TmsItemInfo;

TMS_STATIC_ASSERT(sizeof(TmsItemInfo) == 32, "TmsItemInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsItemInfo, vendor) == 0, "TmsItemInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsItemInfo, type) == 4, "TmsItemInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsItemInfo, device_id) == 8, "TmsItemInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsItemInfo, payload) == 16, "TmsItemInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsItemInfo, payload_len) == 24, "TmsItemInfo layout");

/* One received Envelope: what a state / status callback takes, and what
 * tms_session_query_config() returns. */
typedef struct TmsEnvelopeInfo {
    const char *cell_id;
    const char *sender_id;
    uint64_t session_id;
    uint64_t tick;          /* sender's cmd tick */
    uint64_t timestamp_us;  /* sender wall clock */
    uint32_t stream;        /* transport stream id */
    const TmsItemInfo *items; /* NULL when item_count == 0 */
    size_t item_count;
} TmsEnvelopeInfo;

TMS_STATIC_ASSERT(sizeof(TmsEnvelopeInfo) == 64, "TmsEnvelopeInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsEnvelopeInfo, cell_id) == 0, "TmsEnvelopeInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsEnvelopeInfo, sender_id) == 8, "TmsEnvelopeInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsEnvelopeInfo, session_id) == 16, "TmsEnvelopeInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsEnvelopeInfo, tick) == 24, "TmsEnvelopeInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsEnvelopeInfo, timestamp_us) == 32, "TmsEnvelopeInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsEnvelopeInfo, stream) == 40, "TmsEnvelopeInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsEnvelopeInfo, items) == 48, "TmsEnvelopeInfo layout");
TMS_STATIC_ASSERT(offsetof(TmsEnvelopeInfo, item_count) == 56, "TmsEnvelopeInfo layout");

/* State / status callback: info is borrowed for the call only (see the header
 * comment); user_data is whatever was registered with it. */
typedef void (*TmsStreamCallback)(const TmsEnvelopeInfo *info, void *user_data);

/* Release an envelope handed out by a tms_* call. NULL is a no-op. Never call
 * this on the info a callback receives. */
void tms_envelope_free(TmsEnvelopeInfo *env);

/* ------------------------------------------------------------------------- */
/* Session                                                                   */
/* ------------------------------------------------------------------------- */

/* Opaque: only pointers handed out by tms_session_create are ever used. */
typedef struct TmsSession TmsSession;

/*
 * Open a session: connect, wait up to cfg->agent_wait_ms for the agent's
 * liveliness token, query its CellConfig, and refuse to return unless every
 * cfg->device_ids name is on the agent and its period_ms matches. Blocks for
 * as long as the wait allows.
 *
 * On success *out owns a session (tms_session_destroy). On failure it is left
 * NULL: TMS_ERR_CONFIG for a rejected config or a failed handshake (including
 * the period mismatch), TMS_ERR_AGENT_TIMEOUT when no agent showed up.
 */
int tms_session_create(const TmsControlConfig *cfg, TmsSession **out);

/*
 * Abort the background tasks and free the handle. NULL is a no-op. This must be
 * the last call naming the handle and must not race with another call; a
 * callback that is running when it is called still finishes.
 */
void tms_session_destroy(TmsSession *session);

/* True while an agent is alive on the cell (zenoh liveliness token). */
int tms_session_agent_online(TmsSession *session, bool *out);

/* How many cmd beats the session has sent. A monitoring aid: it advances even
 * when nothing is staged. */
int tms_session_tick(TmsSession *session, uint64_t *out);

/*
 * Stage one already-encoded device target. Prefer the vendor helpers
 * (tms_piper_joint_cmd / tms_realman_movej_canfd / tms_cuarm_2_2_joint_cmd),
 * which encode and stage in one call; this is the raw form for a payload you
 * built yourself.
 *
 * Returns a TmsStatus about the CALL. *out_staged is the library's verdict on
 * the target: 0 staged, 1 locally rejected (vendor is not this library's
 * vendor / device_id was not in the handshake set / type is not a registered
 * vendor command / empty payload).
 */
int tms_session_stage(TmsSession *session, const char *device_id, uint32_t vendor,
                      uint32_t type, const uint8_t *payload, size_t payload_len,
                      int32_t *out_staged);

/* Register the state-stream callback; cb == NULL clears it. */
int tms_session_on_state(TmsSession *session, TmsStreamCallback cb, void *user_data);

/* Register the status-stream callback; cb == NULL clears it. */
int tms_session_on_status(TmsSession *session, TmsStreamCallback cb, void *user_data);

/*
 * Query the agent's CellConfig and return the answering envelope: its first
 * item's payload is the CellConfig protobuf, which
 * tms_decode_cell_config() turns into a TmsCellConfig. On success *out owns an
 * envelope (tms_envelope_free).
 */
int tms_session_query_config(TmsSession *session, TmsEnvelopeInfo **out);

/*
 * Abort the background tasks (beat ticker, subscriptions). The handle stays
 * valid and destroying it is still required; the zenoh session itself closes
 * when the transport's last reference goes.
 */
int tms_session_close(TmsSession *session);

/* ------------------------------------------------------------------------- */
/* CellConfig                                                                */
/* ------------------------------------------------------------------------- */

/* The agent's CellConfig, as served on its config queryable. */
typedef struct TmsCellConfig {
    const char *cell_id;
    uint32_t period_ms;       /* must equal the control's */
    uint32_t state_period_ms; /* 0 = state reporting off */
    uint32_t status_hz;
    uint32_t watchdog_mult;   /* watchdog window = watchdog_mult x period_ms */
    uint32_t max_tick_bytes;
    uint32_t max_tick_items;
    const char *const *device_ids; /* NULL when device_count == 0 */
    size_t device_count;
} TmsCellConfig;

TMS_STATIC_ASSERT(sizeof(TmsCellConfig) == 48, "TmsCellConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsCellConfig, cell_id) == 0, "TmsCellConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsCellConfig, period_ms) == 8, "TmsCellConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsCellConfig, state_period_ms) == 12, "TmsCellConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsCellConfig, status_hz) == 16, "TmsCellConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsCellConfig, watchdog_mult) == 20, "TmsCellConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsCellConfig, max_tick_bytes) == 24, "TmsCellConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsCellConfig, max_tick_items) == 28, "TmsCellConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsCellConfig, device_ids) == 32, "TmsCellConfig layout");
TMS_STATIC_ASSERT(offsetof(TmsCellConfig, device_count) == 40, "TmsCellConfig layout");

/* Decode a CellConfig payload (the first item of a query_config envelope). On
 * success *out owns a config (tms_cell_config_free); on failure it is NULL and
 * the code is TMS_ERR_INVALID_ARG (the bytes are not a CellConfig). */
int tms_decode_cell_config(const uint8_t *payload, size_t payload_len, TmsCellConfig **out);

/* Release a CellConfig. NULL is a no-op. */
void tms_cell_config_free(TmsCellConfig *cfg);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TELEMANIP_CONTROL_H */
