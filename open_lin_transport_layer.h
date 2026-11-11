/**
 * @file open_lin_transport_layer.h
 * @brief LIN transport layer definition
 */
#ifndef OPEN_LIN_TRANSPORT_LAYER_H_
#define OPEN_LIN_TRANSPORT_LAYER_H_

#include "open_lin_cfg.h" /* defines OPEN_LIN_TRANSPORT_LAYER; must be visible here regardless of
                             what the including translation unit pulled in beforehand */
#include "open_lin_types.h"
#include "open_lin_network_layer.h"

#ifdef OPEN_LIN_TRANSPORT_LAYER

typedef l_u8 open_lin_NAD_t;
typedef l_u8 open_lin_SID_t;

typedef enum {
	OPEN_LIN_SID_ASSIGN_NAD = 0xB0,
	OPEN_LIN_SID_ASSIGN_FRAME_ID = 0xB1, /**< LIN 2.0 compatibility, Section 2.5.1 - superseded by Assign Frame ID Range (0xB7) in 2.1+ */
	OPEN_LIN_SID_READ_BY_ID = 0xB2,
	OPEN_LIN_SID_CONDITIONAL_CHANGE_NAD = 0xB3,
	OPEN_LIN_SID_DUMP_DATA = 0xB4,
	OPEN_LIN_SID_SAVED_CONFIGURATION = 0xB6,
	OPEN_LIN_SID_ASSIGN_FRAME_ID_RANGE = 0xB7
}open_lin_SID_enum_t;

typedef struct {
	l_u8 additional: 4;
	l_u8 PCI_type : 4;
} open_lin_PCI_t;

typedef enum {
	OPEN_LIN_PCI_SF = 0,
	OPEN_LIN_PCI_FF = 1,
	OPEN_LIN_PCI_CF = 2
} t_open_lin_pci_type;

typedef enum {
	OPEN_LIN_NAD_SLEEP = 0x00,
	OPEN_LIN_NAD_FUNCTIONAL = 0x7E,
	OPEN_LIN_NAD_BROADCAST = 0x7F
} t_open_lin_nad_type;

typedef struct {
	open_lin_NAD_t nad;
	open_lin_PCI_t pci;
	open_lin_SID_t sid;
	l_u8 data[5];
} t_open_lin_sf_request;

typedef struct {
	open_lin_NAD_t nad;
	open_lin_PCI_t pci;
	l_u8 rsid;
	l_u8 data[5];
} t_open_lin_sf_response;

/**
 * @brief Full transport layer (multi-frame) support, LIN-Spec 2.2A Chapter 3.
 *
 * OPEN_LIN_TP_MAX_MSG_LEN bounds the reassembled/segmented message size
 * (SID/RSID + data). The spec allows up to 4095 bytes; this is intentionally
 * smaller for embedded targets - raise it if a service needs more.
 */
#define OPEN_LIN_TP_MAX_MSG_LEN     64u
#define OPEN_LIN_TP_N_AS_TIMEOUT_US 1000000u /* Table 3.2 */
#define OPEN_LIN_TP_N_CR_TIMEOUT_US 1000000u /* Table 3.2 */

/**
 * @brief Receive (reassembly) context for one in-progress transport layer message.
 */
typedef struct {
	l_bool active;                          /**< true while a FF/CF reassembly is in progress */
	l_u8   buffer[OPEN_LIN_TP_MAX_MSG_LEN];  /**< SID + data, concatenated across SF or FF+CF */
	l_u16  total_len;                        /**< total message length (incl. SID), from SF/FF */
	l_u16  received_len;                     /**< bytes received so far */
	l_u8   next_sn;                          /**< expected next CF sequence number (1..15,0,1,...) */
	l_u32  last_frame_time_us;               /**< timestamp of the last accepted frame, for N_Cr */
} t_open_lin_tp_rx_ctx;

/**
 * @brief Transmit (segmentation) context for one pending transport layer message.
 */
typedef struct {
	l_bool active;         /**< true while there is more of `msg` left to send */
	const l_u8 *msg;        /**< SID/RSID + data to send, owned by the caller */
	l_u16  total_len;
	l_u16  sent_len;
	l_u8   next_sn;
} t_open_lin_tp_tx_ctx;

void   open_lin_tp_rx_init(t_open_lin_tp_rx_ctx *ctx);

/**
 * @brief Feed one already-checksum-validated raw 8-byte diagnostic frame into
 *        the reassembly state machine.
 * @param[in] frame8   NAD, PCI[, LEN], SID/data - exactly 8 bytes.
 * @param[in] own_nad  This node's current NAD (for the addressed check).
 * @param[in] now_us   Caller-supplied timestamp (from open_lin_hw_get_time_us()),
 *                      passed in rather than read internally so this engine
 *                      stays hardware-independent and unit-testable.
 * @return l_true when ctx->buffer[0..received_len) now holds a complete message.
 */
l_bool open_lin_tp_rx_feed(t_open_lin_tp_rx_ctx *ctx, const l_u8 *frame8, open_lin_NAD_t own_nad, l_u32 now_us);

/** @brief N_Cr supervision - call periodically while ctx->active is true. */
l_bool open_lin_tp_rx_timed_out(const t_open_lin_tp_rx_ctx *ctx, l_u32 now_us);

/**
 * @brief N_Cr supervision for the slave's request-reassembly session (Table 3.2).
 *        Call periodically from the app's main loop with the current time. If a
 *        FF+CF request stalls (a Consecutive Frame never arrives), this resets
 *        the reassembly context and reports OPEN_LIN_SLAVE_ERROR_TP_RX_TIMEOUT -
 *        otherwise a stalled session would sit silently forever with no error
 *        reported and no chance to corrupt a later, unrelated CF-shaped frame.
 */
void open_lin_transport_layer_poll(l_u32 now_us);

/**
 * @brief Debug/test only, slave side: the next request that gets a response is answered
 *        with a "response pending" (0x7F, SID, 0x78) first, and the real response follows
 *        once delay_us has passed (sent from open_lin_transport_layer_poll(), so that has
 *        to keep being called). Lets the master's P2* handling be tried on real hardware.
 *        One shot; delay_us = 0 disarms.
 */
void open_lin_transport_layer_debug_response_pending(l_u32 delay_us);

/**
 * @brief Outcome of a service handler, drives what the transport layer sends back on 0x3D.
 */
typedef enum {
	OPEN_LIN_SID_RESULT_NO_RESPONSE = 0, /**< stay silent (e.g. the request is not for this node) */
	OPEN_LIN_SID_RESULT_POSITIVE,        /**< response data is valid, respond RSID = SID + 0x40 followed by it */
	OPEN_LIN_SID_RESULT_NEGATIVE         /**< respond 0x7F, SID, NRC, per Table 4.21 */
} t_open_lin_sid_result;

/**
 * @brief Service the application handles itself (UDS-style services such as 0x22 Read Data By Identifier or
 *        0x2E Write Data By Identifier): called for every request whose SID the library has no handler for (the
 *        library handles 0xB0, 0xB1, 0xB2 and 0xB7 itself). The broadcast NAD (0x7F) counts as the node's own NAD,
 *        answer included (Diagnostic Spec 5.5); a functionally addressed request (NAD 0x7E) is processed too, but
 *        its response is discarded, whatever the application returns.
 * @param[in]     sid             the request's service id
 * @param[in]     request         the data after the SID
 * @param[in]     request_length  length of request
 * @param[out]    response        where to write the data that follows the RSID of a positive response
 * @param[in,out] response_length in: the room in response, out: how many bytes were written
 * @param[out]    nrc             the negative response code when the result is NEGATIVE
 * @note Runs in the context that feeds the library its frames and has to be done well within P2max.
 *       The library adds the RSID (SID + 0x40) or the 0x7F, SID, NRC of a negative response itself.
 */
typedef t_open_lin_sid_result (*open_lin_user_service_fn)(l_u8 sid, const l_u8 *request, l_u16 request_length,
														 l_u8 *response, l_u16 *response_length, l_u8 *nrc);

/** @brief Registers the application's service handler (NULL removes it). Built-in services are never handed over. */
void open_lin_transport_layer_set_user_service(open_lin_user_service_fn fn);
/** @brief N_Cr supervision for the master's response-reassembly session - see open_lin_transport_layer_poll(). */
void open_lin_master_transport_layer_poll(l_u32 now_us);

/** @brief Register a message (SID/RSID + data) to be sent, segmented as needed. */
void   open_lin_tp_tx_start(t_open_lin_tp_tx_ctx *ctx, const l_u8 *msg, l_u16 len);

/**
 * @brief Produce the next 8-byte frame to send (SF, or the next FF/CF chunk).
 *        Fills out_frame8 with 0xFF padding + NAD even when ctx is inactive.
 * @return l_true if there are more frames left to send after this one.
 */
l_bool open_lin_tp_tx_next(t_open_lin_tp_tx_ctx *ctx, open_lin_NAD_t nad, l_u8 *out_frame8);

/** @brief Feed an incoming 0x3C (master request) frame - hooked from the network layer. */
void open_lin_transport_layer_handle(open_lin_frame_slot_t *slot);

/** @brief Fill an outgoing 0x3D (slave response) frame - hooked from the network layer. */
void open_lin_transport_layer_handle_tx(open_lin_frame_slot_t *slot);

/**
 * @brief Master side: originate a diagnostic request (SID/data), segmented as needed.
 * @param[in] nad  Target node's NAD (or OPEN_LIN_NAD_BROADCAST/_FUNCTIONAL).
 * @param[in] msg  SID + data to send, owned by the caller until the request completes.
 * @return l_false if a request is already in progress (single active session, Section 3.2.2).
 */
l_bool open_lin_master_diag_request(open_lin_NAD_t nad, const l_u8 *msg, l_u16 len);

/**
 * @brief Debug/test only: like open_lin_master_diag_request(), but stops after
 *        sending the First Frame instead of continuing on to the Consecutive
 *        Frame(s) - `msg`/`len` must require FF+CF segmentation (len > 6, see
 *        Section 3.2.1). Lets the target's N_Cr supervision
 *        (open_lin_transport_layer_poll()) be exercised on real hardware
 *        without needing to physically fault-inject a dropped frame.
 * @return l_false if a request is already in progress (same rule as open_lin_master_diag_request()).
 */
l_bool open_lin_master_diag_request_debug_drop_cf(open_lin_NAD_t nad, const l_u8 *msg, l_u16 len);

/**
 * @brief Master side: originate a go-to-sleep command (Section 2.6.3, Table 2.1/
 *        Table 4.4) - NAD=0x00 (the dedicated go-to-sleep NAD, distinct from the
 *        0x7F broadcast NAD), D1..D7 all 0xFF. Carries no SID/data, so it
 *        bypasses the tp_tx engine (which treats a zero-length message as
 *        "nothing to send") and is queued as its own one-shot frame instead.
 * @return l_false if a diagnostic request is already in progress (same rule as open_lin_master_diag_request()).
 */
l_bool open_lin_master_send_go_to_sleep(void);

/** @brief Diagnostic Spec 5.6 Table 5.3: P2max, how long the master keeps polling for the start of a response. */
#ifndef OPEN_LIN_MASTER_DIAG_P2_MAX_US
#define OPEN_LIN_MASTER_DIAG_P2_MAX_US 500000u
#endif

/** @brief Diagnostic Spec 5.6 Table 5.3: P2*, how long the master keeps waiting after the slave answered "response pending" (NRC 0x78). */
#ifndef OPEN_LIN_MASTER_DIAG_P2_STAR_US
#define OPEN_LIN_MASTER_DIAG_P2_STAR_US 2000000u
#endif

/** @brief Negative response code "response pending": the final response follows later (Diagnostic Spec Table 4.21 / 5.6). */
#define OPEN_LIN_NRC_RESPONSE_PENDING 0x78u

/**
 * @brief Master side: is there a master request frame (a request's SF/FF/CF, or a
 *        go-to-sleep frame) still waiting to go out? The master data layer uses
 *        this to know when to run the diagnostic master request schedule.
 */
l_bool open_lin_master_diag_tx_pending(void);

/**
 * @brief Master side: is a slave response still to be collected - the request is
 *        fully sent (physical addressing; a functional request gets no response)
 *        and the response is neither complete nor timed out? Used to decide when
 *        to run the diagnostic slave response schedule. Gives up once P2max has
 *        passed without the first response frame arriving; after that, the
 *        multi-frame reassembly timeout (N_Cr) takes over. A "response pending"
 *        negative response (0x7F, SID, 0x78) is not delivered to the application:
 *        it only restarts the wait with the longer P2* limit.
 */
l_bool open_lin_master_diag_response_expected(l_u32 now_us);

/** @brief Fill an outgoing 0x3C (master request) frame - hooked from the master data layer. */
void open_lin_master_transport_layer_handle_tx(open_lin_frame_slot_t *slot);

/** @brief Feed an incoming 0x3D (slave response) frame - hooked from the master data layer. */
void open_lin_master_transport_layer_handle_rx(open_lin_frame_slot_t *slot);

/** @brief App hook: called once a diagnostic response (RSID/data) has been fully reassembled. */
extern void open_lin_master_diag_response_callback(const l_u8 *msg, l_u16 len);

/** @brief Slave side app hook: called once a go-to-sleep command (Section 2.6.2) has been received. */
extern void open_lin_slave_sleep_callback(void);

/**
 * @brief Master side app hook: called once the go-to-sleep frame queued by
 *        open_lin_master_send_go_to_sleep() has actually gone out on the bus
 *        (not just queued) - the right moment to also pause the master's own
 *        schedule via open_lin_master_dl_set_asleep(l_true), Section 2.6.
 */
extern void open_lin_master_sleep_sent_callback(void);

#endif /* OPEN_LIN_TRANSPORT_LAYER */

#endif /* OPEN_LIN_TRANSPORT_LAYER_H_ */


