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


#endif /* OPEN_LIN_TRANSPORT_LAYER */

#endif /* OPEN_LIN_TRANSPORT_LAYER_H_ */


