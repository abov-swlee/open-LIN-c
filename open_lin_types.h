/**
 * @file open_lin_types.h
 * @brief LIN slave data layer implementation
 */

#ifndef OPEN_LIN_OPEN_LIN_TYPES_H_
#define OPEN_LIN_OPEN_LIN_TYPES_H_

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#define OPEN_LIN_MAX_FRAME_LEN 8u
#define OPEN_LIN_MAX_FRAME_CHECK_SUM_SIZE 1u

typedef bool l_bool;
typedef uint8_t l_u8 ;
typedef uint16_t l_u16;
typedef uint32_t l_u32;

#define l_true true
#define l_false false

#define OPEN_LIN_MAX_FRAME_LENGTH 0x8u
#define OPEN_LIN_SYNCH_BYTE 0x55u
#define OPEN_LIN_GET_PID_BIT(x,y) (((x) >> (y)) & 0x01u)
#define OPEN_LIN_ID_MASK 0x3Fu
#define OPEN_LIN_P0_FLAG 6
#define OPEN_LIN_P1_FLAG 7

#define OPEN_LIN_DIAG_REQUEST 0x3C
#define OPEN_LIN_DIAG_RESPONSE 0x3D


#define open_lin_memcpy memcpy

typedef l_u8 open_lin_pid_t;
typedef l_u8 open_lin_checksum_t;
typedef enum {
	OPEN_LIN_FRAME_TYPE_TRANSMIT,
	OPEN_LIN_FRAME_TYPE_RECEIVE
} open_lin_frame_type_t;

/**
 * @brief Per-frame checksum model, LDF "protocol_version"/"checksum" property.
 *        Diagnostic frames (0x3C/0x3D) always use classic regardless of this
 *        setting - see open_lin_data_layer_checksum(). ENHANCED=0 is the
 *        default so existing open_lin_frame_slot_t initializers that omit
 *        this field keep their prior (enhanced) behavior unchanged.
 */
typedef enum {
	OPEN_LIN_CHECKSUM_ENHANCED = 0, /**< LIN 2.x default: parity(pid) + data bytes */
	OPEN_LIN_CHECKSUM_CLASSIC        /**< LIN 1.x: data bytes only */
} open_lin_checksum_type_t;

typedef enum {
	OPEN_LIN_NO_ERROR,
	OPEN_LIN_SLAVE_ERROR_INVALID_DATA_RX,
	OPEN_LIN_SLAVE_ERROR_INVALID_CHECKSUM,
	OPEN_LIN_SLAVE_ERROR_PID_PARITY,
	OPEN_LIN_SLAVE_ERROR_INVALID_SYNCH,
	OPEN_LIN_SLAVE_ERROR_INVALID_BREAK,
	OPEN_LIN_SLAVE_ERROR_ID_NOT_FOUND,
	OPEN_LIN_SLAVE_ERROR_HW_TX,
	OPEN_LIN_MASTER_ERROR_CHECKSUM,
	OPEN_LIN_MASTER_ERROR_HEADER_TX,
	OPEN_LIN_MASTER_ERROR_DATA_TX,
	OPEN_LIN_MASTER_ERROR_DATA_RX,
	OPEN_LIN_MASTER_ERROR_FRAME_SLOT_TIMEOUT,
	OPEN_LIN_SLAVE_ERROR_TP_RX_TIMEOUT,  /**< N_Cr: request FF+CF reassembly stalled, Table 3.2 */
	OPEN_LIN_MASTER_ERROR_TP_RX_TIMEOUT, /**< N_Cr: response FF+CF reassembly stalled, Table 3.2 */
	OPEN_LIN_MASTER_ERROR_EVENT_COLLISION, /**< event-triggered frame: garbled or incomplete response, collision resolving started (Section 2.3.3.2) */
	OPEN_LIN_MASTER_ERROR_EVENT_UNKNOWN_PID /**< event-triggered frame: valid checksum but the PID in data byte 0 matches no associated frame */
} t_open_lin_error;

/**
 * @brief error handler function
 */
void open_lin_error_handler(t_open_lin_error error_code);


#endif /* OPEN_LIN_OPEN_LIN_TYPES_H_ */
