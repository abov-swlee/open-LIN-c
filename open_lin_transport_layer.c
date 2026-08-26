/**
 * @file open_lin_transport_layer.c
 * @brief LIN transport layer implementation
 */

#include "open_lin_transport_layer.h"
#include "open_lin_network_layer.h"
#include "open_lin_hw.h"

#ifdef OPEN_LIN_TRANSPORT_LAYER

extern open_lin_NAD_t open_lin_NAD;
extern l_u16 open_lin_supplier_id;
extern l_u16 open_lin_function_id;
extern void open_lin_sid_callback(open_lin_SID_t sid);

typedef struct {
	open_lin_SID_t sid;
	void (*handler)(const l_u8 *data);
} t_open_lin_sid_handler_item;


/* static function definitions */
static void open_lin_assign_nad_handler(const l_u8 *data);

/* static service handlers */
t_open_lin_sid_handler_item open_lin_sid_handlers[] = {
		{OPEN_LIN_SID_ASSIGN_NAD,open_lin_assign_nad_handler}
};

/* transport layer engine state - one active session at a time, see Section 3.2.2 */
static t_open_lin_tp_rx_ctx s_tp_rx_ctx;
static t_open_lin_tp_tx_ctx s_tp_tx_ctx;

/* assign NAD service implementation
 * data[0..4] = D1..D5 = supplier id (MSB,LSB), function id (MSB,LSB), new NAD */
static void open_lin_assign_nad_handler(const l_u8 *data)
{
	uint16_t sup = (data[1]) | (((uint16_t)data[0]) << 8);
	uint16_t fun = (data[3]) | (((uint16_t)data[2]) << 8);

	if ((open_lin_supplier_id == sup) && (open_lin_function_id == fun))
	{
		/* update NAD */
		open_lin_NAD = data[4];
	} else {
		/* invalid id, this device is not addressed do nothing */
	}
}

/**
 * @brief Dispatch one fully reassembled node-configuration message (SF, or
 *        FF+CF concatenated) to its SID handler.
 * @param[in] msg          msg[0] = SID, msg[1..len-1] = data bytes.
 * @param[in] len           total length including the SID byte.
 * @param[in] requester_nad NAD carried by the frame that completed the message.
 */
static void open_lin_transport_layer_dispatch(const l_u8 *msg, l_u16 len, open_lin_NAD_t requester_nad)
{
	uint8_t i = 0;

	if (len < 1u)
	{
		return; /* nothing to dispatch */
	}

	if ((requester_nad != open_lin_NAD) &&
	    (requester_nad != OPEN_LIN_NAD_BROADCAST) &&
	    (requester_nad != OPEN_LIN_NAD_FUNCTIONAL))
	{
		return; /* different NAD, not addressed to this node */
	}

	for (i = 0; i < (sizeof(open_lin_sid_handlers)/sizeof(open_lin_sid_handlers[0])); i++)
	{
		if (open_lin_sid_handlers[i].sid == msg[0])
		{
			open_lin_sid_handlers[i].handler(&msg[1]);
			open_lin_sid_callback((open_lin_SID_t)msg[0]);
			return;
		}
	}
	/* unsupported SID: negative response (RSID=0x7F) generation belongs to the
	 * response pipeline, not implemented yet - see project backlog. */
}

void open_lin_tp_rx_init(t_open_lin_tp_rx_ctx *ctx)
{
	ctx->active = l_false;
	ctx->total_len = 0u;
	ctx->received_len = 0u;
	ctx->next_sn = 1u;
	ctx->last_frame_time_us = 0u;
}

l_bool open_lin_tp_rx_feed(t_open_lin_tp_rx_ctx *ctx, const l_u8 *frame8, open_lin_NAD_t own_nad, l_u32 now_us)
{
	open_lin_NAD_t nad = frame8[0];
	l_u8 pci_type = (l_u8)((frame8[1] >> 4) & 0x0Fu);
	l_bool addressed = (nad == own_nad) || (nad == OPEN_LIN_NAD_BROADCAST) || (nad == OPEN_LIN_NAD_FUNCTIONAL);

	if (addressed == l_false)
	{
		return l_false; /* meant for another node, ignore without disturbing ctx */
	}

	switch (pci_type)
	{
		case OPEN_LIN_PCI_SF:
		{
			l_u8 len = (l_u8)(frame8[1] & 0x0Fu); /* SID + up to 5 data bytes */
			if ((len == 0u) || (len > 6u))
			{
				return l_false; /* Section 3.2.3: SF with length > 6 is ignored */
			}
			ctx->active = l_false; /* abort any reception in progress, start fresh */
			(void)open_lin_memcpy(ctx->buffer, &frame8[2], len);
			ctx->received_len = len;
			ctx->total_len = len;
			ctx->last_frame_time_us = now_us;
			return l_true; /* complete already */
		}

		case OPEN_LIN_PCI_FF:
		{
			l_u16 len = (l_u16)(((frame8[1] & 0x0Fu) << 8) | frame8[2]);
			if (len < 7u)
			{
				return l_false; /* Section 3.2.3: FF with length < 7 is ignored */
			}
			if (len > OPEN_LIN_TP_MAX_MSG_LEN)
			{
				ctx->active = l_false; /* can't buffer it, don't start reception */
				return l_false;
			}
			(void)open_lin_memcpy(ctx->buffer, &frame8[3], 5u); /* SID + D1..D4 */
			ctx->total_len = len;
			ctx->received_len = 5u;
			ctx->next_sn = 1u;
			ctx->active = l_true;
			ctx->last_frame_time_us = now_us;
			return l_false; /* not complete yet */
		}

		case OPEN_LIN_PCI_CF:
		{
			l_u8 sn = (l_u8)(frame8[1] & 0x0Fu);
			l_u16 remaining;
			l_u8 chunk;

			if (ctx->active == l_false)
			{
				return l_false; /* CF with nothing in progress: ignore */
			}
			if (sn != ctx->next_sn)
			{
				ctx->active = l_false; /* Section 3.2.3: unexpected SN aborts reception */
				return l_false;
			}

			remaining = (l_u16)(ctx->total_len - ctx->received_len);
			chunk = (remaining < 6u) ? (l_u8)remaining : 6u;
			(void)open_lin_memcpy(&ctx->buffer[ctx->received_len], &frame8[2], chunk);
			ctx->received_len = (l_u16)(ctx->received_len + chunk);
			ctx->next_sn = (ctx->next_sn == 15u) ? 0u : (l_u8)(ctx->next_sn + 1u);
			ctx->last_frame_time_us = now_us;

			if (ctx->received_len >= ctx->total_len)
			{
				ctx->active = l_false;
				return l_true;
			}
			return l_false;
		}

		default:
			return l_false; /* unexpected PCI type, ignored per Section 3.2.3 */
	}
}

l_bool open_lin_tp_rx_timed_out(const t_open_lin_tp_rx_ctx *ctx, l_u32 now_us)
{
	if (ctx->active == l_false)
	{
		return l_false;
	}
	return ((now_us - ctx->last_frame_time_us) > OPEN_LIN_TP_N_CR_TIMEOUT_US) ? l_true : l_false;
}

void open_lin_tp_tx_start(t_open_lin_tp_tx_ctx *ctx, const l_u8 *msg, l_u16 len)
{
	ctx->msg = msg;
	ctx->total_len = len;
	ctx->sent_len = 0u;
	ctx->next_sn = 1u;
	ctx->active = (len > 0u) ? l_true : l_false;
}

l_bool open_lin_tp_tx_next(t_open_lin_tp_tx_ctx *ctx, open_lin_NAD_t nad, l_u8 *out_frame8)
{
	l_u8 i;

	for (i = 0u; i < 8u; i++)
	{
		out_frame8[i] = 0xFFu;
	}

	if (ctx->active == l_false)
	{
		return l_false; /* nothing pending: caller gets an all-idle frame */
	}

	out_frame8[0] = nad;

	if ((ctx->sent_len == 0u) && (ctx->total_len <= 6u))
	{
		/* fits in a Single Frame */
		out_frame8[1] = (l_u8)((OPEN_LIN_PCI_SF << 4) | (ctx->total_len & 0x0Fu));
		(void)open_lin_memcpy(&out_frame8[2], ctx->msg, ctx->total_len);
		ctx->sent_len = ctx->total_len;
		ctx->active = l_false;
		return l_false;
	}

	if (ctx->sent_len == 0u)
	{
		/* First Frame: SID + D1..D4 (5 bytes), LEN carries the low 8 bits of length */
		out_frame8[1] = (l_u8)((OPEN_LIN_PCI_FF << 4) | ((ctx->total_len >> 8) & 0x0Fu));
		out_frame8[2] = (l_u8)(ctx->total_len & 0xFFu);
		(void)open_lin_memcpy(&out_frame8[3], ctx->msg, 5u);
		ctx->sent_len = 5u;
		return l_true;
	}

	/* Consecutive Frame: up to 6 more bytes */
	{
		l_u16 remaining = (l_u16)(ctx->total_len - ctx->sent_len);
		l_u8 chunk = (remaining < 6u) ? (l_u8)remaining : 6u;
		out_frame8[1] = (l_u8)((OPEN_LIN_PCI_CF << 4) | (ctx->next_sn & 0x0Fu));
		(void)open_lin_memcpy(&out_frame8[2], &ctx->msg[ctx->sent_len], chunk);
		ctx->sent_len = (l_u16)(ctx->sent_len + chunk);
		ctx->next_sn = (ctx->next_sn == 15u) ? 0u : (l_u8)(ctx->next_sn + 1u);
	}

	if (ctx->sent_len >= ctx->total_len)
	{
		ctx->active = l_false;
		return l_false;
	}
	return l_true;
}

void open_lin_transport_layer_handle(open_lin_frame_slot_t *slot)
{
	if ((slot == OPEN_LIN_NET_SLOT_EMPTY) || (slot->pid != OPEN_LIN_DIAG_REQUEST))
	{
		/* diagnostic request slot not registered by the app, or wrong slot - nothing to do */
		return;
	}

	if (open_lin_tp_rx_feed(&s_tp_rx_ctx, slot->data_ptr, open_lin_NAD, open_lin_hw_get_time_us()) == l_true)
	{
		open_lin_transport_layer_dispatch(s_tp_rx_ctx.buffer, s_tp_rx_ctx.received_len, slot->data_ptr[0]);
	}
}

void open_lin_transport_layer_handle_tx(open_lin_frame_slot_t *slot)
{
	if ((slot == OPEN_LIN_NET_SLOT_EMPTY) || (slot->pid != OPEN_LIN_DIAG_RESPONSE))
	{
		/* diagnostic response slot not registered by the app, or wrong slot - nothing to do */
		return;
	}

	(void)open_lin_tp_tx_next(&s_tp_tx_ctx, open_lin_NAD, slot->data_ptr);
}

#endif
