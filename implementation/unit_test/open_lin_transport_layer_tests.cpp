/*
 * open_lin_transport_layer_tests.cpp
 *
 *  Created on: 21 cze 2018
 *      Author: ay7vi2
 */



#include "catch.hpp"
#include "iostream"

extern "C"{

#include "../open_lin_cfg.h"
#include "../../open_lin_master_data_layer.h"
#include "../../open_lin_transport_layer.h"
#include "../../open_lin_hw.h"

open_lin_NAD_t open_lin_NAD = 0;
uint16_t open_lin_supplier_id = 0x1234;
uint16_t open_lin_function_id = 0x5678;
uint8_t  open_lin_variant_id = 0x00;

/* Assign Frame ID Range's assignable frame list under test - reset by each test case. */
uint8_t afr_data_0, afr_data_1, afr_data_2;
open_lin_frame_slot_t afr_slot_0 = {0x01, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, &afr_data_0};
open_lin_frame_slot_t afr_slot_1 = {0x02, OPEN_LIN_FRAME_TYPE_RECEIVE,  1, &afr_data_1};
open_lin_frame_slot_t afr_slot_2 = {0x0E, OPEN_LIN_FRAME_TYPE_RECEIVE,  1, &afr_data_2};
open_lin_frame_slot_t *open_lin_assignable_frames[] = {&afr_slot_0, &afr_slot_1, &afr_slot_2};
extern const uint8_t open_lin_assignable_frames_count = sizeof(open_lin_assignable_frames) / sizeof(open_lin_assignable_frames[0]); /* C++: plain "const" defaults to internal linkage, even inside extern "C" */

/* Minimal HW time stub for open_lin_transport_layer_handle()'s N_Cr timestamping;
 * the engine-level tests below (open_lin_tp_rx_feed/open_lin_tp_tx_next) take the
 * timestamp as a parameter instead, so they don't depend on this. */
static l_u32 s_fake_time_us = 0;
l_u32 open_lin_hw_get_time_us(void) { return s_fake_time_us; }

void open_lin_sid_callback(open_lin_SID_t sid) { (void)sid; }

/* captures the last call, for the N_Cr poll timeout tests below */
static t_open_lin_error s_last_error = OPEN_LIN_NO_ERROR;
void open_lin_error_handler(t_open_lin_error error_code) { s_last_error = error_code; }

/* captures the last call, for the master diag response callback tests below */
static l_bool s_master_diag_response_called = l_false;
static l_u8   s_master_diag_response_buf[64];
static l_u16  s_master_diag_response_len = 0;
void open_lin_master_diag_response_callback(const l_u8 *msg, l_u16 len)
{
	s_master_diag_response_called = l_true;
	s_master_diag_response_len = len;
	(void)open_lin_memcpy(s_master_diag_response_buf, msg, len);
}

/* captures the last call, for the go-to-sleep tests below */
static l_bool s_slave_sleep_called = l_false;
void open_lin_slave_sleep_callback(void) { s_slave_sleep_called = l_true; }

/* captures the last call, for the master sleep-sent test below */
static l_bool s_master_sleep_sent_called = l_false;
void open_lin_master_sleep_sent_callback(void) { s_master_sleep_sent_called = l_true; }
}

/* Assign NAD request, Table 4.7: D1=supplier LSB, D2=supplier MSB, D3=function LSB,
 * D4=function MSB, D5=new NAD. supplier=0x1234, function=0x5678, matches the globals above. */
uint8_t request_data_1[] = {OPEN_LIN_NAD_BROADCAST, 0x06,0xB0,0x34,0x12,0x78,0x56,0x34};

open_lin_frame_slot_t slot_1 = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, request_data_1};

TEST_CASE("change NAD test", "[open_lin_transport_layer]"){
	open_lin_NAD = 0;
	open_lin_transport_layer_handle(&slot_1);
	CHECK(open_lin_NAD == 0x34);
}

/* ---- Response pipeline: handler -> RSID/negative-response assembly -> 0x3D tx ---- */

TEST_CASE("dispatch: assign NAD responds with the initial NAD, not the new one", "[open_lin_transport_layer][tp]") {
	open_lin_NAD = 0x01;
	open_lin_supplier_id = 0x1234;
	open_lin_function_id = 0x5678;

	uint8_t req[8] = {0x01, 0x06, 0xB0, 0x34, 0x12, 0x78, 0x56, 0x22}; /* new NAD = 0x22 */
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);
	CHECK(open_lin_NAD == 0x22); /* request was applied */

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);

	CHECK(resp[0] == 0x01); /* Table 4.8: response uses the initial NAD, not the new 0x22 */
	CHECK((resp[1] >> 4) == OPEN_LIN_PCI_SF);
	CHECK((resp[1] & 0x0F) == 1); /* RSID only, no data bytes */
	CHECK(resp[2] == 0xF0); /* RSID = SID(0xB0) + 0x40 */
}

TEST_CASE("dispatch: assign NAD with mismatched supplier id stays silent", "[open_lin_transport_layer][tp]") {
	open_lin_NAD = 0x01;
	open_lin_supplier_id = 0x1234;
	open_lin_function_id = 0x5678;

	uint8_t req[8] = {0x01, 0x06, 0xB0, 0xAA, 0xBB, 0x78, 0x56, 0x22}; /* wrong supplier id */
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);
	CHECK(open_lin_NAD == 0x01); /* unchanged, Section 4.2.5.1 */

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	for (int i = 0; i < 8; i++) {
		CHECK(resp[i] == 0xFF); /* nothing queued -> idle frame */
	}
}

TEST_CASE("dispatch: read by identifier id=0 returns the LIN product identification", "[open_lin_transport_layer][tp]") {
	open_lin_NAD = 0x05;
	open_lin_supplier_id = 0x1234;
	open_lin_function_id = 0x5678;
	open_lin_variant_id = 0x03;

	/* Table 4.18: D1=identifier, D2=supplier LSB, D3=supplier MSB, D4=function LSB, D5=function MSB */
	uint8_t req[8] = {0x05, 0x06, 0xB2, 0x00, 0x34, 0x12, 0x78, 0x56};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);

	CHECK(resp[0] == 0x05);
	CHECK((resp[1] & 0x0F) == 6); /* RSID + 5 data bytes, Table 4.20 row id=0 */
	CHECK(resp[2] == 0xF2); /* RSID = SID(0xB2) + 0x40 */
	CHECK(resp[3] == 0x34); /* supplier LSB */
	CHECK(resp[4] == 0x12); /* supplier MSB */
	CHECK(resp[5] == 0x78); /* function LSB */
	CHECK(resp[6] == 0x56); /* function MSB */
	CHECK(resp[7] == 0x03); /* variant */
}

TEST_CASE("dispatch: read by identifier with an unsupported id gets a negative response", "[open_lin_transport_layer][tp]") {
	open_lin_NAD = 0x05;
	open_lin_supplier_id = 0x1234;
	open_lin_function_id = 0x5678;

	uint8_t req[8] = {0x05, 0x06, 0xB2, 0x01 /* id=1, serial number, not implemented */, 0x34, 0x12, 0x78, 0x56};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);

	CHECK(resp[0] == 0x05);
	CHECK((resp[1] & 0x0F) == 3);
	CHECK(resp[2] == 0x7F);
	CHECK(resp[3] == 0xB2);
	CHECK(resp[4] == 0x12); /* Section 4.2.6.1 example error code */
}

TEST_CASE("dispatch: unsupported SID gets a negative response", "[open_lin_transport_layer][tp]") {
	open_lin_NAD = 0x01;

	uint8_t req[8] = {0x01, 0x01, 0xB6, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; /* Save Configuration, not registered yet */
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);

	CHECK(resp[0] == 0x01);
	CHECK((resp[1] & 0x0F) == 3);
	CHECK(resp[2] == 0x7F);
	CHECK(resp[3] == 0xB6);
	CHECK(resp[4] == 0x11);
}

TEST_CASE("dispatch: assign frame id range reassigns, unassigns, and leaves do-not-care slots alone", "[open_lin_transport_layer][tp]") {
	open_lin_NAD = 0x01;
	afr_slot_0.pid = 0x01;
	afr_slot_1.pid = 0x02;
	afr_slot_2.pid = 0x0E;

	/* start=0; D2=0x05 (new pid for slot 0), D3=0x00 (unassign slot 1),
	 * D4=0xFF (do not care, slot 2 untouched), D5=0xFF (out-of-range index 3,
	 * only fulfillable as do not care - Table 4.16 Example 2) */
	uint8_t req[8] = {0x01, 0x06, 0xB7, 0x00, 0x05, 0x00, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	CHECK(afr_slot_0.pid == 0x05);
	CHECK(afr_slot_1.pid == 0xFF); /* OPEN_LIN_PID_UNASSIGNED */
	CHECK(afr_slot_2.pid == 0x0E); /* unchanged */

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);

	CHECK(resp[0] == 0x01);
	CHECK((resp[1] & 0x0F) == 1); /* RSID only, no data - Table 4.17 */
	CHECK(resp[2] == 0xF7); /* RSID = SID(0xB7) + 0x40 */
}

TEST_CASE("dispatch: assign frame id range rejects an out-of-range non-wildcard slot", "[open_lin_transport_layer][tp]") {
	open_lin_NAD = 0x01;
	afr_slot_0.pid = 0x01;
	afr_slot_1.pid = 0x02;
	afr_slot_2.pid = 0x0E;

	/* start=2: D2(idx2, in range) is a real assignment (0x05), D3(idx3, out of
	 * range) is also a real assignment (0x07, not 0xFF) -> whole request
	 * rejected, so even the individually-valid idx2 assignment must not apply */
	uint8_t req[8] = {0x01, 0x06, 0xB7, 0x02, 0x05, 0x07, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	CHECK(afr_slot_2.pid == 0x0E); /* rejected atomically: even the in-range slot is untouched */

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	for (int i = 0; i < 8; i++) {
		CHECK(resp[i] == 0xFF); /* nothing queued -> idle frame */
	}
}

/* ---- Full transport layer (FF/CF) engine tests, LIN-Spec 2.2A Chapter 3 ---- */

TEST_CASE("tp rx: single frame is complete immediately", "[open_lin_transport_layer][tp]") {
	t_open_lin_tp_rx_ctx ctx;
	open_lin_tp_rx_init(&ctx);

	/* NAD, PCI(SF,len=3), SID=0xB2, D1, D2 */
	uint8_t frame[8] = {0x01, 0x03, 0xB2, 0x11, 0x22, 0xFF, 0xFF, 0xFF};
	l_bool complete = open_lin_tp_rx_feed(&ctx, frame, 0x01, 1000);

	CHECK(complete == l_true);
	CHECK(ctx.received_len == 3);
	CHECK(ctx.buffer[0] == 0xB2);
	CHECK(ctx.buffer[1] == 0x11);
	CHECK(ctx.buffer[2] == 0x22);
}

TEST_CASE("tp rx: FF + CF reassembles a 10 byte message", "[open_lin_transport_layer][tp]") {
	t_open_lin_tp_rx_ctx ctx;
	open_lin_tp_rx_init(&ctx);

	/* message = SID(0xB2) + D1..D9, total_len = 10 */
	uint8_t ff[8]  = {0x01, 0x10, 0x0A, 0xB2, 0x01, 0x02, 0x03, 0x04};
	uint8_t cf1[8] = {0x01, 0x21, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF}; /* SN=1, 5 bytes left */

	CHECK(open_lin_tp_rx_feed(&ctx, ff, 0x01, 1000) == l_false);
	CHECK(ctx.active == l_true);
	CHECK(ctx.received_len == 5);

	l_bool complete = open_lin_tp_rx_feed(&ctx, cf1, 0x01, 1500);
	CHECK(complete == l_true);
	CHECK(ctx.received_len == 10);
	uint8_t expected[10] = {0xB2,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
	for (int i = 0; i < 10; i++) {
		CHECK(ctx.buffer[i] == expected[i]);
	}
}

TEST_CASE("tp rx: CF with wrong sequence number aborts reception", "[open_lin_transport_layer][tp]") {
	t_open_lin_tp_rx_ctx ctx;
	open_lin_tp_rx_init(&ctx);

	uint8_t ff[8]  = {0x01, 0x10, 0x0A, 0xB2, 0x01, 0x02, 0x03, 0x04};
	uint8_t bad_cf[8] = {0x01, 0x22 /* SN=2, expected 1 */, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF};

	(void)open_lin_tp_rx_feed(&ctx, ff, 0x01, 1000);
	CHECK(ctx.active == l_true);

	l_bool complete = open_lin_tp_rx_feed(&ctx, bad_cf, 0x01, 1500);
	CHECK(complete == l_false);
	CHECK(ctx.active == l_false); /* reception aborted, Section 3.2.3 */
}

TEST_CASE("tp rx: frame addressed to a different NAD is ignored", "[open_lin_transport_layer][tp]") {
	t_open_lin_tp_rx_ctx ctx;
	open_lin_tp_rx_init(&ctx);

	uint8_t frame[8] = {0x05 /* not us, not broadcast, not functional */, 0x03, 0xB2, 0x11, 0x22, 0xFF, 0xFF, 0xFF};
	l_bool complete = open_lin_tp_rx_feed(&ctx, frame, 0x01, 1000);

	CHECK(complete == l_false);
	CHECK(ctx.active == l_false);
}

TEST_CASE("tp tx: short message fits a Single Frame", "[open_lin_transport_layer][tp]") {
	t_open_lin_tp_tx_ctx ctx;
	uint8_t msg[] = {0xF2, 0x11, 0x22}; /* RSID + 2 data bytes, total_len = 3 */
	open_lin_tp_tx_start(&ctx, msg, sizeof(msg));

	uint8_t frame[8];
	l_bool more = open_lin_tp_tx_next(&ctx, 0x01, frame);

	CHECK(more == l_false);
	CHECK(frame[0] == 0x01);              /* NAD */
	CHECK((frame[1] >> 4) == OPEN_LIN_PCI_SF);
	CHECK((frame[1] & 0x0F) == 3);
	CHECK(frame[2] == 0xF2);
	CHECK(frame[3] == 0x11);
	CHECK(frame[4] == 0x22);
	CHECK(frame[5] == 0xFF);              /* unused bytes recessive, Section 3.2.1.6 */
}

TEST_CASE("tp tx: 10 byte message segments into FF + CF", "[open_lin_transport_layer][tp]") {
	t_open_lin_tp_tx_ctx ctx;
	uint8_t msg[10] = {0xF2,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
	open_lin_tp_tx_start(&ctx, msg, sizeof(msg));

	uint8_t ff[8];
	l_bool more = open_lin_tp_tx_next(&ctx, 0x01, ff);
	CHECK(more == l_true);
	CHECK((ff[1] >> 4) == OPEN_LIN_PCI_FF);
	CHECK(ff[2] == 10); /* LEN */
	CHECK(ff[3] == 0xF2);
	CHECK(ff[4] == 0x01);
	CHECK(ff[7] == 0x04);

	uint8_t cf[8];
	more = open_lin_tp_tx_next(&ctx, 0x01, cf);
	CHECK(more == l_false); /* last frame */
	CHECK((cf[1] >> 4) == OPEN_LIN_PCI_CF);
	CHECK((cf[1] & 0x0F) == 1); /* SN=1 */
	CHECK(cf[2] == 0x05);
	CHECK(cf[6] == 0x09);
	CHECK(cf[7] == 0xFF); /* only 5 of 6 slots used */
}

TEST_CASE("tp tx: nothing pending returns an all-idle frame", "[open_lin_transport_layer][tp]") {
	t_open_lin_tp_tx_ctx ctx;
	open_lin_tp_tx_start(&ctx, nullptr, 0);

	uint8_t frame[8];
	l_bool more = open_lin_tp_tx_next(&ctx, 0x01, frame);

	CHECK(more == l_false);
	for (int i = 0; i < 8; i++) {
		CHECK(frame[i] == 0xFF);
	}
}

/* ---- Master side: originate requests on 0x3C, reassemble responses on 0x3D ---- */

TEST_CASE("master diag: originates a single frame request", "[open_lin_transport_layer][tp][master]") {
	uint8_t msg[] = {0xB0, 0x34, 0x12, 0x78, 0x56, 0x22}; /* Assign NAD request, SID+5 data bytes */
	CHECK(open_lin_master_diag_request(0x05, msg, sizeof(msg)) == l_true);

	uint8_t req[8];
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, req};
	open_lin_master_transport_layer_handle_tx(&req_slot);

	CHECK(req[0] == 0x05);
	CHECK((req[1] >> 4) == OPEN_LIN_PCI_SF);
	CHECK((req[1] & 0x0F) == sizeof(msg));
	CHECK(req[2] == 0xB0);
	CHECK(req[7] == 0x22); /* last of D1..D5 */
}

TEST_CASE("master diag: a second request while one is in flight is refused", "[open_lin_transport_layer][tp][master]") {
	uint8_t msg[10] = {0xB2,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09}; /* needs FF+CF */
	CHECK(open_lin_master_diag_request(0x05, msg, sizeof(msg)) == l_true);

	uint8_t frame[8];
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, frame};
	open_lin_master_transport_layer_handle_tx(&req_slot); /* sends the FF, CF still pending */

	uint8_t other_msg[] = {0xB2, 0x00};
	CHECK(open_lin_master_diag_request(0x06, other_msg, sizeof(other_msg)) == l_false);

	/* drain the pending CF so later tests don't see a stale in-flight session */
	open_lin_master_transport_layer_handle_tx(&req_slot);
}

TEST_CASE("master diag: reassembles a response and hands it to the app callback", "[open_lin_transport_layer][tp][master]") {
	/* consume a small request first, purely to set the expected response NAD to 0x05
	 * (open_lin_master_transport_layer_handle_rx() checks the response against
	 * whatever NAD the most recent open_lin_master_diag_request() targeted) */
	uint8_t msg[] = {0xB2, 0x00};
	CHECK(open_lin_master_diag_request(0x05, msg, sizeof(msg)) == l_true);
	uint8_t req[8];
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, req};
	open_lin_master_transport_layer_handle_tx(&req_slot);

	s_master_diag_response_called = l_false;
	uint8_t resp[8] = {0x05, 0x03, 0xF2, 0x11, 0x22, 0xFF, 0xFF, 0xFF}; /* SF, len=3 */
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, resp};
	open_lin_master_transport_layer_handle_rx(&resp_slot);

	CHECK(s_master_diag_response_called == l_true);
	CHECK(s_master_diag_response_len == 3);
	CHECK(s_master_diag_response_buf[0] == 0xF2);
	CHECK(s_master_diag_response_buf[1] == 0x11);
	CHECK(s_master_diag_response_buf[2] == 0x22);
}

TEST_CASE("master diag: a broadcast request accepts whichever NAD the response carries", "[open_lin_transport_layer][tp][master]") {
	uint8_t msg[] = {0xB0, 0x34, 0x12, 0xFF, 0xFF, 0x7F}; /* Assign NAD, wildcard-addressed */
	CHECK(open_lin_master_diag_request(OPEN_LIN_NAD_BROADCAST, msg, sizeof(msg)) == l_true);
	uint8_t req[8];
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, req};
	open_lin_master_transport_layer_handle_tx(&req_slot); /* consume the request, SF fits in one call */

	s_master_diag_response_called = l_false;
	/* response NAD (0x03) is the slave's own initial NAD, not the broadcast 0x7F used in the request */
	uint8_t resp[8] = {0x03, 0x01, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, resp};
	open_lin_master_transport_layer_handle_rx(&resp_slot);

	CHECK(s_master_diag_response_called == l_true);
	CHECK(s_master_diag_response_len == 1);
	CHECK(s_master_diag_response_buf[0] == 0xF0);
}

TEST_CASE("master diag: debug_drop_cf sends the FF then goes idle instead of the CF", "[open_lin_transport_layer][tp][master][debug]") {
	uint8_t msg[10] = {0xB0,0xFF,0x7F,0xFF,0xFF,0x01,0xAA,0xBB,0xCC,0xDD}; /* needs FF+CF */
	CHECK(open_lin_master_diag_request_debug_drop_cf(0x05, msg, sizeof(msg)) == l_true);

	uint8_t ff[8];
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, ff};
	open_lin_master_transport_layer_handle_tx(&req_slot);
	CHECK((ff[1] >> 4) == OPEN_LIN_PCI_FF); /* the FF still goes out normally */

	uint8_t next[8];
	open_lin_frame_slot_t req_slot2 = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, next};
	open_lin_master_transport_layer_handle_tx(&req_slot2);
	for (int i = 0; i < 8; i++) {
		CHECK(next[i] == 0xFF); /* the CF is dropped: idle frame instead */
	}

	/* a fresh request right after isn't refused as "still in flight" */
	uint8_t other_msg[] = {0xB2, 0x00};
	CHECK(open_lin_master_diag_request(0x06, other_msg, sizeof(other_msg)) == l_true);
	open_lin_master_transport_layer_handle_tx(&req_slot); /* drain it so later tests see a clean state */
}

/* ---- N_Cr supervision: open_lin_transport_layer_poll() / open_lin_master_transport_layer_poll() ---- */

TEST_CASE("slave tp poll: an idle session never reports a timeout", "[open_lin_transport_layer][tp][poll]") {
	s_last_error = OPEN_LIN_NO_ERROR;
	open_lin_transport_layer_poll(0);
	open_lin_transport_layer_poll(0xFFFFFFFFu); /* even far in the future */
	CHECK(s_last_error == OPEN_LIN_NO_ERROR);
}

TEST_CASE("slave tp poll: N_Cr timeout resets a stalled request reassembly and reports it", "[open_lin_transport_layer][tp][poll]") {
	open_lin_NAD = 0x01;
	s_last_error = OPEN_LIN_NO_ERROR;

	/* FF only - a 10 byte message needs a following CF that never arrives */
	s_fake_time_us = 1000;
	uint8_t ff[8] = {0x01, 0x10, 0x0A, 0xB0, 0x34, 0x12, 0x78, 0x56};
	open_lin_frame_slot_t ff_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, ff};
	open_lin_transport_layer_handle(&ff_slot);
	CHECK(open_lin_NAD == 0x01); /* unaffected: still waiting on the CF, not dispatched yet */

	open_lin_transport_layer_poll(s_fake_time_us + 500000); /* within N_Cr (1s) */
	CHECK(s_last_error == OPEN_LIN_NO_ERROR);

	open_lin_transport_layer_poll(s_fake_time_us + 1000001); /* past N_Cr */
	CHECK(s_last_error == OPEN_LIN_SLAVE_ERROR_TP_RX_TIMEOUT);

	/* reassembly context is clean again: a fresh, unrelated SF request dispatches normally */
	uint8_t req[8] = {0x01, 0x06, 0xB0, 0x34, 0x12, 0x78, 0x56, 0x22};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);
	CHECK(open_lin_NAD == 0x22); /* request applied normally, confirms the stale ctx was cleared */
}

TEST_CASE("master tp poll: an idle session never reports a timeout", "[open_lin_transport_layer][tp][master][poll]") {
	s_last_error = OPEN_LIN_NO_ERROR;
	open_lin_master_transport_layer_poll(0);
	open_lin_master_transport_layer_poll(0xFFFFFFFFu);
	CHECK(s_last_error == OPEN_LIN_NO_ERROR);
}

TEST_CASE("master tp poll: N_Cr timeout resets a stalled response reassembly and reports it", "[open_lin_transport_layer][tp][master][poll]") {
	uint8_t msg[] = {0xB2, 0x00};
	CHECK(open_lin_master_diag_request(0x05, msg, sizeof(msg)) == l_true);
	uint8_t req[8];
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, req};
	open_lin_master_transport_layer_handle_tx(&req_slot); /* send the request (fits a SF) */

	s_last_error = OPEN_LIN_NO_ERROR;
	s_master_diag_response_called = l_false;

	/* response FF arrives (10 byte message), but the CF never does */
	s_fake_time_us = 2000;
	uint8_t resp_ff[8] = {0x05, 0x10, 0x0A, 0xF2, 0x01, 0x02, 0x03, 0x04};
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, resp_ff};
	open_lin_master_transport_layer_handle_rx(&resp_slot);
	CHECK(s_master_diag_response_called == l_false); /* not complete yet */

	open_lin_master_transport_layer_poll(s_fake_time_us + 500000); /* within N_Cr (1s) */
	CHECK(s_last_error == OPEN_LIN_NO_ERROR);

	open_lin_master_transport_layer_poll(s_fake_time_us + 1000001); /* past N_Cr */
	CHECK(s_last_error == OPEN_LIN_MASTER_ERROR_TP_RX_TIMEOUT);

	/* reassembly context is clean again: a fresh SF response now completes normally */
	CHECK(open_lin_master_diag_request(0x05, msg, sizeof(msg)) == l_true);
	open_lin_master_transport_layer_handle_tx(&req_slot);
	uint8_t resp_sf[8] = {0x05, 0x03, 0xF2, 0x11, 0x22, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t resp_sf_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, resp_sf};
	open_lin_master_transport_layer_handle_rx(&resp_sf_slot);
	CHECK(s_master_diag_response_called == l_true);
	CHECK(s_master_diag_response_len == 3);
}

/* ---- Go-to-sleep command, Section 2.6.3 ---- */

TEST_CASE("master: send_go_to_sleep produces NAD=0x00, D1..D7=0xFF", "[open_lin_transport_layer][tp][sleep]") {
	CHECK(open_lin_master_send_go_to_sleep() == l_true);

	uint8_t frame[8];
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, frame};
	open_lin_master_transport_layer_handle_tx(&req_slot);

	CHECK(frame[0] == OPEN_LIN_NAD_SLEEP);
	for (int i = 1; i < 8; i++) {
		CHECK(frame[i] == 0xFF);
	}
}

TEST_CASE("master: send_go_to_sleep is refused while a request is already in flight", "[open_lin_transport_layer][tp][sleep]") {
	uint8_t msg[10] = {0xB0,0xFF,0x7F,0xFF,0xFF,0x01,0xAA,0xBB,0xCC,0xDD}; /* needs FF+CF, stays active after the FF */
	CHECK(open_lin_master_diag_request(0x05, msg, sizeof(msg)) == l_true);
	uint8_t frame[8];
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, frame};
	open_lin_master_transport_layer_handle_tx(&req_slot); /* sends the FF, CF still pending */

	CHECK(open_lin_master_send_go_to_sleep() == l_false);

	open_lin_master_transport_layer_handle_tx(&req_slot); /* drain the CF so later tests see a clean state */
}

TEST_CASE("slave: a go-to-sleep frame invokes the sleep callback", "[open_lin_transport_layer][tp][sleep]") {
	s_slave_sleep_called = l_false;
	uint8_t req[8] = {OPEN_LIN_NAD_SLEEP, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);
	CHECK(s_slave_sleep_called == l_true);
}

TEST_CASE("slave: a frame with a real NAD and the same all-0xFF data is not mistaken for go-to-sleep", "[open_lin_transport_layer][tp][sleep]") {
	s_slave_sleep_called = l_false;
	open_lin_NAD = 0x01;
	uint8_t req[8] = {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; /* physical NAD, not the reserved 0x00 */
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);
	CHECK(s_slave_sleep_called == l_false);
}

TEST_CASE("master: sleep_sent callback fires only once the frame is actually transmitted", "[open_lin_transport_layer][tp][sleep]") {
	s_master_sleep_sent_called = l_false;
	CHECK(open_lin_master_send_go_to_sleep() == l_true);
	CHECK(s_master_sleep_sent_called == l_false); /* queued, not sent yet */

	uint8_t frame[8];
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, frame};
	open_lin_master_transport_layer_handle_tx(&req_slot);
	CHECK(s_master_sleep_sent_called == l_true);
}

