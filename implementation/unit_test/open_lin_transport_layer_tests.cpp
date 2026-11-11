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
/* Assign Frame Id's (SID 0xB1) message-id lookup under test, 1:1 with open_lin_assignable_frames[] above. */
extern const uint16_t open_lin_assignable_frame_message_ids[] = {0x0000, 0x0001, 0x0002};

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

TEST_CASE("dispatch: assign frame id (LIN 2.0) reassigns a frame by message id", "[open_lin_transport_layer][tp]") {
	open_lin_NAD = 0x01;
	open_lin_supplier_id = 0x1234;
	afr_slot_0.pid = 0x01;
	afr_slot_1.pid = 0x02;
	afr_slot_2.pid = 0x0E;

	/* Table 2.4: D1/D2=supplier LSB/MSB, D3/D4=message id LSB/MSB, D5=protected id.
	 * message id 1 -> afr_slot_1 (see open_lin_assignable_frame_message_ids[] above). */
	uint8_t req[8] = {0x01, 0x06, 0xB1, 0x34, 0x12, 0x01, 0x00, 0x05};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	CHECK(afr_slot_1.pid == 0x05);
	CHECK(afr_slot_0.pid == 0x01); /* untouched */
	CHECK(afr_slot_2.pid == 0x0E); /* untouched */

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);

	CHECK(resp[0] == 0x01);
	CHECK((resp[1] & 0x0F) == 1); /* RSID only, no data - Table 2.5 */
	CHECK(resp[2] == 0xF1); /* RSID = SID(0xB1) + 0x40 */
}

TEST_CASE("dispatch: assign frame id with an unrecognized message id stays silent", "[open_lin_transport_layer][tp]") {
	open_lin_NAD = 0x01;
	open_lin_supplier_id = 0x1234;
	afr_slot_1.pid = 0x02;

	uint8_t req[8] = {0x01, 0x06, 0xB1, 0x34, 0x12, 0x99, 0x00, 0x05}; /* message id 0x0099, not assigned to any slot */
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	CHECK(afr_slot_1.pid == 0x02); /* unchanged */

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	for (int i = 0; i < 8; i++) {
		CHECK(resp[i] == 0xFF); /* nothing queued -> idle frame */
	}
}

TEST_CASE("dispatch: assign frame id rejects identifiers 0x3C and up", "[open_lin_transport_layer][tp]") {
	open_lin_NAD = 0x01;
	open_lin_supplier_id = 0x1234;
	afr_slot_0.pid = 0x01;

	/* message id 0 (afr_slot_0) but requested protected id's raw id is 0x3C -
	 * Section 2.5.1: "frames with identifier 60 (0x3c) and up can not be changed". */
	uint8_t req[8] = {0x01, 0x06, 0xB1, 0x34, 0x12, 0x00, 0x00, 0x3C};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	CHECK(afr_slot_0.pid == 0x01); /* unchanged */

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

/* ---- Diagnostic schedule triggers (Diagnostic Spec 5.4): tx pending / response expected ---- */
static void drain_master_request(void)
{
	uint8_t frame[8];
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, frame};
	for (int i = 0; (i < 4) && (open_lin_master_diag_tx_pending() == l_true); i++)
	{
		open_lin_master_transport_layer_handle_tx(&slot);
	}
}

TEST_CASE("master diag schedule: a physical request is pending, then its response is awaited until complete", "[open_lin_transport_layer][tp][master][sched]") {
	s_fake_time_us = 10000;
	uint8_t msg[] = {0xB2, 0x00};
	REQUIRE(open_lin_master_diag_request(0x05, msg, sizeof(msg)) == l_true);
	CHECK(open_lin_master_diag_tx_pending() == l_true);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us) == l_false); /* not until the request is fully sent */

	uint8_t req[8];
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, req};
	open_lin_master_transport_layer_handle_tx(&req_slot);
	CHECK(open_lin_master_diag_tx_pending() == l_false);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us + 1000) == l_true);

	uint8_t resp[8] = {0x05, 0x03, 0xF2, 0x11, 0x22, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, resp};
	open_lin_master_transport_layer_handle_rx(&resp_slot);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us + 1000) == l_false); /* complete */
}

TEST_CASE("master diag schedule: a multi-frame request is pending until its last frame, only then is a response awaited", "[open_lin_transport_layer][tp][master][sched]") {
	s_fake_time_us = 20000;
	uint8_t msg[10] = {0xB0, 0xFF, 0x7F, 0xFF, 0xFF, 0x01, 0xAA, 0xBB, 0xCC, 0xDD};
	REQUIRE(open_lin_master_diag_request(0x05, msg, sizeof(msg)) == l_true);

	uint8_t frame[8];
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, frame};
	open_lin_master_transport_layer_handle_tx(&slot); /* FF */
	CHECK(open_lin_master_diag_tx_pending() == l_true);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us) == l_false);
	open_lin_master_transport_layer_handle_tx(&slot); /* CF, the last one */
	CHECK(open_lin_master_diag_tx_pending() == l_false);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us) == l_true);
}

TEST_CASE("master diag schedule: no response to a request is awaited past P2max", "[open_lin_transport_layer][tp][master][sched]") {
	s_fake_time_us = 30000;
	uint8_t msg[] = {0xB2, 0x00};
	REQUIRE(open_lin_master_diag_request(0x05, msg, sizeof(msg)) == l_true);
	drain_master_request();

	CHECK(open_lin_master_diag_response_expected(s_fake_time_us + OPEN_LIN_MASTER_DIAG_P2_MAX_US) == l_true);      /* at P2max, not past it */
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us + OPEN_LIN_MASTER_DIAG_P2_MAX_US + 1u) == l_false); /* gave up */
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us) == l_false);                                        /* and stays given up */
}

TEST_CASE("master diag schedule: a functional request gets no response, so none is awaited", "[open_lin_transport_layer][tp][master][sched]") {
	s_fake_time_us = 40000;
	uint8_t msg[] = {0xB2, 0x00};
	REQUIRE(open_lin_master_diag_request(OPEN_LIN_NAD_FUNCTIONAL, msg, sizeof(msg)) == l_true);
	CHECK(open_lin_master_diag_tx_pending() == l_true);
	drain_master_request();
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us) == l_false);
}

TEST_CASE("master diag schedule: a go-to-sleep frame is pending, and expects no response", "[open_lin_transport_layer][tp][master][sched]") {
	REQUIRE(open_lin_master_send_go_to_sleep() == l_true);
	CHECK(open_lin_master_diag_tx_pending() == l_true);
	drain_master_request();
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us) == l_false);
}

TEST_CASE("master diag schedule: an idle 0x3C slot does not start a response wait", "[open_lin_transport_layer][tp][master][sched]") {
	uint8_t frame[8];
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, frame};
	open_lin_master_transport_layer_handle_tx(&slot); /* nothing queued: legacy tables send this every cycle */
	CHECK(open_lin_master_diag_tx_pending() == l_false);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us) == l_false);
}


/* ---- NRC 0x78 "response pending" (Diagnostic Spec 5.6): keep waiting, now for P2* ---- */
static void send_physical_request_and_drain(l_u32 at_us)
{
	s_fake_time_us = at_us;
	uint8_t msg[] = {0xB2, 0x00};
	REQUIRE(open_lin_master_diag_request(0x05, msg, sizeof(msg)) == l_true);
	drain_master_request();
}

TEST_CASE("master diag pending: NRC 0x78 is not handed to the app and the wait continues for P2*", "[open_lin_transport_layer][tp][master][sched]") {
	send_physical_request_and_drain(100000);
	s_master_diag_response_called = l_false;

	s_fake_time_us = 100000 + 400000; /* the slave answers "pending" within P2max */
	uint8_t pending[8] = {0x05, 0x03, 0x7F, 0xB2, 0x78, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t pending_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, pending};
	open_lin_master_transport_layer_handle_rx(&pending_slot);

	CHECK(s_master_diag_response_called == l_false);
	/* well past P2max counted from the request, but within P2* counted from the pending frame */
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us + OPEN_LIN_MASTER_DIAG_P2_MAX_US + 1u) == l_true);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us + OPEN_LIN_MASTER_DIAG_P2_STAR_US) == l_true);      /* at P2*, not past it */
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us + OPEN_LIN_MASTER_DIAG_P2_STAR_US + 1u) == l_false); /* gave up */
}

TEST_CASE("master diag pending: the final response after NRC 0x78 is delivered and ends the wait", "[open_lin_transport_layer][tp][master][sched]") {
	send_physical_request_and_drain(200000);
	uint8_t pending[8] = {0x05, 0x03, 0x7F, 0xB2, 0x78, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t pending_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, pending};
	open_lin_master_transport_layer_handle_rx(&pending_slot);

	s_master_diag_response_called = l_false;
	s_fake_time_us += 1000000;
	uint8_t final_resp[8] = {0x05, 0x03, 0xF2, 0x11, 0x22, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t final_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, final_resp};
	open_lin_master_transport_layer_handle_rx(&final_slot);

	CHECK(s_master_diag_response_called == l_true);
	CHECK(s_master_diag_response_buf[0] == 0xF2);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us) == l_false);
}

TEST_CASE("master diag pending: a repeated NRC 0x78 restarts the P2* wait", "[open_lin_transport_layer][tp][master][sched]") {
	send_physical_request_and_drain(300000);
	uint8_t pending[8] = {0x05, 0x03, 0x7F, 0xB2, 0x78, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t pending_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, pending};
	open_lin_master_transport_layer_handle_rx(&pending_slot);
	s_fake_time_us += OPEN_LIN_MASTER_DIAG_P2_STAR_US - 1000u;
	open_lin_master_transport_layer_handle_rx(&pending_slot); /* still pending, just before the first wait would have run out */

	CHECK(open_lin_master_diag_response_expected(s_fake_time_us + OPEN_LIN_MASTER_DIAG_P2_STAR_US) == l_true);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us + OPEN_LIN_MASTER_DIAG_P2_STAR_US + 1u) == l_false);
}

TEST_CASE("master diag pending: other negative responses are final and delivered", "[open_lin_transport_layer][tp][master][sched]") {
	send_physical_request_and_drain(400000);
	s_master_diag_response_called = l_false;
	uint8_t negative[8] = {0x05, 0x03, 0x7F, 0xB2, 0x22, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t negative_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, negative};
	open_lin_master_transport_layer_handle_rx(&negative_slot);

	CHECK(s_master_diag_response_called == l_true);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us) == l_false);
}

TEST_CASE("master diag pending: a new request goes back to P2max", "[open_lin_transport_layer][tp][master][sched]") {
	send_physical_request_and_drain(500000);
	uint8_t pending[8] = {0x05, 0x03, 0x7F, 0xB2, 0x78, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t pending_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, pending};
	open_lin_master_transport_layer_handle_rx(&pending_slot);

	send_physical_request_and_drain(s_fake_time_us + 10000);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us + OPEN_LIN_MASTER_DIAG_P2_MAX_US) == l_true);
	CHECK(open_lin_master_diag_response_expected(s_fake_time_us + OPEN_LIN_MASTER_DIAG_P2_MAX_US + 1u) == l_false);
}

/* ---- debug hook: slave answers "response pending" first, the real response after a delay ---- */
TEST_CASE("slave debug pending: NRC 0x78 goes out first, the real response only after the delay", "[open_lin_transport_layer][tp][debug]") {
	open_lin_NAD = 0x01;
	open_lin_supplier_id = 0x1234;
	open_lin_function_id = 0x5678;
	s_fake_time_us = 1000000;
	open_lin_transport_layer_debug_response_pending(300000);

	uint8_t req[8] = {0x01, 0x06, 0xB0, 0x34, 0x12, 0x78, 0x56, 0x22}; /* Assign NAD */
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[2] == 0x7F);
	CHECK(resp[3] == 0xB0);
	CHECK(resp[4] == 0x78);

	open_lin_transport_layer_poll(s_fake_time_us + 299999);   /* too early (poll only, the clock stays) */
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[0] == 0xFF); /* idle */

	s_fake_time_us += 300000; /* the clock moves on with the poll, as on the board */
	open_lin_transport_layer_poll(s_fake_time_us);
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[0] == 0x01);
	CHECK(resp[2] == 0xF0); /* the real positive response */
}

TEST_CASE("slave debug pending: one shot, the next response is immediate again", "[open_lin_transport_layer][tp][debug]") {
	open_lin_NAD = 0x22;
	open_lin_supplier_id = 0x1234;
	open_lin_function_id = 0x5678;
	uint8_t req[8] = {0x22, 0x06, 0xB0, 0x34, 0x12, 0x78, 0x56, 0x22};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[2] == 0xF0);
}

/* ---- slave: a response that is ready later than P2max after its request is discarded ---- */
static int s_last_error_seen;
static void send_assign_nad_request(l_u32 at_us)
{
	open_lin_NAD = 0x01;
	open_lin_supplier_id = 0x1234;
	open_lin_function_id = 0x5678;
	s_fake_time_us = at_us;
	uint8_t req[8] = {0x01, 0x06, 0xB0, 0x34, 0x12, 0x78, 0x56, 0x01}; /* Assign NAD, new NAD = 0x01 */
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);
}

TEST_CASE("slave P2max: a response polled within P2max is sent", "[open_lin_transport_layer][tp][p2max]") {
	send_assign_nad_request(5000000);
	s_fake_time_us += OPEN_LIN_MASTER_DIAG_P2_MAX_US; /* at P2max, not past it */
	s_last_error = OPEN_LIN_NO_ERROR;
	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[2] == 0xF0);
	CHECK(s_last_error == OPEN_LIN_NO_ERROR);
}

TEST_CASE("slave P2max: a response polled after P2max is discarded and reported, the slot goes out idle", "[open_lin_transport_layer][tp][p2max]") {
	send_assign_nad_request(6000000);
	s_fake_time_us += OPEN_LIN_MASTER_DIAG_P2_MAX_US + 1u;
	s_last_error = OPEN_LIN_NO_ERROR;
	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[0] == 0xFF);
	CHECK(s_last_error == OPEN_LIN_SLAVE_ERROR_RESPONSE_DISCARDED);

	/* and it is gone for good: not sent by a later poll either */
	s_fake_time_us -= OPEN_LIN_MASTER_DIAG_P2_MAX_US;
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[0] == 0xFF);
}

TEST_CASE("slave P2max: after a response pending the real response gets P2* from when it was ready", "[open_lin_transport_layer][tp][p2max]") {
	open_lin_transport_layer_debug_response_pending(800000); /* longer than P2max, shorter than P2* */
	send_assign_nad_request(7000000);
	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[4] == 0x78); /* the pending went out within P2max */

	s_fake_time_us += 800000;
	open_lin_transport_layer_poll(s_fake_time_us); /* real response ready now */
	s_fake_time_us += OPEN_LIN_MASTER_DIAG_P2_STAR_US; /* polled at P2*, not past it */
	s_last_error = OPEN_LIN_NO_ERROR;
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[2] == 0xF0);
	CHECK(s_last_error == OPEN_LIN_NO_ERROR);
}

TEST_CASE("slave P2max: a real response that waits past P2* after a response pending is discarded", "[open_lin_transport_layer][tp][p2max]") {
	open_lin_transport_layer_debug_response_pending(100000);
	send_assign_nad_request(8000000);
	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot); /* pending */
	s_fake_time_us += 100000;
	open_lin_transport_layer_poll(s_fake_time_us);
	s_fake_time_us += OPEN_LIN_MASTER_DIAG_P2_STAR_US + 1u;
	s_last_error = OPEN_LIN_NO_ERROR;
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[0] == 0xFF);
	CHECK(s_last_error == OPEN_LIN_SLAVE_ERROR_RESPONSE_DISCARDED);
}

/* ---- The application's own services (0x22 / 0x2E style), open_lin_transport_layer_set_user_service() ---- */
static int s_user_service_calls = 0;
static l_u8 s_user_service_sid = 0;
static l_u8 s_user_service_request[16];
static l_u16 s_user_service_request_length = 0;
static t_open_lin_sid_result s_user_service_result = OPEN_LIN_SID_RESULT_NO_RESPONSE;
static l_u16 s_user_service_response_length = 0;
static l_u8 s_user_service_nrc = 0;

static t_open_lin_sid_result test_user_service(l_u8 sid, const l_u8 *request, l_u16 request_length, l_u8 *response, l_u16 *response_length, l_u8 *nrc)
{
	s_user_service_calls++;
	s_user_service_sid = sid;
	s_user_service_request_length = request_length;
	for (l_u16 i = 0; (i < request_length) && (i < sizeof(s_user_service_request)); i++)
	{
		s_user_service_request[i] = request[i];
	}
	for (l_u16 i = 0; (i < s_user_service_response_length) && (i < *response_length); i++)
	{
		response[i] = (l_u8)(0xA0 + i);
	}
	*response_length = s_user_service_response_length;
	*nrc = s_user_service_nrc;
	return s_user_service_result;
}

static void user_service_setup(t_open_lin_sid_result result, l_u16 response_length, l_u8 nrc)
{
	open_lin_NAD = 0x01;
	s_user_service_calls = 0;
	s_user_service_result = result;
	s_user_service_response_length = response_length;
	s_user_service_nrc = nrc;
	open_lin_transport_layer_set_user_service(test_user_service);
}

TEST_CASE("user service: a request for an unknown SID reaches the application and a short answer is a single frame", "[open_lin_transport_layer][tp][user]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);

	uint8_t req[8] = {0x01, 0x03, 0x22, 0xF1, 0x01, 0xFF, 0xFF, 0xFF}; /* Read Data By Identifier, DID 0xF101 */
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	CHECK(s_user_service_calls == 1);
	CHECK(s_user_service_sid == 0x22);
	REQUIRE(s_user_service_request_length == 2u); /* the data after the SID */
	CHECK(s_user_service_request[0] == 0xF1);
	CHECK(s_user_service_request[1] == 0x01);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[0] == 0x01);
	CHECK((resp[1] >> 4) == OPEN_LIN_PCI_SF);
	CHECK((resp[1] & 0x0F) == 4); /* RSID + 3 data bytes */
	CHECK(resp[2] == 0x62);       /* RSID = SID(0x22) + 0x40 */
	CHECK(resp[3] == 0xA0);
	CHECK(resp[4] == 0xA1);
	CHECK(resp[5] == 0xA2);
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("user service: an answer longer than a single frame goes out as FF + CF", "[open_lin_transport_layer][tp][user]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 10u, 0u);

	uint8_t req[8] = {0x01, 0x03, 0x22, 0xF1, 0x00, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK((resp[1] >> 4) == OPEN_LIN_PCI_FF);
	CHECK(resp[2] == 11); /* RSID + 10 data bytes */
	CHECK(resp[3] == 0x62);
	CHECK(resp[4] == 0xA0);

	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK((resp[1] >> 4) == OPEN_LIN_PCI_CF);
	CHECK(resp[2] == 0xA4); /* the 5th data byte follows the FF's RSID + 4 */
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("user service: a negative result carries the application's NRC", "[open_lin_transport_layer][tp][user]") {
	user_service_setup(OPEN_LIN_SID_RESULT_NEGATIVE, 0u, 0x31);

	uint8_t req[8] = {0x01, 0x03, 0x22, 0x12, 0x34, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK((resp[1] & 0x0F) == 3);
	CHECK(resp[2] == 0x7F);
	CHECK(resp[3] == 0x22);
	CHECK(resp[4] == 0x31);
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("user service: no response from the application keeps the node silent", "[open_lin_transport_layer][tp][user]") {
	user_service_setup(OPEN_LIN_SID_RESULT_NO_RESPONSE, 0u, 0u);

	uint8_t req[8] = {0x01, 0x03, 0x2E, 0xF1, 0x00, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);
	CHECK(s_user_service_calls == 1);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	for (int i = 0; i < 8; i++) {
		CHECK(resp[i] == 0xFF);
	}
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("user service: an answer that does not fit the message buffer becomes a negative response", "[open_lin_transport_layer][tp][user]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 200u, 0u);

	uint8_t req[8] = {0x01, 0x03, 0x22, 0xF1, 0x00, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[2] == 0x7F);
	CHECK(resp[3] == 0x22);
	CHECK(resp[4] == 0x14); /* response too long */
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("addressing: a broadcast request is handled like one for the node's own NAD, answer included", "[open_lin_transport_layer][tp][user][addressing]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);

	uint8_t req[8] = {OPEN_LIN_NAD_BROADCAST, 0x03, 0x22, 0xF1, 0x01, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);
	CHECK(s_user_service_calls == 1);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[0] == OPEN_LIN_NAD_BROADCAST); /* Table 4.8: the response echoes the request's NAD */
	CHECK(resp[2] == 0x62);
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("addressing: a broadcast request for an unknown SID gets the negative response like a physical one", "[open_lin_transport_layer][tp][addressing]") {
	open_lin_NAD = 0x01;
	open_lin_transport_layer_set_user_service(0);

	uint8_t req[8] = {OPEN_LIN_NAD_BROADCAST, 0x01, 0xB6, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[2] == 0x7F);
	CHECK(resp[3] == 0xB6);
	CHECK(resp[4] == 0x11);
}

TEST_CASE("addressing: a functional request is processed but never answered", "[open_lin_transport_layer][tp][user][addressing]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);

	uint8_t req[8] = {OPEN_LIN_NAD_FUNCTIONAL, 0x03, 0x2E, 0xF1, 0x00, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);
	CHECK(s_user_service_calls == 1); /* the application saw it ... */

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	for (int i = 0; i < 8; i++) {
		CHECK(resp[i] == 0xFF); /* ... but whatever it returned is discarded */
	}
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("addressing: a functional request to a built-in service is processed and its response discarded", "[open_lin_transport_layer][tp][addressing]") {
	open_lin_NAD = 0x01;
	open_lin_supplier_id = 0x1234;
	open_lin_function_id = 0x5678;
	open_lin_transport_layer_set_user_service(0);

	uint8_t req[8] = {OPEN_LIN_NAD_FUNCTIONAL, 0x06, 0xB0, 0x34, 0x12, 0x78, 0x56, 0x44}; /* Assign NAD, new NAD 0x44 */
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);
	CHECK(open_lin_NAD == 0x44); /* processed */

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	for (int i = 0; i < 8; i++) {
		CHECK(resp[i] == 0xFF);
	}
	open_lin_NAD = 0x01;
}

TEST_CASE("addressing: a functional First Frame is ignored, functional messages are single frames only", "[open_lin_transport_layer][tp][user][addressing]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);

	uint8_t ff[8] = {OPEN_LIN_NAD_FUNCTIONAL, 0x10, 0x0A, 0x22, 0xF1, 0x00, 0x11, 0x22};
	uint8_t cf[8] = {OPEN_LIN_NAD_FUNCTIONAL, 0x21, 0x33, 0x44, 0x55, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, ff};
	open_lin_transport_layer_handle(&slot);
	slot.data_ptr = cf;
	open_lin_transport_layer_handle(&slot);
	CHECK(s_user_service_calls == 0);
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("addressing: a functional request in the middle of a physical reception is ignored and does not abort it", "[open_lin_transport_layer][tp][user][addressing]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);

	uint8_t ff[8] = {0x01, 0x10, 0x0A, 0x22, 0xF1, 0x00, 0x11, 0x22}; /* physical First Frame, 10 bytes */
	uint8_t functional[8] = {OPEN_LIN_NAD_FUNCTIONAL, 0x03, 0x2E, 0xF1, 0x00, 0xFF, 0xFF, 0xFF};
	uint8_t cf[8] = {0x01, 0x21, 0x33, 0x44, 0x55, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, ff};
	open_lin_transport_layer_handle(&slot);
	slot.data_ptr = functional;
	open_lin_transport_layer_handle(&slot);
	CHECK(s_user_service_calls == 0); /* ignored */
	slot.data_ptr = cf;
	open_lin_transport_layer_handle(&slot);
	CHECK(s_user_service_calls == 1); /* the physical message still completed */
	CHECK(s_user_service_sid == 0x22);
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("addressing: a functional request is ignored while a physical response is pending", "[open_lin_transport_layer][tp][user][addressing]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);

	uint8_t physical[8] = {0x01, 0x03, 0x22, 0xF1, 0x01, 0xFF, 0xFF, 0xFF};
	uint8_t functional[8] = {OPEN_LIN_NAD_FUNCTIONAL, 0x03, 0x2E, 0xF1, 0x00, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, physical};
	open_lin_transport_layer_handle(&slot);
	CHECK(s_user_service_calls == 1); /* the response to it is now pending */

	slot.data_ptr = functional;
	open_lin_transport_layer_handle(&slot);
	CHECK(s_user_service_calls == 1); /* Diagnostic Spec 5.5: ignored, not processed */

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[2] == 0x62); /* the physical response is untouched */
	open_lin_transport_layer_set_user_service(0);
}
/* ---- Slave transmission handler, Diagnostic Spec 5.5 / Transport Layer 3.2.2: another request ends what was going on ---- */
static void expect_idle_response(void)
{
	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	for (int i = 0; i < 8; i++) {
		CHECK(resp[i] == 0xFF);
	}
}

TEST_CASE("handler: a request for another node stops a reception in progress", "[open_lin_transport_layer][tp][user][handler]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);

	uint8_t ff[8]    = {0x01, 0x10, 0x0A, 0x22, 0xF1, 0x00, 0x11, 0x22}; /* for this node, 10 bytes */
	uint8_t other[8] = {0x02, 0x03, 0x22, 0xF1, 0x01, 0xFF, 0xFF, 0xFF}; /* for another node */
	uint8_t cf[8]    = {0x01, 0x21, 0x33, 0x44, 0x55, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, ff};
	open_lin_transport_layer_handle(&slot);
	slot.data_ptr = other;
	open_lin_transport_layer_handle(&slot);
	slot.data_ptr = cf;
	open_lin_transport_layer_handle(&slot);
	CHECK(s_user_service_calls == 0); /* the message was dropped, the late CF completes nothing */
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("handler: a Consecutive Frame for another node also stops a reception in progress", "[open_lin_transport_layer][tp][user][handler]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);

	uint8_t ff[8]       = {0x01, 0x10, 0x0A, 0x22, 0xF1, 0x00, 0x11, 0x22};
	uint8_t other_cf[8] = {0x02, 0x21, 0x99, 0x99, 0x99, 0xFF, 0xFF, 0xFF};
	uint8_t cf[8]       = {0x01, 0x21, 0x33, 0x44, 0x55, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, ff};
	open_lin_transport_layer_handle(&slot);
	slot.data_ptr = other_cf;
	open_lin_transport_layer_handle(&slot);
	slot.data_ptr = cf;
	open_lin_transport_layer_handle(&slot);
	CHECK(s_user_service_calls == 0);
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("handler: a request for another node discards a response that is pending", "[open_lin_transport_layer][tp][user][handler]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);

	uint8_t physical[8] = {0x01, 0x03, 0x22, 0xF1, 0x01, 0xFF, 0xFF, 0xFF};
	uint8_t other[8]    = {0x02, 0x03, 0x22, 0xF1, 0x01, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, physical};
	open_lin_transport_layer_handle(&slot);
	slot.data_ptr = other;
	open_lin_transport_layer_handle(&slot);
	CHECK(s_user_service_calls == 1); /* the other node's request is not ours */
	expect_idle_response();
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("handler: a new request for this node discards the response of the previous one at once", "[open_lin_transport_layer][tp][user][handler]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);

	uint8_t first[8] = {0x01, 0x03, 0x22, 0xF1, 0x01, 0xFF, 0xFF, 0xFF};
	uint8_t ff[8]    = {0x01, 0x10, 0x0A, 0x22, 0xF1, 0x00, 0x11, 0x22}; /* a long new request, not complete yet */
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, first};
	open_lin_transport_layer_handle(&slot);
	slot.data_ptr = ff;
	open_lin_transport_layer_handle(&slot);
	CHECK(s_user_service_calls == 1);
	expect_idle_response(); /* the old response is gone although the new request has not been answered yet */
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("handler: a broadcast request also discards the previous response", "[open_lin_transport_layer][tp][user][handler]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);

	uint8_t first[8]     = {0x01, 0x03, 0x22, 0xF1, 0x01, 0xFF, 0xFF, 0xFF};
	uint8_t broadcast[8] = {OPEN_LIN_NAD_BROADCAST, 0x10, 0x0A, 0x22, 0xF1, 0x00, 0x11, 0x22};
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, first};
	open_lin_transport_layer_handle(&slot);
	slot.data_ptr = broadcast;
	open_lin_transport_layer_handle(&slot);
	expect_idle_response();
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("handler: a response being sent in several frames stops when a request for another node arrives", "[open_lin_transport_layer][tp][user][handler]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 10u, 0u); /* FF + CF response */

	uint8_t physical[8] = {0x01, 0x03, 0x22, 0xF1, 0x00, 0xFF, 0xFF, 0xFF};
	uint8_t other[8]    = {0x02, 0x03, 0x22, 0xF1, 0x01, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, physical};
	open_lin_transport_layer_handle(&slot);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK((resp[1] >> 4) == OPEN_LIN_PCI_FF); /* the first frame went out */

	slot.data_ptr = other;
	open_lin_transport_layer_handle(&slot);
	expect_idle_response(); /* the rest of the response is dropped */
	open_lin_transport_layer_set_user_service(0);
}
/* ---- open_lin_transport_layer_handle_tx() says whether there is a response frame to send (Diagnostic Spec 5.5) ---- */
TEST_CASE("handle_tx: no response pending means no frame to send", "[open_lin_transport_layer][tp][handler][silent]") {
	open_lin_NAD = 0x01;
	open_lin_transport_layer_set_user_service(0);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	/* an earlier test may have left a response behind: take whatever is pending first */
	while (open_lin_transport_layer_handle_tx(&resp_slot) == l_true) {}
	CHECK(open_lin_transport_layer_handle_tx(&resp_slot) == l_false);
}

TEST_CASE("handle_tx: every frame of a response is reported, then silence", "[open_lin_transport_layer][tp][handler][silent]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 10u, 0u); /* FF + CF */

	uint8_t req[8] = {0x01, 0x03, 0x22, 0xF1, 0x00, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	CHECK(open_lin_transport_layer_handle_tx(&resp_slot) == l_true);  /* FF */
	CHECK(open_lin_transport_layer_handle_tx(&resp_slot) == l_true);  /* CF */
	CHECK(open_lin_transport_layer_handle_tx(&resp_slot) == l_false); /* done: the next header is not answered */
	open_lin_transport_layer_set_user_service(0);
}

TEST_CASE("handle_tx: a response discarded for being late is not answered with an idle frame", "[open_lin_transport_layer][tp][handler][silent]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);
	s_fake_time_us = 1000u;

	uint8_t req[8] = {0x01, 0x03, 0x22, 0xF1, 0x01, 0xFF, 0xFF, 0xFF};
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);

	s_fake_time_us += OPEN_LIN_MASTER_DIAG_P2_MAX_US + 1u;
	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	CHECK(open_lin_transport_layer_handle_tx(&resp_slot) == l_false);
	CHECK(s_last_error == OPEN_LIN_SLAVE_ERROR_RESPONSE_DISCARDED);
	open_lin_transport_layer_set_user_service(0);
	s_fake_time_us = 0u;
}
TEST_CASE("user service: a service the library handles itself is not handed over", "[open_lin_transport_layer][tp][user]") {
	user_service_setup(OPEN_LIN_SID_RESULT_POSITIVE, 3u, 0u);
	open_lin_supplier_id = 0x1234;
	open_lin_function_id = 0x5678;

	uint8_t req[8] = {0x01, 0x06, 0xB2, 0x00, 0x34, 0x12, 0x78, 0x56}; /* Read By Identifier id=0 */
	open_lin_frame_slot_t req_slot = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, req};
	open_lin_transport_layer_handle(&req_slot);
	CHECK(s_user_service_calls == 0);

	uint8_t resp[8];
	open_lin_frame_slot_t resp_slot = {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, resp};
	open_lin_transport_layer_handle_tx(&resp_slot);
	CHECK(resp[2] == 0xF2);
	open_lin_transport_layer_set_user_service(0);
}