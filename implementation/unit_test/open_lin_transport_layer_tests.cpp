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

/* Minimal HW time stub for open_lin_transport_layer_handle()'s N_Cr timestamping;
 * the engine-level tests below (open_lin_tp_rx_feed/open_lin_tp_tx_next) take the
 * timestamp as a parameter instead, so they don't depend on this. */
static l_u32 s_fake_time_us = 0;
l_u32 open_lin_hw_get_time_us(void) { return s_fake_time_us; }

void open_lin_sid_callback(open_lin_SID_t sid) { (void)sid; }
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

