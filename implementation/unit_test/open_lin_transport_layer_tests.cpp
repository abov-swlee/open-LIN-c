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

/* Minimal HW time stub for open_lin_transport_layer_handle()'s N_Cr timestamping;
 * the engine-level tests below (open_lin_tp_rx_feed/open_lin_tp_tx_next) take the
 * timestamp as a parameter instead, so they don't depend on this. */
static l_u32 s_fake_time_us = 0;
l_u32 open_lin_hw_get_time_us(void) { return s_fake_time_us; }

void open_lin_sid_callback(open_lin_SID_t sid) { (void)sid; }
}

/* frame from Table 4.7 LIN spec 2.1 */
uint8_t request_data_1[] = {OPEN_LIN_NAD_BROADCAST, 0x06,0xB0,0x12,0x34,0x56,0x78,0x34};

open_lin_frame_slot_t slot_1 = {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, request_data_1};

TEST_CASE("change NAD test", "[open_lin_transport_layer]"){
	open_lin_NAD = 0;
	open_lin_transport_layer_handle(&slot_1);
	CHECK(open_lin_NAD == 0x34);
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

