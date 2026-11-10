#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"
#include "iostream"
#include <vector>

extern "C"{

#include "../open_lin_cfg.h"
#include "../../open_lin_data_layer.h"
#include "../../open_lin_slave_data_layer.h"
#include "../../open_lin_hw.h"
#include "../../open_lin_network_layer.h"

extern l_u8 open_lin_frame_parity(l_u8 pid);
extern void open_lin_frame_byte_process(l_u8 rx_byte);

/* open_lin_network_layer.c routes 0x3C/0x3D through these under
 * OPEN_LIN_TRANSPORT_LAYER (always on via open_lin_cfg.h); this test file
 * only exercises the data layer, so no-op stubs are enough here - the real
 * transport layer engine has its own dedicated tests in
 * open_lin_transport_layer_tests.cpp. open_lin_on_tx_frame is the same idea
 * for non-diagnostic TRANSMIT slots (open_lin_on_rx_frame is already
 * provided by open_lin_frame_winOS.c). */
void open_lin_transport_layer_handle(open_lin_frame_slot_t *slot) { (void)slot; }
void open_lin_transport_layer_handle_tx(open_lin_frame_slot_t *slot) { (void)slot; }
void open_lin_on_tx_frame(open_lin_frame_slot_t *slot) { (void)slot; }

l_u8 dataBuffer1[] = {0,0,0,0};
l_u8 dataBuffer2[] = {0,1,2,3,4,5,6,7};
l_u8 dataBuffer3[] = {0,0,0};
l_u8 dataBuffer4[] = {0};
l_u8 dataBuffer5[] = {0,0}; /* event-triggered frame's associated TX frame: byte 0 is reserved for its protected id */

open_lin_frame_slot_t slot_array[] =
{
		{0x12,OPEN_LIN_FRAME_TYPE_RECEIVE,sizeof(dataBuffer1),dataBuffer1},
		{0x01,OPEN_LIN_FRAME_TYPE_TRANSMIT,sizeof(dataBuffer2),dataBuffer2},
		{OPEN_LIN_DIAG_REQUEST,OPEN_LIN_FRAME_TYPE_RECEIVE,sizeof(dataBuffer3),dataBuffer3},
		{0x08,OPEN_LIN_FRAME_TYPE_RECEIVE,sizeof(dataBuffer4),dataBuffer4,OPEN_LIN_CHECKSUM_CLASSIC},
		{0x05,OPEN_LIN_FRAME_TYPE_TRANSMIT,sizeof(dataBuffer5),dataBuffer5}
};


const l_u8 length_of_slot_array = sizeof( slot_array ) / sizeof( open_lin_frame_slot_t );


}

TEST_CASE( "calc_parity", "[open_lin_data_layer]" ) {

	uint16_t id = open_lin_data_layer_parity(0x80);
	INFO("PID " << std::hex << id);
	CHECK(open_lin_data_layer_parity(0x80) == 0x80);
	CHECK(open_lin_data_layer_parity(0) == 0x80);
	CHECK(open_lin_data_layer_parity(12) == 0x4c);
}

TEST_CASE("calc_checksum", "[open_lin_data_layer]" ) {

	t_open_lin_data_layer_frame frame;
	l_u8 data[] = {0x22,0xAA} ;
	frame.length = sizeof(data);
	frame.data_ptr = data;
	frame.pid = 0x15;
	CHECK((uint16_t)open_lin_data_layer_checksum(frame.pid,OPEN_LIN_CHECKSUM_ENHANCED,frame.length,frame.data_ptr) == (uint16_t)0xDD);

	l_u8 data2[] = {0x11} ;
	frame.length = sizeof(data2);
	frame.data_ptr = data2;
	frame.pid = 0x01;
	CHECK((uint16_t)open_lin_data_layer_checksum(frame.pid,OPEN_LIN_CHECKSUM_ENHANCED,frame.length,frame.data_ptr) == (uint16_t)0x2D);
}

TEST_CASE("calc_checksum classic - excludes the pid, unlike enhanced", "[open_lin_data_layer]" ) {
	/* same data as the enhanced case above (pid=0x15, {0x22,0xAA} -> 0xDD);
	 * classic sums only the data bytes, so the pid change (0x15 vs anything
	 * else) must not affect the result, and the value itself must differ
	 * from the enhanced checksum for the same bytes. */
	l_u8 data[] = {0x22,0xAA};
	CHECK((uint16_t)open_lin_data_layer_checksum(0x15,OPEN_LIN_CHECKSUM_CLASSIC,sizeof(data),data) == (uint16_t)0x33);
	CHECK((uint16_t)open_lin_data_layer_checksum(0x2A,OPEN_LIN_CHECKSUM_CLASSIC,sizeof(data),data) == (uint16_t)0x33); /* pid-independent */

	l_u8 data2[] = {0x11};
	CHECK((uint16_t)open_lin_data_layer_checksum(0x01,OPEN_LIN_CHECKSUM_CLASSIC,sizeof(data2),data2) == (uint16_t)0xEE);
}

TEST_CASE("calc_checksum: diagnostic frames stay classic even if checksum_type says enhanced", "[open_lin_data_layer]" ) {
	/* Section 2.3.1.2: diagnostic frames are always classic - the per-slot
	 * checksum_type must not be able to override that. */
	l_u8 data[] = {0x22,0xAA};
	CHECK((uint16_t)open_lin_data_layer_checksum(OPEN_LIN_DIAG_REQUEST,OPEN_LIN_CHECKSUM_ENHANCED,sizeof(data),data) == (uint16_t)0x33);
	CHECK((uint16_t)open_lin_data_layer_checksum(OPEN_LIN_DIAG_RESPONSE,OPEN_LIN_CHECKSUM_ENHANCED,sizeof(data),data) == (uint16_t)0x33);
}

extern "C"{

extern l_bool sim_break;
extern t_open_lin_error get_and_clear_sim_error();

}

TEST_CASE("slave break detection", "[open_lin_slave]" ) {
	open_lin_net_init(slot_array,length_of_slot_array);
	open_lin_slave_reset();
	/* first brake ok */
//	SECTION( "sec brake cause err wait for sync" ){
		sim_break = true;
		open_lin_slave_rx_header(0);
		{
			CHECK(get_and_clear_sim_error() == OPEN_LIN_NO_ERROR);
		}
//	}

//	SECTION( "sec brake cause err wait for sync" ){
		sim_break = true;
		open_lin_slave_rx_header(0);
		CHECK(get_and_clear_sim_error() == OPEN_LIN_SLAVE_ERROR_INVALID_BREAK);

		sim_break = true;
		open_lin_slave_rx_header(0);
		CHECK(get_and_clear_sim_error() == OPEN_LIN_SLAVE_ERROR_INVALID_BREAK);
//	}

//	SECTION( "sync no err" ){
		sim_break = false;
		open_lin_slave_rx_header(0x55);
		CHECK(get_and_clear_sim_error() == OPEN_LIN_NO_ERROR);
//	}
}

TEST_CASE("reset test", "[open_lin_slave]" ) {
	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	CHECK(get_and_clear_sim_error() == OPEN_LIN_NO_ERROR);
}

l_bool tx_invalid = false;
l_bool tx_done = true;

extern l_u8 dataBuffer2[8];

static std::vector<l_u8> g_tx_log; /* everything the stub was asked to transmit, for the event-triggered tests */

l_u8 open_lin_hw_tx_data(l_u8* data, l_u8 len){ /* return type must match open_lin_hw.h's declaration */
	g_tx_log.insert(g_tx_log.end(), data, data + len);
	if (sizeof(dataBuffer2) == len)
	{
		if (memcmp(data,dataBuffer2,len) != 0)
		{
			tx_invalid = true;
		}

	} else
	{
		if (len == 1)
		{
			// checksum
			if (data[0] != 0x22)
			{
				tx_invalid = true;
			} else
			{
				tx_done = true;
			}
		} else
		{
			tx_invalid = true;
		}
	}
	return true;
}

TEST_CASE("frame reception, tx header", "[open_lin_slave]" ) {
	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(0xC1);
	CHECK(tx_invalid == false);
	CHECK(tx_done == true);
}

TEST_CASE("frame reception, rx header invalid parity", "[open_lin_slave]" ) {
	open_lin_slave_init();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(0x93); // 0x01

	CHECK (get_and_clear_sim_error() == OPEN_LIN_SLAVE_ERROR_PID_PARITY);
}

TEST_CASE("frame reception, rx header invalid checksum", "[open_lin_slave]" ) {
	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(0x92); // 0x01

	open_lin_slave_rx_header(0x00);
	open_lin_slave_rx_header(0x01);
	open_lin_slave_rx_header(0x02);
	open_lin_slave_rx_header(0x03);
	open_lin_slave_rx_header(0x22);

	CHECK (get_and_clear_sim_error() == OPEN_LIN_SLAVE_ERROR_INVALID_CHECKSUM);
}

TEST_CASE("diagnostic frame reception, rx header valid checksum", "[open_lin_slave]" ) {
	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(OPEN_LIN_DIAG_REQUEST);

	open_lin_slave_rx_header(0x01);
	open_lin_slave_rx_header(0x02);
	open_lin_slave_rx_header(0x03);
	open_lin_slave_rx_header(0xF9);


	CHECK (get_and_clear_sim_error() == OPEN_LIN_NO_ERROR);
//	CHECK(false);
}

TEST_CASE("frame reception, rx header valid checksum", "[open_lin_slave]" )
{
	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(0x92); // 0x01

	open_lin_slave_rx_header(0x00);
	open_lin_slave_rx_header(0x01);
	open_lin_slave_rx_header(0x02);
	open_lin_slave_rx_header(0x03);
	open_lin_slave_rx_header(0x67);

	extern l_u8 dataBuffer1[4];
	l_u8 e_dataBuffer1[4] = {0,1,2,3};
	CHECK (get_and_clear_sim_error() == OPEN_LIN_NO_ERROR);

	CHECK(memcmp(dataBuffer1,e_dataBuffer1,sizeof(e_dataBuffer1)) == 0);
}

TEST_CASE("frame reception, classic-checksum slot excludes the pid from the checksum", "[open_lin_slave]" )
{
	/* slot_array's 0x08 entry is OPEN_LIN_CHECKSUM_CLASSIC (Table 4.1's default
	 * ENHANCED would compute a different checksum byte for the same data) */
	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(open_lin_data_layer_parity(0x08));

	open_lin_slave_rx_header(0x11); /* one data byte */
	open_lin_slave_rx_header(0xEE); /* classic checksum of {0x11} alone: ~0x11 = 0xEE */

	extern l_u8 dataBuffer4[1];
	CHECK (get_and_clear_sim_error() == OPEN_LIN_NO_ERROR);
	CHECK(dataBuffer4[0] == 0x11);
}

TEST_CASE("response_error: set on a forced error, folded into the designated frame, cleared after a successful send", "[open_lin_slave][response_error]")
{
	open_lin_slave_dl_set_response_error_frame(0x01, 0, 7); /* same wiring as slave.c */

	open_lin_slave_dl_debug_force_response_error();
	CHECK(open_lin_slave_dl_get_response_error() == l_true);

	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(0xC1); /* parity(0x01) */

	CHECK((dataBuffer2[0] & 0x80) != 0); /* bit folded into the designated byte before tx */
	CHECK(open_lin_slave_dl_get_response_error() == l_false); /* cleared once that send succeeded */

	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(0xC1);

	CHECK((dataBuffer2[0] & 0x80) == 0); /* a clean send now carries the bit cleared */

	/* this test deliberately sends a frame whose checksum won't match the
	 * stub's hardcoded 0x22 (the response_error bit mutates dataBuffer2) -
	 * don't leak that into later tests that check tx_invalid. */
	tx_invalid = false;
}

TEST_CASE("response_error: set on invalid checksum of an owned RX frame", "[open_lin_slave][response_error]")
{
	open_lin_slave_dl_set_response_error_frame(0x01, 0, 7);

	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(0x92); // PID 0x12 (slot_array[0], RECEIVE) - any owned frame counts, not just the designated one

	open_lin_slave_rx_header(0x00);
	open_lin_slave_rx_header(0x01);
	open_lin_slave_rx_header(0x02);
	open_lin_slave_rx_header(0x03);
	open_lin_slave_rx_header(0x22); /* wrong checksum on purpose */

	CHECK(get_and_clear_sim_error() == OPEN_LIN_SLAVE_ERROR_INVALID_CHECKSUM);
	CHECK(open_lin_slave_dl_get_response_error() == l_true);

	/* leave it cleared for whichever test runs next */
	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(0xC1);
	tx_invalid = false;
}

/* ---- Event-triggered frame, slave side (Section 2.3.3.2): event id 0x06, associated TX frame 0x05 ---- */
static void send_header(l_u8 id)
{
	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(open_lin_data_layer_parity(id));
}

static void event_test_setup(void)
{
	open_lin_net_init(slot_array, length_of_slot_array);
	open_lin_slave_dl_clear_event_frames();
	open_lin_slave_dl_set_event_frame(0x06, 0x05);
	dataBuffer5[1] = 0x7E;
	g_tx_log.clear();
	get_and_clear_sim_error();
}

TEST_CASE("event-triggered header with no update stays silent", "[open_lin_slave][event]")
{
	event_test_setup();
	send_header(0x06);
	CHECK(g_tx_log.empty());
	CHECK(get_and_clear_sim_error() == OPEN_LIN_NO_ERROR);
}

TEST_CASE("event-triggered header after an update: protected id in byte 0, checksum over the event id, then silent", "[open_lin_slave][event]")
{
	event_test_setup();
	open_lin_slave_dl_event_updated(0x05);
	send_header(0x06);

	l_u8 expected[2] = {open_lin_data_layer_parity(0x05), 0x7E};
	l_u8 cs = open_lin_data_layer_checksum(0x06, OPEN_LIN_CHECKSUM_ENHANCED, 2, expected);
	REQUIRE(g_tx_log.size() == 3);
	CHECK(g_tx_log[0] == expected[0]);
	CHECK(g_tx_log[1] == expected[1]);
	CHECK(g_tx_log[2] == cs);
	CHECK(get_and_clear_sim_error() == OPEN_LIN_NO_ERROR);

	g_tx_log.clear();
	send_header(0x06); /* the signal is no longer updated */
	CHECK(g_tx_log.empty());
	tx_invalid = false;
}

TEST_CASE("garbled event-triggered response leaves the update pending", "[open_lin_slave][event]")
{
	event_test_setup();
	open_lin_slave_dl_event_updated(0x05);
	open_lin_slave_dl_debug_event_garble();
	send_header(0x06);

	l_u8 data[2] = {open_lin_data_layer_parity(0x05), 0x7E};
	l_u8 cs = open_lin_data_layer_checksum(0x06, OPEN_LIN_CHECKSUM_ENHANCED, 2, data);
	REQUIRE(g_tx_log.size() == 3);
	CHECK(g_tx_log[2] == (l_u8)(cs ^ 0xFF));

	g_tx_log.clear();
	send_header(0x06); /* still pending: answers again, now with a good checksum */
	REQUIRE(g_tx_log.size() == 3);
	CHECK(g_tx_log[2] == cs);
	tx_invalid = false;
}

TEST_CASE("sending the associated frame as an ordinary frame clears the pending update", "[open_lin_slave][event]")
{
	event_test_setup();
	open_lin_slave_dl_event_updated(0x05);
	send_header(0x05); /* polled directly, e.g. by the collision resolving table */
	REQUIRE(g_tx_log.size() == 3);

	g_tx_log.clear();
	send_header(0x06);
	CHECK(g_tx_log.empty());
	tx_invalid = false;
}

TEST_CASE("a node can take part in several event-triggered frames, each answered on its own header", "[open_lin_slave][event]")
{
	event_test_setup();
	REQUIRE(open_lin_slave_dl_set_event_frame(0x07, 0x01) == l_true); /* second event frame, associated TX frame 0x01 (8 bytes) */
	dataBuffer2[1] = 0x33;
	open_lin_slave_dl_event_updated(0x01);

	send_header(0x06); /* the 0x06 frame (0x05) was not updated */
	CHECK(g_tx_log.empty());

	send_header(0x07);
	REQUIRE(g_tx_log.size() == 9);
	l_u8 data[8] = {open_lin_data_layer_parity(0x01), 0x33, 2, 3, 4, 5, 6, 7};
	CHECK(g_tx_log[0] == data[0]);
	CHECK(g_tx_log[1] == 0x33);
	CHECK(g_tx_log[8] == open_lin_data_layer_checksum(0x07, OPEN_LIN_CHECKSUM_ENHANCED, 8, data));

	g_tx_log.clear();
	open_lin_slave_dl_event_updated(0x05);
	send_header(0x06); /* now the other one answers, the first stays done */
	CHECK(g_tx_log.size() == 3);
	g_tx_log.clear();
	send_header(0x07);
	CHECK(g_tx_log.empty());
	tx_invalid = false;
	dataBuffer2[0] = 0;
	dataBuffer2[1] = 1;
}

TEST_CASE("event frame registry: re-registering replaces, a full table refuses, clear empties it", "[open_lin_slave][event]")
{
	event_test_setup();
	CHECK(open_lin_slave_dl_set_event_frame(0x0A, 0x05) == l_true); /* same associated frame: replaces, still one entry */
	open_lin_slave_dl_event_updated(0x05);
	send_header(0x06);
	CHECK(g_tx_log.empty()); /* no longer under 0x06 */
	send_header(0x0A);
	CHECK(g_tx_log.size() == 3);
	tx_invalid = false;

	open_lin_slave_dl_clear_event_frames();
	for (l_u8 i = 0u; i < OPEN_LIN_SLAVE_MAX_EVENT_FRAMES; i++)
	{
		CHECK(open_lin_slave_dl_set_event_frame(0x10, (open_lin_pid_t)(0x20 + i)) == l_true);
	}
	CHECK(open_lin_slave_dl_set_event_frame(0x10, 0x30) == l_false);

	open_lin_slave_dl_clear_event_frames();
	open_lin_slave_dl_event_updated(0x05); /* unknown associated frame: nothing happens */
	CHECK(open_lin_slave_dl_set_event_frame(0x06, 0x05) == l_true);
}
#ifdef OPEN_LIN_DYNAMIC_IDS
	open_lin_id_translation_item_t open_lin_id_translation_tab[] = {
			{0x12,0x12},
			{0x01,0x01},
			{OPEN_LIN_DIAG_REQUEST,OPEN_LIN_DIAG_REQUEST}
	};

TEST_CASE("dynamic id test", "[open_lin_slave]" )
{
	open_lin_id_translation_tab[0] = {0x12,0x22};
	CHECK (get_and_clear_sim_error() == OPEN_LIN_NO_ERROR);

	open_lin_slave_reset();
	sim_break = true;
	open_lin_slave_rx_header(0);
	open_lin_slave_rx_header(0x55);
	open_lin_slave_rx_header(0xe2); // 0x22

	open_lin_slave_rx_header(0x00);
	open_lin_slave_rx_header(0x01);
	open_lin_slave_rx_header(0x02);
	open_lin_slave_rx_header(0x03);
	open_lin_slave_rx_header(0x17);

	extern l_u8 dataBuffer1[4];
	l_u8 e_dataBuffer1[4] = {0,1,2,3};
	CHECK (get_and_clear_sim_error() == OPEN_LIN_NO_ERROR);

	CHECK(memcmp(dataBuffer1,e_dataBuffer1,sizeof(e_dataBuffer1)) == 0);
	CHECK (get_and_clear_sim_error() == OPEN_LIN_NO_ERROR);

	open_lin_id_translation_tab[0] = {0x12,0x12};
}

#endif
