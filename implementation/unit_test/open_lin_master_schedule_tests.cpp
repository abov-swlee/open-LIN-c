/*
 * Schedule-table switching tests for the master data layer.
 * Self-contained: provides its own HW stubs, so it links against only
 * open_lin_master_data_layer.c and open_lin_data_layer.c.
 */
#include "catch.hpp"
#include <vector>

extern "C"{

#include "../open_lin_cfg.h"
#include "../../open_lin_data_layer.h"
#include "../../open_lin_hw.h"
#include "../../open_lin_network_layer.h"
#include "../../open_lin_master_data_layer.h"
#include "../../open_lin_transport_layer.h"

/* ---- HW / callback stubs ---- */
static std::vector<std::vector<uint8_t> > s_frames; /* one entry per break */
static std::vector<uint8_t> s_rx_feed;
static size_t s_rx_pos = 0;
static void (*s_on_rx_callback)(open_lin_frame_slot_t *) = 0;

void open_lin_set_rx_enabled(l_bool status) { (void)status; }
static std::vector<t_open_lin_error> s_errors;
static l_u32 s_time_us = 0;
void open_lin_error_handler(t_open_lin_error error_code) { s_errors.push_back(error_code); }
l_u32 open_lin_hw_get_time_us() { s_time_us += 1000u; return s_time_us; } /* advances so RX timeouts can elapse */
l_bool open_lin_hw_tx_break(void) { s_frames.push_back(std::vector<uint8_t>()); return l_true; }
l_bool open_lin_hw_tx_byte(l_u8 data) { s_frames.back().push_back(data); return l_true; }
l_bool open_lin_hw_rx_byte(l_u8 *data, l_u32 timeout_us)
{
	(void)timeout_us;
	if (s_rx_pos < s_rx_feed.size()) { *data = s_rx_feed[s_rx_pos++]; return l_true; }
	return l_false;
}
/* The hardware stores the bytes that have arrived into the armed buffer; the feed is what the bus delivers. */
static l_u8 *s_hw_rx_buffer = 0;
static l_u8 s_hw_rx_len = 0;
static l_u8 s_hw_rx_count = 0;
static int s_hw_rx_aborts = 0;
void open_lin_hw_rx_data(l_u8 *buffer, l_u8 len) { s_hw_rx_buffer = buffer; s_hw_rx_len = len; s_hw_rx_count = 0; }
l_u8 open_lin_hw_rx_count(void)
{
	while ((s_hw_rx_count < s_hw_rx_len) && (s_rx_pos < s_rx_feed.size())) { s_hw_rx_buffer[s_hw_rx_count++] = s_rx_feed[s_rx_pos++]; }
	return s_hw_rx_count;
}
void open_lin_hw_rx_abort(void) { s_hw_rx_len = 0; s_hw_rx_aborts++; }
/* a background transmission: its bytes join the frame of the last break, it is done when polled */
l_bool open_lin_hw_tx_data(const l_u8 *buffer, l_u8 len) { s_frames.back().insert(s_frames.back().end(), buffer, buffer + len); return l_true; }
l_bool open_lin_hw_tx_is_done(void) { return l_true; }
void open_lin_master_dl_rx_callback(open_lin_frame_slot_t *slot) { if (s_on_rx_callback) s_on_rx_callback(slot); }
/* Diagnostic transport layer stand-in: a request of N frames is pending until N 0x3C slots have run,
 * then a response is awaited until a 0x3D slot delivers one. */
static l_bool s_stub_tx_pending = l_false;
static l_bool s_stub_resp_expected = l_false;
static int s_stub_tx_frames_left = 0;
l_bool open_lin_master_diag_tx_pending(void) { return s_stub_tx_pending; }
l_bool open_lin_master_diag_response_expected(l_u32 now_us) { (void)now_us; return s_stub_resp_expected; }
void open_lin_master_transport_layer_handle_tx(open_lin_frame_slot_t *slot)
{
	(void)slot;
	if ((s_stub_tx_frames_left > 0) && (--s_stub_tx_frames_left == 0))
	{
		s_stub_tx_pending = l_false;
		s_stub_resp_expected = l_true;
	}
}
void open_lin_master_transport_layer_handle_rx(open_lin_frame_slot_t *slot) { (void)slot; s_stub_resp_expected = l_false; }

static l_u8 buf_a1[1] = {0x11}, buf_a2[1] = {0x22};
static l_u8 buf_b1[1] = {0x33}, buf_b2[1] = {0x44}, buf_b3[1] = {0x55};
static l_u8 buf_rx[1];

static t_master_frame_table_item table_a[] = {
	{100000, {0x01, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_a1}, NULL},
	{100000, {0x02, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_a2}, NULL},
};
static t_master_frame_table_item table_b[] = {
	{100000, {0x0A, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_b1}, NULL},
	{100000, {0x0B, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_b2}, NULL},
	{100000, {0x0C, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_b3}, NULL},
};
static t_master_frame_table_item table_c[] = {
	{100000, {0x20, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_a2}, NULL},
};
static t_master_frame_table_item table_rx[] = {
	{100000, {0x03, OPEN_LIN_FRAME_TYPE_RECEIVE, 1, buf_rx}, NULL},
	{100000, {0x04, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_a1}, NULL},
};
}

#define TABLE_SIZE(t) ((l_u8)(sizeof(t) / sizeof((t)[0])))

/* PID byte of the n-th frame sent since reset: frame bytes are 0x55, PID, data..., checksum (the break itself is not recorded) */
static uint8_t sent_pid(size_t n) { return s_frames.at(n).at(1); }

static t_open_lin_master_state s_state = OPEN_LIN_MASTER_IDLE;
static void track_state(t_open_lin_master_state state) { s_state = state; }

static void reset_sim(void)
{
	s_frames.clear();
	s_rx_feed.clear();
	s_rx_pos = 0;
	s_hw_rx_len = 0;
	s_hw_rx_count = 0;
	s_hw_rx_aborts = 0;
	s_on_rx_callback = 0;
	s_errors.clear();
	s_stub_tx_pending = l_false;
	s_stub_resp_expected = l_false;
	s_stub_tx_frames_left = 0;
	s_state = OPEN_LIN_MASTER_IDLE;
	open_lin_master_dl_set_diagnostic_tables(NULL, 0, NULL, 0);
	open_lin_master_dl_set_state_callback(track_state);
}

TEST_CASE("switch_schedule_table rejects NULL and empty tables", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_a, TABLE_SIZE(table_a));
	CHECK(open_lin_master_dl_switch_schedule_table(NULL, 3) == l_false);
	CHECK(open_lin_master_dl_switch_schedule_table(table_b, 0) == l_false);

	/* rejected requests leave the current table running unchanged */
	open_lin_master_dl_handler();
	open_lin_master_dl_handler();
	REQUIRE(s_frames.size() == 2);
	CHECK(sent_pid(0) == open_lin_data_layer_parity(0x01));
	CHECK(sent_pid(1) == open_lin_data_layer_parity(0x02));
}

TEST_CASE("switch_schedule_table between frames starts the new table at entry 0", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_a, TABLE_SIZE(table_a));
	open_lin_master_dl_handler();             /* A[0] */
	REQUIRE(s_frames.size() == 1);

	CHECK(open_lin_master_dl_switch_schedule_table(table_b, TABLE_SIZE(table_b)) == l_true);
	open_lin_master_dl_handler();             /* B[0], not A[1] */
	open_lin_master_dl_handler();             /* B[1] */
	open_lin_master_dl_handler();             /* B[2] */
	open_lin_master_dl_handler();             /* wraps to B[0], stays on B */

	REQUIRE(s_frames.size() == 5);
	CHECK(sent_pid(0) == open_lin_data_layer_parity(0x01));
	CHECK(sent_pid(1) == open_lin_data_layer_parity(0x0A));
	CHECK(sent_pid(2) == open_lin_data_layer_parity(0x0B));
	CHECK(sent_pid(3) == open_lin_data_layer_parity(0x0C));
	CHECK(sent_pid(4) == open_lin_data_layer_parity(0x0A));
}

TEST_CASE("switch_schedule_table requested mid-frame waits for the frame to finish", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_rx, TABLE_SIZE(table_rx));

	/* RX slot 0x03 with a valid response; the switch is requested from inside the rx callback, i.e. mid-frame */
	{
		l_u8 data = 0x05;
		s_rx_feed.push_back(data);
		s_rx_feed.push_back(open_lin_data_layer_checksum(0x03, OPEN_LIN_CHECKSUM_ENHANCED, 1, &data));
	}
	static bool callback_ran;
	callback_ran = false;
	s_on_rx_callback = [](open_lin_frame_slot_t *slot) {
		callback_ran = true;
		CHECK(slot->pid == 0x03);
		CHECK(slot->data_ptr[0] == 0x05);
		open_lin_master_dl_switch_schedule_table(table_b, TABLE_SIZE(table_b));
	};

	open_lin_master_dl_handler();             /* whole RX frame 0x03, request arrives inside it */
	CHECK(callback_ran);
	open_lin_master_dl_handler();             /* next frame comes from table B, not table_rx[1] (0x04) */
	REQUIRE(s_frames.size() == 2);
	CHECK(sent_pid(0) == open_lin_data_layer_parity(0x03));
	CHECK(sent_pid(1) == open_lin_data_layer_parity(0x0A));
}

TEST_CASE("a later switch request replaces an earlier pending one", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_a, TABLE_SIZE(table_a));
	open_lin_master_dl_switch_schedule_table(table_b, TABLE_SIZE(table_b));
	open_lin_master_dl_switch_schedule_table(table_c, TABLE_SIZE(table_c));
	open_lin_master_dl_handler();
	REQUIRE(s_frames.size() == 1);
	CHECK(sent_pid(0) == open_lin_data_layer_parity(0x20));
}

TEST_CASE("init discards a pending switch request", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_a, TABLE_SIZE(table_a));
	open_lin_master_dl_switch_schedule_table(table_b, TABLE_SIZE(table_b));
	open_lin_master_dl_init(table_a, TABLE_SIZE(table_a));
	open_lin_master_dl_handler();
	REQUIRE(s_frames.size() == 1);
	CHECK(sent_pid(0) == open_lin_data_layer_parity(0x01));
}

/* ---- Event-triggered frames and collision resolving (Section 2.3.3.2) ---- */
static l_u8 buf_ev[2], buf_c1[2], buf_c2[2];
static const open_lin_frame_slot_t cand1 = {0x04, OPEN_LIN_FRAME_TYPE_RECEIVE, 2, buf_c1};
static const open_lin_frame_slot_t cand2 = {0x02, OPEN_LIN_FRAME_TYPE_RECEIVE, 2, buf_c2};
static const open_lin_frame_slot_t *cands[] = {&cand1, &cand2};
static l_u8 buf_poll[1] = {0};
static t_master_frame_table_item collision_table[] = {
	{100000, {0x04, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_poll}, NULL},
	{100000, {0x02, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_poll}, NULL},
};
static const open_lin_event_trigger_t event_trigger = {cands, 2, collision_table, TABLE_SIZE(collision_table)};
static t_master_frame_table_item table_ev[] = {
	{100000, {0x01, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_a1}, NULL},
	{100000, {0x06, OPEN_LIN_FRAME_TYPE_RECEIVE,  2, buf_ev}, &event_trigger},
	{100000, {0x02, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_a2}, NULL},
};
static t_master_frame_table_item table_ev_last[] = {
	{100000, {0x01, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_a1}, NULL},
	{100000, {0x06, OPEN_LIN_FRAME_TYPE_RECEIVE,  2, buf_ev}, &event_trigger},
};

static open_lin_pid_t s_rx_pid;
static l_u8 s_rx_data1;
static int s_rx_count;
static void record_rx(open_lin_frame_slot_t *slot) { s_rx_pid = slot->pid; s_rx_data1 = slot->data_ptr[1]; s_rx_count++; }

/* Response to the 0x06 header: data byte 0 is the responder's protected id; the checksum covers the header's PID (0x06) */
static void feed_event_response(l_u8 protected_id, l_u8 value, bool good_checksum = true)
{
	l_u8 data[2] = {protected_id, value};
	l_u8 cs = open_lin_data_layer_checksum(0x06, OPEN_LIN_CHECKSUM_ENHANCED, 2, data);
	s_rx_feed.push_back(data[0]);
	s_rx_feed.push_back(data[1]);
	s_rx_feed.push_back(good_checksum ? cs : (l_u8)(cs ^ 0xFF));
	s_rx_count = 0;
	s_on_rx_callback = record_rx;
}

/* Runs n frames: the handler is called repeatedly, as lin_process() does, until the next frame has started and
 * finished - a response that never comes takes as many calls as the slot time needs to run out. */
static void run(int n)
{
	for (int i = 0; i < n; i++)
	{
		size_t started = s_frames.size();
		int guard = 0;
		do
		{
			open_lin_master_dl_handler();
		} while (((s_frames.size() == started) || (s_state != OPEN_LIN_MASTER_IDLE)) && (++guard < 100000));
		REQUIRE(guard < 100000);
	}
}

TEST_CASE("event-triggered slot with no response stays silent", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_ev, TABLE_SIZE(table_ev));
	run(3);
	REQUIRE(s_frames.size() == 3);
	CHECK(sent_pid(0) == open_lin_data_layer_parity(0x01));
	CHECK(sent_pid(1) == open_lin_data_layer_parity(0x06));
	CHECK(sent_pid(2) == open_lin_data_layer_parity(0x02));
	CHECK(s_errors.empty());
}

TEST_CASE("event-triggered response is dispatched to the candidate named by data byte 0", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_ev, TABLE_SIZE(table_ev));
	run(1);                                                /* 0x01 */
	feed_event_response(open_lin_data_layer_parity(0x04), 0x77);
	run(2);                                                /* 0x06 answered by the first candidate, then 0x02 */
	CHECK(s_errors.empty());
	CHECK(s_rx_count == 1);
	CHECK(s_rx_pid == 0x04);
	CHECK(buf_c1[0] == open_lin_data_layer_parity(0x04));
	CHECK(buf_c1[1] == 0x77);
	CHECK(sent_pid(s_frames.size() - 1) == open_lin_data_layer_parity(0x02));  /* no table switch */

	reset_sim();
	open_lin_master_dl_init(table_ev, TABLE_SIZE(table_ev));
	run(1);
	feed_event_response(open_lin_data_layer_parity(0x02), 0x55);
	run(1);
	CHECK(s_rx_pid == 0x02);
	CHECK(buf_c2[1] == 0x55);
}

TEST_CASE("event-triggered collision runs the collision table once, then resumes after the event entry", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_ev, TABLE_SIZE(table_ev));
	run(1);                                                /* 0x01 */
	feed_event_response(0xAA, 0xBB, false);                /* garbled: bad checksum */
	run(1);                                                /* 0x06 -> collision */
	REQUIRE(s_errors.size() == 1);
	CHECK(s_errors[0] == OPEN_LIN_MASTER_ERROR_EVENT_COLLISION);
	CHECK(s_rx_count == 0);

	run(5);                                                /* 0x04, 0x02, resume: 0x02, 0x01, 0x06 */
	REQUIRE(s_frames.size() == 7);
	CHECK(sent_pid(2) == open_lin_data_layer_parity(0x04));
	CHECK(sent_pid(3) == open_lin_data_layer_parity(0x02));
	CHECK(sent_pid(4) == open_lin_data_layer_parity(0x02)); /* the entry after the event frame, not 0x01 again */
	CHECK(sent_pid(5) == open_lin_data_layer_parity(0x01));
	CHECK(sent_pid(6) == open_lin_data_layer_parity(0x06));
}

TEST_CASE("collision on the last entry resumes at the first entry", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_ev_last, TABLE_SIZE(table_ev_last));
	run(1);
	feed_event_response(0xAA, 0xBB, false);
	run(4);                                                /* 0x06 collision, 0x04, 0x02, back to 0x01 */
	REQUIRE(s_frames.size() == 5);
	CHECK(sent_pid(2) == open_lin_data_layer_parity(0x04));
	CHECK(sent_pid(3) == open_lin_data_layer_parity(0x02));
	CHECK(sent_pid(4) == open_lin_data_layer_parity(0x01));
}

TEST_CASE("a partial event-triggered response counts as a collision", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_ev_last, TABLE_SIZE(table_ev_last));
	run(1);
	s_rx_feed.push_back(0xAA);                             /* one byte, then silence until the slot times out */
	run(2);
	REQUIRE(s_errors.size() == 1);
	CHECK(s_errors[0] == OPEN_LIN_MASTER_ERROR_EVENT_COLLISION);
	CHECK(sent_pid(2) == open_lin_data_layer_parity(0x04));
}

TEST_CASE("event-triggered response with an unknown PID is an error, not a collision", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_ev, TABLE_SIZE(table_ev));
	run(1);
	feed_event_response(open_lin_data_layer_parity(0x09), 0x01);   /* valid checksum, but 0x09 isn't a candidate */
	run(2);
	REQUIRE(s_errors.size() == 1);
	CHECK(s_errors[0] == OPEN_LIN_MASTER_ERROR_EVENT_UNKNOWN_PID);
	CHECK(s_rx_count == 0);
	CHECK(sent_pid(2) == open_lin_data_layer_parity(0x02));        /* no collision switch */
}

TEST_CASE("frame slot time follows the table entry of the frame just started", "[open_lin_master_schedule]")
{
	static t_master_frame_table_item timed_table[] = {
		{15000, {0x01, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_a1}, NULL},
		{10000, {0x02, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_a2}, NULL},
		{25000, {0x03, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_a2}, NULL},
	};
	reset_sim();
	open_lin_master_dl_init(timed_table, TABLE_SIZE(timed_table));
	open_lin_master_dl_handler();
	CHECK(open_lin_master_dl_get_frame_slot_us() == 15000);
	open_lin_master_dl_handler();
	CHECK(open_lin_master_dl_get_frame_slot_us() == 10000);
	open_lin_master_dl_handler();
	CHECK(open_lin_master_dl_get_frame_slot_us() == 25000);
	open_lin_master_dl_handler();                          /* wraps */
	CHECK(open_lin_master_dl_get_frame_slot_us() == 15000);

	/* a table switch brings the new table's own timing */
	open_lin_master_dl_switch_schedule_table(table_c, TABLE_SIZE(table_c));
	open_lin_master_dl_handler();
	CHECK(open_lin_master_dl_get_frame_slot_us() == 100000);
}

/* ---- Diagnostic schedules (Diagnostic Spec 5.4, Diagnostics Interleaved Mode) ---- */
static l_u8 buf_diag_req[8], buf_diag_resp[8];
static t_master_frame_table_item request_table[] = {
	{100000, {OPEN_LIN_DIAG_REQUEST, OPEN_LIN_FRAME_TYPE_TRANSMIT, 8, buf_diag_req}, NULL},
};
static t_master_frame_table_item response_table[] = {
	{100000, {OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_FRAME_TYPE_RECEIVE, 8, buf_diag_resp}, NULL},
};

static void register_diag_tables(void)
{
	open_lin_master_dl_set_diagnostic_tables(request_table, TABLE_SIZE(request_table), response_table, TABLE_SIZE(response_table));
}

/* A request of `frames` frames is queued: pending until that many 0x3C slots have run, then a response is awaited */
static void queue_request(int frames)
{
	s_stub_tx_pending = l_true;
	s_stub_tx_frames_left = frames;
}

static void feed_diag_response(void)
{
	l_u8 data[8] = {0x01, 0x02, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	for (int i = 0; i < 8; i++) s_rx_feed.push_back(data[i]);
	s_rx_feed.push_back(open_lin_data_layer_checksum(OPEN_LIN_DIAG_RESPONSE, OPEN_LIN_CHECKSUM_ENHANCED, 8, data));
}

static void expect_pids(std::initializer_list<uint8_t> ids)
{
	REQUIRE(s_frames.size() == ids.size());
	size_t n = 0;
	for (uint8_t id : ids)
	{
		CHECK(sent_pid(n) == open_lin_data_layer_parity(id));
		n++;
	}
}

TEST_CASE("diagnostic schedules: request schedule per request frame, then response schedule until the response arrives, each between full normal runs", "[open_lin_master_schedule][diag]")
{
	reset_sim();
	register_diag_tables();
	open_lin_master_dl_init(table_a, TABLE_SIZE(table_a));
	run(1);                                    /* 0x01: the normal schedule is mid-run */
	queue_request(2);                          /* a 2 frame request arrives */
	feed_diag_response();
	run(11);
	expect_pids({0x01, 0x02,                   /* the running normal schedule finishes first */
	             0x3C, 0x01, 0x02,             /* request schedule, then a normal run */
	             0x3C, 0x01, 0x02,             /* request schedule for the last request frame, normal run */
	             0x3D, 0x01, 0x02,             /* response schedule collects the response, normal run */
	             0x01});                       /* nothing left to do */
	CHECK(s_errors.empty());
}

TEST_CASE("diagnostic schedules: response schedule repeats between normal runs while no response comes, until none is expected", "[open_lin_master_schedule][diag]")
{
	reset_sim();
	register_diag_tables();
	open_lin_master_dl_init(table_a, TABLE_SIZE(table_a));
	s_stub_resp_expected = l_true;
	run(2);                                    /* 0x01, 0x02 */
	run(1);                                    /* 0x3D, unanswered */
	run(2);
	run(1);                                    /* 0x3D again */
	s_stub_resp_expected = l_false;            /* P2max gives out */
	run(4);
	expect_pids({0x01, 0x02, 0x3D, 0x01, 0x02, 0x3D, 0x01, 0x02, 0x01, 0x02});
	REQUIRE(s_errors.size() == 2);
	CHECK(s_errors[0] == OPEN_LIN_MASTER_ERROR_FRAME_SLOT_TIMEOUT);
	CHECK(s_errors[1] == OPEN_LIN_MASTER_ERROR_FRAME_SLOT_TIMEOUT);
}

TEST_CASE("diagnostic schedules: without registered tables a pending request changes nothing", "[open_lin_master_schedule][diag]")
{
	reset_sim();
	open_lin_master_dl_init(table_a, TABLE_SIZE(table_a));
	queue_request(1);
	run(4);
	expect_pids({0x01, 0x02, 0x01, 0x02});
}

TEST_CASE("diagnostic schedules: a table switch requested by the application cancels a diagnostic schedule about to run", "[open_lin_master_schedule][diag]")
{
	reset_sim();
	register_diag_tables();
	open_lin_master_dl_init(table_a, TABLE_SIZE(table_a));
	queue_request(1);
	run(2);                                    /* 0x01, 0x02: the request schedule is now the active table, not yet run */
	open_lin_master_dl_switch_schedule_table(table_b, TABLE_SIZE(table_b));
	run(3);
	expect_pids({0x01, 0x02, 0x0A, 0x0B, 0x0C});
}

TEST_CASE("diagnostic schedules: a collision on the last entry still lets a waiting request go once resolved", "[open_lin_master_schedule][diag]")
{
	reset_sim();
	register_diag_tables();
	open_lin_master_dl_init(table_ev_last, TABLE_SIZE(table_ev_last));
	queue_request(1);
	run(1);                                    /* 0x01 */
	feed_event_response(0xAA, 0xBB, false);
	run(1);                                    /* 0x06 collides - and being the last entry, the normal run ends here */
	run(3);                                    /* 0x04, 0x02, then the request schedule instead of 0x01 */
	expect_pids({0x01, 0x06, 0x04, 0x02, 0x3C});
}

TEST_CASE("switching tables while a collision is being resolved cancels the resolving", "[open_lin_master_schedule]")
{
	reset_sim();
	open_lin_master_dl_init(table_ev, TABLE_SIZE(table_ev));
	run(1);
	feed_event_response(0xAA, 0xBB, false);
	run(2);                                                /* 0x06 collision, 0x04 */
	open_lin_master_dl_switch_schedule_table(table_c, TABLE_SIZE(table_c));
	run(3);                                                /* table_c only: never back to table_ev */
	REQUIRE(s_frames.size() == 6);
	CHECK(sent_pid(3) == open_lin_data_layer_parity(0x20));
	CHECK(sent_pid(4) == open_lin_data_layer_parity(0x20));
	CHECK(sent_pid(5) == open_lin_data_layer_parity(0x20));
}

/* ---- Event-triggered frame rules (Section 2.3.3.2) checked when a table is handed over ---- */
static l_u8 buf_short[1];
static const open_lin_frame_slot_t cand_short  = {0x03, OPEN_LIN_FRAME_TYPE_RECEIVE, 1, buf_short};
static const open_lin_frame_slot_t cand_empty  = {0x03, OPEN_LIN_FRAME_TYPE_RECEIVE, 0, buf_short};
static const open_lin_frame_slot_t cand_classic = {0x02, OPEN_LIN_FRAME_TYPE_RECEIVE, 2, buf_c2, OPEN_LIN_CHECKSUM_CLASSIC};
static const open_lin_frame_slot_t cand_enhanced = {0x04, OPEN_LIN_FRAME_TYPE_RECEIVE, 2, buf_c1, OPEN_LIN_CHECKSUM_ENHANCED};
static const open_lin_frame_slot_t cand_not_polled = {0x07, OPEN_LIN_FRAME_TYPE_RECEIVE, 2, buf_c2};

static bool event_table_valid(const open_lin_frame_slot_t **candidates, l_u8 count)
{
	open_lin_event_trigger_t trigger = {candidates, count, collision_table, TABLE_SIZE(collision_table)};
	t_master_frame_table_item table[] = {
		{100000, {0x06, OPEN_LIN_FRAME_TYPE_RECEIVE, 2, buf_ev}, &trigger},
	};
	reset_sim();
	return open_lin_master_dl_validate_table(table, TABLE_SIZE(table)) == l_true;
}

TEST_CASE("event-triggered rules: a table that follows them is accepted", "[open_lin_master_schedule]")
{
	CHECK(event_table_valid(cands, 2));
	reset_sim();
	CHECK(open_lin_master_dl_init(table_ev, TABLE_SIZE(table_ev)) == l_true); /* and init reports it */
	CHECK(s_errors.empty());
}

TEST_CASE("event-triggered rules: candidates of different length are rejected", "[open_lin_master_schedule]")
{
	const open_lin_frame_slot_t *mixed[] = {&cand1, &cand_short};
	CHECK_FALSE(event_table_valid(mixed, 2));
	REQUIRE(s_errors.size() == 1);
	CHECK(s_errors[0] == OPEN_LIN_MASTER_ERROR_EVENT_CONFIG);
}

TEST_CASE("event-triggered rules: candidates of a different length than the event response are rejected", "[open_lin_master_schedule]")
{
	const open_lin_frame_slot_t *shorter[] = {&cand_short};
	CHECK_FALSE(event_table_valid(shorter, 1));
}

TEST_CASE("event-triggered rules: a candidate without data byte 0 is rejected", "[open_lin_master_schedule]")
{
	const open_lin_frame_slot_t *empty[] = {&cand_empty};
	CHECK_FALSE(event_table_valid(empty, 1));
}

TEST_CASE("event-triggered rules: mixed checksum models are rejected", "[open_lin_master_schedule]")
{
	const open_lin_frame_slot_t *mixed[] = {&cand_enhanced, &cand_classic};
	CHECK_FALSE(event_table_valid(mixed, 2));
}

TEST_CASE("event-triggered rules: no candidates or a NULL list are rejected", "[open_lin_master_schedule]")
{
	CHECK_FALSE(event_table_valid(cands, 0));
	CHECK_FALSE(event_table_valid(NULL, 2));
}

TEST_CASE("event-triggered rules: a collision table that doesn't poll every candidate is rejected", "[open_lin_master_schedule]")
{
	const open_lin_frame_slot_t *with_unpolled[] = {&cand1, &cand_not_polled};
	CHECK_FALSE(event_table_valid(with_unpolled, 2));
}

TEST_CASE("event-triggered rules: init does not run a table that breaks them", "[open_lin_master_schedule]")
{
	const open_lin_frame_slot_t *mixed[] = {&cand1, &cand_short};
	open_lin_event_trigger_t trigger = {mixed, 2, collision_table, TABLE_SIZE(collision_table)};
	t_master_frame_table_item table[] = {
		{100000, {0x01, OPEN_LIN_FRAME_TYPE_TRANSMIT, 1, buf_a1}, NULL},
		{100000, {0x06, OPEN_LIN_FRAME_TYPE_RECEIVE,  2, buf_ev}, &trigger},
	};
	reset_sim();
	CHECK(open_lin_master_dl_init(table, TABLE_SIZE(table)) == l_false);
	for (int i = 0; i < 3; i++) { open_lin_master_dl_handler(); } /* no frame ever starts, so not run() */
	CHECK(s_frames.empty());
	REQUIRE(s_errors.size() == 1);
	CHECK(s_errors[0] == OPEN_LIN_MASTER_ERROR_EVENT_CONFIG);
}

TEST_CASE("event-triggered rules: switch_schedule_table refuses such a table and keeps the current one", "[open_lin_master_schedule]")
{
	const open_lin_frame_slot_t *mixed[] = {&cand1, &cand_short};
	open_lin_event_trigger_t trigger = {mixed, 2, collision_table, TABLE_SIZE(collision_table)};
	t_master_frame_table_item bad[] = {
		{100000, {0x06, OPEN_LIN_FRAME_TYPE_RECEIVE, 2, buf_ev}, &trigger},
	};
	reset_sim();
	open_lin_master_dl_init(table_a, TABLE_SIZE(table_a));
	CHECK(open_lin_master_dl_switch_schedule_table(bad, TABLE_SIZE(bad)) == l_false);
	run(1);
	REQUIRE(s_frames.size() == 1);
	CHECK(sent_pid(0) == open_lin_data_layer_parity(0x01));
}

/* ---- Non-blocking reception: the handler returns while a response is on its way ---- */
TEST_CASE("a response is collected over several handler calls as its bytes arrive", "[open_lin_master_schedule][rx]")
{
	reset_sim();
	open_lin_master_dl_init(table_rx, TABLE_SIZE(table_rx));
	s_rx_count = 0;
	s_on_rx_callback = record_rx;

	open_lin_master_dl_handler();                          /* header of 0x03 goes out, nothing has arrived yet */
	REQUIRE(s_frames.size() == 1);
	CHECK(s_rx_count == 0);
	CHECK(s_errors.empty());
	CHECK(s_state == OPEN_LIN_MASTER_DATA_RX);

	l_u8 data[1] = {0x5A};
	s_rx_feed.push_back(data[0]);
	open_lin_master_dl_handler();                          /* the data byte is in, the checksum is not */
	CHECK(s_rx_count == 0);
	CHECK(s_state == OPEN_LIN_MASTER_DATA_RX);

	s_rx_feed.push_back(open_lin_data_layer_checksum(0x03, OPEN_LIN_CHECKSUM_ENHANCED, 1, data));
	open_lin_master_dl_handler();                          /* complete */
	CHECK(s_rx_count == 1);
	CHECK(buf_rx[0] == 0x5A);
	CHECK(s_errors.empty());
	CHECK(s_state == OPEN_LIN_MASTER_IDLE);
}

TEST_CASE("a response that never comes times out only once the slot time is over, and the reception is cancelled", "[open_lin_master_schedule][rx]")
{
	reset_sim();
	open_lin_master_dl_init(table_rx, TABLE_SIZE(table_rx));

	int calls = 0;
	do {
		open_lin_master_dl_handler();
		calls++;
	} while (s_errors.empty() && (calls < 100000));

	REQUIRE(s_errors.size() == 1);
	CHECK(s_errors[0] == OPEN_LIN_MASTER_ERROR_FRAME_SLOT_TIMEOUT);
	CHECK(calls > 10);                 /* the handler returned in between, it did not sit inside the slot */
	CHECK(s_hw_rx_aborts == 1);        /* the reception that was still pending was cancelled */
	CHECK(s_state == OPEN_LIN_MASTER_IDLE);
}
