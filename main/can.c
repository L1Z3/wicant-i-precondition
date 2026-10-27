/*
 * This file is part of the WiCAN project.
 *
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include <stdio.h>
#include <string.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "hal/twai_hal.h"
#include "can.h"
#include "hw_config.h"
#if HW_HAS_MCP2515 || HW_HAS_MCP2518FD
#include "driver/spi_master.h"
#endif
#if HW_HAS_MCP2515
#include "esp_twai_mcp2515.h"
#elif HW_HAS_MCP2518FD
#include "esp_twai_mcp251xfd.h"
#include "esp_rom_sys.h"
#endif

#define TAG 		__func__

enum bus_state
{
    OFF_BUS = 0,
    ON_BUS = 1,
	END_BUS
};

typedef struct {
	twai_frame_t frame;
	uint8_t data[TWAI_FRAME_MAX_LEN];
	int64_t queued_us;	// when can_send() claimed the slot, for stall detection
	int8_t wire_slot;	// our own frame's wire_tx slot to restamp when sent, or -1
} can_tx_slot_t;

typedef struct {
	twai_message_t msg;
	can_bus_t bus;
	int64_t us;		// received (from the driver), for the shared-wire guard
} can_rx_item_t;

// Indexed by the CAN_5K..CAN_1000K rate codes in can.h
static const uint32_t can_bitrate_bps[] = {
	5000, 10000, 20000, 25000, 50000, 100000,
	125000, 250000, 500000, 800000, 1000000
};

static can_cfg_t can_cfg[CAN_BUS_COUNT];
static twai_node_handle_t can_node[CAN_BUS_COUNT];
// Mutex guarding each node handle's lifecycle: can_disable() deletes the node,
// so create/delete/transmit are serialized under it to prevent use-after-free.
// Don't hold it across a blocking call.
static SemaphoreHandle_t node_lock[CAN_BUS_COUNT];
// One queue shared by all buses; can_receive() returns the bus tag.
static QueueHandle_t can_rx_queue = NULL;
// Per-bus "enabled" bits (BIT(bus)). These are state flags, not events:
// set by can_enable(), cleared by can_disable(), never consumed by waiters.
// can_receive() parks on wait-any until some bus is up (keeps can_rx_task
// asleep while CAN is off); can_send() reads its bus's bit as a fast-fail
// gate. Only touched from task context (the FromISR set is deferred/lossy).
static EventGroupHandle_t s_can_event_group = NULL;
#define CAN_ENABLE_BIT(bus)		BIT(bus)
#define CAN_ENABLE_BIT_ANY		((1 << CAN_BUS_COUNT) - 1)

// The node driver queues a POINTER to the caller's twai_frame_t and transmits
// from it later (no copy), so every in-flight TX needs to have long enough lifetime.
// can_send() copies the message into a slot from this pool; on_tx_done frees
// it. The pool size doubles as the driver tx_queue_depth, and a counting
// semaphore (taken with the caller's timeout) provides the legacy
// block-until-queue-space behavior.
// 32 is the most the uint32_t slot bitmask can track, and buys ~7 ms of burst
// absorption at 500 kbit/s -- the legacy 8-deep queue (~2 ms) overflowed on
// ordinary vehicle-bus burst clusters.
#define CAN_TX_SLOT_COUNT	32
static can_tx_slot_t tx_slot[CAN_BUS_COUNT][CAN_TX_SLOT_COUNT];
// Counting semaphore whose count == number of free TX slots
// can_send() takes it with the caller's timeout.
// can_on_tx_done() gives it back from the driver callback.
// Ordering rule: take BEFORE node_lock, never while holding it.
static SemaphoreHandle_t tx_slot_sem[CAN_BUS_COUNT];
// Bitmask that tracks which slots are free, guarded by tx_slot_num
static uint32_t tx_slot_used[CAN_BUS_COUNT];
// Spinlock for the slot bitmask above, which is the one piece of TX state
// shared with driver callbacks (can_on_tx_done, possibly in ISR context), so
// we can't use a mutex.
static portMUX_TYPE tx_slot_mux = portMUX_INITIALIZER_UNLOCKED;

// RX frames lost because the shared rx queue was full (rx task too slow).
// Incremented from can_on_rx_done (driver callback), logged rate-limited from
// can_receive() in task context.
static volatile uint32_t rx_drop_count[CAN_BUS_COUNT];
static uint32_t rx_drop_logged[CAN_BUS_COUNT];
static int64_t rx_drop_log_us[CAN_BUS_COUNT];
#if HW_HAS_MCP2518FD
// Bus 1's controller RX FIFO overflows (events, not frames), from the driver
static uint32_t rx_overflow_logged;
static int64_t rx_overflow_log_us;
#endif
// Frames bus 0's controller lost to a full RX FIFO, counted in
// __wrap_twai_hal_read_rx_fifo() and logged like the above
static volatile uint32_t bus0_rx_overruns;
static uint32_t bus0_rx_overrun_logged;
static int64_t bus0_rx_overrun_log_us;

// TX frames dropped by can_send() (bus down, or no free slot within the
// caller's timeout). Counted and logged in place--can_send() is task
// context, unlike the RX drop path above.
static uint32_t tx_drop_count[CAN_BUS_COUNT];
static int64_t tx_drop_log_us[CAN_BUS_COUNT];

// TX stall detection. A bus whose peers have all stopped acknowledging never
// reaches bus-off: an error-passive transmitter's error count stops rising on
// ACK errors (ISO 11898-1), so the controller retries one frame forever. Seen
// on bus 1 with a head unit that went silent after a traffic flood and only
// came back once the bus went quiet. Every CAN_TX_STALL_CHECK_MS the recovery
// task checks each error-passive bus, and recovers it if its oldest
// outstanding frame has waited over CAN_TX_STALL_US. Frames complete in
// submission order, so that frame is the one being retried. Losing
// arbitration on a busy bus never makes a controller error-passive, so that
// alone doesn't trigger it.
#define CAN_TX_STALL_US 1000000
#define CAN_TX_STALL_CHECK_MS 250
// Error-passive per the last state change, set in can_on_state_change()
static volatile bool bus_err_passive[CAN_BUS_COUNT];
// Recovery only helps a bus whose peers are there but starved. With no peer
// at all (a head unit asleep while forwarding continues), every recovery
// stalls again, and each one costs 250 ms of downtime and a flush of the
// shared RX queue. So each stall recovery that no acknowledged frame follows
// doubles the bus's threshold, up to 64 s; an acknowledged frame resets it.
#define CAN_TX_STALL_BACKOFF_MAX 6
static uint8_t tx_stall_backoff[CAN_BUS_COUNT];
// A frame was acknowledged since the last stall check; set in can_on_tx_done()
static bool tx_acked[CAN_BUS_COUNT];

// Bus recovery: a bus-off state change flags the bus and wakes a task that
// recreates the node; the task also finds TX stalls itself (see
// CAN_TX_STALL_US). Full recreate rather than twai_node_recover(): on some
// drivers the frame in flight at bus-off gets no on_tx_done, so bare recovery
// would leak one TX slot per bus-off; teardown + can_tx_slots_reset starts
// clean. The recreate also aborts whatever the controller was retrying and
// leaves the bus quiet.
static TaskHandle_t can_recovery_task_handle = NULL;
static volatile uint32_t can_busoff_pending;	// bitmask, set from driver callbacks and the stall check
static uint32_t can_busoff_count[CAN_BUS_COUNT];

// MITM forwarding lock (can_lock_bridge). The head unit only reaches the rest
// of the bus through us, so while it's set nothing a client does reaches the
// buses: can_disable() refuses (an slcan close, an upload), and so do the
// config setters, even while recovery has a bus down, so a bus always comes
// back as it was locked. ELM327 and SavvyCAN check can_bridge_locked() and
// leave the buses alone; slcan answers a refused change with BEL. The
// recovery task takes buses down through can_disable_node(), and brings back
// any that end up down. Sleep parks the buses (can_park), which also unlocks.
static volatile bool bridge_locked;
// Set for good by can_park(): can_enable() refuses until the restart that
// waking is
static volatile bool can_parked;
#if CAN_BUS_COUNT > 1
// Shared-wire guard for MITM forwarding. A MITM harness splits one bus in two,
// so each frame's sender is on one side only, and neither controller receives
// its own transmissions: no frame ever arrives on both buses. If they share a
// wire instead, both receive every frame, and bridging would copy each one
// back onto the wire it came from. So the same frame (ID, data) arriving on
// both buses within WIRE_MATCH_US isn't bridged, and WIRE_LATCH_MATCHES such
// frames within WIRE_LATCH_US turn forwarding off until restart. RTR frames
// carry no data to tell their senders apart, so they're left out.
// Frames we send ourselves never arrive on their own bus, so they'd loop
// (other bus, bridge, back) without ever matching: one of them arriving on
// the other bus isn't bridged either. That's not counted towards the latch,
// since the far side can send the same frame: for 50 ms after we send one,
// the head unit's all-zero 0x0C7 (stop burst) or the car's idle 0x465 (beep
// release) isn't bridged, but they're the same as ours. As a backstop for
// anything else that circulates (one of ours whose entry another of ours
// overwrote; RTR frames; modified frames), arrivals in one slot faster than
// anything repeats on a real bus also latch. That misses only several of ours
// escaping at once into different slots while nothing else is on the wire
// (full-rate SavvyCAN playback on a bench); any other node's frames latch it.
// Times are when the driver received the frame, not when it's dequeued: the
// queue fills before can_rx_task starts, and a backlog drains in a few ms.
#define WIRE_SLOTS			64		// per bus, direct-mapped by frame hash
#define WIRE_MATCH_US		50000	// bus 1 trails bus 0 by up to ~18 ms during flash writes
#define WIRE_LATCH_MATCHES	3
#define WIRE_LATCH_US		100000
#define WIRE_LOOP_REPEATS	16		// arrivals in one slot this fast: circulating,
#define WIRE_LOOP_US		25000	// one frame comes back about every 0.5 ms
typedef struct {
	int64_t us;					// last seen; 0: empty
	int64_t run_us;				// first of the latest run of arrivals in this slot
	uint32_t id;
	uint8_t extd;
	uint8_t rtr;
	uint8_t dlc;
	uint8_t repeats;
	uint8_t data[TWAI_FRAME_MAX_DLC];
} wire_frame_t;
// Recently received frames: can_bridge_check() only, so no lock
static wire_frame_t wire_rx[CAN_BUS_COUNT][WIRE_SLOTS];
// When the frame can_receive() returned last was received
static int64_t wire_rx_us;
// Our own recently sent frames, while MITM is on: written by any can_send()
// caller, and stamped again by can_on_tx_done() once on the wire, so a frame
// that waited in the TX pool still counts as ours when it comes back
static wire_frame_t wire_tx[CAN_BUS_COUNT][WIRE_SLOTS];
static portMUX_TYPE wire_tx_mux = portMUX_INITIALIZER_UNLOCKED;
#endif
// Latched by can_bridge_check(); outside the #if so status can read it on
// every build
static volatile bool wire_shared;
// Rate limit for the "kept on" warning, which a client can trigger per poll
#define CAN_LOCK_LOG_US 60000000
static int64_t lock_log_us[CAN_BUS_COUNT];
// A locked bus that's down (it failed to come up, or a recreate failed) is
// retried after 1 s, doubling up to 32 s
#define CAN_LOCK_RETRY_BACKOFF_MAX 5
static int64_t lock_retry_at_us[CAN_BUS_COUNT];
static uint8_t lock_retry_backoff[CAN_BUS_COUNT];

static void can_disable_node(can_bus_t bus);

// Driver callbacks run in ISR context for the on-chip controller and in a
// driver task for the SPI controllers, so they pick FreeRTOS APIs to match.
//
// On the S3 the on-chip controller's interrupt also runs while flash writes
// have the cache off (CONFIG_TWAI_ISR_CACHE_SAFE): its FIFO holds about five
// frames, 1.2 ms at 500 kbit/s, and a 4 KB flash write takes ~6 ms. The
// callbacks then run from IRAM and may only touch internal RAM: no logging,
// no constants in flash, nothing in PSRAM.
#if CONFIG_TWAI_ISR_CACHE_SAFE
#define CAN_ISR_ATTR IRAM_ATTR
#else
#define CAN_ISR_ATTR
#endif
// Flash auto-suspend keeps the cache on instead, and hides the option
#if CONFIG_IDF_TARGET_ESP32S3 && !CONFIG_TWAI_ISR_CACHE_SAFE && !CONFIG_SPI_FLASH_AUTO_SUSPEND
#error "CONFIG_TWAI_ISR_CACHE_SAFE is off; an older build.<variant>/sdkconfig? Delete it to pick up sdkconfig.defaults.esp32s3"
#endif

// Callback to receive frames from driver: drain the frame out
// of the hardware, tag it with its bus, and hand it to can_receive() via the
// shared queue--all protocol work happens later in task context
static CAN_ISR_ATTR bool can_on_rx_done(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
	(void)edata;
	// Bus id was registered as the callback's user_ctx in can_enable()
	can_rx_item_t item = { .bus = (can_bus_t)(uintptr_t)user_ctx, .us = esp_timer_get_time() };
	// Point the driver's output buffer directly at the queue item's payload
	// array, so the data lands in its final place with no second copy
	twai_frame_t rx_frame = {
		.buffer = item.msg.data,
		.buffer_len = sizeof(item.msg.data),
	};

	// Only legal inside this callback; pulls the pending frame from hardware
	if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK)
	{
		return false;
	}

	// Convert new twai_frame_t api that the MCP2515 driver uses to the legacy
	// twai_message_t that the rest of the firmware uses.
	// ss/self have no equivalent in the twai_frame_t api
	item.msg.flags = 0;
	item.msg.identifier = rx_frame.header.id;
	item.msg.extd = rx_frame.header.ide;
	item.msg.rtr = rx_frame.header.rtr;
	// For classic CAN, dlc==len. This will need to be updated if we ever support FD
	item.msg.data_length_code =
			(rx_frame.header.dlc > TWAI_FRAME_MAX_DLC) ? TWAI_FRAME_MAX_DLC : rx_frame.header.dlc;

	// Queue full = frame dropped: count it here, log it from task context
	// task_woken -> tell the driver to context-switch on ISR exit if this
	// send unblocked a higher-priority task (i.e. can_rx_task).
	BaseType_t task_woken = pdFALSE;
	BaseType_t sent = xPortInIsrContext() ? xQueueSendFromISR(can_rx_queue, &item, &task_woken)
										  : xQueueSend(can_rx_queue, &item, 0);
	if (sent != pdTRUE)
	{
		rx_drop_count[item.bus]++;
	}
	return (task_woken == pdTRUE);
}

// Frames that arrive while the on-chip controller's FIFO is full are kept as
// overrun slots without data. ESP-IDF's driver reads them with
// twai_hal_read_rx_fifo(), which releases the slot and returns false, and
// moves on with no callback or count. CMakeLists.txt wraps that call so each
// one is counted here. It's a lower bound: the controller marks at most about
// 60 per overrun. If ESP-IDF renames the function or changes its signature,
// the build fails.
__typeof__(twai_hal_read_rx_fifo) __real_twai_hal_read_rx_fifo, __wrap_twai_hal_read_rx_fifo;

CAN_ISR_ATTR bool __wrap_twai_hal_read_rx_fifo(twai_hal_context_t *hal_ctx, twai_hal_frame_t *rx_frame)
{
	bool ok = __real_twai_hal_read_rx_fifo(hal_ctx, rx_frame);
	if (!ok)
	{
		bus0_rx_overruns++;
	}
	return ok;
}

// Callback when the driver finishes (or abandons) a transmitted frame:
// return its backing TX slot to the pool. This is the only thing that
// replenishes the slots a can_send() may be blocked waiting on.
static CAN_ISR_ATTR bool can_on_tx_done(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
	(void)handle;
	// Bus id was registered as the callback's user_ctx in can_enable()
	can_bus_t bus = (can_bus_t)(uintptr_t)user_ctx;
	// The driver hands back the exact twai_frame_t pointer we submitted;
	// recover the slot containing it
	can_tx_slot_t *slot = __containerof(edata->done_tx_frame, can_tx_slot_t, frame);

#if CAN_BUS_COUNT > 1
	if (slot->wire_slot >= 0 && edata->is_tx_success)
	{
		// Rewritten whole: another of ours may have taken the entry since
		wire_frame_t *f = &wire_tx[bus][slot->wire_slot];
		portENTER_CRITICAL_SAFE(&wire_tx_mux);
		f->us = esp_timer_get_time();
		f->id = slot->frame.header.id;
		f->extd = slot->frame.header.ide;
		f->rtr = 0;
		f->dlc = slot->frame.header.dlc;
		for (int i = 0; i < f->dlc; i++)
		{
			f->data[i] = slot->data[i];
		}
		portEXIT_CRITICAL_SAFE(&wire_tx_mux);
	}
#endif
	portENTER_CRITICAL_SAFE(&tx_slot_mux);
	tx_slot_used[bus] &= ~BIT(slot - &tx_slot[bus][0]);
	portEXIT_CRITICAL_SAFE(&tx_slot_mux);
	if (edata->is_tx_success)
	{
		__atomic_store_n(&tx_acked[bus], true, __ATOMIC_RELAXED);
	}

	// Release the count so a blocked can_send() can claim the slot;
	// task_woken as in can_on_rx_done
	BaseType_t task_woken = pdFALSE;
	if (xPortInIsrContext())
	{
		xSemaphoreGiveFromISR(tx_slot_sem[bus], &task_woken);
	}
	else
	{
		xSemaphoreGive(tx_slot_sem[bus]);
	}
	return (task_woken == pdTRUE);
}

// Flag a bus for can_recovery_task(). Returns whether a higher-priority task
// was woken (ISR context only).
static CAN_ISR_ATTR bool can_request_recovery(can_bus_t bus)
{
	__atomic_fetch_or(&can_busoff_pending, BIT(bus), __ATOMIC_SEQ_CST);

	BaseType_t task_woken = pdFALSE;
	if (can_recovery_task_handle != NULL)
	{
		if (xPortInIsrContext())
		{
			vTaskNotifyGiveFromISR(can_recovery_task_handle, &task_woken);
		}
		else
		{
			xTaskNotifyGive(can_recovery_task_handle);
		}
	}
	return (task_woken == pdTRUE);
}

static CAN_ISR_ATTR bool can_on_state_change(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx)
{
	(void)handle;
	can_bus_t bus = (can_bus_t)(uintptr_t)user_ctx;
	bus_err_passive[bus] = (edata->new_sta == TWAI_ERROR_PASSIVE);
	if (edata->new_sta != TWAI_ERROR_BUS_OFF)
	{
		return false;
	}
	return can_request_recovery(bus);
}

// Flags every error-passive bus whose oldest outstanding frame has waited
// over its stall threshold (see CAN_TX_STALL_BACKOFF_MAX).
static void can_check_tx_stalls(void)
{
	int64_t now = esp_timer_get_time();

	for (int bus = 0; bus < CAN_BUS_COUNT; bus++)
	{
		if (__atomic_exchange_n(&tx_acked[bus], false, __ATOMIC_RELAXED))
		{
			tx_stall_backoff[bus] = 0;
		}
		if (!bus_err_passive[bus] || !can_is_enabled((can_bus_t)bus))
		{
			continue;
		}
		int64_t oldest_us = now;
		portENTER_CRITICAL(&tx_slot_mux);
		for (int i = 0; i < CAN_TX_SLOT_COUNT; i++)
		{
			if ((tx_slot_used[bus] & BIT(i)) && tx_slot[bus][i].queued_us < oldest_us)
			{
				oldest_us = tx_slot[bus][i].queued_us;
			}
		}
		portEXIT_CRITICAL(&tx_slot_mux);
		if (now - oldest_us > ((int64_t)CAN_TX_STALL_US << tx_stall_backoff[bus]))
		{
			if (tx_stall_backoff[bus] < CAN_TX_STALL_BACKOFF_MAX)
			{
				tx_stall_backoff[bus]++;
			}
			ESP_LOGW(TAG, "bus %d: error-passive, oldest TX frame waiting %lld ms "
					 "(next stall limit %d s unless a frame is acknowledged)",
					 bus, (now - oldest_us) / 1000,
					 (CAN_TX_STALL_US << tx_stall_backoff[bus]) / 1000000);
			__atomic_fetch_or(&can_busoff_pending, BIT(bus), __ATOMIC_SEQ_CST);
		}
	}
}

// Brings back any bus that's down while MITM forwarding is locked (it failed
// to come up, or a recreate failed), with backoff (CAN_LOCK_RETRY_BACKOFF_MAX).
// Runs in the recovery task, so it never sees a recovery's own 250 ms gap.
static void can_retry_locked_buses(void)
{
	int64_t now = esp_timer_get_time();

	for (int bus = 0; bus < CAN_BUS_COUNT; bus++)
	{
		if (!bridge_locked || can_is_enabled((can_bus_t)bus))
		{
			lock_retry_at_us[bus] = 0;
			lock_retry_backoff[bus] = 0;
			continue;
		}
		if (lock_retry_at_us[bus] == 0)
		{
			lock_retry_at_us[bus] = now + (1000000LL << lock_retry_backoff[bus]);
			continue;
		}
		if (now < lock_retry_at_us[bus])
		{
			continue;
		}
		ESP_LOGW(TAG, "bus %d: down with MITM forwarding locked, bringing it up", bus);
		can_enable((can_bus_t)bus);
		lock_retry_at_us[bus] = 0;
		if (!can_is_enabled((can_bus_t)bus) && lock_retry_backoff[bus] < CAN_LOCK_RETRY_BACKOFF_MAX)
		{
			lock_retry_backoff[bus]++;
		}
	}
}

static void can_recovery_task(void *arg)
{
	(void)arg;
	while (1)
	{
		// Woken for bus-off, and periodically to look for TX stalls and for
		// locked buses that are down
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CAN_TX_STALL_CHECK_MS));
		can_check_tx_stalls();
		can_retry_locked_buses();

		// Tear down first so can_send() fails fast (bus enable bit cleared)
		// instead of burning its full timeout per frame against a dead node.
		uint32_t pending = __atomic_exchange_n(&can_busoff_pending, 0, __ATOMIC_SEQ_CST);
		for (int bus = 0; bus < CAN_BUS_COUNT; bus++)
		{
			if (!(pending & BIT(bus)))
			{
				continue;
			}
			// A driver can report bus-off while can_enable() is still
			// publishing the enabled state; wait for it to finish.
			xSemaphoreTake(node_lock[bus], portMAX_DELAY);
			bool enabled = can_is_enabled((can_bus_t)bus);
			xSemaphoreGive(node_lock[bus]);
			if (!enabled)
			{
				pending &= ~BIT(bus);
				continue;
			}
			can_busoff_count[bus]++;
			ESP_LOGW(TAG, "bus %d: failure #%lu, recreating node (%lu TX dropped so far)",
					 bus, can_busoff_count[bus], tx_drop_count[bus]);
			can_disable_node((can_bus_t)bus);
		}

		if (pending == 0)
		{
			continue;
		}

		// Let the bus settle; also rate-limits recreate flapping when the
		// fault (e.g. shorted bus) persists across recoveries.
		vTaskDelay(pdMS_TO_TICKS(250));

		for (int bus = 0; bus < CAN_BUS_COUNT; bus++)
		{
			if (pending & BIT(bus))
			{
				can_enable((can_bus_t)bus);
			}
		}
	}
}

// Reset TX slot accounting to "all free". Only call with the bus node torn
// down (or never created).
static void can_tx_slots_reset(can_bus_t bus)
{
	portENTER_CRITICAL(&tx_slot_mux);
	tx_slot_used[bus] = 0;
	portEXIT_CRITICAL(&tx_slot_mux);
	bus_err_passive[bus] = false;
	// Frames still queued in the driver at disable time are dropped without 
	// an on_tx_done callback, so the semaphore must be refilled by hand.
	while (xSemaphoreGive(tx_slot_sem[bus]) == pdTRUE)
	{
	}
}

// Best-effort translation of the stored legacy filter/mask (raw SJA1000
// single-filter register values, as set by slcan Fxxx/Mxxx: std ID in bits
// [31:21], mask bit 1 = "don't care") to the node API (bare 11-bit id, mask
// bit 1 = "must match"). Extended-ID acceptance through the legacy register
// layout is not representable; frames are filtered as standard IDs.
// Must be called before twai_node_enable().
static void can_apply_filter(can_bus_t bus)
{
	if (can_cfg[bus].mask == 0xFFFFFFFF)
	{
		return;	// legacy accept-all; node default is already accept-all
	}

	twai_mask_filter_config_t fcfg = {
		.id = (can_cfg[bus].filter >> 21) & 0x7FF,
		.mask = ((~can_cfg[bus].mask) >> 21) & 0x7FF,
		.is_ext = false,
	};

	ESP_LOGW(TAG, "bus %d: legacy filter 0x%08lX/0x%08lX applied as std-id %03lX mask %03lX",
			 bus, can_cfg[bus].filter, can_cfg[bus].mask, fcfg.id, fcfg.mask);

	esp_err_t err = twai_node_config_mask_filter(can_node[bus], 0, &fcfg);
#if HW_HAS_MCP2515
	// The MCP2515 has two independent RX filter groups and an unconfigured
	// group defaults to accept-all (RXM1 = 0), which would bypass group 0
	// entirely--mirror the filter into group 1 so it actually takes effect.
	if (err == ESP_OK && bus == CAN_BUS_1)
	{
		err = twai_node_config_mask_filter(can_node[bus], 1, &fcfg);
	}
#endif
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "bus %d: filter config failed: %s", bus, esp_err_to_name(err));
	}
}

#if HW_HAS_MCP2515

// One-time bring-up shared by every node create/delete cycle: the SPI bus,
// the RST pin state, and the GPIO ISR service all outlive the node
// (twai_node_delete removes only the SPI device and the INT handler).
// Ordering per the reference firmware: RST high -> spi_bus_initialize ->
// gpio_install_isr_service -> twai_new_node_mcp2515.
static esp_err_t can_mcp2515_bus_init(void)
{
	static bool bus_ready = false;

	if (bus_ready)
	{
		return ESP_OK;
	}

	// RST is active-low: hold HIGH for normal run
	gpio_config_t rst_cfg = {
		.pin_bit_mask = BIT64(MCP2515_RST_GPIO_NUM),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	esp_err_t err = gpio_config(&rst_cfg);
	if (err != ESP_OK)
	{
		return err;
	}
	gpio_set_level(MCP2515_RST_GPIO_NUM, 1);

	spi_bus_config_t bus_cfg = {
		.sclk_io_num = MCP2515_SCLK_GPIO_NUM,
		.mosi_io_num = MCP2515_MOSI_GPIO_NUM,
		.miso_io_num = MCP2515_MISO_GPIO_NUM,
		.quadwp_io_num = GPIO_NUM_NC,
		.quadhd_io_num = GPIO_NUM_NC,
	};
	err = spi_bus_initialize(MCP2515_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
	if (err != ESP_OK)
	{
		return err;
	}

	err = gpio_install_isr_service(0);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)	// INVALID_STATE = already installed
	{
		return err;
	}

	bus_ready = true;
	return ESP_OK;
}

static esp_err_t can_bus1_create_node(void)
{
	esp_err_t err = can_mcp2515_bus_init();
	if (err != ESP_OK)
	{
		return err;
	}

	twai_mcp2515_node_config_t config = {
		.io_cfg = {
			.int_gpio = MCP2515_INT_GPIO_NUM,
			.cs_gpio = MCP2515_CS_GPIO_NUM,
		},
		.spi_clock_hz = MCP2515_SPI_CLOCK_HZ,
		.oscillator_hz = MCP2515_OSCILLATOR_HZ,
		.bit_timing = {
			.bitrate = can_bitrate_bps[can_cfg[CAN_BUS_1].rate],
			.sp_permill = CAN_SAMPLE_POINT_PERMILL,
		},
		// MCP2515 hardware supports exactly -1 (auto retry) / 0 (one-shot)
		.fail_retry_cnt = can_cfg[CAN_BUS_1].auto_tx ? -1 : 0,
		.timestamp_resolution_hz = 0,
		.tx_queue_depth = CAN_TX_SLOT_COUNT,
		.flags = {
			// MCP2515 loopback is internal (no ACK needed)
			.enable_loopback = can_cfg[CAN_BUS_1].loopback ? 1 : 0,
			.enable_listen_only = can_cfg[CAN_BUS_1].silent ? 1 : 0,
		},
	};

	return twai_new_node_mcp2515(MCP2515_SPI_HOST, &config, &can_node[CAN_BUS_1]);
}
#elif HW_HAS_MCP2518FD

// One-time bring-up shared by every node create/delete cycle: the SPI bus and
// the GPIO ISR service outlive the node. The MCP2518FD resets over SPI.
static esp_err_t can_mcp2518fd_bus_init(void)
{
	static bool bus_ready = false;

	if (bus_ready)
	{
		return ESP_OK;
	}

	spi_bus_config_t bus_cfg = {
		.sclk_io_num = MCP2518FD_SCLK_GPIO_NUM,
		.mosi_io_num = MCP2518FD_MOSI_GPIO_NUM,
		.miso_io_num = MCP2518FD_MISO_GPIO_NUM,
		.quadwp_io_num = GPIO_NUM_NC,
		.quadhd_io_num = GPIO_NUM_NC,
	};
	// Driver transfers are at most 18 bytes, which the CPU-driven FIFO holds
	esp_err_t err = spi_bus_initialize(MCP2518FD_SPI_HOST, &bus_cfg, SPI_DMA_DISABLED);
	if (err != ESP_OK)
	{
		return err;
	}
	// SDO is high-impedance between transfers, and floating raises the
	// controller's Sleep current (MCP2518FD datasheet, SPI section)
	gpio_pullup_en(MCP2518FD_MISO_GPIO_NUM);
	// esp_restart() resets only the CPUs: INT's level-triggered interrupt
	// stays enabled while the controller, powered separately, may hold INT
	// low. Clear it before the ISR service exists, or it fires without a
	// handler until the interrupt watchdog resets the chip, repeatedly.
	gpio_reset_pin(MCP2518FD_INT_GPIO_NUM);

	err = gpio_install_isr_service(0);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)	// INVALID_STATE = already installed
	{
		return err;
	}

	bus_ready = true;
	return ESP_OK;
}

static esp_err_t can_bus1_create_node(void)
{
	esp_err_t err = can_mcp2518fd_bus_init();
	if (err != ESP_OK)
	{
		return err;
	}

	// Auto-retransmit only; see can_set_auto_retransmit()
	twai_mcp251xfd_node_config_t config = {
		.io_cfg = {
			.int_gpio = MCP2518FD_INT_GPIO_NUM,
			.cs_gpio = MCP2518FD_CS_GPIO_NUM,
		},
		.spi_clock_hz = MCP2518FD_SPI_CLOCK_HZ,
		.oscillator_hz = MCP2518FD_OSCILLATOR_HZ,
		.bit_timing = {
			.bitrate = can_bitrate_bps[can_cfg[CAN_BUS_1].rate],
			.sp_permill = CAN_SAMPLE_POINT_PERMILL,
		},
		.flags = {
			// External loopback: frames also go out on the bus
			.enable_loopback = can_cfg[CAN_BUS_1].loopback ? 1 : 0,
			.enable_listen_only = can_cfg[CAN_BUS_1].silent ? 1 : 0,
		},
	};

	return twai_new_node_mcp251xfd(MCP2518FD_SPI_HOST, &config, &can_node[CAN_BUS_1]);
}
#endif

// Take a bus's transceiver out of standby before enabling its controller, and
// put it back after disabling (or a failed enable). Called under node_lock.
static void can_transceiver_standby(can_bus_t bus, bool standby)
{
#ifdef CAN_STDBY_GPIO_NUM
	if (bus == CAN_BUS_0)
	{
		gpio_set_level(CAN_STDBY_GPIO_NUM, standby);
	}
#endif
#ifdef MCP2518FD_STDBY_GPIO_NUM
	if (bus == CAN_BUS_1)
	{
		gpio_set_level(MCP2518FD_STDBY_GPIO_NUM, standby);
		if (!standby)
		{
			esp_rom_delay_us(30);	// TCAN3413: up to 30 us from standby to normal
		}
	}
#endif
}

// Build the bus-0 on-chip node from the stored per-bus config. Mode flags
// (silent/loopback) are creation-time-only in the node API--that's why
// enable/disable is node create/delete instead of a persistent node.
static esp_err_t can_bus0_create_node(void)
{
	twai_onchip_node_config_t config = {
		.io_cfg = {
			.tx = TX_GPIO_NUM,
			.rx = RX_GPIO_NUM,
			.quanta_clk_out = GPIO_NUM_NC,
			.bus_off_indicator = GPIO_NUM_NC,
		},
		.bit_timing = {
			.bitrate = can_bitrate_bps[can_cfg[CAN_BUS_0].rate],
			.sp_permill = CAN_SAMPLE_POINT_PERMILL,
		},
		.fail_retry_cnt = can_cfg[CAN_BUS_0].auto_tx ? -1 : 0,	// -1 = retry forever
		// == slot pool size, so a held tx_slot_sem count implies queue space
		.tx_queue_depth = CAN_TX_SLOT_COUNT,
		.intr_priority = 0,
		.flags = {
			.enable_listen_only = can_cfg[CAN_BUS_0].silent ? 1 : 0,
			// self_test (no ACK needed) so loopback works partner-free
			.enable_loopback = can_cfg[CAN_BUS_0].loopback ? 1 : 0,
			.enable_self_test = can_cfg[CAN_BUS_0].loopback ? 1 : 0,
		},
	};

	return twai_new_node_onchip(&config, &can_node[CAN_BUS_0]);
}

// Bring a bus up from its stored config: create node -> filter -> callbacks
// -> enable -> publish the enable bit.
void can_enable(can_bus_t bus)
{
	// s_can_event_group is NULL only before can_init() (config server
	// starts earlier in app_main and could call in during that window)
	if (bus >= CAN_BUS_COUNT || s_can_event_group == NULL)
	{
		return;
	}
	// node_lock synchronizes the enable sequence with
	// can_send()/can_disable() touching the node handle.
	xSemaphoreTake(node_lock[bus], portMAX_DELAY);

	// Bus is already on, or parked for sleep (checked under node_lock, so a
	// retry or recovery enable racing can_park() can't win)
	if (can_cfg[bus].bus_state == ON_BUS || can_parked)
	{
		xSemaphoreGive(node_lock[bus]);
		return;
	}

	esp_err_t err;
	if (bus == CAN_BUS_0)
	{
		err = can_bus0_create_node();
	}
	else
	{
#if HW_HAS_MCP2515 || HW_HAS_MCP2518FD
		err = can_bus1_create_node();
#else
		err = ESP_ERR_NOT_SUPPORTED;
#endif
	}

	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "bus %d: node create failed: %s", bus, esp_err_to_name(err));
		can_node[bus] = NULL;
		xSemaphoreGive(node_lock[bus]);
		return;
	}

	// Filter config is only legal while the node is stopped, so it must
	// happen here, before twai_node_enable()
	can_apply_filter(bus);

	// Register the ISR callbacks with the bus id as their user_ctx
	twai_event_callbacks_t cbs = {
		.on_rx_done = can_on_rx_done,
		.on_tx_done = can_on_tx_done,
		.on_state_change = can_on_state_change,
	};
	err = twai_node_register_event_callbacks(can_node[bus], &cbs, (void *)(uintptr_t)bus);
	if (err == ESP_OK)
	{
		can_tx_slots_reset(bus);	// fresh node: all slots are free
		can_transceiver_standby(bus, false);
		err = twai_node_enable(can_node[bus]);
	}
	if (err != ESP_OK)
	{
		// Don't panic: bus 1 comes up at every boot by default, and a wedged
		// MCP2515 (created OK over SPI but refusing to enable) must degrade
		// to a dead bus, not a boot loop.
		ESP_LOGE(TAG, "bus %d: node enable failed: %s", bus, esp_err_to_name(err));
		can_transceiver_standby(bus, true);
		twai_node_delete(can_node[bus]);
		can_node[bus] = NULL;
		xSemaphoreGive(node_lock[bus]);
		return;
	}

	// Clear the queue on enable. The queue is shared, so this also drops
	// any pending frames of the other bus. This can happen at boot, or at
	// runtime during bus recovery. In-flight frames on the other bus will be
	// lost. TODO(ejones): maybe eventually we can split into one rx queue per bus?
	xQueueReset(can_rx_queue);

	can_cfg[bus].bus_state = ON_BUS;
	// Publish "bus up" last: unparks can_receive(), opens can_send()'s gate
	xEventGroupSetBits(s_can_event_group, CAN_ENABLE_BIT(bus));

	xSemaphoreGive(node_lock[bus]);
}

// Tear a bus down: stops the interrupt sources, then delete frees the node (no
// callbacks after this).
static void can_disable_node(can_bus_t bus)
{
	if (bus >= CAN_BUS_COUNT || s_can_event_group == NULL)
	{
		return;
	}

	// The node is deleted with node_lock held so a concurrent can_send() can never touch a freed handle
	xSemaphoreTake(node_lock[bus], portMAX_DELAY);

	if (can_cfg[bus].bus_state != ON_BUS)
	{
		// Bus is already off
		xSemaphoreGive(node_lock[bus]);
		return;
	}
	// The enable bit goes first so new senders fail fast instead of piling up on the lock.
	xEventGroupClearBits(s_can_event_group, CAN_ENABLE_BIT(bus));

	twai_node_disable(can_node[bus]);
	can_transceiver_standby(bus, true);
	twai_node_delete(can_node[bus]);
	can_node[bus] = NULL;
	// Some drivers drop frames still queued at disable without calling
	// on_tx_done, so reset the tx slots
	can_tx_slots_reset(bus);
	can_cfg[bus].bus_state = OFF_BUS;

	xSemaphoreGive(node_lock[bus]);
}

void can_disable(can_bus_t bus)
{
	if (bridge_locked && bus < CAN_BUS_COUNT)
	{
		int64_t now = esp_timer_get_time();
		if (lock_log_us[bus] == 0 || now - lock_log_us[bus] >= CAN_LOCK_LOG_US)
		{
			lock_log_us[bus] = now;
			ESP_LOGW(TAG, "bus %d: kept on, MITM forwarding is locked", bus);
		}
		return;
	}
	can_disable_node(bus);
}

void can_park(void)
{
	if (!can_parked)
	{
		ESP_LOGI(TAG, "buses parked for sleep");
	}
	// Before taking the buses down, so a retry or recovery re-enable
	// already in progress can't bring one back
	can_parked = true;
	bridge_locked = false;
	for (int bus = 0; bus < CAN_BUS_COUNT; bus++)
	{
		can_disable_node((can_bus_t)bus);
	}
}

void can_lock_bridge(void)
{
	// Set first, so nothing reconfigures a bus between here and its enable
	bridge_locked = true;
	ESP_LOGI(TAG, "MITM forwarding locked");

	for (int bus = 0; bus < CAN_BUS_COUNT; bus++)
	{
		// Forwarding needs normal mode, every frame and retransmission.
		// Modes and filters are creation-time: recreate the node if needed.
		// The settings go in before the node comes down, so a recovery
		// re-enable in between brings it back with them.
		bool recreate = can_cfg[bus].silent || can_cfg[bus].loopback || !can_cfg[bus].auto_tx
						|| can_cfg[bus].filter != 0 || can_cfg[bus].mask != 0xFFFFFFFF;
		if (can_cfg[bus].silent)
		{
			ESP_LOGI(TAG, "bus %d: configured listen-only, but MITM forwarding needs normal mode", bus);
		}
		can_cfg[bus].silent = 0;
		can_cfg[bus].loopback = 0;
		can_cfg[bus].auto_tx = 1;
		can_cfg[bus].filter = 0;
		can_cfg[bus].mask = 0xFFFFFFFF;
		if (recreate)
		{
			can_disable_node((can_bus_t)bus);
		}
		can_enable((can_bus_t)bus);
		if (!can_is_enabled((can_bus_t)bus))
		{
			ESP_LOGE(TAG, "bus %d: down with MITM forwarding locked, will retry", bus);
		}
	}
#if CAN_BUS_COUNT > 1
	if (can_cfg[CAN_BUS_0].rate != can_cfg[CAN_BUS_1].rate)
	{
		ESP_LOGW(TAG, "MITM: bus 0 and bus 1 bitrates differ; check the CAN settings");
	}
#endif
}

bool can_bridge_locked(void)
{
	return bridge_locked;
}

// Config setters. As in the legacy driver, they are no-ops while the bus is
// ON: values are latched into the node at the next can_enable(). Callers
// that reconfigure at runtime (slcan/gvret/elm327) disable first. While MITM
// forwarding is locked they're no-ops whether the bus is on or not.
static bool can_cfg_frozen(can_bus_t bus)
{
	return bus >= CAN_BUS_COUNT || bridge_locked || can_cfg[bus].bus_state == ON_BUS;
}

void can_set_silent(can_bus_t bus, uint8_t flag)
{
	if (can_cfg_frozen(bus))
	{
		return;
	}
	can_cfg[bus].silent = flag;
}

void can_set_loopback(can_bus_t bus, uint8_t flag)
{
	if (can_cfg_frozen(bus))
	{
		return;
	}
	can_cfg[bus].loopback = flag;
}

uint8_t can_is_silent(can_bus_t bus)
{
	if (bus >= CAN_BUS_COUNT)
	{
		return 0;
	}
	return can_cfg[bus].silent;
}

void can_set_auto_retransmit(can_bus_t bus, uint8_t flag)
{
	if (can_cfg_frozen(bus))
	{
		return;
	}
#if HW_HAS_MCP2515 || HW_HAS_MCP2518FD
	// The MCP2518FD driver has no one-shot mode.
	// One-shot mode (auto_tx = 0) is unusable on the MCP2515: the driver
	// only completes a frame (on_tx_done + start of the next queued TX) on
	// TX0IF, which a failed one-shot transmit never raises (only MERRF), so
	// every failure would leak a TX slot until the pool starves and bus-1 TX
	// stalls for good. Keep retry-forever; nothing calls this today anyway.
	if (bus == CAN_BUS_1 && flag == 0)
	{
		ESP_LOGW(TAG, "bus %d: one-shot TX unsupported on SPI controller, keeping auto-retransmit", bus);
		return;
	}
#endif
	can_cfg[bus].auto_tx = flag;
}

void can_set_filter(can_bus_t bus, uint32_t f)
{
	if (can_cfg_frozen(bus))
	{
		return;
	}
	can_cfg[bus].filter = f;
}

void can_set_mask(can_bus_t bus, uint32_t m)
{
	if (can_cfg_frozen(bus))
	{
		return;
	}
	can_cfg[bus].mask = m;
}

void can_set_bitrate(can_bus_t bus, uint8_t rate)
{
	if (can_cfg_frozen(bus))
	{
		return;
	}
	if (rate >= CAN_AUTO)	// auto-baud not supported; also guards table bounds
	{
		rate = CAN_500K;
	}
	can_cfg[bus].rate = rate;
}

uint8_t can_get_bitrate(can_bus_t bus)
{
	if (bus >= CAN_BUS_COUNT)
	{
		return CAN_500K;
	}
	return can_cfg[bus].rate;
}

// One-time OS-resource setup; touches no hardware (per-bus bring-up is
// can_enable()). Idempotent so a stray second call can't leak the handles.
void can_init(void)
{
	if (s_can_event_group != NULL)
	{
		return;
	}

	s_can_event_group = xEventGroupCreate();
	// Deeper than the legacy driver's 100: at 500 kbit/s a saturated bus
	// delivers ~4 frames/ms, so 256 rides out multi-ms scheduling stalls of
	// can_rx_task under TCP streaming load instead of silently dropping RX.
	can_rx_queue = xQueueCreate(256, sizeof(can_rx_item_t));
	xTaskCreate(can_recovery_task, "can_recovery", 3072, NULL, 6, &can_recovery_task_handle);

	for (int bus = 0; bus < CAN_BUS_COUNT; bus++)
	{
		node_lock[bus] = xSemaphoreCreateMutex();
		tx_slot_sem[bus] = xSemaphoreCreateCounting(CAN_TX_SLOT_COUNT, CAN_TX_SLOT_COUNT);
		can_cfg[bus] = (can_cfg_t){
			.bus_state = OFF_BUS,
			.auto_tx = 1,
			.rate = CAN_500K,
			.filter = 0,
			.mask = 0xFFFFFFFF,
		};
	}
}

// Pop the next frame (any bus, global arrival order) from the shared queue;
// *bus reports where it came from.
esp_err_t can_receive(twai_message_t *message, can_bus_t *bus, TickType_t ticks_to_wait)
{
	if (s_can_event_group == NULL)
	{
		return ESP_ERR_INVALID_STATE;
	}

	// Park until at least one bus is enabled
	xEventGroupWaitBits(s_can_event_group,
						CAN_ENABLE_BIT_ANY,
						pdFALSE,	// don't consume the bits: they're state, not events
						pdFALSE,	// wait-any: one live bus is enough
						portMAX_DELAY);

	// Log rx drops, max 1 log/s/bus
	for (int b = 0; b < CAN_BUS_COUNT; b++)
	{
		uint32_t dropped = rx_drop_count[b];
		if (dropped != rx_drop_logged[b])
		{
			int64_t now = esp_timer_get_time();
			if (now - rx_drop_log_us[b] > 1000000)
			{
				rx_drop_log_us[b] = now;
				rx_drop_logged[b] = dropped;
				ESP_LOGW(TAG, "bus %d: rx queue full, %lu frames dropped total", b, dropped);
			}
		}
	}
#if HW_HAS_MCP2518FD
	uint32_t overflows = twai_mcp251xfd_get_rx_overflow_count();
	if (overflows != rx_overflow_logged)
	{
		int64_t now = esp_timer_get_time();
		if (now - rx_overflow_log_us > 1000000)
		{
			rx_overflow_log_us = now;
			rx_overflow_logged = overflows;
			ESP_LOGW(TAG, "bus %d: controller RX FIFO overflowed (frames lost), %lu events total",
					 CAN_BUS_1, overflows);
		}
	}
#endif
	uint32_t overruns = bus0_rx_overruns;
	if (overruns != bus0_rx_overrun_logged)
	{
		int64_t now = esp_timer_get_time();
		if (now - bus0_rx_overrun_log_us > 1000000)
		{
			bus0_rx_overrun_log_us = now;
			bus0_rx_overrun_logged = overruns;
			ESP_LOGW(TAG, "bus %d: controller RX FIFO overflowed, %lu frames lost total",
					 CAN_BUS_0, overruns);
		}
	}

	// The caller's timeout applies to the actual wait-for-data
	can_rx_item_t item;
	if (xQueueReceive(can_rx_queue, &item, ticks_to_wait) != pdTRUE)
	{
		return ESP_ERR_TIMEOUT;
	}

#if CAN_BUS_COUNT > 1
	wire_rx_us = item.us;
#endif
	*message = item.msg;
	if (bus != NULL)	// bus tag is optional
	{
		*bus = item.bus;
	}
	return ESP_OK;
}

#if CAN_BUS_COUNT > 1
static uint32_t wire_slot(const twai_message_t *msg, uint8_t dlc)
{
	// FNV-1a over the ID and data
	uint32_t h = 2166136261u ^ (msg->identifier | ((uint32_t)msg->extd << 31)
								| ((uint32_t)msg->rtr << 30));
	h *= 16777619u;
	for (int i = 0; i < dlc; i++)
	{
		h = (h ^ msg->data[i]) * 16777619u;
	}
	return (h ^ (h >> 16)) % WIRE_SLOTS;
}

static bool wire_match(const wire_frame_t *f, const twai_message_t *msg, uint8_t dlc, int64_t now)
{
	return f->us != 0 && now - f->us <= WIRE_MATCH_US
		&& f->id == msg->identifier && f->extd == msg->extd && f->rtr == msg->rtr && f->dlc == dlc
		&& memcmp(f->data, msg->data, dlc) == 0;
}

static void wire_store(wire_frame_t *f, const twai_message_t *msg, uint8_t dlc, int64_t now)
{
	f->us = now;
	f->id = msg->identifier;
	f->extd = msg->extd;
	f->rtr = msg->rtr;
	f->dlc = dlc;
	memcpy(f->data, msg->data, dlc);
}
#endif

#if CAN_BUS_COUNT > 1
static void wire_latch(const twai_message_t *msg, uint8_t dlc, can_bus_t rx_bus, const char *why)
{
	wire_shared = true;
	char hex[3 * TWAI_FRAME_MAX_DLC + 1] = "";
	for (int i = 0; i < dlc && !msg->rtr; i++)
	{
		snprintf(hex + 3 * i, sizeof(hex) - 3 * i, " %02x", msg->data[i]);
	}
	ESP_LOGE(TAG, "ID 0x%03lx%s%s on bus %d %s: bus 0 and bus 1 share a wire, so MITM forwarding "
			 "is off until restart. Check the harness, or set the forwarding mode to parallel",
			 (unsigned long)msg->identifier, msg->rtr ? " RTR" : "", hex, rx_bus, why);
}
#endif

bool can_bridge_check(const twai_message_t *msg, can_bus_t rx_bus)
{
#if CAN_BUS_COUNT > 1
	if (wire_shared || rx_bus >= CAN_BUS_COUNT)
	{
		return false;
	}
	uint8_t dlc = (msg->data_length_code > TWAI_FRAME_MAX_DLC)
					  ? TWAI_FRAME_MAX_DLC : msg->data_length_code;
	if (msg->rtr)
	{
		dlc = 0;	// no data to compare
	}
	can_bus_t other = (rx_bus == CAN_BUS_0) ? CAN_BUS_1 : CAN_BUS_0;
	uint32_t slot = wire_slot(msg, dlc);
	int64_t now = wire_rx_us;

	// Counted per slot, not per frame: frames looping together that share a
	// slot would otherwise keep restarting each other's run
	wire_frame_t *own = &wire_rx[rx_bus][slot];
	if (own->us != 0 && now - own->run_us <= WIRE_LOOP_US)
	{
		if (own->repeats < UINT8_MAX)
		{
			own->repeats++;
		}
	}
	else
	{
		own->run_us = now;
		own->repeats = 1;
	}
	wire_store(own, msg, dlc, now);
	if (own->repeats >= WIRE_LOOP_REPEATS)
	{
		wire_latch(msg, dlc, rx_bus, "keeps coming back");
		return false;
	}
	if (msg->rtr)
	{
		return true;
	}

	portENTER_CRITICAL(&wire_tx_mux);
	bool ours = wire_match(&wire_tx[other][slot], msg, dlc, now);
	portEXIT_CRITICAL(&wire_tx_mux);
	if (ours)
	{
		return false;
	}
	if (!wire_match(&wire_rx[other][slot], msg, dlc, now))
	{
		return true;
	}

	static int64_t first_us;
	static int matches;
	if (matches == 0 || now - first_us > WIRE_LATCH_US)
	{
		first_us = now;
		matches = 0;
	}
	if (++matches >= WIRE_LATCH_MATCHES)
	{
		wire_latch(msg, dlc, rx_bus, "was received on both");
	}
	return false;
#else
	(void)msg;
	(void)rx_bus;
	return false;
#endif
}

bool can_bridge_shared_wire(void)
{
	return wire_shared;
}

static esp_err_t can_send_frame(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait, bool ours);

// Queue a frame for transmission. Fire-and-forget: the message is copied
// into a pool slot the driver transmits from later, so ticks_to_wait is
// spent waiting for a free slot--not for the frame to reach the wire.
esp_err_t can_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait)
{
	return can_send_frame(bus, message, ticks_to_wait, true);
}

esp_err_t can_bridge_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait)
{
	return can_send_frame(bus, message, ticks_to_wait, false);
}

static esp_err_t can_send_frame(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait, bool ours)
{
	if (bus >= CAN_BUS_COUNT || s_can_event_group == NULL)
	{
		return ESP_ERR_INVALID_STATE;
	}

	// Advisory fast-fail so "bus not open" returns instantly without
	// blocking (gvret/slcan test for exactly this error); the
	// authoritative re-check happens under node_lock below
	if (!(xEventGroupGetBits(s_can_event_group) & CAN_ENABLE_BIT(bus)))
	{
		tx_drop_count[bus]++;
		int64_t now = esp_timer_get_time();
		if (now - tx_drop_log_us[bus] > 1000000)
		{
			tx_drop_log_us[bus] = now;
			ESP_LOGW(TAG, "bus %d: down (disabled), %lu frames dropped total",
					 bus, tx_drop_count[bus]);
		}
		return ESP_ERR_INVALID_STATE;
	}

	// Claim a TX slot; this is where the caller's ticks_to_wait is spent.
	// Ordering matters: never hold node_lock while blocking here, or a
	// full TX queue could stall can_disable() indefinitely.
	if (xSemaphoreTake(tx_slot_sem[bus], ticks_to_wait) != pdTRUE)
	{
		tx_drop_count[bus]++;
		int64_t now = esp_timer_get_time();
		if (now - tx_drop_log_us[bus] > 1000000)
		{
			tx_drop_log_us[bus] = now;
			ESP_LOGW(TAG, "bus %d: TX queue full, %lu frames dropped total", bus, tx_drop_count[bus]);
		}
		return ESP_ERR_TIMEOUT;
	}

	// Synchronize against enable/disable deleting the node out from under
	// us. Safe to wait unbounded: every holder keeps this lock only for
	// short non-blocking sections.
	xSemaphoreTake(node_lock[bus], portMAX_DELAY);

	if (can_cfg[bus].bus_state != ON_BUS || can_node[bus] == NULL)
	{
		xSemaphoreGive(node_lock[bus]);
		xSemaphoreGive(tx_slot_sem[bus]);
		return ESP_ERR_INVALID_STATE;
	}

	// Semaphore normally guarantees a free slot exists; find it. Completion
	// order can diverge from allocation order when a transmit fails, hence
	// the bitmask instead of a ring index. The pool can still come up empty
	// if can_tx_slots_reset() refilled the semaphore while we held a permit
	// waiting on node_lock (bus recreated under us) -- drop, don't assert.
	can_tx_slot_t *slot = NULL;
	int64_t now = esp_timer_get_time();
	portENTER_CRITICAL(&tx_slot_mux);
	for (int i = 0; i < CAN_TX_SLOT_COUNT; i++)
	{
		if (!(tx_slot_used[bus] & BIT(i)))
		{
			tx_slot_used[bus] |= BIT(i);
			slot = &tx_slot[bus][i];
			// Stamped with the claim, so the stall check never sees a used
			// slot with a stale time
			slot->queued_us = now;
			break;
		}
	}
	portEXIT_CRITICAL(&tx_slot_mux);
	if (slot == NULL)
	{
		xSemaphoreGive(node_lock[bus]);
		xSemaphoreGive(tx_slot_sem[bus]);	// no-op if reset already refilled it
		return ESP_ERR_TIMEOUT;
	}

	// Copy legacy twai_message_t -> the slot's stable twai_frame_t + data
	// (the driver only keeps a pointer, hence the copy into pool storage).
	// RTR frames carry no payload: buffer_len must be 0 or the driver
	// rejects the dlc/len mismatch. ss/self have no node-API equivalent.
	uint8_t dlc = (message->data_length_code > TWAI_FRAME_MAX_DLC)
					  ? TWAI_FRAME_MAX_DLC : message->data_length_code;
	memset(&slot->frame, 0, sizeof(slot->frame));
	slot->frame.header.id = message->identifier;
	slot->frame.header.dlc = dlc;
	slot->frame.header.ide = message->extd;
	slot->frame.header.rtr = message->rtr;
	memcpy(slot->data, message->data, dlc);
	slot->frame.buffer = slot->data;
	slot->frame.buffer_len = message->rtr ? 0 : dlc;
	slot->wire_slot = -1;
#if CAN_BUS_COUNT > 1
	if (ours && bridge_locked && !wire_shared && !message->rtr)
	{
		uint32_t wslot = wire_slot(message, dlc);
		slot->wire_slot = (int8_t)wslot;
		portENTER_CRITICAL(&wire_tx_mux);
		wire_store(&wire_tx[bus][wslot], message, dlc, now);
		portEXIT_CRITICAL(&wire_tx_mux);
	}
#else
	(void)ours;
#endif

	// Timeout 0: the slot semaphore already did the waiting (queue space is
	// guaranteed), and staying non-blocking keeps the node_lock hold short
	esp_err_t err = twai_node_transmit(can_node[bus], &slot->frame, 0);

	if (err != ESP_OK)
	{
		// Frame never reached the driver: put the slot straight back, before
		// releasing node_lock so a bus recreate can't reuse it meanwhile
		portENTER_CRITICAL(&tx_slot_mux);
		tx_slot_used[bus] &= ~BIT(slot - &tx_slot[bus][0]);
		portEXIT_CRITICAL(&tx_slot_mux);
		xSemaphoreGive(tx_slot_sem[bus]);
		// Callers test for ESP_ERR_INVALID_STATE (legacy "bus not enabled")
		if (err != ESP_ERR_INVALID_STATE && err != ESP_ERR_TIMEOUT)
		{
			ESP_LOGE(TAG, "bus %d: transmit failed: %s", bus, esp_err_to_name(err));
		}
	}
	xSemaphoreGive(node_lock[bus]);
	return err;
}

bool can_is_enabled(can_bus_t bus)
{
	if (bus >= CAN_BUS_COUNT)
	{
		return false;
	}
	return can_cfg[bus].bus_state == ON_BUS;
}

// True if any bus is up--one atomic read instead of a per-bus loop
bool can_any_enabled(void)
{
	if (s_can_event_group == NULL)
	{
		return false;
	}
	return (xEventGroupGetBits(s_can_event_group) & CAN_ENABLE_BIT_ANY) != 0;
}
