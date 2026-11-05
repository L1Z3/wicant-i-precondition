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


#ifndef __CAN_H__
#define __CAN_H__
#include "driver/twai.h"
#include "hw_config.h"

typedef enum {
	// OBD-II pins, on the WiCAN's own plug: bus 0 is 6 (H)/14 (L). On eb-fd,
	// bus 1 is 12 (H)/13 (L). The harness socket's numbering is mirrored; see
	// README.md.
	CAN_BUS_0 = 0,   // on-chip TWAI controller
	CAN_BUS_1 = 1,   // SPI CAN: MCP2515 on proto, MCP2518FD on eb-fd
} can_bus_t;

#define CAN_5K				0
#define CAN_10K				1
#define CAN_20K				2
#define CAN_25K				3
#define CAN_50K				4
#define CAN_100K			5
#define CAN_125K			6
#define CAN_250K			7
#define CAN_500K			8
#define CAN_800K			9
#define CAN_1000K			10
#define CAN_AUTO			11
typedef struct {
	uint8_t bus_state;
	uint8_t silent;
	uint8_t loopback;
	uint8_t auto_tx;
	uint8_t rate;
	uint8_t fd_enabled;
	uint32_t fd_data_rate;
	uint32_t filter;
	uint32_t mask;
}can_cfg_t;

#define CAN_FD_MAX_DLC		15
#define CAN_FD_MAX_DATA		64
#define CAN_FD_FLAG_IDE		0x01
#define CAN_FD_FLAG_BRS		0x02
#define CAN_FD_FLAG_ESI		0x04

// CAN-FD frame used by the host/GVRET path. FDF is implicit; RTR is not
// valid in CAN-FD.
typedef struct {
	uint32_t id;
	uint8_t dlc;
	uint8_t flags;
	uint8_t data[CAN_FD_MAX_DATA];
} can_fd_message_t;

void can_init(void);
void can_enable(can_bus_t bus);
void can_disable(can_bus_t bus);
// MITM: brings every bus up in normal mode, accepting every frame, and keeps
// it so: can_disable() and the config setters refuse, and a bus that goes
// down is brought back. Until can_park().
void can_lock_bridge(void);
// Sleep: takes every bus down and unlocks MITM forwarding. No bus comes back
// up until the board wakes, which is a restart.
void can_park(void);
// So clients can tell a refused change from one that took
bool can_bridge_locked(void);
// MITM forwarding: whether to bridge a received frame to the other bus. False
// for a frame seen on both buses, which share a wire then, or for one of our
// own frames coming back; such a frame gets parallel-mode handling. Call
// only from the task that forwards, with the frame can_receive() returned last.
bool can_bridge_check(const twai_message_t *msg, can_bus_t rx_bus);
// Set for good, until restart, once bus 0 and bus 1 turn out to share a wire
bool can_bridge_shared_wire(void);
// can_send() for frames the bridge forwards; can_bridge_check() treats only
// can_send()'s frames as our own
esp_err_t can_bridge_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait);
void can_set_silent(can_bus_t bus, uint8_t flag);
void can_set_loopback(can_bus_t bus, uint8_t flag);
void can_set_auto_retransmit(can_bus_t bus, uint8_t flag);
void can_set_filter(can_bus_t bus, uint32_t f);
void can_set_mask(can_bus_t bus, uint32_t m);
void can_set_bitrate(can_bus_t bus, uint8_t rate);
void can_set_fd(can_bus_t bus, uint8_t enabled, uint32_t data_rate);
bool can_is_fd_enabled(can_bus_t bus);
uint32_t can_get_fd_bitrate(can_bus_t bus);
uint8_t can_fd_dlc_to_len(uint8_t dlc);
uint8_t can_fd_len_to_dlc(uint8_t len);
esp_err_t can_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait);
esp_err_t can_send_fd(can_bus_t bus, const can_fd_message_t *message, TickType_t ticks_to_wait);
esp_err_t can_receive(twai_message_t *message, can_bus_t *bus, TickType_t ticks_to_wait);
esp_err_t can_receive_fd(can_fd_message_t *message, can_bus_t *bus, TickType_t ticks_to_wait);
uint8_t can_is_silent(can_bus_t bus);
bool can_is_enabled(can_bus_t bus);
bool can_is_forwardable(can_bus_t bus);
bool can_any_enabled(void);
uint8_t can_get_bitrate(can_bus_t bus);
// Verbose bring-up diagnostics: call from task context only (not an ISR and
// not while holding a node lock). Safe with buses up or down.
void can_dump_diagnostics(const char *why);
#endif
