/*
 * This file is part of the WiCAN project.
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

// When sleep_mode.c's adc_task wants the board asleep or awake. Kept free of
// ESP-IDF so test/host can cover it.

#ifndef SLEEP_POLICY_H
#define SLEEP_POLICY_H

#include <stdbool.h>
#include <stdint.h>

// No frame on either bus for this long: the car's network is asleep. Its
// periodic frames come many times a second while it's awake.
#define SLEEP_BUS_IDLE_US (10 * 1000000LL)

typedef struct {
	bool sleep;		// start counting down to sleep
	bool wake;		// cancel the countdown, or wake up
} sleep_decision_t;

// mode: 1 = low-voltage sleep, 2 = car-off sleep, anything else = never.
// bus_sleep: MITM forwarding on a board whose buses can wake it. car_on_pin:
// the car-on sense wire is high (false on boards without one). car_ready: the
// CAN-derived READY state. bus_idle: no frame for SLEEP_BUS_IDLE_US.
// Exactly at the voltage threshold neither is wanted, which keeps the state.
static inline sleep_decision_t sleep_policy_decide(uint8_t mode, bool bus_sleep, float vbatt,
												   float sleep_volt, bool car_on_pin,
												   bool car_ready, bool bus_idle)
{
	sleep_decision_t d = { false, false };

	if (mode != 1 && mode != 2)
	{
		return d;
	}
	if (bus_sleep)
	{
		// The head unit reaches the car only through us, and must be able to
		// wake it at any time (Bluelink): follow the car's network. Sleep
		// once it's quiet, and wake on its traffic. The board is then awake
		// only while the car's network is, which bounds its draw, so the
		// battery voltage doesn't matter: sleeping on it while the network
		// is live would cut the head unit off, and its next frame would wake
		// the board again. READY is left out too: only a frame updates it,
		// so on quiet buses it keeps its last value, and a stale READY would
		// keep the board awake for good.
		d.sleep = bus_idle && !car_on_pin;
		d.wake = !bus_idle || car_on_pin;
	}
	else if (mode == 1)
	{
		// Low-voltage sleep: the battery alone, against the setpoint
		d.sleep = vbatt < sleep_volt;
		d.wake = vbatt > sleep_volt;
	}
	else
	{
		// Car-off sleep: same setpoint, but never sleep while the car looks
		// on, and wake up as soon as it does
		bool car_awake = car_on_pin || car_ready;
		d.sleep = (vbatt < sleep_volt) && !car_awake;
		d.wake = (vbatt > sleep_volt) || car_awake;
	}
	return d;
}

// The 12 V battery reads above this only while the car charges it (driving,
// charging, remote climate); a resting one stays below. Fixed, not the
// configured sleep voltage, which can be set as low as 12.0 V.
#define SLEEP_CHARGING_V 13.2f

// Whether our own polling (AutoPID) may run. With bus sleep, its requests
// would wake the car's network and the replies keep the buses busy, holding
// the car and the board awake for good. So it waits for a sign that the car is
// on by itself, which our frames can't cause: the car-on wire, or the 12 V
// battery being charged.
static inline bool sleep_policy_polling_allowed(bool bus_sleep, float vbatt, bool car_on_pin)
{
	return !bus_sleep || car_on_pin || vbatt > SLEEP_CHARGING_V;
}

#endif
