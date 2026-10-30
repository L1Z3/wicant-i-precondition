// sleep_policy_decide(): when sleep_mode.c wants the board asleep or awake.
// Every combination of mode, bus sleep, battery, car-on pin, READY and bus
// activity is checked against the rules written out below.
#include <stdbool.h>
#include <stdint.h>
#include "test_support.h"
#include "sleep_policy.h"

#define SLEEP_VOLT 13.1f

static const float volts[] = { 12.5f, SLEEP_VOLT, 14.2f };

// the rules, restated
static sleep_decision_t expected(uint8_t mode, bool bus_sleep, float vbatt, bool pin, bool ready,
                                 bool idle)
{
    sleep_decision_t d = { false, false };
    if (mode != 1 && mode != 2) {
        return d;
    }
    if (bus_sleep) {
        // buses only: voltage and READY ignored
        d.sleep = idle && !pin;
        d.wake = !idle || pin;
    } else if (mode == 1) {
        d.sleep = vbatt < SLEEP_VOLT;
        d.wake = vbatt > SLEEP_VOLT;
    } else {
        d.sleep = vbatt < SLEEP_VOLT && !pin && !ready;
        d.wake = vbatt > SLEEP_VOLT || pin || ready;
    }
    return d;
}

int main(void) {
    for (uint8_t mode = 0; mode <= 3; mode++) {
        for (int bs = 0; bs < 2; bs++) {
            for (size_t v = 0; v < sizeof(volts) / sizeof(volts[0]); v++) {
                for (int pin = 0; pin < 2; pin++) {
                    for (int ready = 0; ready < 2; ready++) {
                        for (int idle = 0; idle < 2; idle++) {
                            sleep_decision_t got = sleep_policy_decide(
                                mode, bs, volts[v], SLEEP_VOLT, pin, ready, idle);
                            sleep_decision_t want = expected(mode, bs, volts[v], pin, ready,
                                                             idle);
                            CHECK_MSG(got.sleep == want.sleep && got.wake == want.wake,
                                      "mode %u bus_sleep %d vbatt %.1f pin %d ready %d idle %d:"
                                      " got sleep %d wake %d",
                                      mode, bs, volts[v], pin, ready, idle, got.sleep,
                                      got.wake);
                            // never both
                            CHECK(!(got.sleep && got.wake));
                        }
                    }
                }
            }
        }
    }

    // the cases this exists for, spelled out
    // Bluelink: car off, battery low, the head unit wakes the network
    sleep_decision_t d = sleep_policy_decide(2, true, 12.4f, SLEEP_VOLT, false, false, false);
    CHECK(!d.sleep && d.wake);
    // car's network asleep: sleep, whatever the battery reads
    d = sleep_policy_decide(2, true, 14.4f, SLEEP_VOLT, false, false, true);
    CHECK(d.sleep && !d.wake);
    // a READY that went stale on quiet buses doesn't hold the board awake
    d = sleep_policy_decide(2, true, 12.4f, SLEEP_VOLT, false, true, true);
    CHECK(d.sleep);
    // the car-on wire still does
    d = sleep_policy_decide(1, true, 12.4f, SLEEP_VOLT, true, false, true);
    CHECK(!d.sleep && d.wake);
    // without bus sleep, car-off sleep is unchanged: READY holds it awake
    d = sleep_policy_decide(2, false, 12.4f, SLEEP_VOLT, false, true, true);
    CHECK(!d.sleep && d.wake);
    // exactly at the threshold, neither (keeps the state)
    d = sleep_policy_decide(1, false, SLEEP_VOLT, SLEEP_VOLT, false, false, true);
    CHECK(!d.sleep && !d.wake);
    // sleep off
    d = sleep_policy_decide(0, true, 12.4f, SLEEP_VOLT, false, false, true);
    CHECK(!d.sleep && !d.wake);

    return test_report("sleep policy");
}
