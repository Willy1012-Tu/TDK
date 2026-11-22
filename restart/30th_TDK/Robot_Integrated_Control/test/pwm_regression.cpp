#include <Arduino.h>
#include "pwm_controller.h"
#include "system_config.h"
#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <type_traits>
#define CHECK(expr) do { if (!(expr)) throw std::runtime_error(std::to_string(__LINE__) + ": " + #expr); } while (0)

static_assert(std::is_same<decltype(PwmController::currentValue()), uint8_t>::value,
              "PWM status uses an unsigned 8-bit duty");
static_assert(SystemConfig::Pwm::PIN == 14 && SystemConfig::Pwm::RESOLUTION_BITS == 8 &&
              SystemConfig::Pwm::STEP_INTERVAL_MS == 5 && SystemConfig::Pwm::MAX_VALUE == 55 &&
              SystemConfig::Pwm::STROBE_INTERVAL_MS == 50 && SystemConfig::Pwm::STROBE_VALUE == 255 &&
              SystemConfig::Cylinder::IN1_PIN == 17 && SystemConfig::Cylinder::IN2_PIN == 16,
              "PWM and cylinder pin ownership must match the requested wiring");
void pollAfter(uint32_t elapsed) {
    mockMillis += elapsed;
    const uint32_t before = mockMillis;
    PwmController::poll();
    CHECK(mockMillis == before); // Poll never advances time or waits for a deadline.
}
void assertPwmWrites() {
    for (const auto &event : Mock::pwmWrites)
        CHECK(event.pin == 14 && event.value >= 0 && event.value <= 255);
    CHECK(Mock::gpioWrites.empty());
    CHECK(Mock::gpioState[16] == LOW && Mock::gpioState[17] == LOW);
}
void checkConfiguration(uint8_t peak, uint32_t interval) {
    const auto state = PwmController::status();
    CHECK(state.pin == 14 && state.maxValue == peak && state.stepIntervalMs == interval);
    CHECK(state.value == PwmController::currentValue() && state.value <= peak);
    CHECK(Mock::pwmDuty[14] == state.value);
}
void test(const std::string &name) {
    if (name == "pwm_safe_boot_zero") {
        mockMillis = 37; PwmController::begin();
        CHECK(PwmController::currentValue() == 0 && Mock::pwmDuty[14] == 0);
        CHECK(Mock::gpioMode[14] == OUTPUT && Mock::pwmResolution == 8);
        CHECK(Mock::pwmWrites.size() == 1 && Mock::pwmWrites[0].at == 37);
        CHECK(Mock::pwmResolutions.size() == 1 && Mock::pwmResolutions[0].bits == 8);
        CHECK(Mock::gpioModes.size() == 1 && Mock::gpioModes[0].pin == 14);
    } else if (name == "pwm_interval_and_no_early_step") {
        PwmController::begin();
        for (uint32_t elapsed : {0U, 1U, 2U, 1U}) pollAfter(elapsed);
        CHECK(PwmController::currentValue() == 0 && Mock::pwmWrites.size() == 1);
        pollAfter(1); CHECK(PwmController::currentValue() == 1 && Mock::pwmWrites.size() == 2);
        pollAfter(0); pollAfter(4); CHECK(PwmController::currentValue() == 1);
        pollAfter(1); CHECK(PwmController::currentValue() == 2 && Mock::pwmWrites.size() == 3);
        CHECK(Mock::pwmWrites[1].at == 5 && Mock::pwmWrites[2].at == 10);
    } else if (name == "pwm_late_poll_one_step_only") {
        PwmController::begin(); pollAfter(5000);
        CHECK(PwmController::currentValue() == 1 && Mock::pwmWrites.size() == 2);
        pollAfter(0); pollAfter(4); CHECK(PwmController::currentValue() == 1);
        pollAfter(1); CHECK(PwmController::currentValue() == 2);
        CHECK(Mock::pwmWrites.size() == 3 && Mock::pwmWrites.back().at == 5005);
    } else if (name == "pwm_full_breath_endpoint_reversals") {
        PwmController::begin();
        for (unsigned tick = 1; tick <= 220; ++tick) {
            pollAfter(5);
            const unsigned cycle = tick % 110;
            const int expected = cycle <= 55 ? static_cast<int>(cycle) : static_cast<int>(110 - cycle);
            CHECK(PwmController::currentValue() == expected && Mock::pwmDuty[14] == expected);
            CHECK(Mock::pwmWrites.back().value == expected);
            CHECK(Mock::pwmWrites.back().at == tick * 5U);
            if (tick > 1) CHECK(std::abs(Mock::pwmWrites[tick].value - Mock::pwmWrites[tick - 1].value) == 1);
        }
        CHECK(Mock::pwmWrites.size() == 221 && PwmController::currentValue() == 0);
        pollAfter(5); CHECK(PwmController::currentValue() == 1);
    } else if (name == "pwm_begin_resets_value_direction_and_clock") {
        PwmController::begin();
        for (unsigned tick = 0; tick < 60; ++tick) pollAfter(5);
        CHECK(PwmController::currentValue() == 50);
        mockMillis += 2; PwmController::begin();
        CHECK(PwmController::currentValue() == 0 && Mock::pwmWrites.back().value == 0);
        pollAfter(4); CHECK(PwmController::currentValue() == 0);
        pollAfter(1); CHECK(PwmController::currentValue() == 1);
        CHECK(Mock::pwmResolutions.size() == 2 && Mock::pwmResolution == 8);
    } else if (name == "pwm_millis_rollover") {
        mockMillis = UINT32_MAX - 2U; PwmController::begin();
        pollAfter(4); CHECK(mockMillis == 1 && PwmController::currentValue() == 0);
        pollAfter(1); CHECK(mockMillis == 2 && PwmController::currentValue() == 1);
        pollAfter(4); CHECK(PwmController::currentValue() == 1);
        pollAfter(1); CHECK(mockMillis == 7 && PwmController::currentValue() == 2);
        CHECK(Mock::pwmWrites.size() == 3);
    } else if (name == "pwm_gpio_cylinder_state_is_independent") {
        Mock::gpioState[17] = HIGH; Mock::gpioState[16] = LOW;
        Mock::gpioState[14] = LOW; PwmController::begin();
        for (unsigned tick = 0; tick < 55; ++tick) pollAfter(5);
        CHECK(PwmController::currentValue() == 55 && Mock::pwmDuty[14] == 55);
        CHECK(Mock::gpioState[14] == LOW && Mock::gpioState[17] == HIGH && Mock::gpioState[16] == LOW);
        Mock::gpioState[17] = LOW; // Restore the independent fixture before checking the common trace.
    } else if (name == "pwm_config_status_defaults") {
        PwmController::begin(); checkConfiguration(55,5);
        const size_t writes=Mock::pwmWrites.size();
        CHECK(PwmController::configure(55,5)); checkConfiguration(55,5);
        CHECK(Mock::pwmWrites.size()==writes);
    } else if (name == "pwm_config_raise_peak_full_255_cycle") {
        PwmController::begin(); CHECK(PwmController::configure(255,1));
        for(unsigned tick=1;tick<=1020;++tick) {
            pollAfter(1); const unsigned cycle=tick%510;
            const int expected=cycle<=255 ? static_cast<int>(cycle) : static_cast<int>(510-cycle);
            CHECK(PwmController::currentValue()==expected); checkConfiguration(255,1);
        }
        CHECK(PwmController::currentValue()==0); pollAfter(1); CHECK(PwmController::currentValue()==1);
    } else if (name == "pwm_config_lower_peak_clamps_and_descends") {
        PwmController::begin(); for(unsigned tick=0;tick<30;++tick) pollAfter(5);
        CHECK(PwmController::currentValue()==30); const size_t writes=Mock::pwmWrites.size();
        CHECK(PwmController::configure(10,5)); checkConfiguration(10,5);
        CHECK(PwmController::currentValue()==10 && Mock::pwmWrites.size()==writes+1);
        for(int expected=9;expected>=0;--expected) { pollAfter(5); CHECK(PwmController::currentValue()==expected); }
        pollAfter(5); CHECK(PwmController::currentValue()==1);
        for(unsigned tick=0;tick<4;++tick) pollAfter(5);
        CHECK(PwmController::currentValue()==5); CHECK(PwmController::configure(5,5));
        pollAfter(5); CHECK(PwmController::currentValue()==4); checkConfiguration(5,5);
    } else if (name == "pwm_config_peak_zero_off_and_resume") {
        PwmController::begin(); for(unsigned tick=0;tick<10;++tick) pollAfter(5);
        CHECK(PwmController::configure(0,5)); checkConfiguration(0,5);
        const size_t writes=Mock::pwmWrites.size(); pollAfter(1000000);
        CHECK(PwmController::currentValue()==0 && Mock::pwmWrites.size()==writes);
        CHECK(PwmController::configure(7,5)); pollAfter(5);
        CHECK(PwmController::currentValue()==1); checkConfiguration(7,5);
    } else if (name == "pwm_config_same_settings_preserve_phase") {
        PwmController::begin(); pollAfter(5); pollAfter(4);
        const size_t writes=Mock::pwmWrites.size(); CHECK(PwmController::configure(55,5));
        CHECK(Mock::pwmWrites.size()==writes); pollAfter(1); CHECK(PwmController::currentValue()==2);
        for(unsigned tick=0;tick<58;++tick) pollAfter(5);
        CHECK(PwmController::currentValue()==50); CHECK(PwmController::configure(55,5));
        pollAfter(5); CHECK(PwmController::currentValue()==49); checkConfiguration(55,5);
    } else if (name == "pwm_config_peak_only_restarts_clock") {
        PwmController::begin(); pollAfter(5); pollAfter(3);
        CHECK(PwmController::configure(30,5)); pollAfter(4); CHECK(PwmController::currentValue()==1);
        pollAfter(1); CHECK(PwmController::currentValue()==2); checkConfiguration(30,5);
    } else if (name == "pwm_config_changed_interval_new_deadline") {
        PwmController::begin(); pollAfter(5); pollAfter(2); CHECK(PwmController::configure(55,20));
        pollAfter(19); CHECK(PwmController::currentValue()==1);
        pollAfter(1); CHECK(PwmController::currentValue()==2 && Mock::pwmWrites.back().at==27);
        CHECK(PwmController::configure(55,1)); pollAfter(0); CHECK(PwmController::currentValue()==2);
        pollAfter(1); CHECK(PwmController::currentValue()==3);
        CHECK(PwmController::configure(55,1000)); pollAfter(999); CHECK(PwmController::currentValue()==3);
        pollAfter(1); CHECK(PwmController::currentValue()==4); checkConfiguration(55,1000);
    } else if (name == "pwm_config_invalid_nonmutating") {
        PwmController::begin(); CHECK(PwmController::configure(60,7)); pollAfter(6);
        const size_t writes=Mock::pwmWrites.size();
        struct Invalid {uint16_t peak;uint32_t interval;};
        for(const auto invalid : {Invalid{256,5},Invalid{65535,1},Invalid{55,0},Invalid{1,1001},Invalid{255,UINT32_MAX}}) {
            CHECK(!PwmController::configure(invalid.peak,invalid.interval)); checkConfiguration(60,7);
            CHECK(PwmController::currentValue()==0 && Mock::pwmWrites.size()==writes);
        }
        pollAfter(1); CHECK(PwmController::currentValue()==1);
    } else if (name == "pwm_config_begin_restores_defaults") {
        PwmController::begin(); CHECK(PwmController::configure(255,1));
        for(unsigned tick=0;tick<80;++tick) pollAfter(1);
        CHECK(PwmController::currentValue()==80); PwmController::begin(); checkConfiguration(55,5);
        CHECK(PwmController::currentValue()==0); pollAfter(4); CHECK(PwmController::currentValue()==0);
        pollAfter(1); CHECK(PwmController::currentValue()==1);
    } else if (name == "pwm_config_changed_interval_rollover") {
        mockMillis=UINT32_MAX-3U; PwmController::begin(); pollAfter(1);
        CHECK(PwmController::configure(60,7)); pollAfter(6); CHECK(PwmController::currentValue()==0);
        pollAfter(1); CHECK(mockMillis==4 && PwmController::currentValue()==1); checkConfiguration(60,7);
        CHECK(PwmController::configure(0,1)); CHECK(PwmController::configure(1,1));
        pollAfter(1); CHECK(PwmController::currentValue()==1); pollAfter(1); CHECK(PwmController::currentValue()==0);
    } else if (name == "pwm_strobe_start_full_255_interval") {
        PwmController::begin(); PwmController::setRcStrobe(true);
        CHECK(PwmController::currentValue()==255 && Mock::pwmDuty[14]==255);
        CHECK(PwmController::status().strobeEnabled && PwmController::status().rcControlled);
        pollAfter(49); CHECK(PwmController::currentValue()==255);
        pollAfter(1); CHECK(PwmController::currentValue()==0);
        pollAfter(49); CHECK(PwmController::currentValue()==0);
        pollAfter(1); CHECK(PwmController::currentValue()==255);
        CHECK(Mock::pwmWrites.size()==4);
    } else if (name == "pwm_strobe_repeated_high_preserves_phase") {
        PwmController::begin(); PwmController::setRcStrobe(true); pollAfter(40);
        const size_t writes=Mock::pwmWrites.size(); PwmController::setRcStrobe(true);
        CHECK(Mock::pwmWrites.size()==writes); pollAfter(9); CHECK(PwmController::currentValue()==255);
        pollAfter(1); CHECK(PwmController::currentValue()==0);
        PwmController::setRcStrobe(true); CHECK(PwmController::currentValue()==0);
        pollAfter(50); CHECK(PwmController::currentValue()==255);
        CHECK(Mock::pwmWrites.size()==writes+2);
    } else if (name == "pwm_strobe_off_stays_off_gui_can_breathe") {
        PwmController::begin(); PwmController::setRcStrobe(true); pollAfter(50);
        PwmController::setRcStrobe(false);
        CHECK(PwmController::currentValue()==0 && !PwmController::status().strobeEnabled);
        CHECK(PwmController::status().rcControlled);
        const size_t writes=Mock::pwmWrites.size(); pollAfter(5000);
        PwmController::setRcStrobe(false);
        CHECK(Mock::pwmWrites.size()==writes && PwmController::currentValue()==0);
        CHECK(PwmController::configure(55,5));
        CHECK(!PwmController::status().strobeEnabled && !PwmController::status().rcControlled);
        pollAfter(5); CHECK(PwmController::currentValue()==1);
    } else if (name == "pwm_strobe_config_saved_without_dimming") {
        PwmController::begin(); PwmController::setRcStrobe(true);
        CHECK(PwmController::configure(7,9));
        CHECK(PwmController::status().maxValue==7 && PwmController::status().stepIntervalMs==9);
        CHECK(PwmController::currentValue()==255 && PwmController::status().strobeEnabled);
        pollAfter(49); CHECK(PwmController::currentValue()==255); pollAfter(1);
        CHECK(PwmController::currentValue()==0); CHECK(PwmController::configure(0,1));
        pollAfter(50); CHECK(PwmController::currentValue()==255);
        CHECK(!PwmController::configure(256,5) && !PwmController::configure(55,0));
        CHECK(PwmController::currentValue()==255 && PwmController::status().strobeEnabled);
        PwmController::setRcStrobe(false); CHECK(PwmController::currentValue()==0);
        CHECK(PwmController::configure(7,9)); pollAfter(8); CHECK(PwmController::currentValue()==0);
        pollAfter(1); CHECK(PwmController::currentValue()==1 && !PwmController::status().rcControlled);
    } else if (name == "pwm_strobe_rollover_late_poll_one_toggle") {
        mockMillis=UINT32_MAX-20U; PwmController::begin(); PwmController::setRcStrobe(true);
        pollAfter(49); CHECK(mockMillis==28 && PwmController::currentValue()==255);
        pollAfter(1); CHECK(PwmController::currentValue()==0);
        const size_t writes=Mock::pwmWrites.size(); pollAfter(5000);
        CHECK(PwmController::currentValue()==255 && Mock::pwmWrites.size()==writes+1);
        pollAfter(0); pollAfter(49); CHECK(PwmController::currentValue()==255);
        pollAfter(1); CHECK(PwmController::currentValue()==0);
    } else if (name == "pwm_strobe_gpio_cylinder_independent") {
        Mock::gpioState[17]=HIGH; Mock::gpioState[16]=LOW;
        PwmController::begin(); PwmController::setRcStrobe(true);
        for(unsigned i=0;i<12;++i) {
            pollAfter(50);
            CHECK(Mock::gpioState[17]==HIGH && Mock::gpioState[16]==LOW);
            CHECK(PwmController::currentValue()==(i%2 ? 255 : 0));
        }
        PwmController::setRcStrobe(false); CHECK(Mock::gpioState[17]==HIGH);
        Mock::gpioState[17]=LOW;
    } else if (name == "pwm_strobe_begin_restores_breathe_defaults") {
        PwmController::begin(); PwmController::setRcStrobe(true);
        CHECK(PwmController::configure(10,20)); pollAfter(50); PwmController::begin();
        checkConfiguration(55,5);
        CHECK(!PwmController::status().strobeEnabled && !PwmController::status().rcControlled);
        pollAfter(5); CHECK(PwmController::currentValue()==1);
    } else throw std::runtime_error("Unknown PWM case");
    assertPwmWrites();
}
int main(int argc, char **argv) {
    try { CHECK(argc == 2); test(argv[1]); printf("PASS %s\n", argv[1]); return 0; }
    catch (const std::exception &error) { fprintf(stderr, "FAIL %s: %s\n", argc > 1 ? argv[1] : "?", error.what()); return 1; }
}
