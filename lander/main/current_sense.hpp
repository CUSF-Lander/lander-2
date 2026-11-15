#ifndef CURRENT_SENSE_HPP
#define CURRENT_SENSE_HPP

// Latest ESC current reading in amps, or CURRENT_SENSE_INVALID_A when the sense
// input is saturated at the ADC rail (lead disconnected / ESC unpowered).
// Written by the current-sense task at ~10 Hz and read when telemetry is built.
extern float latest_current_a;

// Sentinel for "no valid reading" (a disconnected lead used to read as a
// constant full-scale current).
static constexpr float CURRENT_SENSE_INVALID_A = -1.0f;

// Create the ESC current-sense task (ADC sampling + 1 Hz log line).
void init_current_sense();

#endif // CURRENT_SENSE_HPP
