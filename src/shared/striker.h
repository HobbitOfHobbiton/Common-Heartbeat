#pragma once

// Drum-strike actuation layer (gravity + elastic hammer, capstan-cocked).
//
// Replaces the old continuous-rotation cam approach (motorPhaseLock.*). A string
// winds onto the motor spool: winding lifts the mallet and stretches an elastic
// (COCK); rapidly unwinding throws the string slack so gravity + the elastic snap
// the mallet into the drum (STRIKE), with the rotor decoupled from the hit.
//
// The strike is scheduled a fixed lead time *ahead* of the heartbeat's predicted
// beat, so the hit lands on the beat despite the swing taking tens of ms.
//
// All the rig geometry (cock revolutions, speeds, release lead) is unknown until
// the mechanism is built, so it's tunable live over the serial console — see the
// help text in striker.cpp (type '?' in the monitor) for the command list.

// Set up the stepper for cock/release moves and print the console help. Call once
// from setup(), after motorInit().
void strikerInit();

// Run one iteration of the state machine: poll the serial console, fold in beat
// timing, and step the cock/release move. Call as often as possible from loop().
void strikerUpdate();
