/*
  config.h — Omnibot ELRS tuning parameters

  This is the only file you should need to edit for your specific build.
  Start here if the robot moves the wrong way, a channel feels backwards,
  or you want to adjust speed or sensitivity.
*/

// ── Speed ─────────────────────────────────────────────────────────────────────
// Maximum PWM duty cycle sent to the drive motors [0–255].
// 255 = full speed. Start lower (150–200) while tuning direction flags, then
// increase once everything moves the right way.
uint8_t maxSpeed = 200;

// ── Motor direction ───────────────────────────────────────────────────────────
// If a drive wheel spins the wrong way, flip its flag here rather than
// rewiring. Flip LIFT (M4) if the fork goes down when you expect up.
bool flipM1 = false;
bool flipM2 = false;
bool flipM3 = false;
bool flipM4 = true;

// ── Channel assignments ───────────────────────────────────────────────────────
// CRSF channels are 0-indexed. These defaults match Radiomaster Mode 2:
//
//   Left stick                Right stick
//   ┌──────┐                  ┌──────┐
//   │  CH4 │ ← rotate CW/CCW  │  CH1 │ ← strafe left/right
//   │  CH3 │ ← fork up/down   │  CH2 │ ← forward/back
//   └──────┘  (throttle,      └──────┘  (auto-centering)
//              non-centering)
//
// CH3 on the throttle axis is intentional: the non-centering ratchet makes
// the fork hold its last commanded speed when you release the stick, giving
// you precise lift control without fighting a return spring.
const uint8_t CH_STRAFE = 0;  // right stick X
const uint8_t CH_DRIVE  = 1;  // right stick Y
const uint8_t CH_FORK   = 2;  // left stick Y  (throttle)
const uint8_t CH_ROTATE = 3;  // left stick X  (yaw)

// ── Axis inversion ────────────────────────────────────────────────────────────
// If pushing a stick forward makes the robot go backward (or similar), set
// the corresponding flag to true. Test one axis at a time.
const bool INV_STRAFE = true;
const bool INV_DRIVE  = true;   // stick up → robot forward (CRSF counts up = pull back)
const bool INV_FORK   = false;
const bool INV_ROTATE = false;

// ── Dead zone ─────────────────────────────────────────────────────────────────
// Sticks never return to a perfect electrical center. This threshold (in the
// internal ±1000 scale) suppresses that drift so the robot stays still when
// you release a stick. 120 ≈ 12% of full throw.
// Raise it if the robot creeps when sticks are released.
// Lower it if the robot feels sluggish to start moving.
const int CRSF_DEADBAND = 120;

// The throttle stick is non-centering so it rarely rests at exactly zero.
// A wider dead zone here stops the forklift from creeping when the stick
// is sitting near but not at the bottom of its travel. 250 ≈ 25% of throw.
const int FORK_DEADBAND = 250;
