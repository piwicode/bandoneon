# Features

## Keyboard tuning (FN2)

The wing boards report which side they are (left or right); the notes those
buttons play are chosen by the `keyboard_tuning` property, so one firmware
covers several bandoneon systems on the same hardware. The **right function
button (FN2)** cycles through them, wrapping after the last, and the tuning now
selected is reported on the console. FN2 only writes the property, so the button
and `set keyboard_tuning` stay in agreement.

| Value | System | |
|---|---|---|
| 0 | **Rheinische Tonlage**, 142 tones (default) | bisonoric |
| 1 | **Peguri** | unisonoric |
| 2 | **Manoury** | unisonoric |

*Bisonoric* means a key sounds a different note depending on whether the bellows
is pushed or pulled — the traditional Argentine bandoneon. The two French
systems are *unisonoric*: every key sounds the same note in both directions.
All three share the same button disposition, so switching tuning changes only
which note each button plays.

Two notes of the published Peguri/Manoury right-hand keyboard are missing here.
Those layouts use 40 right-hand buttons where this instrument has 38, and the
two absent buttons are the lowest of their rows (D4 and D♯4). What remains is
chromatically complete from E4 upward.

Changing the tuning while keys are held is safe: a sounding note keeps its
original pitch until released, and the next press uses the new tuning.

Note that property storage is still RAM-only, so the tuning returns to
Rheinische after a power cycle.

## MIDI Active Sensing

A heartbeat the device sends to the host so the host can detect a dropped
connection and cut the sound (silence stuck notes) when the heartbeat stops.

Settings:
- `midi_active_sensing_enable` — send the heartbeat (default on).
- `midi_active_sensing_period` — milliseconds between bytes (default 200).

## Table mode

The **left function button (FN0)** toggles table mode on and off; the current
state is reported on the console.

Normally a note only sounds while the bellows is moving, and its pitch and
velocity follow the bellows direction and intensity. Table mode lets the
instrument be played flat on a table with the bellows at rest: each key press
sounds immediately and the bellows is treated as always **pulled**, so every key
plays its pull note. This makes it practical to type a score into a DAW or
notation software one note at a time, without having to work the bellows.

With the bellows at rest it can shape neither loudness nor expression, so table
mode holds both constant and stops the bellows driving them:

- every note plays at the same velocity;
- the expression CC (**CC#11**) is no longer sent from the bellows. It is pinned
  once, on entering table mode, to that same value. Without this the bellows
  would sit at rest, hold CC#11 at 0, and silence every note.

Both take their value from `tablemode_velocity` (default 80).

Toggling table mode off re-evaluates the keys currently held so sounding notes
follow the real bellows again, and hands CC#11 back to it.

## Bellows sensitivity

The **middle function button (FN1)** cycles bellows sensitivity through three
levels, wrapping back to the first; the current level is reported on the
console. Each level scales the bellows signal (the 0..1024 intensity that drives
both note velocity and the expression CC), so a higher level reaches full
velocity and full expression with less bellows travel — useful for quiet playing
or a stiff bellows.

Level 1 is unity (no scaling). Levels 2 and 3 multiply by `bellow_scale_mid`
(default x1.5) and `bellow_scale_high` (default x2.0); both are stored as a /256
fixed point value (256 = x1.0), so 384 and 512. The scaled intensity is clamped
to its full range, so beyond the point that reaches maximum the signal simply
saturates.

Table mode has no bellows signal to scale, so the sensitivity levels do not
affect it; it plays at `tablemode_velocity` whatever the level.

## Bellows inertia mode

Set `bellow_inertia_enable` to turn bellows inertia mode on; it is **off by
default**. Off, the bellows reading drives the sound
directly. On, the reading is run through a virtual-bellows pressure model that
gives the light blade spring the feel of an acoustic bandoneon's bellows: a quick
impulse stores energy that stays available for the note played just after, held
chords soften as air escapes through the open pallets (pressed keys), and the
push/pull direction is committed so it does not flicker near rest.

The behaviour is shaped by the `bellow_inertia_*` properties (responsiveness,
damping, impulse strength, air leak, and the direction deadzone/hysteresis). See
[bellow_simulation.md](bellow_simulation.md) for the principle, the model, and a
tuning guide.

## Pedals

Two pedal inputs, each accepting an expression pedal wired like the M-Audio
EX-P (mode switch set to **M-Audio**). A plugged-in pedal is read continuously
and sent on both keyboard channels: pedal 1 on the **Modulation wheel (CC#1)**,
pedal 2 on the **Foot Controller (CC#4)**. Both are pre-mapped in most
instruments, so the pedals are expressive out of the box, and either can be
MIDI-learned to another VST parameter in the DAW.

Per pedal, `pedalN_min`/`pedalN_max` set the raw ADC readings that map to CC 0
and CC 127; values between are interpolated linearly and values outside are
clamped. Setting this window inside the pedal's full travel lets the usable
stroke span the whole MIDI range. To calibrate, push the pedal to each extreme
and use the raw ADC readings there as min and max.
