/* Key-to-note mapping for every supported bandoneon tuning.
 *
 * The note data was originally digitized from photographs of an acoustic
 * instrument; it is hand-maintained here. */

#ifndef KEYBOARD_LAYOUT_H_
#define KEYBOARD_LAYOUT_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MIDI note numbers by name: NOTE(<letter>, <octave>), C4 = 60 (middle C).
 * Append 's' to the letter for sharp, e.g. NOTE(As, 3) = 58.
 * NOTE_NONE marks a key id with no assigned note. */
#define NOTE_NONE 0
#define NOTE_C  0
#define NOTE_Cs 1
#define NOTE_D  2
#define NOTE_Ds 3
#define NOTE_E  4
#define NOTE_F  5
#define NOTE_Fs 6
#define NOTE_G  7
#define NOTE_Gs 8
#define NOTE_A  9
#define NOTE_As 10
#define NOTE_B  11
#define NOTE(name, octave) (((octave) + 1) * 12 + NOTE_##name)

typedef enum { BELLOWS_PULL = 0, BELLOWS_PUSH = 1, BELLOWS_NEUTRAL = 2 } bellows_t;

#define NUM_KEYS 40

/* Tunings sharing the same button disposition. Rheinische is bisonoric (a key
 * sounds a different note on push and pull); the two French systems are
 * unisonoric, so a key sounds the same note either way. Selected at runtime by
 * the keyboard_tuning property. */
typedef enum {
  TUNING_RHEINISCHE = 0,   /* Rheinische Tonlage, 142 tones, bisonoric */
  TUNING_PEGURI     = 1,   /* Peguri, unisonoric */
  TUNING_MANOURY    = 2,   /* Manoury revision of the Peguri system, unisonoric */
  NUM_TUNINGS       = 3,
} tuning_t;

/* The keyboard sides, indexing note_table. A wing announces itself with a wing
 * id, which is not the same numbering: map it with wing_side(). */
typedef enum { SIDE_RIGHT = 0, SIDE_LEFT = 1, NUM_SIDES = 2 } side_t;

/* [tuning][side][bellows] -> the NUM_KEYS notes for that combination. Entries
 * share their note array wherever the data repeats: a unisonoric tuning points
 * push and pull at the same notes, and Peguri and Manoury differ only in the
 * left hand, so they name the same right-hand array. */
extern const uint8_t *const note_table[NUM_TUNINGS][NUM_SIDES][2];

/* Returns the note_table side for wing_id, or -1 if wing_id is not a wing this
 * firmware knows. Also the validity check for a received wing id: a corrupted
 * frame, or a bus that has not yet had a good frame, yields -1. */
static inline int wing_side(uint8_t wing_id)
{
  switch (wing_id)
  {
    case 1: return SIDE_RIGHT;
    case 2: return SIDE_LEFT;
    default: return -1;
  }
}

/* Returns the keyboard side's name for wing_id, or NULL if wing_id is unknown. */
static inline const char *wing_name(uint8_t wing_id)
{
  switch (wing_side(wing_id))
  {
    case SIDE_RIGHT: return "right";
    case SIDE_LEFT:  return "left";
    default: return NULL;
  }
}

/* Returns the tuning's name, or NULL if tuning is out of range. */
static inline const char *tuning_name(uint8_t tuning)
{
  switch (tuning)
  {
    case TUNING_RHEINISCHE: return "rheinische_tonlage_142_tones";
    case TUNING_PEGURI:     return "peguri";
    case TUNING_MANOURY:    return "manoury";
    default: return NULL;
  }
}

#ifdef __cplusplus
}
#endif

#endif /* KEYBOARD_LAYOUT_H_ */
