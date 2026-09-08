#include "keyboard_layout.h"

/* Note arrays are indexed by key id, i.e. by the wing's scan position, not by
 * where the button sits on the keyboard. Key ids the wing does not populate
 * hold NOTE_NONE: the right wing populates ids 0..37, the left id 0 and ids
 * 8..39. */

/* ---- Rheinische Tonlage, 142 tones (bisonoric) --------------------------- */

static const uint8_t rheinische_right_pull[NUM_KEYS] =
{
  /*  0 */ NOTE(A,3), NOTE(B,3), NOTE(Ds,4), NOTE(E,4), NOTE(F,4),
  /*  5 */ NOTE(F,5), NOTE(As,3), NOTE(As,4), NOTE(Ds,5), NOTE(D,4),
  /* 10 */ NOTE(Gs,4), NOTE(B,4), NOTE(Cs,5), NOTE(Cs,4), NOTE(C,4),
  /* 15 */ NOTE(Fs,5), NOTE(G,4), NOTE(A,5), NOTE(A,6), NOTE(As,5),
  /* 20 */ NOTE(B,6), NOTE(D,5), NOTE(Fs,4), NOTE(A,4), NOTE(Gs,6),
  /* 25 */ NOTE(C,5), NOTE(Cs,6), NOTE(G,6), NOTE(Gs,5), NOTE(C,6),
  /* 30 */ NOTE(Fs,6), NOTE(E,6), NOTE(G,5), NOTE(D,6), NOTE(B,5),
  /* 35 */ NOTE(F,6), NOTE(E,5), NOTE(Ds,6), NOTE_NONE, NOTE_NONE,
};

static const uint8_t rheinische_right_push[NUM_KEYS] =
{
  /*  0 */ NOTE(A,3), NOTE(B,3), NOTE(Ds,4), NOTE(Fs,4), NOTE(F,4),
  /*  5 */ NOTE(F,5), NOTE(As,3), NOTE(E,4), NOTE(E,5), NOTE(Cs,4),
  /* 10 */ NOTE(A,4), NOTE(Cs,5), NOTE(Fs,5), NOTE(C,4), NOTE(D,4),
  /* 15 */ NOTE(Gs,5), NOTE(Gs,4), NOTE(B,5), NOTE(G,6), NOTE(As,4),
  /* 20 */ NOTE(A,6), NOTE(E,5), NOTE(G,4), NOTE(B,4), NOTE(Gs,6),
  /* 25 */ NOTE(D,5), NOTE(E,6), NOTE(Fs,6), NOTE(A,5), NOTE(C,5),
  /* 30 */ NOTE(As,5), NOTE(C,6), NOTE(Ds,5), NOTE(D,6), NOTE(Cs,6),
  /* 35 */ NOTE(F,6), NOTE(G,5), NOTE(Ds,6), NOTE_NONE, NOTE_NONE,
};

static const uint8_t rheinische_left_pull[NUM_KEYS] =
{
  /*  0 */ NOTE(D,2), NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
  /*  5 */ NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE(E,2), NOTE(A,2),
  /* 10 */ NOTE(B,2), NOTE(G,4), NOTE(D,3), NOTE(Gs,2), NOTE(E,3),
  /* 15 */ NOTE(Gs,3), NOTE(B,3), NOTE(A,4), NOTE(G,3), NOTE(Cs,3),
  /* 20 */ NOTE(As,2), NOTE(C,4), NOTE(A,3), NOTE(Ds,3), NOTE(E,4),
  /* 25 */ NOTE(Ds,4), NOTE(F,4), NOTE(As,3), NOTE(F,3), NOTE(Fs,4),
  /* 30 */ NOTE(D,4), NOTE(C,3), NOTE(Fs,3), NOTE(F,2), NOTE(Cs,4),
  /* 35 */ NOTE(C,2), NOTE(G,2), NOTE(Ds,2), NOTE(Gs,4), NOTE(Fs,2),
};

static const uint8_t rheinische_left_push[NUM_KEYS] =
{
  /*  0 */ NOTE(E,2), NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
  /*  5 */ NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE(D,2), NOTE(D,3),
  /* 10 */ NOTE(B,4), NOTE(Fs,4), NOTE(G,2), NOTE(Gs,2), NOTE(A,2),
  /* 15 */ NOTE(E,3), NOTE(A,3), NOTE(Cs,2), NOTE(As,3), NOTE(Ds,3),
  /* 20 */ NOTE(As,2), NOTE(B,3), NOTE(G,3), NOTE(C,4), NOTE(D,4),
  /* 25 */ NOTE(F,2), NOTE(Cs,3), NOTE(C,3), NOTE(Ds,4), NOTE(E,4),
  /* 30 */ NOTE(Cs,4), NOTE(F,4), NOTE(F,3), NOTE(Fs,2), NOTE(Gs,3),
  /* 35 */ NOTE(F,2), NOTE(Fs,3), NOTE(Cs,2), NOTE(G,4), NOTE(B,2),
};

/* ---- Peguri / Manoury (unisonoric) --------------------------------------- */

/* Shared by both French systems: their right-hand keyboards are identical, and
 * being unisonoric each is used for push and pull alike.
 *
 * The published Peguri/Manoury right keyboard has 40 buttons, arranged in rows
 * of 4,5,6,8,8,9. This instrument's right wing has 38, in rows of 4,5,6,7,8,8:
 * the lowest button of row 4 (D4) and of row 6 (Ds4) are absent from the board,
 * so those two notes cannot be played. What remains is chromatically complete
 * from E4 upward. */
static const uint8_t unisonoric_right[NUM_KEYS] =
{
  /*  0 */ NOTE(E,4), NOTE(F,4), NOTE(A,4), NOTE(Gs,4), NOTE(G,4),
  /*  5 */ NOTE(C,5), NOTE(Fs,4), NOTE(As,4), NOTE(Ds,5), NOTE(C,4),
  /* 10 */ NOTE(Cs,5), NOTE(E,5), NOTE(B,4), NOTE(A,3), NOTE(B,3),
  /* 15 */ NOTE(Fs,5), NOTE(Cs,4), NOTE(A,5), NOTE(As,3), NOTE(D,6),
  /* 20 */ NOTE(Gs,6), NOTE(G,5), NOTE(D,5), NOTE(F,5), NOTE(A,6),
  /* 25 */ NOTE(Gs,5), NOTE(C,6), NOTE(As,6), NOTE(As,5), NOTE(Ds,6),
  /* 30 */ NOTE(F,6), NOTE(Fs,6), NOTE(Ds,6), NOTE(E,6), NOTE(Cs,6),
  /* 35 */ NOTE(B,6), NOTE(B,5), NOTE(G,6), NOTE_NONE, NOTE_NONE,
};

/* Peguri left hand. Its top two rows carry high notes (A4, As4, B4) that the
 * Manoury revision replaces with the low notes the system was missing. */
static const uint8_t peguri_left[NUM_KEYS] =
{
  /*  0 */ NOTE(A,4), NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
  /*  5 */ NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE(Gs,4), NOTE(B,4),
  /* 10 */ NOTE(Fs,4), NOTE(Ds,4), NOTE(F,4), NOTE(Cs,2), NOTE(G,4),
  /* 15 */ NOTE(E,4), NOTE(Cs,4), NOTE(C,4), NOTE(As,4), NOTE(Gs,2),
  /* 20 */ NOTE(C,2), NOTE(B,3), NOTE(D,4), NOTE(A,4), NOTE(Gs,3),
  /* 25 */ NOTE(A,3), NOTE(B,2), NOTE(As,2), NOTE(G,2), NOTE(G,3),
  /* 30 */ NOTE(As,3), NOTE(F,3), NOTE(Fs,3), NOTE(A,2), NOTE(E,3),
  /* 35 */ NOTE(C,3), NOTE(D,3), NOTE(Ds,3), NOTE(Fs,2), NOTE(Cs,3),
};

/* Manoury left hand. Differs from peguri_left at seven key ids only:
 * 9, 13, 18, 19, 20, 23 and 28. */
static const uint8_t manoury_left[NUM_KEYS] =
{
  /*  0 */ NOTE(A,4), NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE_NONE,
  /*  5 */ NOTE_NONE, NOTE_NONE, NOTE_NONE, NOTE(Gs,4), NOTE(F,2),
  /* 10 */ NOTE(Fs,4), NOTE(Ds,4), NOTE(F,4), NOTE(D,2), NOTE(G,4),
  /* 15 */ NOTE(E,4), NOTE(Cs,4), NOTE(C,4), NOTE(E,2), NOTE(C,2),
  /* 20 */ NOTE(Cs,2), NOTE(B,3), NOTE(D,4), NOTE(Ds,2), NOTE(Gs,3),
  /* 25 */ NOTE(A,3), NOTE(B,2), NOTE(As,2), NOTE(Gs,2), NOTE(G,3),
  /* 30 */ NOTE(As,3), NOTE(F,3), NOTE(Fs,3), NOTE(A,2), NOTE(E,3),
  /* 35 */ NOTE(C,3), NOTE(D,3), NOTE(Ds,3), NOTE(Fs,2), NOTE(Cs,3),
};

const uint8_t *const note_table[NUM_TUNINGS][NUM_SIDES][2] =
{
  [TUNING_RHEINISCHE] =
  {
    [SIDE_RIGHT] = { [BELLOWS_PULL] = rheinische_right_pull, [BELLOWS_PUSH] = rheinische_right_push },
    [SIDE_LEFT]  = { [BELLOWS_PULL] = rheinische_left_pull,  [BELLOWS_PUSH] = rheinische_left_push  },
  },
  [TUNING_PEGURI] =
  {
    /* Unisonoric: the same notes whichever way the bellows moves. */
    [SIDE_RIGHT] = { unisonoric_right, unisonoric_right },
    [SIDE_LEFT]  = { peguri_left,      peguri_left      },
  },
  [TUNING_MANOURY] =
  {
    [SIDE_RIGHT] = { unisonoric_right, unisonoric_right },
    [SIDE_LEFT]  = { manoury_left,     manoury_left     },
  },
};
