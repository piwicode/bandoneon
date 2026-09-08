/* Host unit tests for the note tables. Pure C, no HAL — compile and run
 * natively (see `just test`). Same framework as hysteresis_test.c.
 *
 * These guard the shape of the data rather than individual notes: a tuning that
 * puts a note on a key id the wing does not populate would be silently
 * unplayable, and a unisonoric tuning that lost its push/pull sharing would
 * behave like a bisonoric one. */

#include "keyboard_layout.h"

#include <stdio.h>
#include <stdint.h>

static int g_checks;
static int g_failures;

#define CHECK(cond)                                                \
  do {                                                             \
    g_checks++;                                                    \
    if (!(cond)) {                                                 \
      g_failures++;                                                \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
    }                                                              \
  } while (0)

/* Every tuning has to leave exactly the same key ids unmapped as Rheinische:
 * those are the scan positions the wing hardware does not populate, so a note
 * placed there could never sound. */
static void test_unmapped_keys_agree(void)
{
  for (int side = 0; side < NUM_SIDES; side++)
  {
    for (int t = 0; t < NUM_TUNINGS; t++)
    {
      for (int dir = 0; dir <= 1; dir++)
      {
        const uint8_t *ref = note_table[TUNING_RHEINISCHE][side][dir];
        const uint8_t *cur = note_table[t][side][dir];
        for (int k = 0; k < NUM_KEYS; k++)
          CHECK((ref[k] == NOTE_NONE) == (cur[k] == NOTE_NONE));
      }
    }
  }
}

/* Peguri and Manoury are unisonoric: a key sounds the same note whichever way
 * the bellows moves. Sharing the array makes that true by construction, so
 * check the sharing itself. */
static void test_unisonoric_shares_push_and_pull(void)
{
  const int unisonoric[] = { TUNING_PEGURI, TUNING_MANOURY };
  for (size_t i = 0; i < sizeof unisonoric / sizeof *unisonoric; i++)
    for (int side = 0; side < NUM_SIDES; side++)
      CHECK(note_table[unisonoric[i]][side][BELLOWS_PULL] ==
            note_table[unisonoric[i]][side][BELLOWS_PUSH]);

  /* Rheinische is bisonoric, so its two directions must NOT be shared. */
  for (int side = 0; side < NUM_SIDES; side++)
    CHECK(note_table[TUNING_RHEINISCHE][side][BELLOWS_PULL] !=
          note_table[TUNING_RHEINISCHE][side][BELLOWS_PUSH]);
}

/* The two French systems differ only in the left hand; their right-hand
 * keyboards are the same array. */
static void test_peguri_manoury_share_right_hand(void)
{
  CHECK(note_table[TUNING_PEGURI][SIDE_RIGHT][BELLOWS_PUSH] ==
        note_table[TUNING_MANOURY][SIDE_RIGHT][BELLOWS_PUSH]);

  /* ...and their left hands differ at exactly seven key ids. */
  const uint8_t *p = note_table[TUNING_PEGURI][SIDE_LEFT][BELLOWS_PUSH];
  const uint8_t *m = note_table[TUNING_MANOURY][SIDE_LEFT][BELLOWS_PUSH];
  int differing = 0;
  for (int k = 0; k < NUM_KEYS; k++)
    differing += (p[k] != m[k]);
  CHECK(differing == 7);
}

/* Wing ids are not side indices, and everything outside {1,2} — including 0,
 * which is what a bus reports before its first good frame — has to be rejected
 * rather than indexing the table. */
static void test_wing_side(void)
{
  CHECK(wing_side(1) == SIDE_RIGHT);
  CHECK(wing_side(2) == SIDE_LEFT);
  CHECK(wing_side(0) < 0);
  CHECK(wing_side(3) < 0);
  CHECK(wing_side(255) < 0);
}

static void test_names(void)
{
  CHECK(wing_name(0) == NULL);
  CHECK(wing_name(1) != NULL);
  CHECK(wing_name(2) != NULL);
  CHECK(wing_name(3) == NULL);

  for (int t = 0; t < NUM_TUNINGS; t++)
    CHECK(tuning_name((uint8_t)t) != NULL);
  CHECK(tuning_name(NUM_TUNINGS) == NULL);
}

int main(void)
{
  test_unmapped_keys_agree();
  test_unisonoric_shares_push_and_pull();
  test_peguri_manoury_share_right_hand();
  test_wing_side();
  test_names();

  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
