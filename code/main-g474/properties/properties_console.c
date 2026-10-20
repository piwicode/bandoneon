#include "properties_console.h"
#include "properties.h"
#include "ansi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *type_name(property_type_t t)
{
  return t == PROPERTY_TYPE_BOOL ? "bool" : "u16";
}

/* Current value of a property regardless of type (bool -> 0/1). */
static uint16_t current_value(size_t index)
{
  const property_desc_t *d = property_at(index);
  if (d->type == PROPERTY_TYPE_BOOL)
  {
    bool b = false;
    property_get_bool(index, &b);
    return b ? 1u : 0u;
  }
  uint16_t v = 0;
  property_get_u16(index, &v);
  return v;
}

/* "name = value  (min M, max X, default D)", the default being the saved
 * value if there is one: "(..., saved S, factory F)". */
static void print_one(size_t index)
{
  const property_desc_t *d = property_at(index);
  uint16_t saved;
  if (property_get_saved(index, &saved))
    printf("%s = %u  (min %u, max %u, saved %u, factory %u)\r\n",
           d->name, current_value(index), d->min, d->max, saved, d->factory_value);
  else
    printf("%s = %u  (min %u, max %u, default %u)\r\n",
           d->name, current_value(index), d->min, d->max, d->factory_value);
}

/* Shell-style glob match supporting '*' (any run) and '?' (one char). Used so
 * `get log_*` / `set log_* 0` can address a whole family of properties at once. */
static bool name_match(const char *pat, const char *name)
{
  while (*pat)
  {
    if (*pat == '*')
    {
      while (*pat == '*') pat++;
      if (!*pat) return true;
      for (; *name; name++)
        if (name_match(pat, name)) return true;
      return false;
    }
    if (!*name || (*pat != '?' && *pat != *name)) return false;
    pat++; name++;
  }
  return !*name;
}

static bool has_wildcard(const char *s)
{
  return strchr(s, '*') || strchr(s, '?');
}

/* One row of the `get` table; `row` alternates the background. The default
 * column is the saved value, starred, if there is one, else the firmware's
 * factory value. */
static void print_row(size_t index, size_t row)
{
  const property_desc_t *d = property_at(index);
  uint16_t saved;
  bool has = property_get_saved(index, &saved);
  char range[16];
  snprintf(range, sizeof range, "[%u, %u]", d->min, d->max);
  printf("%s%-28s %5s %-12s %6u %7u%c %8u  %s" ANSI_RESET "\r\n", (row & 1) ? ANSI_BG_GREY236 : "",
         d->name, type_name(d->type), range, current_value(index),
         has ? saved : d->factory_value, has ? '*' : ' ', d->factory_value, d->description);
}

static bool property_cmd_get(int argc, const char *const *argv)
{
  if (argc < 2) { printf("usage: get <name>\r\n"); return false; }
  size_t matched = 0;
  for (size_t i = 0; i < property_count(); i++)
  {
    if (!name_match(argv[1], property_at(i)->name)) continue;
    if (!matched)
      printf(ANSI_BOLD ANSI_FG_GREEN "%-28s %5s %-12s %6s %8s %8s  %s" ANSI_RESET "\r\n", "name", "type", "range", "value", "default",
             "firmware", "description");
    print_row(i, matched++);
  }
  if (!matched) { printf("no property matches: %s\r\n", argv[1]); return false; }
  return true;
}

/* Parse a property value string. Returns false (with a message) if not a number. */
static bool parse_value(const char *s, unsigned long *out)
{
  char *end;
  unsigned long v = strtoul(s, &end, 0);
  if (s[0] == '\0' || *end != '\0') { printf("not a number: %s\r\n", s); return false; }
  *out = v;
  return true;
}

static void set_one(size_t i, unsigned long v)
{
  if (property_at(i)->type == PROPERTY_TYPE_BOOL) property_set_bool(i, v != 0);
  else property_set_u16(i, v > 0xFFFFu ? 0xFFFFu : (uint16_t)v);
  print_one(i);
}

static bool property_cmd_set(int argc, const char *const *argv)
{
  /* `set <bool name>` with no value flips the boolean. */
  if (argc == 2 && !has_wildcard(argv[1]))
  {
    size_t i;
    bool cur;
    if (!property_by_name(argv[1], &i)) { printf("unknown property: %s\r\n", argv[1]); return false; }
    if (!property_get_bool(i, &cur)) { printf("usage: set <name> <value>  (%s is not a boolean)\r\n", argv[1]); return false; }
    set_one(i, !cur);
    return true;
  }
  if (argc < 3) { printf("usage: set <name> [<value>]  (the value may be left out for a boolean)\r\n"); return false; }
  unsigned long v;
  if (!parse_value(argv[2], &v)) return false;
  if (has_wildcard(argv[1]))
  {
    size_t matched = 0;
    for (size_t i = 0; i < property_count(); i++)
      if (name_match(argv[1], property_at(i)->name)) { set_one(i, v); matched++; }
    if (!matched) { printf("no property matches: %s\r\n", argv[1]); return false; }
    return true;
  }
  size_t i;
  if (!property_by_name(argv[1], &i)) { printf("unknown property: %s\r\n", argv[1]); return false; }
  set_one(i, v);
  return true;
}

static bool property_cmd_reset(int argc, const char *const *argv)
{
  if (argc < 2) { printf("usage: reset <name>\r\n"); return false; }
  if (has_wildcard(argv[1]))
  {
    size_t matched = 0;
    for (size_t i = 0; i < property_count(); i++)
      if (name_match(argv[1], property_at(i)->name)) { property_reset(i); print_one(i); matched++; }
    if (!matched) { printf("no property matches: %s\r\n", argv[1]); return false; }
    return true;
  }
  size_t i;
  if (!property_by_name(argv[1], &i)) { printf("unknown property: %s\r\n", argv[1]); return false; }
  property_reset(i);
  print_one(i);
  return true;
}

/* The default column is the saved value, starred, if there is one. */
static void property_cmd_show(void)
{
  printf("%-18s %6s %6s %6s %8s %8s\r\n", "name", "value", "min", "max", "default", "factory");
  for (size_t i = 0; i < property_count(); i++)
  {
    const property_desc_t *d = property_at(i);
    uint16_t saved;
    bool has = property_get_saved(i, &saved);
    printf("%-18s %6u %6u %6u %7u%c %8u\r\n", d->name, current_value(i), d->min, d->max,
           has ? saved : d->factory_value, has ? '*' : ' ', d->factory_value);
  }
}

static const char *save_error(property_save_result_t r)
{
  switch (r)
  {
    case PROPERTY_SAVE_OK:          return "ok";
    case PROPERTY_SAVE_BAD_INDEX:   return "bad index";
    case PROPERTY_SAVE_TRANSIENT:   return "not a saved property";
    case PROPERTY_SAVE_FLASH_ERROR: return "flash write failed";
    case PROPERTY_SAVE_DISABLED:    return "saving disabled (flash not in 2 KB pages)";
  }
  return "?";
}

static void property_cmd_save(void)
{
  size_t n = 0;
  property_save_result_t r = property_save_all(&n);
  if (r != PROPERTY_SAVE_OK) printf("save: %s after %u changes\r\n", save_error(r), (unsigned)n);
  else printf("saved %u change%s\r\n", (unsigned)n, n == 1 ? "" : "s");
}

static void property_cmd_factory_reset(void)
{
  property_save_result_t r = property_factory_reset();
  if (r != PROPERTY_SAVE_OK) printf("factory_reset: %s\r\n", save_error(r));
  else printf("saved values erased, every property back to its factory value\r\n");
}

static void property_cmd_store(void)
{
  property_store_status_t st = property_store_status();
  if (!st.enabled) { printf("store: %s\r\n", save_error(PROPERTY_SAVE_DISABLED)); return; }
  if (st.partition < 0) { printf("store: nothing saved\r\n"); return; }
  printf("store: partition %c, generation %u, %u/%u slots used, %u saved value%s\r\n",
         'A' + st.partition, st.generation, st.used, st.capacity, st.saved_count,
         st.saved_count == 1 ? "" : "s");
}

bool properties_execute(int argc, const char *const *argv)
{
  if (argc < 1) return false;
  if (strcmp(argv[0], "show") == 0)  { property_cmd_show();        return true; }
  if (strcmp(argv[0], "get") == 0)   { property_cmd_get(argc, argv);   return true; }
  if (strcmp(argv[0], "set") == 0)   { property_cmd_set(argc, argv);   return true; }
  if (strcmp(argv[0], "reset") == 0) { property_cmd_reset(argc, argv); return true; }
  if (strcmp(argv[0], "save") == 0)  { property_cmd_save();         return true; }
  if (strcmp(argv[0], "factory_reset") == 0) { property_cmd_factory_reset(); return true; }
  if (strcmp(argv[0], "store") == 0) { property_cmd_store();        return true; }
  return false;
}

void properties_help(void)
{
  printf("Property commands:\r\n");
  printf("  show                 list all properties with current value (* = saved default)\r\n");
  printf("  get <name>           table of value, range, default, firmware value and description\r\n");
  printf("                       (name may glob, e.g. log_* or *)\r\n");
  printf("  set <name> [<value>] set a property, clamped to its range (name may glob, e.g. log_*);\r\n");
  printf("                       without a value, a boolean is toggled\r\n");
  printf("  reset <name>         restore default(s): saved value, else factory (name may glob)\r\n");
  printf("  save                 save current values as defaults, in flash\r\n");
  printf("  factory_reset        erase saved values, restore every factory value\r\n");
  printf("  store                state of the saved-values flash store\r\n");
  printf("  help                 this help\r\n");
}

size_t properties_complete(const char *prefix, const char **out, size_t cap)
{
  size_t n = 0;
  size_t plen = prefix ? strlen(prefix) : 0;
  for (size_t i = 0; i < property_count() && n < cap; i++)
  {
    const char *name = property_at(i)->name;
    if (strncmp(name, prefix ? prefix : "", plen) == 0) out[n++] = name;
  }
  return n;
}
