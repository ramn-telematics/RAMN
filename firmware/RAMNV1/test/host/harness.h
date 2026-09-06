/* Tiny assertion harness.
 *
 * CHECK      -- must pass; a failure fails the build.
 * CHECK_BUG  -- a known defect. Failing is expected and does NOT fail the
 *               build; but if it starts PASSING the build fails, telling you
 *               to delete the marker. A known-bug marker cannot rot silently.
 */
#pragma once
#include <stdio.h>

extern int h_checks, h_failures, h_bugs_confirmed, h_bugs_fixed;
extern const char *h_case;

void h_case_begin(const char *name);
void h_report(int ok, int is_bug, const char *msg, const char *note,
              const char *file, int line);

#define CHECK(cond, msg) \
    h_report((cond) ? 1 : 0, 0, (msg), NULL, __FILE__, __LINE__)

#define CHECK_BUG(cond, msg, note) \
    h_report((cond) ? 1 : 0, 1, (msg), (note), __FILE__, __LINE__)
