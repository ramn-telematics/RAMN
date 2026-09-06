#include "harness.h"

int h_checks = 0, h_failures = 0, h_bugs_confirmed = 0, h_bugs_fixed = 0;
const char *h_case = "";

void h_case_begin(const char *name) { h_case = name; printf("\n  %s\n", name); }

void h_report(int ok, int is_bug, const char *msg, const char *note,
              const char *file, int line)
{
    h_checks++;
    if (is_bug) {
        if (!ok) {
            h_bugs_confirmed++;
            printf("    KNOWN BUG   %s\n", msg);
            if (note) printf("                %s\n", note);
        } else {
            h_bugs_fixed++;
            h_failures++;
            printf("    NOW PASSING %s  (%s:%d)\n", msg, file, line);
            printf("                This defect appears fixed. Remove the\n");
            printf("                CHECK_BUG marker and make it a CHECK.\n");
        }
        return;
    }
    if (!ok) {
        h_failures++;
        printf("    FAIL        %s  (%s:%d)\n", msg, file, line);
    }
}
