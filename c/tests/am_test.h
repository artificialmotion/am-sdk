/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal test harness: no dependencies, one executable per test file, exit code = failures. */
#ifndef AM_TEST_H
#define AM_TEST_H

#include <math.h>
#include <stdio.h>

static int am_test_failures = 0;
static int am_test_checks = 0;

#define CHECK(cond)                                                                       \
    do {                                                                                  \
        am_test_checks++;                                                                 \
        if (!(cond)) {                                                                    \
            am_test_failures++;                                                           \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                                 \
    } while (0)

#define CHECK_ST(expr, want)                                                              \
    do {                                                                                  \
        int got_ = (int)(expr);                                                           \
        am_test_checks++;                                                                 \
        if (got_ != (int)(want)) {                                                        \
            am_test_failures++;                                                           \
            fprintf(stderr, "%s:%d: %s returned %d, expected %d\n", __FILE__, __LINE__,   \
                    #expr, got_, (int)(want));                                            \
        }                                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                             \
    do {                                                                                  \
        double a_ = (double)(a), b_ = (double)(b);                                        \
        am_test_checks++;                                                                 \
        if (!(fabs(a_ - b_) <= (tol))) {                                                  \
            am_test_failures++;                                                           \
            fprintf(stderr, "%s:%d: %s = %.9g, expected %.9g (tol %g)\n", __FILE__,       \
                    __LINE__, #a, a_, b_, (double)(tol));                                 \
        }                                                                                 \
    } while (0)

#define RUN(fn)                                                                           \
    do {                                                                                  \
        int before_ = am_test_failures;                                                   \
        fn();                                                                             \
        printf("%-48s %s\n", #fn, am_test_failures == before_ ? "ok" : "FAILED");        \
    } while (0)

#define TEST_EXIT()                                                                       \
    do {                                                                                  \
        printf("%d checks, %d failures\n", am_test_checks, am_test_failures);             \
        return am_test_failures ? 1 : 0;                                                  \
    } while (0)

#endif
