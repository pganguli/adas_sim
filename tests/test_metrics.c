/**
 * @file test_metrics.c
 * @brief Tests for the run metrics.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/metrics.h"
#include "test_util.h"

void test_metrics(void) {
  RunMetrics m;
  metrics_init(&m);
  CHECK(m.n_ticks == 0);
  CHECK(!m.collision);

  /* A run that never ticked must not print NaN/inf percentages. */
  char *buf = NULL;
  size_t len = 0;
  FILE *fp = open_memstream(&buf, &len);
  CHECK(fp != NULL);
  if (fp != NULL) {
    metrics_print(&m, fp);
    CHECK(fclose(fp) == 0);
    CHECK(buf != NULL && strstr(buf, "nan") == NULL &&
          strstr(buf, "inf") == NULL);
    free(buf);
  }
}
