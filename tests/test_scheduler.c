/**
 * @file test_scheduler.c
 * @brief Tests for the DAG scheduler.
 */
#include <stdio.h>

#include "pipeline/scheduler.h"
#include "test_util.h"

/** Execution log shared by the test node functions. */
typedef struct {
  int order[SCHED_MAX_NODES];
  int n;
} RunLog;

/** Node function that records its own id (passed via the node table index). */
static void record_a(void *ctx) {
  ((RunLog *)ctx)->order[((RunLog *)ctx)->n++] = 0;
}
static void record_b(void *ctx) {
  ((RunLog *)ctx)->order[((RunLog *)ctx)->n++] = 1;
}
static void record_c(void *ctx) {
  ((RunLog *)ctx)->order[((RunLog *)ctx)->n++] = 2;
}
static void record_d(void *ctx) {
  ((RunLog *)ctx)->order[((RunLog *)ctx)->n++] = 3;
}

void test_scheduler(void) {
  /* Diamond: A -> {B, C} -> D */
  const TaskNode diamond[4] = {
      {"A", record_a, 0, {0}, 1.0f},
      {"B", record_b, 1, {0}, 2.0f},
      {"C", record_c, 1, {0}, 3.0f},
      {"D", record_d, 2, {1, 2}, 1.0f},
  };
  Scheduler s;
  SchedulerStats stats;
  RunLog log = {{0}, 0};

  CHECK(scheduler_init(&s, diamond, 4) == 0);
  CHECK(s.n_levels == 3);
  CHECK(s.level[0] == 0 && s.level[1] == 1 && s.level[2] == 1 &&
        s.level[3] == 2);

  scheduler_run(&s, &log, &stats);
  CHECK(log.n == 4);
  CHECK(log.order[0] == 0 && log.order[3] == 3); /* deps first */
  CHECK_NEAR(stats.makespan_ms, 5.0f, 1e-6);     /* 1 + max(2, 3) + 1 */
  CHECK_NEAR(stats.serial_ms, 7.0f, 1e-6);
  CHECK_NEAR(stats.start_ms[3], 4.0f, 1e-6);

  /* Invalid graphs are rejected (the scheduler reports them on stderr). */
  const TaskNode cycle[2] = {
      {"X", record_a, 1, {1}, 1.0f},
      {"Y", record_b, 1, {0}, 1.0f},
  };
  const TaskNode self_dep[1] = {{"S", record_a, 1, {0}, 1.0f}};
  const TaskNode bad_dep[1] = {{"B", record_a, 1, {5}, 1.0f}};
  const TaskNode no_fn[1] = {{"N", NULL, 0, {0}, 1.0f}};
  CHECK(scheduler_init(&s, cycle, 2) == -1);
  CHECK(scheduler_init(&s, self_dep, 1) == -1);
  CHECK(scheduler_init(&s, bad_dep, 1) == -1);
  CHECK(scheduler_init(&s, no_fn, 1) == -1);
  CHECK(scheduler_init(&s, diamond, 0) == -1);
  CHECK(scheduler_init(&s, diamond, SCHED_MAX_NODES + 1) == -1);
}
