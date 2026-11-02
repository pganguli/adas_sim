/**
 * @file plan_delay.h
 * @brief Delay line between the pipeline and the low-level controllers.
 *
 * Every plan is pushed with the sim time at which it becomes available
 * (capture time + injected latency, not rounded to sim steps). The
 * controllers use the newest plan that is already due, and hold it until the
 * next one is due (zero-order hold). Plans are released in push order.
 */
#ifndef ADAS_PLAN_DELAY_H
#define ADAS_PLAN_DELAY_H

#include <stdbool.h>

#include "types/planning_types.h"

/** Plans in flight at most (8 x 50 ms covers latencies up to 400 ms). */
#define PLAN_DELAY_CAPACITY 8

/** FIFO of in-flight plans plus the plan currently applied. */
typedef struct {
  PlanOutput active;                       /**< plan the controllers use */
  PlanOutput pending[PLAN_DELAY_CAPACITY]; /**< in-flight plans (ring)   */
  double due_s[PLAN_DELAY_CAPACITY];       /**< release time of each [s] */
  int head;                                /**< index of the oldest      */
  int count;                               /**< number in flight         */
} PlanDelay;

/** Empty the queue; the active plan is all-zero (mode OFF, LKA inactive). */
void plan_delay_init(PlanDelay *q);

/** Queue `plan` for release at `due_s`; drops the oldest when full. */
void plan_delay_push(PlanDelay *q, const PlanOutput *plan, double due_s);

/** Release every plan due by `t_s` and return the one to apply now. */
const PlanOutput *plan_delay_poll(PlanDelay *q, double t_s);

/** Release time of the oldest plan in flight; false if none is. */
bool plan_delay_next_due(const PlanDelay *q, double *due_s);

#endif /* ADAS_PLAN_DELAY_H */
