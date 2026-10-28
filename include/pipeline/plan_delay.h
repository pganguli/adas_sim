/**
 * @file plan_delay.h
 * @brief Delay line between the pipeline and the low-level controllers.
 *
 * Every plan is pushed with the sim step at which it becomes available
 * (capture step + injected latency). The controllers use the newest plan
 * that is already due, and hold it until the next one is due (zero-order
 * hold). Plans are released in push order.
 */
#ifndef ADAS_PLAN_DELAY_H
#define ADAS_PLAN_DELAY_H

#include "types/planning_types.h"

/** Plans in flight at most (8 x 50 ms covers latencies up to 400 ms). */
#define PLAN_DELAY_CAPACITY 8

/** FIFO of in-flight plans plus the plan currently applied. */
typedef struct {
  PlanOutput active;                       /**< plan the controllers use */
  PlanOutput pending[PLAN_DELAY_CAPACITY]; /**< in-flight plans (ring)   */
  int due_step[PLAN_DELAY_CAPACITY];       /**< release step of each     */
  int head;                                /**< index of the oldest      */
  int count;                               /**< number in flight         */
} PlanDelay;

/** Empty the queue; the active plan is all-zero (mode OFF, LKA inactive). */
void plan_delay_init(PlanDelay *q);

/** Queue `plan` for release at `due_step`; drops the oldest when full. */
void plan_delay_push(PlanDelay *q, const PlanOutput *plan, int due_step);

/** Release every plan due by `step` and return the one to apply now. */
const PlanOutput *plan_delay_poll(PlanDelay *q, int step);

#endif /* ADAS_PLAN_DELAY_H */
