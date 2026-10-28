/**
 * @file plan_delay.c
 * @brief Plan delay line implementation (fixed-capacity ring buffer).
 */
#include "pipeline/plan_delay.h"

#include <string.h>

#include "types/planning_types.h"

void plan_delay_init(PlanDelay *q) { memset(q, 0, sizeof(*q)); }

void plan_delay_push(PlanDelay *q, const PlanOutput *plan, int due_step) {
  if (q->count == PLAN_DELAY_CAPACITY) {
    q->head = (q->head + 1) % PLAN_DELAY_CAPACITY; /* overrun: drop oldest */
    q->count--;
  }
  const int tail = (q->head + q->count) % PLAN_DELAY_CAPACITY;
  q->pending[tail] = *plan;
  q->due_step[tail] = due_step;
  q->count++;
}

const PlanOutput *plan_delay_poll(PlanDelay *q, int step) {
  while (q->count > 0 && q->due_step[q->head] <= step) {
    q->active = q->pending[q->head];
    q->head = (q->head + 1) % PLAN_DELAY_CAPACITY;
    q->count--;
  }
  return &q->active;
}
