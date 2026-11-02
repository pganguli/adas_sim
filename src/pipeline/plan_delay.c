/**
 * @file plan_delay.c
 * @brief Plan delay line implementation (fixed-capacity ring buffer).
 */
#include "pipeline/plan_delay.h"

#include <stdbool.h>
#include <string.h>

#include "types/planning_types.h"

void plan_delay_init(PlanDelay *q) { memset(q, 0, sizeof(*q)); }

void plan_delay_push(PlanDelay *q, const PlanOutput *plan, double due_s) {
  if (q->count == PLAN_DELAY_CAPACITY) {
    q->head = (q->head + 1) % PLAN_DELAY_CAPACITY; /* overrun: drop oldest */
    q->count--;
  }
  const int tail = (q->head + q->count) % PLAN_DELAY_CAPACITY;
  q->pending[tail] = *plan;
  q->due_s[tail] = due_s;
  q->count++;
}

const PlanOutput *plan_delay_poll(PlanDelay *q, double t_s) {
  while (q->count > 0 && q->due_s[q->head] <= t_s) {
    q->active = q->pending[q->head];
    q->head = (q->head + 1) % PLAN_DELAY_CAPACITY;
    q->count--;
  }
  return &q->active;
}

bool plan_delay_next_due(const PlanDelay *q, double *due_s) {
  if (q->count == 0) {
    return false;
  }
  *due_s = q->due_s[q->head];
  return true;
}
