/**
 * @file test_plan_delay.c
 * @brief Tests for the plan delay line.
 */
#include "pipeline/plan_delay.h"
#include "test_util.h"
#include "types/planning_types.h"

/** A plan tagged with `a_des` so released plans can be told apart. */
static PlanOutput tagged_plan(float a_des) {
  PlanOutput p = {0};
  p.mode = LON_MODE_FOLLOW;
  p.a_des_mps2 = a_des;
  return p;
}

void test_plan_delay(void) {
  PlanDelay q;
  plan_delay_init(&q);

  /* Before the first plan is due the controllers see the all-zero plan. */
  const PlanOutput p1 = tagged_plan(1.0f);
  plan_delay_push(&q, &p1, 4);
  CHECK(plan_delay_poll(&q, 0)->mode == LON_MODE_OFF);
  CHECK(plan_delay_poll(&q, 3)->a_des_mps2 == 0.0f);
  CHECK(plan_delay_poll(&q, 4)->a_des_mps2 == 1.0f);

  /* Zero-order hold until the next plan is due. */
  const PlanOutput p2 = tagged_plan(2.0f);
  plan_delay_push(&q, &p2, 9);
  CHECK(plan_delay_poll(&q, 8)->a_des_mps2 == 1.0f);
  CHECK(plan_delay_poll(&q, 9)->a_des_mps2 == 2.0f);

  /* Several plans due at once: the newest wins. */
  const PlanOutput p3 = tagged_plan(3.0f);
  const PlanOutput p4 = tagged_plan(4.0f);
  plan_delay_push(&q, &p3, 10);
  plan_delay_push(&q, &p4, 11);
  CHECK(plan_delay_poll(&q, 20)->a_des_mps2 == 4.0f);
  CHECK(q.count == 0);

  /* Zero delay: a plan pushed and polled in the same step is applied. */
  const PlanOutput p5 = tagged_plan(5.0f);
  plan_delay_push(&q, &p5, 21);
  CHECK(plan_delay_poll(&q, 21)->a_des_mps2 == 5.0f);

  /* Overrun: a full queue drops its oldest plan. */
  for (int i = 0; i <= PLAN_DELAY_CAPACITY; ++i) {
    const PlanOutput p = tagged_plan((float)(100 + i));
    plan_delay_push(&q, &p, 100 + i);
  }
  CHECK(q.count == PLAN_DELAY_CAPACITY);
  CHECK(plan_delay_poll(&q, 101)->a_des_mps2 == 101.0f); /* 100 was dropped */
}
