/**
 * @file scheduler.h
 * @brief Generic, deterministic DAG (task-graph) scheduler.
 *
 * Nodes declare their dependencies; scheduler_init() topologically sorts them
 * and assigns each a "level" (nodes on the same level have no dependency on
 * each other and could run in parallel on separate cores/accelerators).
 *
 * scheduler_run() executes nodes one by one in topological order (so results
 * are reproducible) and simulates parallel timing: every node starts when its
 * slowest dependency finishes and takes its nominal latency_ms. This yields
 * the end-to-end latency of the parallel pipeline (critical path) next to the
 * time a purely serial execution would take.
 *
 * The scheduler knows nothing about ADAS; it only calls fn(ctx).
 */
#ifndef ADAS_SCHEDULER_H
#define ADAS_SCHEDULER_H

#include <stdio.h>

/** Maximum number of nodes in a graph. */
#define SCHED_MAX_NODES 32
/** Maximum number of dependencies of one node. */
#define SCHED_MAX_DEPS 4

/** Work function of a node; receives the user context given to scheduler_run. */
typedef void (*NodeFn)(void *ctx);

/** One node of the task graph. */
typedef struct {
  const char *name;         /**< display name                */
  NodeFn fn;                /**< work function               */
  int n_deps;               /**< number of valid deps        */
  int deps[SCHED_MAX_DEPS]; /**< indices into the node table */
  float latency_ms;         /**< nominal execution time      */
} TaskNode;

/** A validated, topologically sorted task graph. */
typedef struct {
  const TaskNode *nodes; /**< node table, not owned */
  int n_nodes;
  int order[SCHED_MAX_NODES]; /**< topological execution order */
  int level[SCHED_MAX_NODES]; /**< parallel stage of each node */
  int n_levels;
} Scheduler;

/** Simulated timing of one scheduler_run(). */
typedef struct {
  float start_ms[SCHED_MAX_NODES];  /**< simulated start of each node  */
  float finish_ms[SCHED_MAX_NODES]; /**< simulated finish of each node */
  float makespan_ms; /**< critical-path latency (parallel execution) */
  float serial_ms;   /**< sum of all node latencies                  */
} SchedulerStats;

/** Validates the graph and sorts it. Returns 0 on success, -1 on error/cycle. */
int scheduler_init(Scheduler *s, const TaskNode *nodes, int n_nodes);

/** Execute every node once in topological order and fill `stats`. */
void scheduler_run(const Scheduler *s, void *ctx, SchedulerStats *stats);

/** Print the stage layout (nodes and their dependencies) to `fp`. */
void scheduler_print(const Scheduler *s, FILE *fp);

#endif /* ADAS_SCHEDULER_H */
