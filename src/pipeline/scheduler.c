/**
 * @file scheduler.c
 * @brief DAG scheduler implementation (Kahn-style topological sort).
 */
#include "pipeline/scheduler.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

int scheduler_init(Scheduler *s, const TaskNode *nodes, int n_nodes) {
  bool done[SCHED_MAX_NODES] = {false};

  memset(s, 0, sizeof(*s));
  if (n_nodes <= 0 || n_nodes > SCHED_MAX_NODES) {
    fprintf(stderr, "scheduler: invalid node count %d\n", n_nodes);
    return -1;
  }
  s->nodes = nodes;
  s->n_nodes = n_nodes;

  /* Validate dependency indices. */
  for (int i = 0; i < n_nodes; ++i) {
    if (nodes[i].fn == NULL || nodes[i].n_deps < 0 ||
        nodes[i].n_deps > SCHED_MAX_DEPS) {
      fprintf(stderr, "scheduler: node %d (%s) is malformed\n", i,
              nodes[i].name);
      return -1;
    }
    for (int d = 0; d < nodes[i].n_deps; ++d) {
      const int dep = nodes[i].deps[d];
      if (dep < 0 || dep >= n_nodes || dep == i) {
        fprintf(stderr, "scheduler: node %s has invalid dependency %d\n",
                nodes[i].name, dep);
        return -1;
      }
    }
  }

  /* Repeatedly pick the first node whose dependencies are all done. */
  for (int count = 0; count < n_nodes; ++count) {
    int pick = -1;
    for (int i = 0; i < n_nodes && pick < 0; ++i) {
      bool ready = !done[i];
      for (int d = 0; d < nodes[i].n_deps && ready; ++d) {
        ready = done[nodes[i].deps[d]];
      }
      if (ready) {
        pick = i;
      }
    }
    if (pick < 0) {
      fprintf(stderr, "scheduler: dependency cycle detected\n");
      return -1;
    }

    int lvl = 0;
    for (int d = 0; d < nodes[pick].n_deps; ++d) {
      const int dep_lvl = s->level[nodes[pick].deps[d]] + 1;
      lvl = (dep_lvl > lvl) ? dep_lvl : lvl;
    }
    s->level[pick] = lvl;
    s->n_levels = (lvl + 1 > s->n_levels) ? lvl + 1 : s->n_levels;
    s->order[count] = pick;
    done[pick] = true;
  }
  return 0;
}

void scheduler_run(const Scheduler *s, void *ctx, SchedulerStats *stats) {
  memset(stats, 0, sizeof(*stats));

  for (int k = 0; k < s->n_nodes; ++k) {
    const int i = s->order[k];
    const TaskNode *node = &s->nodes[i];

    float start = 0.0f;
    for (int d = 0; d < node->n_deps; ++d) {
      const float dep_finish = stats->finish_ms[node->deps[d]];
      start = (dep_finish > start) ? dep_finish : start;
    }

    node->fn(ctx);

    stats->start_ms[i] = start;
    stats->finish_ms[i] = start + node->latency_ms;
    stats->serial_ms += node->latency_ms;
    if (stats->finish_ms[i] > stats->makespan_ms) {
      stats->makespan_ms = stats->finish_ms[i];
    }
  }
}

void scheduler_print(const Scheduler *s, FILE *fp) {
  fprintf(fp,
          "Pipeline DAG (%d nodes, %d stages; nodes in one stage may run in "
          "parallel)\n",
          s->n_nodes, s->n_levels);
  for (int lvl = 0; lvl < s->n_levels; ++lvl) {
    fprintf(fp, "  stage %d:", lvl);
    for (int k = 0; k < s->n_nodes; ++k) {
      const int i = s->order[k];
      if (s->level[i] != lvl) {
        continue;
      }
      const TaskNode *n = &s->nodes[i];
      fprintf(fp, "  %s", n->name);
      if (n->n_deps > 0) {
        fprintf(fp, " <- (");
        for (int d = 0; d < n->n_deps; ++d) {
          fprintf(fp, "%s%s", d ? ", " : "", s->nodes[n->deps[d]].name);
        }
        fprintf(fp, ")");
      }
      fprintf(fp, ";");
    }
    fprintf(fp, "\n");
  }
}
