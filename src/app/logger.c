/**
 * @file logger.c
 * @brief CSV logger implementation.
 */
#include "app/logger.h"

#include <stdio.h>

#include "pipeline/adas_pipeline.h"
#include "types/control_types.h"
#include "types/world_types.h"

/** printf is variadic, so floats are promoted to double: make that explicit. */
#define D(x) ((double)(x))

int logger_open(Logger *log, const char *path) {
  log->fp = fopen(path, "w");
  if (log->fp == NULL) {
    perror(path);
    return -1;
  }
  fprintf(log->fp, "t_s,mode,"
                   "v_ego_mps,a_ego_mps2,v_lead_mps,"
                   "gap_true_m,gap_est_m,v_rel_true_mps,v_rel_est_mps,"
                   "lead_valid,d_des_m,ttc_s,"
                   "a_des_mps2,throttle,brake,"
                   "e_y_true_m,e_y_est_m,e_psi_true_rad,e_psi_est_rad,"
                   "kappa_true_1pm,kappa_est_1pm,lane_valid,"
                   "steer_cmd_rad,steer_act_rad,n_det2d,n_det3d,lidar_used,"
                   "camera_used,latency_ms\n");
  return 0;
}

void logger_write(Logger *log, const WorldState *w, const GroundTruth *gt,
                  const AdasPipeline *p, const ActuatorCmd *cmd) {
  const PipelineData *d = &p->data;
  if (log->fp == NULL) {
    return;
  }
  fprintf(log->fp,
          "%.2f,%d,"
          "%.3f,%.3f,%.3f,"
          "%.3f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,"
          "%.3f,%.3f,%.3f,"
          "%.4f,%.4f,%.5f,%.5f,%.6f,%.6f,%d,"
          "%.5f,%.5f,%d,%d,%d,%d,%.2f\n",
          w->t_s, (int)d->plan.mode, D(w->ego.v_mps), D(w->ego.a_mps2),
          D(gt->v_lead_mps), D(gt->gap_m), D(d->lead.d_m),
          D(gt->v_lead_mps - w->ego.v_mps), D(d->lead.v_rel_mps),
          (int)d->lead.valid, D(d->acc.d_des_m),
          D(d->acc.ttc_s > 99.0f ? 99.0f : d->acc.ttc_s), D(d->plan.a_des_mps2),
          D(cmd->throttle), D(cmd->brake), D(gt->e_y_m), D(d->lane.e_y_m),
          D(gt->e_psi_rad), D(d->lane.e_psi_rad), D(gt->kappa_1pm),
          D(d->lane.kappa_1pm), (int)d->lane.valid, D(cmd->steer_rad),
          D(w->ego.delta_rad), d->det2d.n, d->det3d.n, (int)d->lead.lidar_used,
          (int)d->lead.camera_used, D(p->stats.makespan_ms));
}

int logger_close(Logger *log) {
  int status = 0;
  if (log->fp != NULL) {
    /* Buffered write errors only surface at flush/close time. */
    if (ferror(log->fp) != 0 || fclose(log->fp) != 0) {
      status = -1;
    }
    log->fp = NULL;
  }
  return status;
}
