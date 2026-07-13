#include "foc.h"

void foc_init(foc_t *handle, pid_controller_t *pid_id, pid_controller_t *pid_iq, pid_controller_t *pid_speed) {
    handle->target_speed    = 0.0f;
    handle->target_position = 0.0f;
    handle->target_id       = 0.0f;
    handle->target_iq       = 0.0f;

    handle->v_d_cmd = 0.0f;
    handle->v_q_cmd = 0.0f;
    handle->v_d_out = 0.0f;
    handle->v_q_out = 0.0f;
    handle->v_d_pi  = 0.0f;
    handle->v_q_pi  = 0.0f;
    handle->v_d_ff  = 0.0f;
    handle->v_q_ff  = 0.0f;

    handle->pid_id       = pid_id;
    handle->pid_iq       = pid_iq;
    handle->pid_speed    = pid_speed;
    handle->pid_position = NULL;
    handle->flux_weak    = NULL;

    handle->duty_cycle.a = 0.0f;
    handle->duty_cycle.b = 0.0f;
    handle->duty_cycle.c = 0.0f;

    handle->angle_offset = 0.0f;
}

void foc_set_id(foc_t *handle, float id) {
    handle->target_id = id;
}

void foc_set_iq(foc_t *handle, float iq) {
    handle->target_iq = iq;
}

void foc_set_speed(foc_t *handle, float speed_rpm) {
    handle->target_speed = speed_rpm;
}

void foc_set_position(foc_t *handle, float position_rad) {
    handle->target_position = position_rad;
}

void foc_set_positionPid(foc_t *handle, pid_controller_t *pid_position) {
    handle->pid_position = pid_position;
}

void foc_set_fluxWeak(foc_t *handle, flux_weak_t *flux_weak) {
    handle->flux_weak = flux_weak;
}
