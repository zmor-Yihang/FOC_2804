/*
 * Copyright 2021 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief State Machine Framework header file (minimal, hierarchical-only)
 */

#ifndef USER_UTILS_SMF_H_
#define USER_UTILS_SMF_H_

#include "smf_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Return value of a state's run action.
 *
 * SMF_EVENT_HANDLED    - the event was consumed, stop propagating to parents.
 * SMF_EVENT_PROPAGATE  - let the parent state's run action handle it.
 */
enum smf_state_result {
    SMF_EVENT_HANDLED,
    SMF_EVENT_PROPAGATE,
};

/**
 * @brief Function pointer implementing the entry/exit actions of a state.
 *
 * @param obj pointer to user defined object
 */
typedef void (*state_method)(void *obj);

/**
 * @brief Function pointer implementing the run action of a state.
 *
 * @param obj pointer to user defined object
 * @return Whether the event was handled or should propagate to the parent.
 */
typedef enum smf_state_result (*state_execution)(void *obj);

/** General state that can be used in multiple state machines. */
struct smf_state {
    /** Optional method run when this state is entered. */
    const state_method entry;

    /** Optional method run repeatedly during the state machine loop. */
    const state_execution run;

    /** Optional method run when this state is exited. */
    const state_method exit;

    /**
     * Optional parent state providing common entry/run/exit behaviour.
     * entry: parent runs BEFORE child.
     * run:   parent runs AFTER child.
     * exit:  parent runs AFTER child.
     *
     * Note: when transitioning between two sibling child states that share
     * a parent, that parent's exit and entry actions do not execute.
     */
    const struct smf_state *parent;
};

/** Defines the current context of the state machine. */
struct smf_ctx {
    /** Current (leaf) state the state machine is executing. */
    const struct smf_state *current;
    /** Previous state the state machine executed. */
    const struct smf_state *previous;
    /** Currently executing state (may be a parent during propagation). */
    const struct smf_state *executing;
    /**
     * Set by smf_set_terminate(); a non-zero value returned by
     * smf_run_state() terminates the state machine.
     */
    int32_t terminate_val;
    /**
     * Cast internally to "struct internal_ctx" to track transition state.
     */
    uint32_t internal;
};

/**
 * @brief Create a state descriptor.
 *
 * @param _entry  entry action (or NULL)
 * @param _run    run action (or NULL)
 * @param _exit   exit action (or NULL)
 * @param _parent parent state (or NULL for a root state)
 */
#define SMF_CREATE_STATE(_entry, _run, _exit, _parent) {.entry = (_entry), .run = (_run), .exit = (_exit), .parent = (_parent)}

/**
 * @brief Cast a user object to its embedded state-machine context.
 *
 * The struct smf_ctx member must be the first member of the user object.
 */
#define SMF_CTX(object) ((struct smf_ctx *)(void *)(object))

/**
 * @brief Initializes the state machine and sets its initial state.
 *
 * @param ctx        State machine context
 * @param init_state Initial state the state machine starts in.
 */
void smf_set_initial(struct smf_ctx *ctx, const struct smf_state *init_state);

/**
 * @brief Changes the state machine's state. Handles exiting the previous
 *        state and entering the target state. Entry/exit actions of the
 *        Least Common Ancestor are not run.
 *
 * @param ctx       State machine context
 * @param new_state State to transition to.
 */
void smf_set_state(struct smf_ctx *ctx, const struct smf_state *new_state);

/**
 * @brief Terminate the state machine.
 *
 * @param ctx  State machine context
 * @param val  Non-zero termination value returned by smf_run_state().
 */
void smf_set_terminate(struct smf_ctx *ctx, int32_t val);

/**
 * @brief Runs one iteration of the state machine (including parent states).
 *
 * @param ctx State machine context
 * @return    A non-zero value (from smf_set_terminate()) terminates the
 *            state machine.
 */
int32_t smf_run_state(struct smf_ctx *ctx);

/**
 * @brief Get the current leaf state.
 *
 * @param ctx State machine context
 * @return    The current leaf state.
 */
static inline const struct smf_state *smf_get_current_leaf_state(const struct smf_ctx *const ctx) {
    return ctx->current;
}

#ifdef __cplusplus
}
#endif

#endif /* USER_UTILS_SMF_H_ */
