#include "smf_test.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "../utils/smf.h"

#define SMF_TEST_TERMINATE_VALUE (77)
#define SMF_TEST_TRACE_CAPACITY (16U)

typedef enum { SMF_TEST_STATE_ROOT = 0, SMF_TEST_STATE_IDLE, SMF_TEST_STATE_ACTIVE, SMF_TEST_STATE_DONE, SMF_TEST_STATE_COUNT } smf_test_state_id_t;

typedef enum { SMF_TEST_EVENT_NONE = 0, SMF_TEST_EVENT_PING, SMF_TEST_EVENT_START, SMF_TEST_EVENT_RESTART, SMF_TEST_EVENT_COMPLETE, SMF_TEST_EVENT_TERMINATE } smf_test_event_t;

typedef enum {
    SMF_TEST_PHASE_INITIAL = 0,
    SMF_TEST_PHASE_PARENT_PROPAGATION,
    SMF_TEST_PHASE_TO_ACTIVE,
    SMF_TEST_PHASE_SELF_TRANSITION,
    SMF_TEST_PHASE_TO_DONE,
    SMF_TEST_PHASE_TERMINATE,
    SMF_TEST_PHASE_SUMMARY,
    SMF_TEST_PHASE_FINISHED
} smf_test_phase_t;

typedef enum {
    SMF_TEST_TRACE_ROOT_ENTRY = 1,
    SMF_TEST_TRACE_ROOT_RUN,
    SMF_TEST_TRACE_ROOT_EXIT,
    SMF_TEST_TRACE_IDLE_ENTRY,
    SMF_TEST_TRACE_IDLE_RUN,
    SMF_TEST_TRACE_IDLE_EXIT,
    SMF_TEST_TRACE_ACTIVE_ENTRY,
    SMF_TEST_TRACE_ACTIVE_RUN,
    SMF_TEST_TRACE_ACTIVE_EXIT,
    SMF_TEST_TRACE_DONE_ENTRY,
    SMF_TEST_TRACE_DONE_RUN,
    SMF_TEST_TRACE_DONE_EXIT
} smf_test_trace_t;

typedef struct {
    struct smf_ctx   ctx;
    smf_test_event_t event;
    smf_test_phase_t phase;
    uint32_t         pass_count;
    uint32_t         fail_count;
    uint32_t         parent_handled_count;
    uint32_t         root_entry_count;
    uint32_t         root_run_count;
    uint32_t         root_exit_count;
    uint32_t         idle_entry_count;
    uint32_t         idle_run_count;
    uint32_t         idle_exit_count;
    uint32_t         active_entry_count;
    uint32_t         active_run_count;
    uint32_t         active_exit_count;
    uint32_t         done_entry_count;
    uint32_t         done_run_count;
    uint32_t         done_exit_count;
    uint8_t          trace[SMF_TEST_TRACE_CAPACITY];
    uint8_t          trace_count;
} smf_test_context_t;

static smf_test_context_t     smf_test;
static const struct smf_state smf_test_states[SMF_TEST_STATE_COUNT];

static const char *smf_test_event_name(smf_test_event_t event) {
    switch (event) {
        case SMF_TEST_EVENT_PING:
            return "PING";
        case SMF_TEST_EVENT_START:
            return "START";
        case SMF_TEST_EVENT_RESTART:
            return "RESTART";
        case SMF_TEST_EVENT_COMPLETE:
            return "COMPLETE";
        case SMF_TEST_EVENT_TERMINATE:
            return "TERMINATE";
        case SMF_TEST_EVENT_NONE:
        default:
            return "NONE";
    }
}

static void smf_test_trace_append(smf_test_context_t *test, smf_test_trace_t value) {
    if (test->trace_count < SMF_TEST_TRACE_CAPACITY) {
        test->trace[test->trace_count++] = (uint8_t)value;
    }
}

static void smf_test_trace_reset(smf_test_context_t *test) {
    test->trace_count = 0U;
}

static void smf_test_log_action(const char *state, const char *action, smf_test_event_t event) {
    printf("[SMF TEST][ACTION] %s.%s event=%s\r\n", state, action, smf_test_event_name(event));
}

static void smf_test_expect(smf_test_context_t *test, const char *name, bool condition) {
    if (condition) {
        test->pass_count++;
        printf("[SMF TEST][PASS] %s\r\n", name);
    } else {
        test->fail_count++;
        printf("[SMF TEST][FAIL] %s\r\n", name);
    }
}

static void smf_test_expect_trace(smf_test_context_t *test, const char *name, const uint8_t *expected, uint8_t expected_count) {
    bool matches = (test->trace_count == expected_count);

    if (matches) {
        for (uint8_t i = 0U; i < expected_count; i++) {
            if (test->trace[i] != expected[i]) {
                matches = false;
                break;
            }
        }
    }

    smf_test_expect(test, name, matches);

    if (!matches) {
        printf("[SMF TEST][TRACE] actual=");
        for (uint8_t i = 0U; i < test->trace_count; i++) {
            printf("%u%s", test->trace[i], (i + 1U < test->trace_count) ? "," : "");
        }
        printf(" expected=");
        for (uint8_t i = 0U; i < expected_count; i++) {
            printf("%u%s", expected[i], (i + 1U < expected_count) ? "," : "");
        }
        printf("\r\n");
    }
}

static void smf_test_root_entry(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->root_entry_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_ROOT_ENTRY);
    smf_test_log_action("root", "entry", test->event);
}

static enum smf_state_result smf_test_root_run(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->root_run_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_ROOT_RUN);
    smf_test_log_action("root", "run", test->event);

    if (test->event == SMF_TEST_EVENT_PING) {
        test->event = SMF_TEST_EVENT_NONE;
        test->parent_handled_count++;
        return SMF_EVENT_HANDLED;
    }

    return SMF_EVENT_PROPAGATE;
}

static void smf_test_root_exit(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->root_exit_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_ROOT_EXIT);
    smf_test_log_action("root", "exit", test->event);
}

static void smf_test_idle_entry(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->idle_entry_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_IDLE_ENTRY);
    smf_test_log_action("idle", "entry", test->event);
}

static enum smf_state_result smf_test_idle_run(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->idle_run_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_IDLE_RUN);
    smf_test_log_action("idle", "run", test->event);

    if (test->event == SMF_TEST_EVENT_START) {
        test->event = SMF_TEST_EVENT_NONE;
        smf_set_state(SMF_CTX(test), &smf_test_states[SMF_TEST_STATE_ACTIVE]);
        return SMF_EVENT_HANDLED;
    }

    return SMF_EVENT_PROPAGATE;
}

static void smf_test_idle_exit(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->idle_exit_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_IDLE_EXIT);
    smf_test_log_action("idle", "exit", test->event);
}

static void smf_test_active_entry(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->active_entry_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_ACTIVE_ENTRY);
    smf_test_log_action("active", "entry", test->event);
}

static enum smf_state_result smf_test_active_run(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->active_run_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_ACTIVE_RUN);
    smf_test_log_action("active", "run", test->event);

    if (test->event == SMF_TEST_EVENT_RESTART) {
        test->event = SMF_TEST_EVENT_NONE;
        smf_set_state(SMF_CTX(test), &smf_test_states[SMF_TEST_STATE_ACTIVE]);
        return SMF_EVENT_HANDLED;
    }

    if (test->event == SMF_TEST_EVENT_COMPLETE) {
        test->event = SMF_TEST_EVENT_NONE;
        smf_set_state(SMF_CTX(test), &smf_test_states[SMF_TEST_STATE_DONE]);
        return SMF_EVENT_HANDLED;
    }

    return SMF_EVENT_PROPAGATE;
}

static void smf_test_active_exit(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->active_exit_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_ACTIVE_EXIT);
    smf_test_log_action("active", "exit", test->event);
}

static void smf_test_done_entry(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->done_entry_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_DONE_ENTRY);
    smf_test_log_action("done", "entry", test->event);
}

static enum smf_state_result smf_test_done_run(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->done_run_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_DONE_RUN);
    smf_test_log_action("done", "run", test->event);

    if (test->event == SMF_TEST_EVENT_TERMINATE) {
        test->event = SMF_TEST_EVENT_NONE;
        smf_set_terminate(SMF_CTX(test), SMF_TEST_TERMINATE_VALUE);
        return SMF_EVENT_HANDLED;
    }

    return SMF_EVENT_PROPAGATE;
}

static void smf_test_done_exit(void *obj) {
    smf_test_context_t *test = (smf_test_context_t *)obj;

    test->done_exit_count++;
    smf_test_trace_append(test, SMF_TEST_TRACE_DONE_EXIT);
    smf_test_log_action("done", "exit", test->event);
}

static const struct smf_state smf_test_states[SMF_TEST_STATE_COUNT] = {
    [SMF_TEST_STATE_ROOT] = SMF_CREATE_STATE(smf_test_root_entry, smf_test_root_run, smf_test_root_exit, NULL),
    [SMF_TEST_STATE_IDLE] = SMF_CREATE_STATE(smf_test_idle_entry, smf_test_idle_run, smf_test_idle_exit, &smf_test_states[SMF_TEST_STATE_ROOT]),
    [SMF_TEST_STATE_ACTIVE] = SMF_CREATE_STATE(smf_test_active_entry, smf_test_active_run, smf_test_active_exit, &smf_test_states[SMF_TEST_STATE_ROOT]),
    [SMF_TEST_STATE_DONE] = SMF_CREATE_STATE(smf_test_done_entry, smf_test_done_run, smf_test_done_exit, &smf_test_states[SMF_TEST_STATE_ROOT]),
};

void smf_test_init(void) {
    memset(&smf_test, 0, sizeof(smf_test));
    smf_test.phase = SMF_TEST_PHASE_INITIAL;

    printf("\r\n[SMF TEST] start, UART2 printf output\r\n");
    smf_set_initial(SMF_CTX(&smf_test), &smf_test_states[SMF_TEST_STATE_IDLE]);
}

void smf_test_poll(void) {
    int32_t run_result;

    switch (smf_test.phase) {
        case SMF_TEST_PHASE_INITIAL: {
            static const uint8_t expected[] = {
                SMF_TEST_TRACE_ROOT_ENTRY,
                SMF_TEST_TRACE_IDLE_ENTRY,
            };

            smf_test_expect(&smf_test, "initial leaf is idle", smf_get_current_leaf_state(SMF_CTX(&smf_test)) == &smf_test_states[SMF_TEST_STATE_IDLE]);
            smf_test_expect(&smf_test, "parent and child entry called once", (smf_test.root_entry_count == 1U) && (smf_test.idle_entry_count == 1U));
            smf_test_expect_trace(&smf_test, "initial entry order", expected, ARRAY_SIZE(expected));
            smf_test_trace_reset(&smf_test);
            smf_test.phase = SMF_TEST_PHASE_PARENT_PROPAGATION;
            break;
        }

        case SMF_TEST_PHASE_PARENT_PROPAGATION: {
            static const uint8_t expected[] = {
                SMF_TEST_TRACE_IDLE_RUN,
                SMF_TEST_TRACE_ROOT_RUN,
            };

            smf_test.event = SMF_TEST_EVENT_PING;
            run_result = smf_run_state(SMF_CTX(&smf_test));
            smf_test_expect(&smf_test, "child event propagates to parent", (run_result == 0) && (smf_test.parent_handled_count == 1U) && (smf_test.root_run_count == 1U));
            smf_test_expect_trace(&smf_test, "child then parent run order", expected, ARRAY_SIZE(expected));
            smf_test_trace_reset(&smf_test);
            smf_test.phase = SMF_TEST_PHASE_TO_ACTIVE;
            break;
        }

        case SMF_TEST_PHASE_TO_ACTIVE: {
            static const uint8_t expected[] = {
                SMF_TEST_TRACE_IDLE_RUN,
                SMF_TEST_TRACE_IDLE_EXIT,
                SMF_TEST_TRACE_ACTIVE_ENTRY,
            };

            smf_test.event = SMF_TEST_EVENT_START;
            run_result = smf_run_state(SMF_CTX(&smf_test));
            smf_test_expect(&smf_test, "idle transitions to active",
                            (run_result == 0) && (smf_test.ctx.current == &smf_test_states[SMF_TEST_STATE_ACTIVE]) && (smf_test.ctx.previous == &smf_test_states[SMF_TEST_STATE_IDLE]));
            smf_test_expect(&smf_test, "shared parent is not re-entered", (smf_test.root_entry_count == 1U) && (smf_test.root_exit_count == 0U) && (smf_test.root_run_count == 1U));
            smf_test_expect_trace(&smf_test, "sibling transition order", expected, ARRAY_SIZE(expected));
            smf_test_trace_reset(&smf_test);
            smf_test.phase = SMF_TEST_PHASE_SELF_TRANSITION;
            break;
        }

        case SMF_TEST_PHASE_SELF_TRANSITION: {
            static const uint8_t expected[] = {
                SMF_TEST_TRACE_ACTIVE_RUN,
                SMF_TEST_TRACE_ACTIVE_EXIT,
                SMF_TEST_TRACE_ACTIVE_ENTRY,
            };

            smf_test.event = SMF_TEST_EVENT_RESTART;
            run_result = smf_run_state(SMF_CTX(&smf_test));
            smf_test_expect(&smf_test, "self transition exits and re-enters", (run_result == 0) && (smf_test.active_exit_count == 1U) && (smf_test.active_entry_count == 2U));
            smf_test_expect_trace(&smf_test, "self transition order", expected, ARRAY_SIZE(expected));
            smf_test_trace_reset(&smf_test);
            smf_test.phase = SMF_TEST_PHASE_TO_DONE;
            break;
        }

        case SMF_TEST_PHASE_TO_DONE: {
            static const uint8_t expected[] = {
                SMF_TEST_TRACE_ACTIVE_RUN,
                SMF_TEST_TRACE_ACTIVE_EXIT,
                SMF_TEST_TRACE_DONE_ENTRY,
            };

            smf_test.event = SMF_TEST_EVENT_COMPLETE;
            run_result = smf_run_state(SMF_CTX(&smf_test));
            smf_test_expect(&smf_test, "active transitions to done",
                            (run_result == 0) && (smf_test.ctx.current == &smf_test_states[SMF_TEST_STATE_DONE]) && (smf_test.ctx.previous == &smf_test_states[SMF_TEST_STATE_ACTIVE]));
            smf_test_expect_trace(&smf_test, "done transition order", expected, ARRAY_SIZE(expected));
            smf_test_trace_reset(&smf_test);
            smf_test.phase = SMF_TEST_PHASE_TERMINATE;
            break;
        }

        case SMF_TEST_PHASE_TERMINATE: {
            static const uint8_t expected[] = {
                SMF_TEST_TRACE_DONE_RUN,
            };
            uint32_t done_run_count;
            int32_t  second_run_result;

            smf_test.event = SMF_TEST_EVENT_TERMINATE;
            run_result = smf_run_state(SMF_CTX(&smf_test));
            done_run_count = smf_test.done_run_count;
            second_run_result = smf_run_state(SMF_CTX(&smf_test));

            smf_test_expect(&smf_test, "terminate value is persistent", (run_result == SMF_TEST_TERMINATE_VALUE) && (second_run_result == SMF_TEST_TERMINATE_VALUE));
            smf_test_expect(&smf_test, "terminated machine does not run callbacks", smf_test.done_run_count == done_run_count);
            smf_test_expect_trace(&smf_test, "terminate callback order", expected, ARRAY_SIZE(expected));
            smf_test.phase = SMF_TEST_PHASE_SUMMARY;
            break;
        }

        case SMF_TEST_PHASE_SUMMARY:
            printf("[SMF TEST] complete: pass=%lu fail=%lu result=%s\r\n", (unsigned long)smf_test.pass_count, (unsigned long)smf_test.fail_count, (smf_test.fail_count == 0U) ? "PASS" : "FAIL");
            smf_test.phase = SMF_TEST_PHASE_FINISHED;
            break;

        case SMF_TEST_PHASE_FINISHED:
        default:
            break;
    }
}

uint8_t smf_test_is_done(void) {
    return (smf_test.phase == SMF_TEST_PHASE_FINISHED) ? 1U : 0U;
}

uint32_t smf_test_get_pass_count(void) {
    return smf_test.pass_count;
}

uint32_t smf_test_get_fail_count(void) {
    return smf_test.fail_count;
}