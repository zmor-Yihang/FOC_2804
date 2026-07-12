#ifndef USER_TEST_SMF_TEST_H_
#define USER_TEST_SMF_TEST_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Call after usart_init(); all test progress is reported through printf/UART2. */
void     smf_test_init(void);
void     smf_test_poll(void);
uint8_t  smf_test_is_done(void);
uint32_t smf_test_get_pass_count(void);
uint32_t smf_test_get_fail_count(void);

#ifdef __cplusplus
}
#endif

#endif /* USER_TEST_SMF_TEST_H_ */