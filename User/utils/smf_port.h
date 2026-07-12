#ifndef SMF_PORT_H_
#define SMF_PORT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* Minimal compatibility shims for the portable SMF implementation. */

#ifndef LOG_ERR
#define LOG_ERR(fmt, ...) fprintf(stderr, "[SMF ERROR] " fmt "\r\n", ##__VA_ARGS__)
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif

#endif /* SMF_PORT_H_ */
