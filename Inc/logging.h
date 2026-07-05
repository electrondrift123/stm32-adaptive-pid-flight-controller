#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void log_printf(const char *format, ...); // custom printf function for logging

#ifdef __cplusplus
}
#endif

#endif /* LOGGING_H */