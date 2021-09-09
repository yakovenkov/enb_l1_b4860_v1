/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __LOG_H_
#define __LOG_H_

#include <smartdsp_os.h>
#include <app_config.h>
#include <log_ipc.h>
#include <log_event.h>
#include <narg.h>

typedef enum
{
	DTRX = 0, DCONFIG, DRACH, DFAPI, DLASTCOMP
} log_comp_id_t;

typedef enum
{
	LOGL_DEBUG = 0, LOGL_INFO, LOGL_WARN, LOGL_ERROR, LOGL_NONE
} log_level_t;

typedef struct
{
	const char *name;
	log_level_t loglevel;
	uint32_t enabled;
} log_info_cat_t;

void log_init();
os_status log_set_level(log_comp_id_t comp, log_level_t level);
void log_ipc(const int8_t *file, const int8_t *func, int32_t line, log_comp_id_t comp,
		log_level_t level, char *format, ...);

uint32_t log_get_timer_value();

void log_ipc_raw(char *format, ...);
void log_ipc_bit_array(char *str, int8_t *bits, int32_t len, int32_t need_cr);

void log_line_string(int nargs, log_comp_id_t comp, log_level_t level, const int8_t *file, const int8_t *func, int32_t line, ...);
void log_line_direct(int nargs, log_comp_id_t comp, log_level_t level, const int8_t *file, const int8_t *func, int32_t line, const char *msg, ...);
void log_hex(int nargs, log_comp_id_t comp, log_level_t level, const int8_t *file, const int8_t *func, int32_t line, void *hexbuf, uint32_t hexlen, char *fmt, ...);
//void log_hex(int nargs, log_comp_id_t comp, log_level_t level, const int8_t *file, const int8_t *func, int32_t line,  ...);

//#define ERROR(component, format, ...) log_line(PP_NARG(__VA_ARGS__), component, LOGL_ERROR, __FILE__, __FUNCTION__, __LINE__, format, __VA_ARGS__)
#ifdef USE_DIRECT_LOG

#define ERROR(component, msg, ...) log_line_direct(PP_NARG(__VA_ARGS__), component, LOGL_ERROR, __FILE__, __FUNCTION__, __LINE__, "[%06u:%04u:%u] " msg, ##__VA_ARGS__)
#define WARN(component, msg, ...) log_line_direct(PP_NARG(__VA_ARGS__), component, LOGL_WARN, __FILE__, __FUNCTION__, __LINE__, "[%06u:%04u:%u] " msg, ##__VA_ARGS__)
#define INFO(component, msg, ...) log_line_direct(PP_NARG(__VA_ARGS__), component, LOGL_INFO, __FILE__, __FUNCTION__, __LINE__, "[%06u:%04u:%u] " msg, ##__VA_ARGS__)
#ifdef DEBUG_BUILD
#define DBG(component, msg, ...) log_line_direct(PP_NARG(__VA_ARGS__), component, LOGL_DEBUG, __FILE__, __FUNCTION__, __LINE__, "[%06u:%04u:%u] " msg, ##__VA_ARGS__)
#else
#define DBG(component, ...)
#endif

#else

#define ERROR(component, ...) log_line_string(PP_NARG(__VA_ARGS__), component, LOGL_ERROR, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define WARN(component, ...) log_line_string(PP_NARG(__VA_ARGS__), component, LOGL_WARN, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define INFO(component, ...) log_line_string(PP_NARG(__VA_ARGS__), component, LOGL_INFO, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#ifdef DEBUG_BUILD
#define DBG(component, ...) log_line_string(PP_NARG(__VA_ARGS__), component, LOGL_DEBUG, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define DBG(component, ...)
#endif

#endif

#define ERROR_HEX(component, hexbuf, hexlen, format, ...) log_hex(PP_NARG(__VA_ARGS__), component, LOGL_ERROR, __FILE__, __FUNCTION__, __LINE__, hexbuf, hexlen, format, ##__VA_ARGS__)
#define WARN_HEX(component, hexbuf, hexlen, format, ...) log_hex(PP_NARG(__VA_ARGS__), component, LOGL_WARN, __FILE__, __FUNCTION__, __LINE__, hexbuf, hexlen, format, ##__VA_ARGS__)
#define INFO_HEX(component, hexbuf, hexlen, format, ...) log_hex(PP_NARG(__VA_ARGS__), component, LOGL_INFO, __FILE__, __FUNCTION__, __LINE__, hexbuf, hexlen, format, ##__VA_ARGS__)
#define DBG_HEX(component, hexbuf, hexlen, format, ...) log_hex(PP_NARG(__VA_ARGS__), component, LOGL_DEBUG, __FILE__, __FUNCTION__, __LINE__, hexbuf, hexlen, format, ##__VA_ARGS__)

#endif /* LOG_H_ */
