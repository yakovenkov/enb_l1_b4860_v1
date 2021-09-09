/* 
 * Copyright (c) 2013-2021 Valentin Yakovenkov
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <stdint.h>
#include <smartdsp_os.h>
#include "os_runtime.h"
#include "smartdsp_os_device.h"

#include <log_event.h>

#pragma opt_level = "O3"

#define NUM_OF_LOG_EVENTS (1024*1024)

//log_event_t log_events[NUM_OF_LOG_EVENTS] __attribute__((section(".log_event_ddr1")));
log_event_t log_events[NUM_OF_LOG_EVENTS] __attribute__((section(".shared_data_ddr0_cacheable_bss")));
#pragma align log_events 0x1000000 //ARCH_CACHE_LINE_SIZE

static uint32_t log_events_ptr = 0;
static uint64_t log_events_counter = 0;

extern uint32_t g_tick_parameter;

extern os_timer_handle g_system_timer;
extern volatile uint64_t g_os_ticks;

void log_event_reset()
{
	osHwiSwiftDisable();
	log_events_ptr = 0;
	log_events_counter = 0;
	memset(log_events, 0, NUM_OF_LOG_EVENTS * sizeof(log_event_t));
	osHwiSwiftEnable();
}

void log_event(uint32_t id, uint32_t sub)
{
	uint64_t tick = log_get_timer_value();
	
	osHwiSwiftDisable();

	log_events[log_events_ptr].id = id;
	log_events[log_events_ptr].param = sub;
	log_events[log_events_ptr].tick = tick;
	log_events_counter++;

	log_events_ptr = (log_events_ptr + 1) & (NUM_OF_LOG_EVENTS - 1);
	osHwiSwiftEnable();
	


}
