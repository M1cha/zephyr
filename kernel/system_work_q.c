/*
 * Copyright (c) 2016 Wind River Systems, Inc.
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * System workqueue.
 */

#include <zephyr/kernel.h>
#include <kernel_internal.h>

static const struct k_work_queue_config cfg = {
	.name = "sysworkq",
	.no_yield = IS_ENABLED(CONFIG_SYSTEM_WORKQUEUE_NO_YIELD),
	.essential = true,
	.work_timeout_ms = CONFIG_SYSTEM_WORKQUEUE_WORK_TIMEOUT_MS,
};
K_WORK_QUEUE_DEFINE(k_sys_work_q);

#ifndef CONFIG_SYSTEM_WORKQUEUE_USE_EXTERNAL_THREAD

static K_KERNEL_STACK_DEFINE(sys_work_q_stack,
			     CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE);

static void sys_work_q_init(void)
{
	k_work_queue_start(&k_sys_work_q,
			    sys_work_q_stack,
			    K_KERNEL_STACK_SIZEOF(sys_work_q_stack),
			    CONFIG_SYSTEM_WORKQUEUE_PRIORITY, &cfg);
}

/* Registered into the kernel post-init section. The entry lives in this TU,
 * so it (and this init) is linked only when something references the system
 * work queue (e.g. k_sys_work_q), preserving pay-per-use linkage.
 */
K_KERNEL_INIT_POST(sys_work_q_init);

#else

void k_sys_work_q_run_on_current_thread(void) {
	k_thread_priority_set(_current, CONFIG_SYSTEM_WORKQUEUE_PRIORITY);
	k_work_queue_run(&k_sys_work_q, &cfg);
}

#endif /* CONFIG_SYSTEM_WORKQUEUE_CREATE_THREAD */
