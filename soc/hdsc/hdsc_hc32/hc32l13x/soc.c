#include "soc.h"

#include <zephyr/init.h>
#include <zephyr/irq.h>

void soc_early_init_hook(void)
{
	extern int z_clock_hw_cycles_per_sec;
	z_clock_hw_cycles_per_sec = 4000000;
}
