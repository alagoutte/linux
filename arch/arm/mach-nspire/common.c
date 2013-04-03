/*
 *	linux/arch/arm/mach-nspire/common.c
 *
 *	Copyright (C) 2012 Daniel Tang <tangrs@tangrs.id.au>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/clkdev.h>
#include <linux/platform_device.h>
#include <linux/usb/ehci_pdriver.h>
#include <linux/usb/chipidea.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/dma-mapping.h>

#include <mach/nspire_mmio.h>
#include <mach/nspire_clock.h>
#include <mach/irqs.h>
#include <mach/clkdev.h>
#include <mach/keypad.h>
#include <mach/sram.h>

#include <asm/mach/time.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>

#include "common.h"
#include "clock.h"

/* Clocks */

struct nspire_clk_speeds (*nspire_io_to_clocks)(unsigned long);
unsigned long (*nspire_clocks_to_io)(struct nspire_clk_speeds *);

/* AHB clock */
static void ahb_get_rate(struct clk *clk)
{
	struct nspire_clk_speeds speeds = nspire_get_clocks();
	clk->rate = CLK_GET_AHB(&speeds);
}

static struct clk ahb_clk = {
	.get_rate = ahb_get_rate,
};

/* APB clock */

static void apb_get_rate(struct clk *clk)
{
	clk->rate = clk_get_rate(&ahb_clk) / 2;
}

static struct clk apb_clk = {
	.get_rate = apb_get_rate
};

/* Misc */

static struct clk systimer_clk = {
	.rate	= 32768,
};

static struct clk uart_clk = {
	.rate	= 12000000,
};

#ifdef CONFIG_MACH_NSPIRECX
static struct clk i2c_clk = {
	/* Doesn't matter, we set it manually */
	.rate	= 250000,
};
#endif

static struct clk_lookup nspire_clk_lookup[] = {
	{
		.dev_id = "uart",
		.clk = &uart_clk
	},
	{
		.dev_id = "fb",
		.clk = &ahb_clk
	},
	{
		.con_id = "ahb",
		.clk = &ahb_clk
	},
	{
		.dev_id = "watchdog",
		.clk = &apb_clk
	},
	{
		.dev_id = "nspire-keypad.0",
		.clk = &apb_clk
	},
#ifdef CONFIG_MACH_NSPIRECX
	{
		.dev_id = "sp804",
		.con_id = "timer2",
		.clk = &systimer_clk
	},
	{
		.dev_id = "i2c_designware.0",
		.clk = &i2c_clk
	},
#endif
#if defined(CONFIG_MACH_NSPIRECLP) || defined(CONFIG_MACH_NSPIRETP)
	{
		.dev_id = NULL,
		.con_id = "timer2",
		.clk = &systimer_clk
	},
#endif
};

/* Keypad */
static struct resource nspire_keypad_resources[] = {
	{
		.start	= NSPIRE_APB_PHYS(NSPIRE_APB_KEYPAD),
		.end	= NSPIRE_APB_PHYS(NSPIRE_APB_KEYPAD + SZ_4K - 1),
		.flags	= IORESOURCE_MEM,
	},
	RESOURCE_ENTRY_IRQ(KEYPAD)
};

struct nspire_keypad_data nspire_keypad_data = {
	.scan_interval	= 1000,
	.row_delay	= 200
};

struct platform_device nspire_keypad_device = {
	.name		= "nspire-keypad",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(nspire_keypad_resources),
	.resource	= nspire_keypad_resources,
	.dev = {
		.platform_data = &nspire_keypad_data
	}
};


/* GPIO */
static struct resource nspire_gpio_resources[] = {
	{
		.start	= NSPIRE_APB_PHYS(NSPIRE_APB_GPIO),
		.end	= NSPIRE_APB_PHYS(NSPIRE_APB_GPIO + SZ_4K - 1),
		.flags	= IORESOURCE_MEM,
	},
	RESOURCE_ENTRY_IRQ(GPIO)
};

static struct platform_device nspire_gpio_device = {
	.name		= "gpio-nspire",
	.resource	= nspire_gpio_resources,
	.num_resources	= ARRAY_SIZE(nspire_gpio_resources),
};

/* ADC */
static struct resource nspire_adc_resources[] = {
	RESOURCE_ENTRY_MEM(ADC),
	RESOURCE_ENTRY_IRQ(ADC)
};

static struct platform_device nspire_adc_device = {
	.name		= "nspire-adc",
	.resource	= nspire_adc_resources,
	.num_resources	= ARRAY_SIZE(nspire_adc_resources)
};

/* Framebuffer */
int nspire_clcd_setup(struct clcd_fb *fb, unsigned panel_size,
	struct clcd_panel *panel)
{
	dma_addr_t dma;

	fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev,
		panel_size, &dma, GFP_KERNEL);
	if (!fb->fb.screen_base) {
		pr_err("CLCD: unable to map framebuffer\n");
		return -ENOMEM;
	}

	fb->fb.fix.smem_start = dma;
	fb->fb.fix.smem_len = panel_size;
	fb->panel = panel;

	return 0;
}

int nspire_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(&fb->dev->dev, vma,
		fb->fb.screen_base, fb->fb.fix.smem_start,
		fb->fb.fix.smem_len);
}

void nspire_clcd_remove(struct clcd_fb *fb)
{
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
		fb->fb.screen_base, fb->fb.fix.smem_start);
}

/* Watchdog */

AMBA_APB_DEVICE(watchdog, "watchdog", 0, NSPIRE_APB_PHYS(NSPIRE_APB_WATCHDOG),
	{ NSPIRE_IRQ_WATCHDOG }, NULL);

/* Generic OTG */

u64 nspire_usb_dma_mask = ~(u32)0;

static struct resource otg_resources[] = {
	RESOURCE_ENTRY_MEM(OTG),
	RESOURCE_ENTRY_IRQ(OTG)
};

static struct ci13xxx_platform_data otg_pdata = {
	.name = "nspire_usb",
	.capoffset = 0x100,
	.flags = CI13XXX_REGS_SHARED,
};


struct platform_device nspire_otg_device = {
	.name		= "ci_hdrc",
	.id		= 0,
	.dev = {
		.platform_data = &otg_pdata,
		.coherent_dma_mask = ~0,
		.dma_mask = &nspire_usb_dma_mask
	},
	.resource = otg_resources,
	.num_resources = ARRAY_SIZE(otg_resources)
};

struct platform_device nspire_usb_nop_xceiver = {
	.name		= "nop_usb_xceiv",
};

/* RTC */
static struct resource nspire_rtc_resources[] = {
	{
		.start	= NSPIRE_APB_PHYS(NSPIRE_APB_RTC),
		.end	= NSPIRE_APB_PHYS(NSPIRE_APB_RTC + SZ_4K - 1),
		.flags	= IORESOURCE_MEM,
	},
	RESOURCE_ENTRY_IRQ(RTC)
};

static struct platform_device nspire_rtc_device = {
	.name		= "nspire-rtc",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(nspire_rtc_resources),
	.resource	= nspire_rtc_resources,
};

/* Memory mapped IO */
struct map_desc nspire_io_regs[] __initdata = {
	IOTABLE_ENTRY(ADC),
	IOTABLE_ENTRY(APB),
	IOTABLE_ENTRY(INTERRUPT),
};

void __init nspire_map_io(void)
{
	iotable_init(nspire_io_regs, ARRAY_SIZE(nspire_io_regs));
}

/* Clocks */
void __init nspire_init_early(void)
{
	clkdev_add_table(nspire_clk_lookup, ARRAY_SIZE(nspire_clk_lookup));

	/* Renable bus access to everything in case the OS disabled them */
	writel(0, NSPIRE_APB_VIRTIO(NSPIRE_APB_POWER + 0x18));
	writel(0, NSPIRE_APB_VIRTIO(NSPIRE_APB_POWER + 0x20));

	/*
	 * Ack some non-maskable clock speed change interrupts before cpufreq
	 * driver is brought up to avoid a race condition between an interrupt
	 * happening and driver init.
	 */

	writel(3, NSPIRE_APB_VIRTIO(NSPIRE_APB_POWER + 0x14));
}

/* Common init */
void __init nspire_init(void)
{
	sram_init(NSPIRE_SRAM_PHYS_BASE, NSPIRE_SRAM_SIZE);
	amba_device_register(&watchdog_device, &iomem_resource);

	platform_device_register(&nspire_gpio_device);
	platform_device_register(&nspire_rtc_device);
	platform_device_register(&nspire_adc_device);

}

void __init nspire_init_late(void)
{
}

/* Restart */
void nspire_restart(char mode, const char *cmd)
{
	writel(2, NSPIRE_APB_VIRTIO(NSPIRE_APB_MISC + 0x8));
}
