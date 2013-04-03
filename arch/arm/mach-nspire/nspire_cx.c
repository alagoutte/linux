/*
 *	linux/arch/arm/mach-nspire/nspire_cx.c
 *
 *	Copyright (C) 2012 Daniel Tang <tangrs@tangrs.id.au>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/clkdev.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/input.h>
#include <linux/usb/ehci_pdriver.h>
#include <linux/mtd/nand.h>
#include <linux/irqchip/arm-vic.h>

#include <mach/nspire_mmio.h>
#include <mach/nspire_clock.h>
#include <mach/irqs.h>
#include <mach/clkdev.h>
#include <mach/sram.h>
#include <mach/keypad.h>

#include <asm/mach/time.h>
#include <asm/hardware/timer-sp.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "common.h"
#include "touchpad.h"

/* Clock */

union reg_clk_speed {
	unsigned long raw;
	struct {
		unsigned long __padding0:1;
		unsigned long base_cpu_ratio:7;
		unsigned long is_base_48mhz:1;
		unsigned long __padding1:3;
		unsigned long cpu_ahb_ratio:3;
		unsigned long base_val:6;
		unsigned long unknown:2;
	} val;
};

static struct nspire_clk_speeds cx_io_to_clocks(unsigned long val)
{
	struct nspire_clk_speeds clks;
	union reg_clk_speed reg;

	reg.raw = val;
	reg.val.base_cpu_ratio *= reg.val.unknown;
	reg.val.cpu_ahb_ratio++;

	BUG_ON(reg.val.base_cpu_ratio == 0);

	clks.base = reg.val.is_base_48mhz ? 48 : 6*reg.val.base_val;
	clks.base *= 1000000; /* Convert to Hz */

	clks.div.base_cpu = reg.val.base_cpu_ratio;
	clks.div.cpu_ahb = reg.val.cpu_ahb_ratio;

	return clks;
}

static unsigned long cx_clocks_to_io(struct nspire_clk_speeds *clks)
{
	union reg_clk_speed reg;

	BUG_ON(clks->div.base_cpu < 1);
	BUG_ON(clks->div.cpu_ahb < 1);

	reg.raw = 0;
	reg.val.unknown = (clks->div.base_cpu & 0x1) ? 0b01 : 0b10;
	reg.val.base_cpu_ratio = clks->div.base_cpu / reg.val.unknown;
	reg.val.cpu_ahb_ratio = clks->div.cpu_ahb - 1;
	reg.val.is_base_48mhz = (clks->base <= 48000000);
	reg.val.base_val = (clks->base / 6000000);

	return reg.raw;
}

/* IRQ */
static void __init cx_init_irq(void)
{
	vic_init(IOMEM(NSPIRE_INTERRUPT_VIRT_BASE), 0, NSPIRE_IRQ_MASK, 0);
}

/* UART */

static AMBA_APB_DEVICE(uart, "uart", 0, NSPIRE_APB_PHYS(NSPIRE_APB_UART),
	{ NSPIRE_IRQ_UART }, NULL);

/* TIMER */

void __init cx_timer_init(void)
{
	sp804_clockevents_init(NSPIRE_APB_VIRTIO(NSPIRE_APB_TIMER2),
		NSPIRE_IRQ_TIMER2, "timer2");
}

/* FRAMEBUFFER */
static struct clcd_panel cx_lcd_panel = {
	.mode		= {
		.name		= "color lcd",
		.refresh	= 60,
		.xres		= 320,
		.yres		= 240,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
		.pixclock	= 1,
		.hsync_len	= 6,
		.vsync_len	= 1,
		.right_margin	= 50,
		.left_margin	= 38,
		.lower_margin	= 3,
		.upper_margin	= 17,
	},
	.width		= 65, /* ~6.50 cm */
	.height		= 49, /* ~4.87 cm */
	.tim2		= TIM2_IPC,
	.cntl		= (CNTL_BGR | CNTL_LCDTFT | CNTL_LCDVCOMP(1) |
				CNTL_LCDBPP16_565),
	.bpp		= 16,
};
#define PANEL_SIZE (38 * SZ_4K)

static int cx_clcd_setup(struct clcd_fb *fb)
{
	return nspire_clcd_setup(fb, PANEL_SIZE, &cx_lcd_panel);
}

static struct clcd_board cx_clcd_data = {
	.name		= "lcd controller",
	.check		= clcdfb_check,
	.decode		= clcdfb_decode,
	.setup		= cx_clcd_setup,
	.mmap		= nspire_clcd_mmap,
	.remove		= nspire_clcd_remove,
};

static AMBA_AHB_DEVICE(fb, "fb", 0, NSPIRE_LCD_PHYS_BASE,
	{ NSPIRE_IRQ_LCD }, &cx_clcd_data);

/* USB HOST */

static struct usb_ehci_pdata cxusbhost_pdata = {
	.has_tt = 1,
	.caps_offset = 0x100
};

static struct resource cxusbhost_resources_pdata[] = {
	RESOURCE_ENTRY_MEM(OTG),
	RESOURCE_ENTRY_IRQ(OTG)
};

static struct platform_device usbhost_device = {
	.name		= "ehci-platform",
	.id		= 0,
	.dev = {
		.platform_data = &cxusbhost_pdata,
		.coherent_dma_mask = ~0,
		.dma_mask = &nspire_usb_dma_mask
	},
	.resource = cxusbhost_resources_pdata,
	.num_resources = ARRAY_SIZE(cxusbhost_resources_pdata)
};


static __init int cx_usb_init(void)
{
	int err = 0;
	unsigned val;
	void __iomem *hostusb_addr =
		ioremap(NSPIRE_OTG_PHYS_BASE, NSPIRE_OTG_SIZE);

	if (!hostusb_addr) {
		pr_warn("Could not allocate enough memory to initialize NSPIRE host USB\n");
		err = -ENOMEM;
		goto out;
	}

	/* Disable OTG interrupts */
	pr_info("Disable OTG interrupts\n");
	val	 = readl(hostusb_addr + 0x1a4);
	val &= ~(0x7f<<24);
	writel(val, hostusb_addr + 0x1a4);

	iounmap(hostusb_addr);

	pr_info("Adding USB controller as platform device\n");
	err = platform_device_register(&usbhost_device);
out:

	return err;
}

static __init int cx_usb_workaround(void)
{
	int err = 0;
	unsigned val;
	void __iomem *hostusb_addr =
		ioremap(NSPIRE_OTG_PHYS_BASE, NSPIRE_OTG_SIZE);

	if (!hostusb_addr) {
		pr_warn("Could not do USB workaround\n");
		err = -ENOMEM;
		goto out;
	}

	pr_info("Temporary USB hack to force USB to connect as fullspeed\n");
	val	 = readl(hostusb_addr + 0x184);
	val |= (1<<24);
	writel(val, hostusb_addr + 0x184);

	iounmap(hostusb_addr);
out:

	return err;
}

/* Backlight driver */

#define CX_BACKLIGHT_UPPER	0x1d0
#define CX_BACKLIGHT_LOWER	0x100 /* Should be (around about) off */

static void cx_set_backlight(int val)
{
	val += CX_BACKLIGHT_LOWER;

	if (val <= CX_BACKLIGHT_UPPER)
		writel(val, NSPIRE_APB_VIRTIO(NSPIRE_APB_CONTRAST + 0x20));
}

static struct generic_bl_info cx_bl = {
	.name		= "nspire_backlight",
	.max_intensity	= CX_BACKLIGHT_UPPER - CX_BACKLIGHT_LOWER,
	.default_intensity = (CX_BACKLIGHT_UPPER - CX_BACKLIGHT_LOWER) / 2,
	.set_bl_intensity = cx_set_backlight
};

static struct platform_device bl_device = {
	.name		= "generic-bl",
	.id		= 0,
	.dev = {
		.platform_data = &cx_bl,
	}
};

/* Init */

bool cx_use_otg;
static int __init set_cx_otg(char *dummy __attribute__((unused)))
{
	cx_use_otg = 1;
	return 0;
}
early_param("cx_use_otg", set_cx_otg);

static void __init cx_early_init(void)
{
	nspire_io_to_clocks = cx_io_to_clocks;
	nspire_clocks_to_io = cx_clocks_to_io;

	nspire_init_early();
}

static void __init cx_init(void)
{
	nspire_init();
	amba_device_register(&fb_device, &iomem_resource);
	amba_device_register(&uart_device, &iomem_resource);

	nspire_keypad_data.evtcodes = nspire_touchpad_evtcode_map;
	platform_device_register(&nspire_keypad_device);
	platform_device_register(&bl_device);
	nspire_touchpad_init();

	if (!cx_use_otg) {
		pr_info("Selecting USB host only driver for CX\n");
		cx_usb_init();
	} else {
		pr_info("Selecting USB OTG driver for CX\n");
		platform_device_register(&nspire_otg_device);
		platform_device_register(&nspire_usb_nop_xceiver);
	}
}

static void __init cx_init_late(void)
{
	if (!cx_use_otg)
		cx_usb_workaround();
	nspire_init_late();
}

MACHINE_START(NSPIRECX, "TI-NSPIRE CX Calculator")
	.nr_irqs	= NR_IRQS,
	.map_io		= nspire_map_io,
	.init_irq	= cx_init_irq,
	.init_time	= cx_timer_init,
	.init_early	= cx_early_init,
	.init_machine	= cx_init,
	.init_late	= cx_init_late,
	.restart	= nspire_restart,
MACHINE_END
