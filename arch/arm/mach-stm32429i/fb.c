#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>

#undef DEBUG
#define DEBUG

#ifdef DEBUG
#define debug(fmt,args...)	    printk(KERN_INFO fmt, ##args)
#else
#define debug(fmt,args...)
#endif	/* DEBUG */
#define info(fmt,args...)       printk(KERN_INFO fmt, ##args)
#define warning(fmt,args...)    printk(KERN_WARNING fmt, ##args)
#define error(fmt,args...)	    printk(KERN_ERR fmt, ##args)

#define VIDEO_MEMORY_ADDRESS    (0xc0000000 + 31*1024*1024)
#define VIDEO_MEMORY_SIZE       (1024*1024)

struct stm32_ltdc_layer_regs {
    u32	cr;             /* 0x00 *//* LTDC Layerx Control Register */
    u32	whpcr;          /* 0x04 *//* LTDC Layerx Window Horizontal Position Configuration Register */
    u32	wvpcr;          /* 0x08 *//* LTDC Layerx Window Vertical Position Configuration Register */
    u32 ckcr;           /* 0x0c *//* LTDC Layerx Color Keying Configuration Register */
    u32 pfcr;           /* 0x10 *//* LTDC Layerx Pixel Format Configuration Register */
    u32 cacr;           /* 0x14 *//* LTDC Layerx Constant Alpha Configuration Register */
    u32 dccr;           /* 0x18 *//* LTDC Layerx Default Color Configuration Register */
    u32 bfcr;           /* 0x1c *//* LTDC Layerx Blending Factors Configuration Register */
    u32	reserved0[(0x28 - 0x1c) / sizeof(u32) - 1];
    u32 cfbar;          /* 0x28 *//* LTDC Layerx Color Frame Buffer Address Register */
    u32 cfblr;          /* 0x28 *//* LTDC Layerx Color Frame Buffer Length Register */
    u32 cfblnr;         /* 0x2c *//* LTDC Layerx Color Frame Buffer Line Number Register */
    u32	reserved1[(0x3c - 0x2c) / sizeof(u32) - 1];
    u32 clutwr;         /* 0x3c *//* LTDC Layerx CLUT Write Register */
    u32	reserved2[(0x84 - 0x3c) / sizeof(u32) - 1];
};

struct stm32_ltdc_regs {
    u32	reserved0[(0x08 - 0x00) / sizeof(u32) - 1 + 1];
	u32	sscr;           /* 0x08 *//* LTDC Synchronization Size Configuration Register */
	u32	bpcr;           /* 0x0c *//* LTDC Back Porch Configuration Register */
    u32	awcr;           /* 0x10 *//* LTDC Active Width Configuration Register */
    u32	twcr;           /* 0x14 *//* LTDC Total Width Configuration Register */   
    u32	gcr;            /* 0x18 *//* LTDC Global Control Register */ 
    u32	reserved1[(0x24 - 0x18) / sizeof(u32) - 1];
    u32	srcr;           /* 0x24 *//* LTDC Shadow Reload Configuration Register */
    u32	reserved2[(0x2c - 0x24) / sizeof(u32) - 1];
    u32	bccr;           /* 0x2c *//* LTDC Background Color Configuration Register */
    u32	reserved3[(0x34 - 0x2c) / sizeof(u32) - 1];
    u32	ier;            /* 0x34 *//* LTDC Interrupt Enable Register */
    u32	isr;            /* 0x38 *//* LTDC Interrupt Status Register */
    u32	icr;            /* 0x3c *//* LTDC Interrupt Clear Register */
    u32	lipcr;          /* 0x40 *//* LTDC Line Interrupt Position Configuration Register */
    u32	cpsr;           /* 0x44 *//* LTDC Current Position Status Register */
    u32	cdsr;           /* 0x48 *//* LTDC Current Display Status Register */
    u32	reserved4[(0x84 - 0x48) / sizeof(u32) - 1];
    struct stm32_ltdc_layer_regs layer[2];
};

struct fb_par {
    struct platform_device *pdev;
    volatile struct stm32_ltdc_regs *regs;
    u32 pseudo_palette[16];
};

static int stm32429i_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
    error("stm32429i_fb_check_var not yet implemented\n");

    return -1;
}

static int stm32429i_fb_set_par(struct fb_info *info)
{
    error("stm32429i_fb_set_par not yet implemented\n");

    return 0;	
}

void stm32429i_fb_fillrect(struct fb_info *p, const struct fb_fillrect *region)
{
    cfb_fillrect(p, region);
}

void stm32429i_fb_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
    cfb_copyarea(p, area);
}

void stm32429i_fb_imageblit(struct fb_info *p, const struct fb_image *image)
{
    cfb_imageblit(p, image);
}

static int stm32429i_fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue, u_int trans, struct fb_info *info)
{
    if (regno >= 16)
        return 1;
    
    red = red >> 8;
    green = green >> 8;
    blue = blue >> 8;

    ((u32 *)info->pseudo_palette)[regno] = (red << info->var.red.offset) |
                                            (green << info->var.green.offset) |
                                            (blue << info->var.blue.offset);

    return 0;
}

static struct fb_ops stm32429i_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= stm32429i_fb_check_var,
	.fb_set_par	= stm32429i_fb_set_par,
	.fb_setcolreg	= stm32429i_fb_setcolreg,
	.fb_fillrect	= stm32429i_fb_fillrect, 	/* Needed !!! */
	.fb_copyarea	= stm32429i_fb_copyarea,	/* Needed !!! */
	.fb_imageblit	= stm32429i_fb_imageblit,	/* Needed !!! */
};

static void init_hw(struct fb_info *info)
{
    struct fb_par *par = info->par;

    /* enable clock */
    *((volatile unsigned int *) 0x40023844) |= (1 << 26);

    /* display settings */
    par->regs->sscr = 0x280009;
    par->regs->bpcr = 0x2a000b;
    par->regs->awcr = 0x20a011b;
    par->regs->twcr = 0x20c011d;
    par->regs->gcr = 0x2220;
    par->regs->bccr = 0x808080;/* grey */

    /* setup layer 0 */
    par->regs->layer[0].whpcr = 0x20a002b;
    par->regs->layer[0].wvpcr = 0x11b000c;
    par->regs->layer[0].pfcr = 1;//RGB888
    par->regs->layer[0].bfcr = 0x405;//only layer 0 is display
    par->regs->layer[0].cfbar = info->fix.smem_start;
    par->regs->layer[0].cfblr = 0x5a005a3;
    par->regs->layer[0].cfblnr = 0x110;
    /* enable layer */
    par->regs->layer[0].cr |= 1;

    par->regs->srcr = 1;/* update layer registers */
    /* enable display */
    par->regs->gcr |= 1;
}

static int stm32429i_fb_probe(struct platform_device *dev)
{
    struct fb_info *info;
    struct device *device = &dev->dev;
    struct fb_par *par;
    int ret;
    
    /* frame buffer info */
    info = framebuffer_alloc(sizeof(struct fb_par), device);
    if (!info) {
        error("Unable to framebuffer_alloc\n");
        ret = -ENOMEM;
		goto error_allocate_info;
    }
    par = info->par;
    par->pdev = dev;
    par->regs = (struct stm32_ltdc_regs *) 0x40016800;

    /* fillup structure */
    info->pseudo_palette = par->pseudo_palette;
    info->screen_base = ioremap(VIDEO_MEMORY_ADDRESS, VIDEO_MEMORY_SIZE);
    info->fbops = &stm32429i_fb_ops;
    info->flags = FBINFO_DEFAULT;
    strcpy(info->fix.id, "STM32");
    info->fix.smem_start = VIDEO_MEMORY_ADDRESS;
    info->fix.smem_len = 480*272*3;
    info->fix.type = FB_TYPE_PACKED_PIXELS;
    info->fix.type_aux = 0;
    info->fix.visual = FB_VISUAL_TRUECOLOR;
    info->fix.xpanstep = 0;
    info->fix.ypanstep = 0;
    info->fix.ywrapstep = 0;
    info->fix.line_length = 480 * 3;
    info->fix.accel = FB_ACCEL_NONE;
    info->var.xres = 480;
    info->var.yres = 272;
    info->var.xres_virtual = 480;
    info->var.yres_virtual = 272;
    info->var.xoffset = 0;
    info->var.yoffset = 0;
    info->var.bits_per_pixel = 24;
    info->var.grayscale = 0;
    info->var.red.offset = 16;
    info->var.red.length = 8;
    info->var.red.msb_right = 0;
    info->var.green.offset = 8;
    info->var.green.length = 8;
    info->var.green.msb_right = 0;
    info->var.blue.offset = 0;
    info->var.blue.length = 8;
    info->var.blue.msb_right = 0;

    /* register fb */
    ret = register_framebuffer(info);
    if (ret < 0) {
        error("Unable to register_framebuffer\n");
        goto error_register_framebuffer;
    }

    init_hw(info);

    printk(KERN_INFO "fb%d: %s frame buffer device\n", info->node, info->fix.id);
    platform_set_drvdata(dev, info);

    ret = 0;

error_register_framebuffer:
    if (ret)
        framebuffer_release(info);

error_allocate_info:
    
    return ret;
}

static struct platform_driver stm32429i_fb_driver = {
	.probe = stm32429i_fb_probe,
	.driver = {
		.name = "stm32429i_fb",
	},
};

static struct platform_device stm32429i_fb_device = {
	.name           = "stm32429i_fb",
	.id             = 0,
};

static int __init stm32429i_fb_init(void)
{
    int ret;

    ret = platform_driver_register(&stm32429i_fb_driver);
    if (!ret) {
        ret = platform_device_register(&stm32429i_fb_device);
        if (ret)
            error("Unable to register stm32429i_fb_device\n");
    } else
        error("Unable to register stm32429i_fb_driver\n");

    return ret;
}

module_init(stm32429i_fb_init);

