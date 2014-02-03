#include "../../../sound/soc/codecs/wm8994.h"
#include <sound/pcm_params.h>
#include <linux/module.h>

static const struct snd_pcm_hardware dma_hardware = {
	.info			=   SNDRV_PCM_INFO_INTERLEAVED,
	.formats		=   SNDRV_PCM_FMTBIT_S16_LE |
				        SNDRV_PCM_FMTBIT_U16_LE,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 32*1024,
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= PAGE_SIZE*2,
	.periods_min		= 4,
	.periods_max		= 128,
	.fifo_size		= 32,
};

static int dma_open(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    
    dev_err(NULL, "dma_open() call\n");

    snd_soc_set_runtime_hwparams(substream, &dma_hardware);

    return 0;
}

static int dma_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
    dev_err(NULL, "dma_hw_params() call\n");

    return 0;
}

static int dma_close(struct snd_pcm_substream *substream)
{
	dev_err(NULL, "dma_close() call\n");

	return 0;
}

static int dma_hw_free(struct snd_pcm_substream *substream)
{
    dev_err(NULL, "dma_hw_free() call\n");

	return 0;
}

static int dma_prepare(struct snd_pcm_substream *substream)
{
    dev_err(NULL, "dma_prepare() call\n");

	return 0;
}

static int dma_trigger(struct snd_pcm_substream *substream, int cmd)
{
    dev_err(NULL, "dma_trigger() call\n");

	return 0;
}

static int dma_copy(struct snd_pcm_substream *substream, int channel, snd_pcm_uframes_t pos, void __user *buf, snd_pcm_uframes_t count)
{
    dev_err(NULL, "dma_copy() call\n");

	return 0;
}

static struct snd_pcm_ops dma_ops = {
	.open		= dma_open,
	.close		= dma_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= dma_hw_params,
    .copy       = dma_copy,
	.hw_free	= dma_hw_free,
	.prepare	= dma_prepare,
	.trigger	= dma_trigger,
//	.pointer	= dma_pointer,
//	.mmap		= dma_mmap,
};

static int dma_new(struct snd_soc_pcm_runtime *rtd)
{
    dev_err(NULL, "dma_new() call\n");
    
    return 0;
}

static void dma_free_dma_buffers(struct snd_pcm *pcm)
{
    dev_err(NULL, "dma_free_dma_buffers() call\n");
}

/* dma stuff */
static struct snd_soc_platform_driver stm32429i_asoc_platform = {
	.ops		= &dma_ops,
	.pcm_new	= dma_new,
	.pcm_free	= dma_free_dma_buffers,
};

/* i2s stuff */
struct i2s_dai {
    struct platform_device *pdev;
    struct snd_soc_dai_driver i2s_dai_drv;
};

static const struct snd_soc_component_driver stm32429i_i2s_component = {
	.name		= "stm32429i-i2s",
};

static inline struct i2s_dai *to_info(struct snd_soc_dai *dai)
{
	return snd_soc_dai_get_drvdata(dai);
}

static int stm32429i_i2s_dai_probe(struct snd_soc_dai *dai)
{
    struct i2s_dai *i2s = to_info(dai);

    dev_err(&i2s->pdev->dev, "stm32429i_i2s_dai_probe() call\n");
    
    return 0;
}

static int i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
    struct i2s_dai *i2s = to_info(dai);

    dev_err(&i2s->pdev->dev, "i2s_trigger() call\n");
    
    return 0;
}

static int i2s_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
    struct i2s_dai *i2s = to_info(dai);

    dev_err(&i2s->pdev->dev, "i2s_hw_params() call\n");
    
    return 0;
}

static int i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    struct i2s_dai *i2s = to_info(dai);

    dev_err(&i2s->pdev->dev, "i2s_set_fmt() call\n");
    
    return 0;
}

static int i2s_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
    struct i2s_dai *i2s = to_info(dai);

    dev_err(&i2s->pdev->dev, "i2s_set_clkdiv() call\n");
    
    return 0;
}

static int i2s_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int rfs, int dir)
{
    struct i2s_dai *i2s = to_info(dai);

    dev_err(&i2s->pdev->dev, "i2s_set_sysclk() call\n");
    
    return 0;
}

static int i2s_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    struct i2s_dai *i2s = to_info(dai);

    dev_err(&i2s->pdev->dev, "i2s_startup() call\n");
    
    return 0;
}

static void i2s_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    struct i2s_dai *i2s = to_info(dai);

    dev_err(&i2s->pdev->dev, "i2s_shutdown() call\n");
}

static const struct snd_soc_dai_ops stm32429i_i2s_dai_ops = {
	.trigger = i2s_trigger,
	.hw_params = i2s_hw_params,
	.set_fmt = i2s_set_fmt,
	.set_clkdiv = i2s_set_clkdiv,
	.set_sysclk = i2s_set_sysclk,
	.startup = i2s_startup,
	.shutdown = i2s_shutdown,
	//.delay = i2s_delay,
};


static int stm32429i_i2s_probe(struct platform_device *pdev)
{
    int ret;
    struct i2s_dai *i2s;

    dev_err(&pdev->dev, "stm32429i_i2s_probe\n");

    i2s = devm_kzalloc(&pdev->dev, sizeof(struct i2s_dai), GFP_KERNEL);
    if (!i2s) {
        dev_err(&pdev->dev, "stm32429i_i2s_probe() fail to alloc i2s\n");
        ret = -ENOMEM;
        goto out;
    }

    i2s->pdev = pdev;
    i2s->i2s_dai_drv.symmetric_rates = 1;
	i2s->i2s_dai_drv.probe = stm32429i_i2s_dai_probe;
	i2s->i2s_dai_drv.ops = &stm32429i_i2s_dai_ops;
	i2s->i2s_dai_drv.playback.channels_min = 2;
	i2s->i2s_dai_drv.playback.channels_max = 2;
	i2s->i2s_dai_drv.playback.rates = SNDRV_PCM_RATE_8000_96000;
	i2s->i2s_dai_drv.playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
    dev_set_drvdata(&i2s->pdev->dev, i2s);

    ret = snd_soc_register_component(&i2s->pdev->dev, &stm32429i_i2s_component, &i2s->i2s_dai_drv, 1);
    if (ret) {
        dev_err(&pdev->dev, "snd_soc_register_component() fail with error %d\n", ret);
        goto out;
    }

    ret = snd_soc_register_platform(&pdev->dev, &stm32429i_asoc_platform);
    if (ret) {
        dev_err(&pdev->dev, "snd_soc_register_platform() fail with error %d\n", ret);
        goto out;
    }

    ret = 0;
    out:
	return ret;
}

static struct platform_driver stm32429i_i2s_driver = {
	.driver		= {
		.name	= "stm32429i-i2s",
		.owner	= THIS_MODULE,
	},
	.probe		= stm32429i_i2s_probe,
};

module_platform_driver(stm32429i_i2s_driver);

/* device on platform */
static struct platform_device i2s_device = {
	.name		= "stm32429i-i2s",
	.id		    = 0,
};

void __init stm32429i_i2s_init(void)
{
    platform_device_register(&i2s_device);
}

