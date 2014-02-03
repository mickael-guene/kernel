#include "../../../sound/soc/codecs/wm8994.h"
#include <sound/pcm_params.h>
#include <linux/module.h>

/* FIXME : need to setup i2s pll */
#define STM32429I_WM8994_FREQ   25000000

static int stm32429i_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int pll_out;
	int ret;

    dev_err(NULL, "stm32429i_hw_params()\n");
	/* AIF1CLK should be >=3MHz for optimal performance */
	if (params_format(params) == SNDRV_PCM_FORMAT_S24_LE)
		pll_out = params_rate(params) * 384;
	else if (params_rate(params) == 8000 || params_rate(params) == 11025)
		pll_out = params_rate(params) * 512;
	else
		pll_out = params_rate(params) * 256;

	ret = snd_soc_dai_set_pll(codec_dai, WM8994_FLL1, WM8994_FLL_SRC_MCLK1,
					STM32429I_WM8994_FREQ, pll_out);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_FLL1,
					pll_out, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops stm32429i_ops = {
    .hw_params = stm32429i_hw_params,
};

#if 0
static int stm32429i_init_paiftx(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* HeadPhone */
	snd_soc_dapm_enable_pin(dapm, "HPOUT1R");
	snd_soc_dapm_enable_pin(dapm, "HPOUT1L");

	/* Other pins NC */
	snd_soc_dapm_nc_pin(dapm, "HPOUT2P");
	snd_soc_dapm_nc_pin(dapm, "HPOUT2N");
	snd_soc_dapm_nc_pin(dapm, "SPKOUTLN");
	snd_soc_dapm_nc_pin(dapm, "SPKOUTLP");
	snd_soc_dapm_nc_pin(dapm, "SPKOUTRP");
	snd_soc_dapm_nc_pin(dapm, "SPKOUTRN");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT1N");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT1P");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT2N");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT2P");
	snd_soc_dapm_nc_pin(dapm, "IN1LP");
	snd_soc_dapm_nc_pin(dapm, "IN2LP:VXRN");
	snd_soc_dapm_nc_pin(dapm, "IN1RP");
	snd_soc_dapm_nc_pin(dapm, "IN2RP:VXRP");

	return 0;
}
#endif

static struct snd_soc_dai_link stm32429i_dais[] = {
	{ /* Primary DAI i/f */
		.name = "WM8994 AIF1",
		.stream_name = "Pri_Dai",
		.cpu_dai_name = "stm32429i-i2s.0",
		.codec_dai_name = "wm8994-aif1",
		.platform_name = "stm32429i-i2s.0",
		.codec_name = "wm8994-codec",
        //.init = stm32429i_init_paiftx,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM,
		.ops = &stm32429i_ops,
	},
};

static struct snd_soc_card stm32429i_card = {
	.name = "stm32429i-i2s",
	.owner = THIS_MODULE,
	.dai_link = stm32429i_dais,
	.num_links = ARRAY_SIZE(stm32429i_dais),
};

static int stm32429i_audio_probe(struct platform_device *pdev)
{
    int ret;

    stm32429i_card.dev = &pdev->dev;
    ret = snd_soc_register_card(&stm32429i_card);

    if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed:%d\n", ret);

	return ret;
}

static struct platform_driver stm32429i_audio_driver = {
	.driver		= {
		.name	= "stm32429i-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= stm32429i_audio_probe,
};

module_platform_driver(stm32429i_audio_driver);


/* device on platform */
static struct platform_device sound_device = {
	.name		= "stm32429i-audio",
	.id		    = 0,
};

void __init stm32429i_sound_init(void)
{
    platform_device_register(&sound_device);
}

