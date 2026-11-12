// SPDX-License-Identifier: GPL-2.0-only
/*
 * realme GT7 (RMX6688 / MT6991) speaker machine driver.
 *
 * Two NXP TFA9873 mono smart amplifiers on i2c6 (0x34/0x35), both fed
 * from AFE I2S output 4.  Use a DL1 front end (memory interface, which
 * provides the PCM buffer/period limits) and an I2SOUT4 back end for the
 * codecs, like the other MediaTek machines.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "mt8196-reg.h"
#include "mt8196-interconnection.h"

#define RMX6688_FE_DAI		"DL1"
#define RMX6688_CPU_DAI		"I2SOUT4"
#define RMX6688_CODEC_DAI	"tfa98xx-aif"

struct rmx6688_sound {
	struct snd_soc_card card;
	struct snd_soc_dai_link links[2];
	struct snd_soc_dai_link_component fe_cpus[1];
	struct snd_soc_dai_link_component fe_codecs[1];
	struct snd_soc_dai_link_component be_cpus[1];
	struct snd_soc_dai_link_component be_codecs[2];
};

/*
 * The AFE routes DL1 into I2SOUT4 through the AFE_CONN026/027 mixer
 * switches.  They are off by default and the DPCM core does not turn
 * them on.  Writing the connection bits directly (instead of poking the
 * DAPM mixer kcontrol) avoids taking the DAPM mutex while the DPCM
 * open path already holds its own locks.
 */
static void rmx6688_enable_i2s_conn(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt = snd_soc_rtd_to_cpu(rtd, 0)->component;

	snd_soc_component_update_bits(cmpnt, AFE_CONN026_1,
				      BIT(I_DL1_CH1), BIT(I_DL1_CH1));
	snd_soc_component_update_bits(cmpnt, AFE_CONN027_1,
				      BIT(I_DL1_CH2), BIT(I_DL1_CH2));
}

static int rmx6688_be_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);

	rmx6688_enable_i2s_conn(rtd);
	return 0;
}

static int rmx6688_be_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	unsigned int mclk = params_rate(params) * 128;

	return snd_soc_dai_set_sysclk(cpu_dai, 0, mclk, SND_SOC_CLOCK_OUT);
}

static const struct snd_soc_ops rmx6688_be_ops = {
	.startup = rmx6688_be_startup,
	.hw_params = rmx6688_be_hw_params,
};

static int rmx6688_be_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	snd_mask_reset_range(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
			     0, SNDRV_PCM_FORMAT_LAST);
	params_set_format(params, SNDRV_PCM_FORMAT_S32_LE);
	return 0;
}

/*
 * The AFE's I2SOUT4_CH1 mixer path from DL1 is a DAPM mixer control and is
 * disconnected at PCM-open time, so the DPCM path walk finds no BE.  Add
 * an explicit connected route from the FE DAI to the BE DAI as well.
 */
static const struct snd_soc_dapm_route rmx6688_routes[] = {
	{ "I2SOUT4_CH1", "DL1_CH1", "DL1" },
	{ "I2SOUT4", NULL, "I2SOUT4_CH1" },
	{ "I2SOUT4", NULL, "DL1" },
};

static int rmx6688_sound_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *codec_l, *codec_r;
	struct rmx6688_sound *priv;
	struct snd_soc_dai_link *fe, *be;
	int ret;

	if (!np)
		return -ENODEV;

	codec_l = of_parse_phandle(np, "realme,codec-left", 0);
	codec_r = of_parse_phandle(np, "realme,codec-right", 0);
	if (!codec_l || !codec_r) {
		of_node_put(codec_l);
		of_node_put(codec_r);
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing realme,codec-left/right\n");
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		of_node_put(codec_l);
		of_node_put(codec_r);
		return -ENOMEM;
	}

	fe = &priv->links[0];
	priv->fe_cpus[0].dai_name = RMX6688_FE_DAI;
	priv->fe_codecs[0].name = "snd-soc-dummy";
	priv->fe_codecs[0].dai_name = "snd-soc-dummy-dai";
	fe->name = "Speaker Playback";
	fe->stream_name = "Speaker";
	fe->trigger[0] = SND_SOC_DPCM_TRIGGER_PRE;
	fe->trigger[1] = SND_SOC_DPCM_TRIGGER_PRE;
	fe->dynamic = 1;
	fe->playback_only = 1;
	fe->cpus = priv->fe_cpus;
	fe->num_cpus = ARRAY_SIZE(priv->fe_cpus);
	fe->codecs = priv->fe_codecs;
	fe->num_codecs = ARRAY_SIZE(priv->fe_codecs);

	be = &priv->links[1];
	priv->be_cpus[0].dai_name = RMX6688_CPU_DAI;
	priv->be_codecs[0].of_node = codec_l;
	priv->be_codecs[0].dai_name = RMX6688_CODEC_DAI;
	priv->be_codecs[1].of_node = codec_r;
	priv->be_codecs[1].dai_name = RMX6688_CODEC_DAI;
	be->name = "SPK";
	be->no_pcm = 1;
	be->playback_only = 1;
	be->ignore_suspend = 1;
	be->ignore_pmdown_time = 1;
	be->dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		      SND_SOC_DAIFMT_CBC_CFC | SND_SOC_DAIFMT_GATED;
	be->ops = &rmx6688_be_ops;
	be->be_hw_params_fixup = rmx6688_be_fixup;
	be->cpus = priv->be_cpus;
	be->num_cpus = ARRAY_SIZE(priv->be_cpus);
	be->codecs = priv->be_codecs;
	be->num_codecs = ARRAY_SIZE(priv->be_codecs);

	priv->card.dev = &pdev->dev;
	priv->card.name = "rmx6688-speaker";
	priv->card.owner = THIS_MODULE;
	priv->card.dai_link = priv->links;
	priv->card.num_links = ARRAY_SIZE(priv->links);
	priv->card.dapm_routes = rmx6688_routes;
	priv->card.num_dapm_routes = ARRAY_SIZE(rmx6688_routes);

	ret = devm_snd_soc_register_card(&pdev->dev, &priv->card);
	if (ret)
		dev_err_probe(&pdev->dev, ret, "failed to register sound card\n");

	return ret;
}

static const struct of_device_id rmx6688_sound_of_match[] = {
	{ .compatible = "realme,rmx6688-sound" },
	{ }
};
MODULE_DEVICE_TABLE(of, rmx6688_sound_of_match);

static struct platform_driver rmx6688_sound_driver = {
	.probe = rmx6688_sound_probe,
	.driver = {
		.name = "rmx6688-sound",
		.of_match_table = rmx6688_sound_of_match,
	},
};
module_platform_driver(rmx6688_sound_driver);

MODULE_DESCRIPTION("realme RMX6688 / MT6991 speaker machine");
MODULE_LICENSE("GPL v2");
