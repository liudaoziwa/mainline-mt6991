// SPDX-License-Identifier: GPL-2.0-only
/*
 * realme GT7 (RMX6688 / MT6991) machine driver.
 *
 * Speaker: two NXP TFA9873 mono smart amplifiers on i2c6 (0x34/0x35), both
 * fed from AFE I2S output 4.  Use a DL1 front end (memory interface, which
 * provides the PCM buffer/period limits) and an I2SOUT4 back end for the
 * codecs, like the other MediaTek machines.
 *
 * Microphones: the MT6681 audio PMIC ADCs (analog/DCC mics) are delivered to
 * the SoC over MTKAIF into the AFE ADDA block.  Use a UL0 front end and an
 * ADDA back end whose codec DAI is "mt6681-snd-codec-aif1".
 */

#include <linux/clk.h>
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

#define RMX6688_MIC_FE_DAI	"UL0"
#define RMX6688_MIC_CPU_DAI	"ADDA"
#define RMX6688_MIC_CODEC_DAI	"mt6681-snd-codec-aif1"

/* MTKAIF protocol 2 with CLK phase 2, same as the vendor machine. */
#define RMX6688_MTKAIF_PROTOCOL	2

extern int mt6681_set_mtkaif_protocol(struct snd_soc_component *cmpnt,
				      int mtkaif_protocol);
static const char * const rmx6688_adc_clks[] = {
	"aud_ul0_adc_clk", "aud_ul0_adc_hires_clk",
	"aud_ul1_adc_clk", "aud_ul1_adc_hires_clk",
	"aud_ul2_adc_clk", "aud_ul2_adc_hires_clk",
};


struct rmx6688_sound {
	struct snd_soc_card card;
	struct snd_soc_dai_link links[4];
	struct snd_soc_dai_link_component fe_cpus[1];
	struct snd_soc_dai_link_component fe_codecs[1];
	struct snd_soc_dai_link_component be_cpus[1];
	struct snd_soc_dai_link_component be_codecs[2];
	struct snd_soc_dai_link_component mic_fe_cpus[1];
	struct snd_soc_dai_link_component mic_fe_codecs[1];
	struct snd_soc_dai_link_component mic_be_cpus[1];
	struct snd_soc_dai_link_component mic_be_codecs[1];
	struct clk *adc_clks[ARRAY_SIZE(rmx6688_adc_clks)];
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

/* Tell the codec which MTKAIF framing the AFE is using before capture. */
/*
 * Same problem as the speaker path: UL0_CH1/CH2 take ADDA_UL_CH1/CH2 through
 * AFE_CONN018/019 mixer bits which are off at reset and are not turned on
 * by the DPCM open path.  Write them directly, like rmx6688_enable_i2s_conn().
 *
 * The UL ADC gate clocks sit in audiosys, inside the AUDIO power domain.
 * They can only be touched while that domain is powered, which is
 * guaranteed here by the FE open having resumed the AFE device -- not at
 * card probe time, where an audiosys access with the domain off raises an
 * SError (that panic is why these are enabled lazily).
 */
static int rmx6688_adda_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_component *cmpnt = snd_soc_rtd_to_cpu(rtd, 0)->component;
	struct snd_soc_card *card = rtd->card;
	struct rmx6688_sound *priv = snd_soc_card_get_drvdata(card);
	int i, ret;

	snd_soc_component_update_bits(cmpnt, AFE_CONN018_0,
				      BIT(I_ADDA_UL_CH1), BIT(I_ADDA_UL_CH1));
	snd_soc_component_update_bits(cmpnt, AFE_CONN019_0,
				      BIT(I_ADDA_UL_CH2), BIT(I_ADDA_UL_CH2));

	for (i = 0; i < ARRAY_SIZE(priv->adc_clks); i++) {
		if (!priv->adc_clks[i])
			continue;
		ret = clk_prepare_enable(priv->adc_clks[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static void rmx6688_adda_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct rmx6688_sound *priv = snd_soc_card_get_drvdata(rtd->card);
	int i;

	for (i = 0; i < ARRAY_SIZE(priv->adc_clks); i++) {
		if (priv->adc_clks[i])
			clk_disable_unprepare(priv->adc_clks[i]);
	}
}

static const struct snd_soc_ops rmx6688_adda_ops = {
	.startup = rmx6688_adda_startup,
	.shutdown = rmx6688_adda_shutdown,
};

static int rmx6688_adda_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt = snd_soc_rtd_to_codec(rtd, 0)->component;

	return mt6681_set_mtkaif_protocol(cmpnt, RMX6688_MTKAIF_PROTOCOL);
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
	/* UL0 <- ADDA_UL_Mux: mixer controls are off at reset, same as DL1. */
	{ "UL0_CH1", "ADDA_UL_CH1", "ADDA_UL_Mux" },
	{ "UL0_CH2", "ADDA_UL_CH2", "ADDA_UL_Mux" },
	{ "UL0", NULL, "UL0_CH1" },
	{ "UL0", NULL, "UL0_CH2" },
	/* Direct FE<->BE connection for the DPCM path walk (same as DL1). */
	{ "UL0", NULL, "ADDA Capture" },
};

static int rmx6688_sound_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *codec_l, *codec_r, *codec_mic;
	struct rmx6688_sound *priv;
	struct snd_soc_dai_link *fe, *be;
	struct clk *clk;
	int ret, i, num_links = 2;

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

	codec_mic = of_parse_phandle(np, "realme,codec-mic", 0);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		of_node_put(codec_l);
		of_node_put(codec_r);
		of_node_put(codec_mic);
		return -ENOMEM;
	}

	/*
	 * The mt8196 AFE driver does not manage the UL ADC gate clocks; the
	 * card holds the references and enables them at capture start
	 * (see rmx6688_adda_startup).  Optional: a board without ADC clocks
	 * still gets the speaker card.
	 */
	for (i = 0; i < ARRAY_SIZE(rmx6688_adc_clks); i++) {
		clk = devm_clk_get_optional(&pdev->dev, rmx6688_adc_clks[i]);
		if (IS_ERR(clk))
			return dev_err_probe(&pdev->dev, PTR_ERR(clk),
					     "failed to get %s\n",
					     rmx6688_adc_clks[i]);
		priv->adc_clks[i] = clk;
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

	if (codec_mic) {
		fe = &priv->links[2];
		priv->mic_fe_cpus[0].dai_name = RMX6688_MIC_FE_DAI;
		priv->mic_fe_codecs[0].name = "snd-soc-dummy";
		priv->mic_fe_codecs[0].dai_name = "snd-soc-dummy-dai";
		fe->name = "Mic Capture";
		fe->stream_name = "Mic";
		fe->trigger[0] = SND_SOC_DPCM_TRIGGER_PRE;
		fe->trigger[1] = SND_SOC_DPCM_TRIGGER_PRE;
		fe->dynamic = 1;
		fe->capture_only = 1;
		fe->cpus = priv->mic_fe_cpus;
		fe->num_cpus = ARRAY_SIZE(priv->mic_fe_cpus);
		fe->codecs = priv->mic_fe_codecs;
		fe->num_codecs = ARRAY_SIZE(priv->mic_fe_codecs);

		be = &priv->links[3];
		priv->mic_be_cpus[0].dai_name = RMX6688_MIC_CPU_DAI;
		priv->mic_be_codecs[0].of_node = codec_mic;
		priv->mic_be_codecs[0].dai_name = RMX6688_MIC_CODEC_DAI;
		be->name = "MT6681 Capture";
		be->stream_name = "MT6681 Capture";
		be->no_pcm = 1;
		be->capture_only = 1;
		be->ignore_suspend = 1;
		be->init = rmx6688_adda_init;
		be->ops = &rmx6688_adda_ops;
		be->cpus = priv->mic_be_cpus;
		be->num_cpus = ARRAY_SIZE(priv->mic_be_cpus);
		be->codecs = priv->mic_be_codecs;
		be->num_codecs = ARRAY_SIZE(priv->mic_be_codecs);

		num_links = 4;
	}

	priv->card.dev = &pdev->dev;
	priv->card.name = "rmx6688-speaker";
	priv->card.owner = THIS_MODULE;
	snd_soc_card_set_drvdata(&priv->card, priv);
	priv->card.dai_link = priv->links;
	priv->card.num_links = num_links;
	priv->card.dapm_routes = rmx6688_routes;
	priv->card.num_dapm_routes = ARRAY_SIZE(rmx6688_routes);

	ret = devm_snd_soc_register_card(&pdev->dev, &priv->card);
	if (ret)
		dev_err_probe(&pdev->dev, ret, "failed to register sound card\n");

	of_node_put(codec_mic);
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

MODULE_DESCRIPTION("realme RMX6688 / MT6991 sound machine");
MODULE_LICENSE("GPL v2");
