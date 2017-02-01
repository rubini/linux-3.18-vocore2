#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "es8388.h"

void es8388_reset(struct snd_soc_codec *codec)
{
	pr_info("es8388_reset\n");
	snd_soc_write(codec, ES8388_CTRL1, ES8388_CTRL1_SCP_RESET);
	snd_soc_write(codec, ES8388_CTRL1,
		      ES8388_CTRL1_VMIDSEL_500k | ES8388_CTRL1_ENREF);
}

static int es8388_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	if (dai->capture_active){
		// power on AD
		snd_soc_write(codec, 3, 0);
	}else{
		// power off AD
		snd_soc_write(codec, 3, 0xff);
	}
	if (dai->playback_active){
		// power on DA
		snd_soc_write(codec, 4, 0x3c);
	} else {
		// power off DA
		snd_soc_write(codec, 4, 0xc0);
	}

	pr_info("es8388_hw_params: %d, AD: %d, DA: %d\n",
		params_width(params), dai->capture_active, dai->playback_active);

	return 0;
}

static int es8388_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	int reg = mute ? ES8388_CTRL3_DACMUTE : 0;
	snd_soc_update_bits(codec, ES8388_CTRL3, ES8388_CTRL3_DACMUTE, reg);
	pr_info("es8388_mute %d\n", mute);
	return 0;
}

static const struct snd_soc_dai_ops es8388_dai_ops = {
	.hw_params = es8388_hw_params,
	.digital_mute = es8388_mute,
};

static struct snd_soc_dai_driver es8388_dai = {
	.name = "es8388-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &es8388_dai_ops,
	.symmetric_rates = 1,
};

static int es8388_suspend(struct snd_soc_codec *codec)
{
	pr_info("es8388_suspend\n");
	return 0;
}

static int es8388_resume(struct snd_soc_codec *codec)
{
	pr_info("es8388_resume\n");
	return 0;
}

static int es8388_probe(struct snd_soc_codec *codec)
{
	pr_info("es8388_codec_probe\n");
	es8388_reset(codec);

	snd_soc_write(codec, 0x00, 0x05);
	snd_soc_write(codec, 0x01, 0x40);
	snd_soc_write(codec, 0x02, 0x00);
	// AD and DA power on when ES8388 power on
	snd_soc_write(codec, 0x03, 0x0);
	snd_soc_write(codec, 0x04, 0x3c);

	// slave mode
	snd_soc_write(codec, 0x08, 0x00);

	// AD
	snd_soc_write(codec, 0x09, 0x44);   // power gain!
	// L use L2, R use R2
	// snd_soc_write(codec, 0x0a, 0x51);   // pin L2
	// snd_soc_write(codec, 0x0b, 0x88);   //
	// snd_soc_write(codec, 0x0a, 0xf4);   // differential
	snd_soc_write(codec, 0x0a, 0x04);   // L1
	snd_soc_write(codec, 0x0b, 0x80);   //
	snd_soc_write(codec, 0x0c, 0x4c);   // 16 bit, CLK,DAT,LRCLK use I2S format
	snd_soc_write(codec, 0x0d, 0x33);
	snd_soc_write(codec, 0x0e, 0x00);

	// snd_soc_write(codec, 0x10, 0xc0);
	// snd_soc_write(codec, 0x11, 0xc0);
	snd_soc_write(codec, 0x10, 0x00);   // L vol, 00 0dB, c0 -96dB
	snd_soc_write(codec, 0x11, 0x00);   // R vol, 00 0dB, c0 -96dB
	snd_soc_write(codec, 0x12, 0xe2);
	snd_soc_write(codec, 0x13, 0xc0);
	snd_soc_write(codec, 0x14, 0x11);
	snd_soc_write(codec, 0x15, 0x06);
	snd_soc_write(codec, 0x16, 0xf3);

	// DA
	snd_soc_write(codec, 0x17, 0x18);
	snd_soc_write(codec, 0x18, 0x33);
	snd_soc_write(codec, 0x19, 0x20);
	snd_soc_write(codec, 0x1a, 0x00);
	snd_soc_write(codec, 0x1b, 0x00);
	// L use L1, R use R1
	snd_soc_write(codec, 0x26, 0x00);
	snd_soc_write(codec, 0x27, 0x80);
	snd_soc_write(codec, 0x2a, 0x80);
	snd_soc_write(codec, 0x2b, 0x80);
	snd_soc_write(codec, 0x2e, 0x1e);
	snd_soc_write(codec, 0x2f, 0x1e);
	snd_soc_write(codec, 0x30, 0x1e);
	snd_soc_write(codec, 0x31, 0x1e);
	return 0;
}

static int es8388_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static int es8388_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level)
{
	return 0;
}

static struct snd_soc_codec_driver codec_es8388 = {
	.probe =	es8388_probe,
	.remove =	es8388_remove,
	.suspend =	es8388_suspend,
	.resume =	es8388_resume,
	.set_bias_level = es8388_set_bias_level,
};

static const struct regmap_config es8388_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ES8388_REG_MAX,
	.cache_type = REGCACHE_RBTREE,
};

static int es8388_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(i2c, &es8388_regmap);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	// call snd_soc_cache_sync, we do not need regmap handler.
	return snd_soc_register_codec(&i2c->dev, &codec_es8388, &es8388_dai, 1);
}

static int es8388_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id es8388_i2c_id[] = {
	{ "es8388", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es8388_i2c_id);

static const struct of_device_id es8388_ofs[] = {
	{ .compatible = "everest,es8388", },
	{ }
};
MODULE_DEVICE_TABLE(of, es8388_ofs);

static struct i2c_driver es8388_i2c_driver = {
	.driver = {
		.name = "es8388",
		.of_match_table = es8388_ofs,
	},
	.probe = es8388_i2c_probe,
	.remove = es8388_i2c_remove,
	.id_table = es8388_i2c_id,
};

module_i2c_driver(es8388_i2c_driver);

MODULE_DESCRIPTION("Audio codec ES8388 driver");
MODULE_AUTHOR("Qin Wei <me@vocore.io>");
MODULE_LICENSE("GPL");
