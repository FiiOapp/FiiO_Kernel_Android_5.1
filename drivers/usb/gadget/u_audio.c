/*
 * u_audio.c -- ALSA audio utilities for Gadget stack
 *
 * Copyright (C) 2008 Bryan Wu <cooloney@kernel.org>
 * Copyright (C) 2008 Analog Devices, Inc
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/random.h>
#include <linux/syscalls.h>
#include <linux/cred.h>

#include "u_audio.h"

/*
 * This component encapsulates the ALSA devices for USB audio gadget
 */

#define FILE_PCM_PLAYBACK	"/dev/snd/pcmC0D0p"
//#define FILE_PCM_CAPTURE	"/dev/snd/pcmC0D0c"
#define FILE_CONTROL		"/dev/snd/controlC0"

static char *fn_play = FILE_PCM_PLAYBACK;
module_param(fn_play, charp, S_IRUGO);
MODULE_PARM_DESC(fn_play, "Playback PCM device file name");

#if 0
static char *fn_cap = FILE_PCM_CAPTURE;
module_param(fn_cap, charp, S_IRUGO);
MODULE_PARM_DESC(fn_cap, "Capture PCM device file name");
#endif

static char *fn_cntl = FILE_CONTROL;
module_param(fn_cntl, charp, S_IRUGO);
MODULE_PARM_DESC(fn_cntl, "Control device file name");

extern int actual_rate;
bool uac_rate_falg = true;
extern int audio_thread_wakeup_needed;

#if defined(CONFIG_LIDA_MACH_X5)
extern void set_codec_to_mute(bool on);
#endif

/*-------------------------------------------------------------------------*/

/**
 * Some ALSA internal helper functions
 */
static int snd_interval_refine_set(struct snd_interval *i, unsigned int val)
{
	struct snd_interval t;
	t.empty = 0;
	t.min = t.max = val;
	t.openmin = t.openmax = 0;
	t.integer = 1;
	return snd_interval_refine(i, &t);
}

static int _snd_pcm_hw_param_set(struct snd_pcm_hw_params *params,
				 snd_pcm_hw_param_t var, unsigned int val,
				 int dir)
{
	int changed;
	if (hw_is_mask(var)) {
		struct snd_mask *m = hw_param_mask(params, var);
		if (val == 0 && dir < 0) {
			changed = -EINVAL;
			snd_mask_none(m);
		} else {
			if (dir > 0)
				val++;
			else if (dir < 0)
				val--;
			changed = snd_mask_refine_set(
					hw_param_mask(params, var), val);
		}
	} else if (hw_is_interval(var)) {
		struct snd_interval *i = hw_param_interval(params, var);
		if (val == 0 && dir < 0) {
			changed = -EINVAL;
			snd_interval_none(i);
		} else if (dir == 0)
			changed = snd_interval_refine_set(i, val);
		else {
			struct snd_interval t;
			t.openmin = 1;
			t.openmax = 1;
			t.empty = 0;
			t.integer = 0;
			if (dir < 0) {
				t.min = val - 1;
				t.max = val;
			} else {
				t.min = val;
				t.max = val+1;
			}
			changed = snd_interval_refine(i, &t);
		}
	} else
		return -EINVAL;
	if (changed) {
		params->cmask |= 1 << var;
		params->rmask |= 1 << var;
	}
	return changed;
}
/*-------------------------------------------------------------------------*/

static inline
struct snd_interval *param_to_interval(struct snd_pcm_hw_params *p, int n)
{
	return &(p->intervals[n - SNDRV_PCM_HW_PARAM_FIRST_INTERVAL]);
}

static int pcm_buffer_size(struct snd_pcm_hw_params *params)
{
	struct snd_interval *i =
		param_to_interval(params, SNDRV_PCM_HW_PARAM_BUFFER_BYTES);
	pr_debug("buffer_bytes = (%d,%d) omin=%d omax=%d int=%d empty=%d\n",
		i->min, i->max, i->openmin, i->openmax, i->integer, i->empty);
	return i->min;
}

static int pcm_period_size(struct snd_pcm_hw_params *params)
{
	struct snd_interval *i =
		param_to_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES);
	return i->min;
}

/**
 * Set default hardware params
 */

static int playback_prepare_params(struct gaudio_snd_dev *snd, int sample_rate)
{
	struct snd_pcm_substream *substream = snd->substream;
	struct snd_pcm_hw_params *params;
	struct snd_pcm_sw_params *swparams;
	unsigned long period_size;
	unsigned long buffer_size;
	snd_pcm_sframes_t result;

       /*
	* SNDRV_PCM_ACCESS_RW_INTERLEAVED,
	* SNDRV_PCM_FORMAT_S16_LE
	* CHANNELS: 1
	* RATE: 16K default, user configurable
	*/
	snd->access = SNDRV_PCM_ACCESS_RW_INTERLEAVED;
	snd->format = SNDRV_PCM_FORMAT_S32_LE;//SNDRV_PCM_FORMAT_S16_LE;
	snd->channels = 2;
	snd->rate = sample_rate;

	params = kzalloc(sizeof(*params), GFP_KERNEL);
	if (!params)
		return -ENOMEM;

	_snd_pcm_hw_params_any(params);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_ACCESS,
			snd->access, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_FORMAT,
			snd->format, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_CHANNELS,
			snd->channels, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_RATE,
			snd->rate, 0);

	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
                         4*1024, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
                         16*1024, 0);

	result = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_DROP, NULL);
	if (result < 0)
		ERROR(snd->card,
			"SNDRV_PCM_IOCTL_DROP failed: %d\n", (int)result);

	result = snd_pcm_kernel_ioctl(substream,
			SNDRV_PCM_IOCTL_HW_PARAMS, params);
	if (result < 0) {
		ERROR(snd->card,
			"SNDRV_PCM_IOCTL_HW_PARAMS failed: %d\n", (int)result);
		kfree(params);
		return result;
	}

	result = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
	if (result < 0)
		ERROR(snd->card,
			"Preparing sound card failed: %d\n", (int)result);

	/* Store the hardware parameters */
	snd->access = params_access(params);
	snd->format = params_format(params);
	snd->channels = params_channels(params);
	snd->rate = params_rate(params);
#if 0
	/* Set SW params */
	swparams = kzalloc(sizeof(*swparams), GFP_KERNEL);
	if (!swparams) {
		pr_err("Failed to allocate sw params");
		kfree(params);
		return -ENOMEM;
	}

	buffer_size = pcm_buffer_size(params);
	period_size = pcm_period_size(params);
	swparams->avail_min = period_size/2;
	swparams->xfer_align = period_size/2;

	swparams->tstamp_mode = SNDRV_PCM_TSTAMP_NONE;
	swparams->period_step = 1;
	swparams->start_threshold = 1;
	swparams->stop_threshold = INT_MAX;
	swparams->silence_size = 0;
	swparams->silence_threshold = 0;

	result = snd_pcm_kernel_ioctl(substream,
				      SNDRV_PCM_IOCTL_SW_PARAMS, params/*swparams*/);
	if (result < 0)
		pr_err("SNDRV_PCM_IOCTL_SW_PARAMS failed: %d\n", (int)result);
	kfree(swparams);
#endif
	kfree(params);

	INFO(snd->card, "playback params: access %x, format %x, channels %d, rate %d\n",
		snd->access, snd->format, snd->channels, snd->rate);

	return 0;
}

static int playback_default_hw_params(struct gaudio_snd_dev *snd)
{
	struct snd_pcm_substream *substream = snd->substream;
	struct snd_pcm_hw_params *params;
	snd_pcm_sframes_t result;

       /*
	* SNDRV_PCM_ACCESS_RW_INTERLEAVED,
	* SNDRV_PCM_FORMAT_S16_LE
	* CHANNELS: 2
	* RATE: 48000
	*/
	snd->access = SNDRV_PCM_ACCESS_RW_INTERLEAVED;
	snd->format = SNDRV_PCM_FORMAT_S16_LE;
	snd->channels = 2;
	snd->rate = 44100;

	params = kzalloc(sizeof(*params), GFP_KERNEL);
	if (!params)
		return -ENOMEM;

	_snd_pcm_hw_params_any(params);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_ACCESS,
			snd->access, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_FORMAT,
			snd->format, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_CHANNELS,
			snd->channels, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_RATE,
			snd->rate, 0);

#if 0
	snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_DROP, NULL);
	snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_HW_PARAMS, params);

	result = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
	if (result < 0) {
		ERROR(snd->card,
			"Preparing sound card failed: %d\n", (int)result);
		kfree(params);
		return result;
	}
#endif

	/* Store the hardware parameters */
	snd->access = params_access(params);
	snd->format = params_format(params);
	snd->channels = params_channels(params);
	snd->rate = params_rate(params);

	kfree(params);

	INFO(snd->card,
		"Hardware params: access %x, format %x, channels %d, rate %d\n",
		snd->access, snd->format, snd->channels, snd->rate);

	return 0;
}

static int gaudio_open_playback_streams(struct gaudio *the_card)
{
	struct gaudio_snd_dev *snd;
	int res = 0;

	if (!the_card) {
		pr_err("%s: Card is NULL", __func__);
		return -ENODEV;
	}

	pr_debug("Initialize hw params");

	/* Open PCM playback device and setup substream */
	snd = &the_card->playback;
	res = playback_prepare_params(snd, actual_rate);
	if (res) {
		pr_err("Setting playback params failed: err %d", res);
		return res;
	}

	pr_debug("Initialized playback params");

	return 0;
}

/**
 * Playback audio buffer data by ALSA PCM device
 */
static size_t u_audio_playback(struct gaudio *card, void *buf, size_t count)
{
	struct gaudio_snd_dev	*snd = &card->playback;
	struct snd_pcm_substream *substream = snd->substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	mm_segment_t old_fs;
	ssize_t result;
	snd_pcm_sframes_t frames;
        int err = 0;

	if (!count) {
		pr_err("Buffer is empty, no data to play\n");
		return 0;
	}

        if (audio_thread_wakeup_needed == 0) {
                pr_err("Sampling rate switching, don`t transport data to play\n");
                return 0;
        }

        if (runtime->rate != actual_rate)
            uac_rate_falg = true;
        //else 
        //    uac_rate_falg = false;

	if (uac_rate_falg) {
#if defined(CONFIG_LIDA_MACH_X5)
                set_codec_to_mute(true);
#endif
                uac_rate_falg = false;
                runtime->rate = actual_rate;
		err = gaudio_open_playback_streams(card);
		if (err) {
			pr_err("Failed to init audio streams\n");
			return 0;
		}
#if defined(CONFIG_LIDA_MACH_X5)
                set_codec_to_mute(false);
#endif
	}

try_again:
	if (runtime->status->state == SNDRV_PCM_STATE_XRUN ||
		runtime->status->state == SNDRV_PCM_STATE_SUSPENDED ||
		runtime->status->state == SNDRV_PCM_STATE_SETUP) {
		result = snd_pcm_kernel_ioctl(substream,
				SNDRV_PCM_IOCTL_PREPARE, NULL);
		if (result < 0) {
			ERROR(card, "Preparing sound card failed: %d\n",
					(int)result);
			return result;
		}
	}

	if (!runtime->frame_bits) {
		ERROR(card, "SND failure - runtime->frame_bits == 0");
		return 0;
	}

	frames = bytes_to_frames(runtime, count);
	pr_debug("runtime->frame_bits = %d, count = %d, frames = %d",
		runtime->frame_bits, (int)count, (int)frames);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	result = snd_pcm_lib_write(snd->substream, (void __user *)buf, frames);
	if (result != frames) {
		ERROR(card, "Playback error: %d\n", (int)result);
		set_fs(old_fs);
		goto try_again;
	}
	set_fs(old_fs);

	pr_debug("Done. Sent %d frames", (int)frames);

	return 0;
}

static int u_audio_get_playback_channels(struct gaudio *card)
{
	return card->playback.channels;
}

static int u_audio_get_playback_rate(struct gaudio *card)
{
	return card->playback.rate;
}

/**
 * Open ALSA PCM and control device files
 * Initial the PCM or control device
 */
static int gaudio_open_snd_dev(struct gaudio *card)
{
	struct snd_pcm_file *pcm_file;
	struct gaudio_snd_dev *snd;
        struct cred *cred = (struct cred *)get_current_cred();
        mm_segment_t orig_fs = get_fs();
        uid_t orig_uid;
        gid_t orig_gid;

        printk("%s:   enter  !!!\n", __FUNCTION__);
	if (!card)
		return -ENODEV;

        printk("%s:  current->fsuid  is  %d \n", __FUNCTION__, cred->fsuid);
        orig_uid = cred->fsuid;
        orig_gid = cred->fsgid;
        cred->fsuid = cred->fsgid = 0;
        printk("%s:  1111 current->fsuid  is  %d \n", __FUNCTION__, cred->fsuid);
        set_fs(get_ds());

	/* Open control device */
	snd = &card->control;
	snd->filp = filp_open(fn_cntl, O_RDWR, 0);
	if (IS_ERR(snd->filp)) {
		int ret = PTR_ERR(snd->filp);
		ERROR(card, "unable to open sound control device file: %s\n",
				fn_cntl);
		snd->filp = NULL;
                set_fs(orig_fs);
		return ret;
	}
	snd->card = card;

	/* Open PCM playback device and setup substream */
	snd = &card->playback;
	snd->filp = filp_open(fn_play, O_RDWR/*O_WRONLY*/, 0);
	if (IS_ERR(snd->filp)) {
		ERROR(card, "No such PCM playback device: %s\n", fn_play);
		snd->filp = NULL;
	}

        set_fs(orig_fs);
        cred->fsuid = orig_uid;
        cred->fsgid = orig_gid;
	pcm_file = snd->filp->private_data;
	snd->substream = pcm_file->substream;
	snd->card = card;
        printk("%s: actual_rate is  %d \n", __func__, actual_rate);
	playback_default_hw_params(snd);

	/* Open PCM capture device and setup substream */
#if 0
	snd = &card->capture;
	snd->filp = filp_open(fn_cap, O_RDONLY, 0);
	if (IS_ERR(snd->filp)) {
		ERROR(card, "No such PCM capture device: %s\n", fn_cap);
		snd->substream = NULL;
		snd->card = NULL;
		snd->filp = NULL;
	} else {
		pcm_file = snd->filp->private_data;
		snd->substream = pcm_file->substream;
		snd->card = card;
	}
#endif
	return 0;
}

extern bool uac_flags;
/**
 * Close ALSA PCM and control device files
 */
static int gaudio_close_snd_dev(struct gaudio *gau)
{
	struct gaudio_snd_dev	*snd;
        printk("%s:   enter  !!!\n", __FUNCTION__);

	/* Close control device */
	snd = &gau->control;
	if (snd->filp)
		filp_close(snd->filp, NULL/*current->files*/);

	/* Close PCM playback device and setup substream */
	snd = &gau->playback;
	if (snd->filp)
		filp_close(snd->filp, NULL/*current->files*/);

	snd->substream = NULL;
	snd->card = NULL;
	snd->filp = NULL;

	uac_flags = false;

	/* Close PCM capture device and setup substream */
#if 0
	snd = &gau->capture;
	if (snd->filp)
		filp_close(snd->filp, current->files);
#endif

	return 0;
}

static struct gaudio *the_card;
/**
 * gaudio_setup - setup ALSA interface and preparing for USB transfer
 *
 * This sets up PCM, mixer or MIDI ALSA devices fore USB gadget using.
 *
 * Returns negative errno, or zero on success
 */
int /*__init*/ gaudio_setup(struct gaudio *card)
{
	struct gaudio_snd_dev *snd;
	int	ret;

	snd = &card->control;
	if (snd->card) {
		pr_debug("snd devices already opened\n");
		return 0;
	}

	ret = gaudio_open_snd_dev(card);
	if (ret)
		ERROR(card, "we need at least one control device\n");
	else if (!the_card)
		the_card = card;

	return ret;

}

/**
 * gaudio_cleanup - remove ALSA device interface
 *
 * This is called to free all resources allocated by @gaudio_setup().
 */
void gaudio_cleanup(void)
{
        printk("%s:   enter  !!!\n", __FUNCTION__);
	if (the_card) {
		gaudio_close_snd_dev(the_card);
		the_card = NULL;
	}
}

