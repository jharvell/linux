/*
 *  Force feedback support for Linux input subsystem
 *
 *  Copyright (c) 2006 Anssi Hannula <anssi.hannula@gmail.com>
 *  Copyright (c) 2006 Dmitry Torokhov <dtor@mail.ru>
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* #define DEBUG */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>


static const char* ff_names[]=
{
        "Rumble"
        ,"Periodic"
        ,"Constant"
        ,"Spring"
        ,"Friction"
        ,"Damper"
        ,"Inertia"
        ,"Ramp"
        ,"Square"
        ,"Triangle"
        ,"Sine"
        ,"SawUp"
        ,"SawDown"
        ,"Custom"
        ,"<Unknown=0x5e>"
        ,"<Unknown=0x5f>"
        ,"Gain"
        ,"Autocenter"
};

inline
static const char* ff_name(__u16 val)
{
    __u16 idx = val - 0x50;
    if(idx < sizeof(ff_names)/sizeof(ff_names[0]))
    {
        return ff_names[idx];
    }
    return "<Invalid>";
}

#define debug_print_effect(idev,effect)\
do{\
    do_debug_print_effect(__FILE__,__LINE__,idev,effect);\
} while(0)

static void do_debug_print_effect(const char* file, unsigned line,
        const struct input_dev *idev,
        const struct ff_effect *effect)
{
    const struct device* dev = &idev->dev;
#define FF_EFFECT_PRINT_FORMAT "file=%s line=%u type=%s id=%d dir=%u\ntrigger{btn=%u,intvl=%u}\nreplay{len=%u,delay=%u}"
#define FF_EFFECT_PRINT_ARGS file,line,ff_name(effect->type), effect->id, effect->direction\
        ,effect->trigger.button, effect->trigger.interval\
        ,effect->replay.length, effect->replay.delay
#define FF_ENV_PRINT_FORMAT "env{aLen=%u,aLvl=%u,fadeLen=%u,fadeLvl=%u}"
#define FF_ENV_PRINT_ARGS env->attack_length,env->attack_level\
    ,env->fade_length,env->fade_level
#define FF_COND_PRINT_FORMAT "cond[0]{left{sat=%u,k=%d} right{sat=%u,k=%d} dead=%u, ctr=%d}\n\
cond[1]{left{sat=%u,k=%d} right{sat=%u,k=%d} dead=%u, ctr=%d}"

#define FF_COND_PRINT_ARGS cond[0].left_saturation, cond[1].left_coeff\
    ,cond[0].right_saturation, cond[0].right_coeff\
    ,cond[0].deadband, cond[0].center\
    ,cond[1].left_saturation, cond[1].left_coeff\
    ,cond[1].right_saturation, cond[1].right_coeff\
    ,cond[1].deadband, cond[1].center

    switch(effect->type)
    {
    case FF_RUMBLE:
    {
        const struct ff_rumble_effect* re = &effect->u.rumble;
        dev_dbg(dev,FF_EFFECT_PRINT_FORMAT
                "\nrumble{strong_magnitude=%u, weak_magnitudue=%u}\n"
                ,FF_EFFECT_PRINT_ARGS
                ,re->strong_magnitude,re->weak_magnitude);
        break;
    }
    case FF_PERIODIC:
    {
        const struct ff_periodic_effect* pe = &effect->u.periodic;
        const struct ff_envelope* env = &pe->envelope;
        dev_dbg(dev,FF_EFFECT_PRINT_FORMAT
                "\nperiodic{waveform=%s,period=%u,magnitude=%u,offset=%d,phase=%u}"
                "\n" FF_ENV_PRINT_FORMAT "\n"
                ,FF_EFFECT_PRINT_ARGS
                ,ff_name(pe->waveform),pe->period,pe->magnitude
                ,pe->offset,pe->phase
                ,FF_ENV_PRINT_ARGS);
        break;
    }
    case FF_CONSTANT:
    {
        const struct ff_constant_effect* ce = &effect->u.constant;
        const struct ff_envelope* env = &ce->envelope;
        dev_dbg(dev,FF_EFFECT_PRINT_FORMAT
                "\nconstant: lvl=%d\n"
                FF_ENV_PRINT_FORMAT "\n"
                ,FF_EFFECT_PRINT_ARGS
                ,ce->level
                ,FF_ENV_PRINT_ARGS);
        break;
    }
    case FF_SPRING:
    case FF_FRICTION:
    case FF_DAMPER:
    case FF_INERTIA:
    {
        const struct ff_condition_effect* cond = &effect->u.condition[0];
        dev_dbg(dev,FF_EFFECT_PRINT_FORMAT
                "\n" FF_COND_PRINT_FORMAT "\n"
                ,FF_EFFECT_PRINT_ARGS
                ,FF_COND_PRINT_ARGS);
        break;
    }
    case FF_RAMP:
    {
        const struct ff_ramp_effect* re = &effect->u.ramp;
        const struct ff_envelope* env = &re->envelope;
        dev_dbg(dev,FF_EFFECT_PRINT_FORMAT
                "\nramp{startLvl=%d,endLvl=%d}"
                "\n" FF_ENV_PRINT_FORMAT "\n"
                ,FF_EFFECT_PRINT_ARGS
                ,re->start_level,re->end_level
                ,FF_ENV_PRINT_ARGS);
        break;
    }
    default:
    {
        dev_dbg(dev,FF_EFFECT_PRINT_FORMAT
                "\nUnknown Effect format\n"
                ,FF_EFFECT_PRINT_ARGS);
        break;
    }
    }
}

/*
 * Check that the effect_id is a valid effect and whether the user
 * is the owner
 */
static int check_effect_access(struct ff_device *ff, int effect_id,
				struct file *file)
{
	if (effect_id < 0 || effect_id >= ff->max_effects ||
	    !ff->effect_owners[effect_id])
		return -EINVAL;

	if (file && ff->effect_owners[effect_id] != file)
		return -EACCES;

	return 0;
}

/*
 * Checks whether 2 effects can be combined together
 */
static inline int check_effects_compatible(struct ff_effect *e1,
					   struct ff_effect *e2)
{
	return e1->type == e2->type &&
	       (e1->type != FF_PERIODIC ||
		e1->u.periodic.waveform == e2->u.periodic.waveform);
}

/*
 * Convert an effect into compatible one
 */
static int compat_effect(struct ff_device *ff, struct ff_effect *effect)
{
	int magnitude;

	switch (effect->type) {
	case FF_RUMBLE:
		if (!test_bit(FF_PERIODIC, ff->ffbit))
			return -EINVAL;

		/*
		 * calculate manginude of sine wave as average of rumble's
		 * 2/3 of strong magnitude and 1/3 of weak magnitude
		 */
		magnitude = effect->u.rumble.strong_magnitude / 3 +
			    effect->u.rumble.weak_magnitude / 6;

		effect->type = FF_PERIODIC;
		effect->u.periodic.waveform = FF_SINE;
		effect->u.periodic.period = 50;
		effect->u.periodic.magnitude = max(magnitude, 0x7fff);
		effect->u.periodic.offset = 0;
		effect->u.periodic.phase = 0;
		effect->u.periodic.envelope.attack_length = 0;
		effect->u.periodic.envelope.attack_level = 0;
		effect->u.periodic.envelope.fade_length = 0;
		effect->u.periodic.envelope.fade_level = 0;

		return 0;

	default:
		/* Let driver handle conversion */
		return 0;
	}
}

/**
 * input_ff_upload() - upload effect into force-feedback device
 * @dev: input device
 * @effect: effect to be uploaded
 * @file: owner of the effect
 */
int input_ff_upload(struct input_dev *dev, struct ff_effect *effect,
		    struct file *file)
{
	struct ff_device *ff = dev->ff;
	struct ff_effect *old = NULL;
	int ret = 0;
	int id;

	debug_print_effect(dev,effect);

	if (!test_bit(EV_FF, dev->evbit)) {
	    dev_dbg(&dev->dev, "Force-feedback not supported for this device.\n");
	    return -ENOSYS;
	}


	if (effect->type < FF_EFFECT_MIN || effect->type > FF_EFFECT_MAX ||
	    !test_bit(effect->type, dev->ffbit)) {
		dev_dbg(&dev->dev, "invalid or not supported effect type in upload\n");
		return -EINVAL;
	}

	if (effect->type == FF_PERIODIC &&
	    (effect->u.periodic.waveform < FF_WAVEFORM_MIN ||
	     effect->u.periodic.waveform > FF_WAVEFORM_MAX ||
	     !test_bit(effect->u.periodic.waveform, dev->ffbit))) {
		dev_dbg(&dev->dev, "invalid or not supported wave form in upload\n");
		return -EINVAL;
	}

	if (!test_bit(effect->type, ff->ffbit)) {
		ret = compat_effect(ff, effect);
		debug_print_effect(dev,effect);
		if (ret) {
		    dev_dbg(&dev->dev
		            ,"unable to map unsupported effect %s to compatible effect: %d\n"
		            ,ff_name(effect->type), ret);
		    return ret;
		}
	}

	mutex_lock(&ff->mutex);

	if (effect->id == -1) {
		for (id = 0; id < ff->max_effects; id++)
			if (!ff->effect_owners[id])
				break;

		if (id >= ff->max_effects) {
			ret = -ENOSPC;
			goto out;
		}

		effect->id = id;
		old = NULL;

	} else {
		id = effect->id;

		ret = check_effect_access(ff, id, file);
		if (ret)
			goto out;

		old = &ff->effects[id];

		if (!check_effects_compatible(effect, old)) {
			ret = -EINVAL;
			goto out;
		}
	}

	ret = ff->upload(dev, effect, old);
	if (ret)
		goto out;

	spin_lock_irq(&dev->event_lock);
	ff->effects[id] = *effect;
	ff->effect_owners[id] = file;
	spin_unlock_irq(&dev->event_lock);

 out:
	mutex_unlock(&ff->mutex);
	debug_print_effect(dev,effect);
	if(ret) {
	    dev_dbg(&dev->dev,"ret=%d\n",ret);
	    if(old != NULL) {
	        debug_print_effect(dev,old);
	    }
	}
	return ret;
}
EXPORT_SYMBOL_GPL(input_ff_upload);

/*
 * Erases the effect if the requester is also the effect owner. The mutex
 * should already be locked before calling this function.
 */
static int erase_effect(struct input_dev *dev, int effect_id,
			struct file *file)
{
	struct ff_device *ff = dev->ff;
	int error;

	dev_dbg(&dev->dev,"Erasing effect %d\n",effect_id);
	error = check_effect_access(ff, effect_id, file);
	if (error)
		return error;

	spin_lock_irq(&dev->event_lock);
	ff->playback(dev, effect_id, 0);
	ff->effect_owners[effect_id] = NULL;
	spin_unlock_irq(&dev->event_lock);

	if (ff->erase) {
		error = ff->erase(dev, effect_id);
		if (error) {
			spin_lock_irq(&dev->event_lock);
			ff->effect_owners[effect_id] = file;
			spin_unlock_irq(&dev->event_lock);

			return error;
		}
	}

	return 0;
}

/**
 * input_ff_erase - erase a force-feedback effect from device
 * @dev: input device to erase effect from
 * @effect_id: id of the ffect to be erased
 * @file: purported owner of the request
 *
 * This function erases a force-feedback effect from specified device.
 * The effect will only be erased if it was uploaded through the same
 * file handle that is requesting erase.
 */
int input_ff_erase(struct input_dev *dev, int effect_id, struct file *file)
{
	struct ff_device *ff = dev->ff;
	int ret;

	if (!test_bit(EV_FF, dev->evbit))
		return -ENOSYS;

	mutex_lock(&ff->mutex);
	ret = erase_effect(dev, effect_id, file);
	mutex_unlock(&ff->mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(input_ff_erase);

/*
 * flush_effects - erase all effects owned by a file handle
 */
static int flush_effects(struct input_dev *dev, struct file *file)
{
	struct ff_device *ff = dev->ff;
	int i;

	dev_dbg(&dev->dev, "flushing now\n");

	mutex_lock(&ff->mutex);

	for (i = 0; i < ff->max_effects; i++)
		erase_effect(dev, i, file);

	mutex_unlock(&ff->mutex);

	return 0;
}

/**
 * input_ff_event() - generic handler for force-feedback events
 * @dev: input device to send the effect to
 * @type: event type (anything but EV_FF is ignored)
 * @code: event code
 * @value: event value
 */
int input_ff_event(struct input_dev *dev, unsigned int type,
		   unsigned int code, int value)
{
	struct ff_device *ff = dev->ff;

    dev_dbg(&dev->dev,"Set %s to %d\n",ff_name(code),value);
	if (type != EV_FF)
		return 0;

	switch (code) {
	case FF_GAIN:
		if (!test_bit(FF_GAIN, dev->ffbit) || value > 0xffff)
			break;

		ff->set_gain(dev, value);
		break;

	case FF_AUTOCENTER:
		if (!test_bit(FF_AUTOCENTER, dev->ffbit) || value > 0xffff)
			break;

		ff->set_autocenter(dev, value);
		break;

	default:
		if (check_effect_access(ff, code, NULL) == 0)
			ff->playback(dev, code, value);
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(input_ff_event);

/**
 * input_ff_create() - create force-feedback device
 * @dev: input device supporting force-feedback
 * @max_effects: maximum number of effects supported by the device
 *
 * This function allocates all necessary memory for a force feedback
 * portion of an input device and installs all default handlers.
 * @dev->ffbit should be already set up before calling this function.
 * Once ff device is created you need to setup its upload, erase,
 * playback and other handlers before registering input device
 */
int input_ff_create(struct input_dev *dev, unsigned int max_effects)
{
	struct ff_device *ff;
	size_t ff_dev_size;
	int i;

	if (!max_effects) {
		dev_err(&dev->dev, "cannot allocate device without any effects\n");
		return -EINVAL;
	}

	ff_dev_size = sizeof(struct ff_device) +
				max_effects * sizeof(struct file *);
	if (ff_dev_size < max_effects) /* overflow */
		return -EINVAL;

	ff = kzalloc(ff_dev_size, GFP_KERNEL);
	if (!ff)
		return -ENOMEM;

	ff->effects = kcalloc(max_effects, sizeof(struct ff_effect),
			      GFP_KERNEL);
	if (!ff->effects) {
		kfree(ff);
		return -ENOMEM;
	}

	ff->max_effects = max_effects;
	mutex_init(&ff->mutex);

	dev->ff = ff;
	dev->flush = flush_effects;
	dev->event = input_ff_event;
	__set_bit(EV_FF, dev->evbit);

	/* Copy "true" bits into ff device bitmap */
	for (i = 0; i <= FF_MAX; i++)
		if (test_bit(i, dev->ffbit))
			__set_bit(i, ff->ffbit);

	/* we can emulate RUMBLE with periodic effects */
	if (test_bit(FF_PERIODIC, ff->ffbit))
		__set_bit(FF_RUMBLE, dev->ffbit);

	return 0;
}
EXPORT_SYMBOL_GPL(input_ff_create);

/**
 * input_ff_destroy() - frees force feedback portion of input device
 * @dev: input device supporting force feedback
 *
 * This function is only needed in error path as input core will
 * automatically free force feedback structures when device is
 * destroyed.
 */
void input_ff_destroy(struct input_dev *dev)
{
	struct ff_device *ff = dev->ff;

	__clear_bit(EV_FF, dev->evbit);
	if (ff) {
		if (ff->destroy)
			ff->destroy(ff);
		kfree(ff->private);
		kfree(ff->effects);
		kfree(ff);
		dev->ff = NULL;
	}
}
EXPORT_SYMBOL_GPL(input_ff_destroy);
