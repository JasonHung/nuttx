/*
 * Copyright (c) 2015 Google, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include "rt5647.h"

// ignore unused warning message for current stage
#pragma GCC diagnostic ignored "-Wunused-function"

#define RT5647_CODEC_NAME   "rt5647"

#define AUDIO_ACCESS_RW (GB_AUDIO_ACCESS_READ | GB_AUDIO_ACCESS_WRITE)

struct rt5647_reg {
    uint8_t reg;
    uint16_t val;
};

struct rt5647_info {
    struct device *dev;
    uint8_t name[AUDIO_CODEC_NAME_MAX];

    struct gb_audio_dai *dai;

    struct gb_audio_control *controls;
    int num_controls;
    struct gb_audio_widget *widgets;
    int num_widgets;
    struct gb_audio_route *routes;
    int num_routes;
};

struct rt5647_reg rt5647_init_regs[] = {
    // {xxxx, xxxx},
};

struct gb_audio_dai rt5647_dai = {
    .name = "rt5647-aif1",
    .cport = 0,
    .capture = {
        .stream_name = "AIF1 Capture",
        .formats = RT5647_FORMATS,
        .rates = RT5647_STEREO_RATES,
        .chan_min = 1,
        .chan_max = 2,
    },
    .playback = {
        .stream_name = "AIF1 Playback",
        .formats = RT5647_FORMATS,
        .rates = RT5647_STEREO_RATES,
        .chan_min = 1,
        .chan_max = 2,
    },
};

struct gb_audio_control rt5647_controls[] = {
    CONTROL("I2S Switch", 0, MIXER, ALL, 0, BI), //SINGLE
    /* DAC Digital Volume */
    CONTROL("Mono DAC Playback Volume", 1, MIXER, ALL, 0, BI), //SOC_DOUBLE_TLV
    CONTROL("DAC2 Playback Switch", 2, MIXER, ALL, 0, BI), //SOC_DOUBLE
    CONTROL("DAC2 L source", 3, MIXER, ALL, 0, BI), //ENUM_SINGLE
    CONTROL("DAC2 R source", 4, MIXER, ALL, 0, BI), //ENUM_SINGLE
    /* Stereo DAC MIXL */
    CONTROL("DAC L1 Switch", 5, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("DAC L2 Switch", 6, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("DAC R1 Switch", 7, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("ANC L Switch", 8, MIXER, ALL, 0, BI), //SINGLE
    /* Stereo DAC MIXR */
    CONTROL("DAC R1 Switch", 9, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("DAC R2 Switch", 10, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("DAC L1 Switch", 11, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("ANC R Switch", 12, MIXER, ALL, 0, BI), //SINGLE
    /* DACs */
    CONTROL("DAC L1 Control", 13, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("DAC L2 Control", 14, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("DAC R1 Control", 15, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("DAC R2 Control", 16, MIXER, ALL, 0, BI), //SINGLE

    /* SPK MIXL */
    CONTROL("DAC L1 Switch", 17, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("DAC L2 Switch", 18, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("INL Switch", 19, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("BST3 L Switch", 20, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("BST1 L Switch", 21, MIXER, ALL, 0, BI), //SINGLE

    /* SPK MIXR */
    CONTROL("DAC R1 Switch", 22, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("DAC R2 Switch", 23, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("INR Switch", 24, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("BST3 R Switch", 25, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("BST2 R Switch", 26, MIXER, ALL, 0, BI), //SINGLE

    /* SPOL MIX */
    CONTROL("DAC R1 Switch", 27, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("DAC L1 Switch", 28, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("SPKVOL R Switch", 29, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("SPKVOL L Switch", 30, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("BST3 Switch", 31, MIXER, ALL, 0, BI), //SINGLE

    /* SPOR MIX */
    CONTROL("DAC R1 Switch", 32, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("SPKVOL R Switch", 33, MIXER, ALL, 0, BI), //SINGLE
    CONTROL("BST3 Switch", 34, MIXER, ALL, 0, BI), //SINGLE

    CONTROL("SPK Channel Switch", 35, MIXER, ALL, 0, BI), //SOC_DOUBLE
    CONTROL("SPO Playback Volume", 36, MIXER, ALL, 0, BI), //SOC_DOUBLE_TLV
    CONTROL("SPO Playback Switch", 37, MIXER, ALL, 0, BI), //SOC_DOUBLE
};

struct gb_audio_widget rt5647_widgets[] = {
    /* Audio Interface */
    WIDGET("AIF1RX", 0, INPUT, DISABLED, 0),
    WIDGET("AIF1TX", 0, OUTPUT, DISABLED, 0),
    /* Digital Interface */
    WIDGET("I2S1", 0, PGA, DISABLED, 0),
    WIDGET("IF1 DAC1", 0, PGA, DISABLED, 0),
    WIDGET("IF1 DAC2", 0, PGA, DISABLED, 0),
    WIDGET("IF1 DAC1 L", 0, PGA, DISABLED, 0),
    WIDGET("IF1 DAC1 R", 0, PGA, DISABLED, 0),
    WIDGET("IF1 DAC2 L", 0, PGA, DISABLED, 0),
    WIDGET("IF1 DAC2 R", 0, PGA, DISABLED, 0),
    WIDGET("IF1 ADC", 0, PGA, DISABLED, 0),
    /* DAC2 channel Mux */
    WIDGET("DAC L2 Mux", 0, MUX, DISABLED, 0),
    WIDGET("DAC R2 Mux", 0, MUX, DISABLED, 0),
    /* DAC Mixer */
    WIDGET("Stereo DAC MIXL", 0, MIXER, DISABLED, 0),
    WIDGET("Stereo DAC MIXR", 0, MIXER, DISABLED, 0),
    /* DACs */
    WIDGET("DAC L1", 0, DAC, DISABLED, 0),
    WIDGET("DAC L2", 0, DAC, DISABLED, 0),
    WIDGET("DAC R1", 0, DAC, DISABLED, 0),
    WIDGET("DAC R2", 0, DAC, DISABLED, 0),
    /* OUT Mixer */
    WIDGET("SPK MIXL", 0, MIXER, DISABLED, 0),
    WIDGET("SPK MIXR", 0, MIXER, DISABLED, 0),

    WIDGET("SPK amp", 0, PGA, DISABLED, 0),
    /* SPO Mixer */
    WIDGET("SPOL MIX", 0, MIXER, DISABLED, 0),
    WIDGET("SPOR MIX", 0, MIXER, DISABLED, 0),
    /* Output Lines */
    WIDGET("SPOL", 0, OUTPUT, DISABLED, 0),
    WIDGET("SPOR", 0, OUTPUT, DISABLED, 0),
};

struct gb_audio_route rt5647_routes[] = {
    // { src, dest, control },
};

/* TODO: copy from Linux source code temporarily,
 * we should modify this function by ourself.
 */
static int fls(int x)
{
    int r = 32;

    if (!x)
        return 0;
    if (!(x & 0xffff0000u)) {
        x <<= 16;
        r -= 16;
    }
    if (!(x & 0xff000000u)) {
        x <<= 8;
        r -= 8;
    }
    if (!(x & 0xf0000000u)) {
        x <<= 4;
        r -= 4;
    }
    if (!(x & 0xc0000000u)) {
        x <<= 2;
        r -= 2;
    }
    if (!(x & 0x80000000u)) {
        x <<= 1;
        r -= 1;
    }
    return r;
}

static uint32_t audcodec_read(uint32_t reg, uint32_t *value) {
    // read register via i2c
    return 0;
}

static uint32_t audcodec_write(uint32_t reg, uint32_t value) {
    // write register via i2c
    return 0;
}

static int audcodec_read_bit(struct bitctl *ctl, uint32_t *value)
{
    uint32_t reg = 0, inv = 0, shift = 0, data = 0;
    int ret = 0;

    if (!ctl || !value) {
        return -EINVAL;
    }

    reg = ctl->reg;
    shift = ctl->shift;
    inv = ctl->inv;

    ret = audcodec_read(reg, &data);
    if (ret) {
        return -EIO;
    }
    data = (data >> shift) & 0x01;
    if (inv) {
        data = (data)? 0: 1;
    }

    // TODO: need to fill return value to 'value'.
    return 0;
}

static int audcodec_read_bits(struct bitctl *ctl, uint32_t *value)
{
    uint32_t reg1 = 0, reg2 = 0, inv = 0;
    uint32_t shift1 = 0, shift2 = 0;
    uint32_t data1 = 0, data2 = 0;
    int ret = 0;

    if (!ctl || !value) {
        return -EINVAL;
    }

    reg1 = ctl->reg;
    reg2 = ctl->reg2;
    shift1 = ctl->shift;
    shift2 = ctl->shift2;
    inv = ctl->inv;

    ret = audcodec_read(reg1, &data1);
    if (ret) {
        return -EIO;
    }
    data1 = (data1 >> shift1) & 0x01;
    if (inv) {
        data1 = (data1)? 0: 1;
    }
    if (((reg1 == reg2) && (shift1 != shift2)) || (reg1 != reg2)) {
        /* two controls */
        ret = audcodec_read(reg2, &data2);
        if (ret) {
            return -EIO;
        }
        data2 = (data2 >> shift2) & 0x01;
        if (inv) {
            data2 = (data2)? 0: 1;
        }
    }
    // TODO: need to fill return value to 'value'.
    return 0;
}

static int audcodec_read_value(struct bitctl *ctl, uint32_t *value)
{
    uint32_t reg1 = 0, reg2 = 0, inv = 0;
    uint32_t shift1 = 0, shift2 = 0;
    uint32_t max = 0, min = 0, mask = 0;
    uint32_t data1 = 0, data2 = 0;
    int ret = 0;

    if (!ctl || !value) {
        return -EINVAL;
    }

    reg1 = ctl->reg;
    reg2 = ctl->reg2;
    shift1 = ctl->shift;
    shift2 = ctl->shift2;
    inv = ctl->inv;
    max = ctl->max;
    min = ctl->min;
    mask = (1 << fls(max)) - 1;

    ret = audcodec_read(reg1, &data1);
    if (ret) {
        return -EIO;
    }
    data1 = (data1 >> shift1) & mask;
    if (inv) {
        data1 = max - data1;
    }
    data1 = data1 - min;

    if (((reg1 == reg2) && (shift1 != shift2)) || (reg1 != reg2)) {
        /* two controls */
        ret = audcodec_read(reg2, &data2);
        if (ret) {
            return -EIO;
        }
        data2 = (data2 >> shift2) & mask;
        if (inv) {
            data2 = max - data2;
        }
        data2 = data2 - min;
    }
    // TODO: need to fill return value to 'value'.
    return 0;
}

static int audcodec_read_enum_bits(struct enumctl *ctl, uint32_t *value)
{
    uint32_t reg1 = 0, reg2 = 0;
    uint32_t shift1 = 0, shift2 = 0;
    uint32_t mask = 0;
    uint32_t data1 = 0, data2 = 0;
    int ret = 0;

    if (!ctl || !value) {
        return -EINVAL;
    }

    reg1 = ctl->reg;
    reg2 = ctl->reg2;
    shift1 = ctl->shift;
    shift2 = ctl->shift2;
    mask = ctl->mask;

    ret = audcodec_read(reg1, &data1);
    if (ret) {
        return -EIO;
    }
    data1 = (data1 >> shift1) & mask;

    if (((reg1 == reg2) && (shift1 != shift2)) || (reg1 != reg2)) {
        /* two controls */
        ret = audcodec_read(reg2, &data2);
        if (ret) {
            return -EIO;
        }
        data2 = (data2 >> shift2) & mask;
    }
    // TODO: need to fill return value to 'value'.
    return 0;
}

static int rt5647_get_topology_size(struct device *dev, uint16_t *size)
{
    struct rt5647_info *info = NULL;
    int tpg_size = 0;

    if (!dev || !device_get_private(dev) || !size) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    tpg_size = sizeof(struct gb_audio_topology);
    tpg_size += sizeof(struct gb_audio_dai);
    tpg_size += info->num_controls * sizeof(struct gb_audio_control);
    tpg_size += info->num_widgets * sizeof(struct gb_audio_widget);
    tpg_size += info->num_routes * sizeof(struct gb_audio_route);

    *size = tpg_size;
    return 0;
}

static int rt5647_get_topology(struct device *dev,
                               struct gb_audio_topology *topology)
{
    struct rt5647_info *info = NULL;
    int len = 0;
    uint8_t *data = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    topology->num_dais = 1;
    topology->num_controls = info->num_controls;
    topology->num_widgets = info->num_widgets;
    topology->num_routes = info->num_routes;

    data = topology->data;
    /* fill dai object */
    len = sizeof(struct gb_audio_dai);
    memcpy(data, info->dai, len);
    data += len;

    /* fill audio control object */
    len = topology->num_controls * sizeof(struct gb_audio_control);
    memcpy(data, info->controls, len);
    data += len;

    /* fill audio widget object */
    len = topology->num_widgets * sizeof(struct gb_audio_widget);
    memcpy(data, info->widgets, len);
    data += len;

    /* fill audio route object */
    len = topology->num_routes * sizeof(struct gb_audio_route);
    memcpy(data, info->routes, len);
    return 0;
}

static int rt5647_get_dai_config(struct device *dev,
                                 struct device_codec_dai_config *dai_config)
{
    return 0;
}

static int rt5647_set_dai_config(struct device *dev,
                                 struct device_codec_dai_config *dai_config)
{
    return 0;
}

/*DAC2 L/R source*/ //MX-1B [6:4] [2:0]
static const char *rt5647_dac12_src[] = {
    "IF1 DAC", "IF2 DAC", "IF3 DAC", "Mono ADC", "VAD_ADC"
};
static const char *rt5647_dacr2_src[] = {
    "IF1 DAC", "IF2 DAC", "IF3 DAC", "Mono ADC", "Haptic"
};

static int rt5647_get_control(struct device *dev, uint8_t control_id,
                              uint32_t *value)
{
    int ret = 0;
    struct bitctl ctl;
    struct enumctl e;

    /* TODO: control_id should define as '#define ctl_xxx' or enum ctl_xxx. */
    switch (control_id) {
        case 0: /* I2S Switch */
            BITCTL(ctl, RT5647_PWR_DIG1, RT5647_PWR_I2S1_BIT, 0);
            audcodec_read_bit(&ctl, value);
        break;
        case 1: /* Mono DAC Playback Volume */
            RANGCTL(ctl, RT5647_DAC2_DIG_VOL, RT5647_L_VOL_SFT,
                    RT5647_R_VOL_SFT, 0, 175, 0); // TODO: TLV
            audcodec_read_value(&ctl, value);
        break;
        case 2: /* DAC2 Playback Switch */
            BITSCTL(ctl, RT5647_DAC_CTRL, RT5647_M_DAC_L2_VOL_SFT,
                    RT5647_M_DAC_R2_VOL_SFT, 1);
            audcodec_read_bits(&ctl, value);
        break;
        case 3: /* DAC2 L source */
            ENUMCTL(e, RT5647_DAC_CTRL, RT5647_DAC2_L_SEL_SFT, 5, 0x7,
                    rt5647_dac12_src);
            audcodec_read_enum_bits(&e, value);
        break;
        case 4: /* DAC2 R source */
            ENUMCTL(e, RT5647_DAC_CTRL, RT5647_DAC2_R_SEL_SFT, 5, 0x7,
                    rt5647_dacr2_src);
            audcodec_read_enum_bits(&e, value);
        break;
        case 5: /* DAC L1 Switch */
            BITCTL(ctl, RT5647_STO_DAC_MIXER, RT5647_M_DAC_L1_SFT, 1);
            audcodec_read_bit(&ctl, value);
        break;
        case 6: /* DAC L2 Switch */
            BITCTL(ctl, RT5647_STO_DAC_MIXER, RT5647_M_DAC_L2_SFT, 1);
            audcodec_read_bit(&ctl, value);
        break;
        case 7: /* DAC R1 Switch */
            BITCTL(ctl, RT5647_STO_DAC_MIXER, RT5647_M_DAC_R1_STO_L_SFT, 1);
            audcodec_read_bit(&ctl, value);
        break;
        case 8: /* ANC L Switch */
            BITCTL(ctl, RT5647_STO_DAC_MIXER, RT5647_M_ANC_DAC_L_SFT, 1);
            audcodec_read_bit(&ctl, value);
        break;
        case 9: /* DAC R1 Switch */
            BITCTL(ctl, RT5647_STO_DAC_MIXER, RT5647_M_DAC_R1_SFT, 1);
            audcodec_read_bit(&ctl, value);
        break;
        case 10: /* DAC R2 Switch */
            BITCTL(ctl, RT5647_STO_DAC_MIXER, RT5647_M_DAC_R2_SFT, 1);
            audcodec_read_bit(&ctl, value);
        break;
        case 11: /* DAC L1 Switch */
            BITCTL(ctl, RT5647_STO_DAC_MIXER, RT5647_M_DAC_L1_STO_R_SFT, 1);
            audcodec_read_bit(&ctl, value);
        break;
        case 12: /* ANC R Switch */
            BITCTL(ctl, RT5647_STO_DAC_MIXER, RT5647_M_ANC_DAC_R_SFT, 1);
            audcodec_read_bit(&ctl, value);
        break;
        case 13: /* DAC L1 Control */
            BITCTL(ctl, RT5647_PWR_DIG1, RT5647_PWR_DAC_L1_BIT, 0);
            audcodec_read_bit(&ctl, value);
        break;
        case 14: /* DAC L2 Control */
            BITCTL(ctl, RT5647_PWR_DIG1, RT5647_PWR_DAC_L2_BIT, 0);
            audcodec_read_bit(&ctl, value);
        break;
        case 15: /* DAC R1 Control */
            BITCTL(ctl, RT5647_PWR_DIG1, RT5647_PWR_DAC_R1_BIT, 0);
            audcodec_read_bit(&ctl, value);
        break;
        case 16: /* DAC R2 Control */
            BITCTL(ctl, RT5647_PWR_DIG1, RT5647_PWR_DAC_R2_BIT, 0);
            audcodec_read_bit(&ctl, value);
        break;
    }
    return ret;
}

static int rt5647_set_control(struct device *dev, uint8_t control_id,
                              uint32_t value)
{
    int ret = 0;
    struct bitctl ctl;

    /* TODO: control_id should define as '#define ctl_xxx' or enum ctl_xxx. */
    switch (control_id) {
        case 0:

        break;
    }
    return ret;
}

static int rt5647_enable_widget(struct device *dev, uint8_t widget_id)
{
    switch (widget_id) {
        case 0:
        break;
    }
    return 0;
}

static int rt5647_disable_widget(struct device *dev, uint8_t widget_id)
{
    return 0;
}

static int rt5647_get_tx_delay(struct device *dev, uint32_t *delay)
{
    return 0;
}

static int rt5647_start_tx(struct device *dev)
{
    return 0;
}

static int rt5647_stop_tx(struct device *dev)
{
    return 0;
}

static int rt5647_register_tx_callback(struct device *dev,
                                       device_codec_event_callback *callback)
{
    return 0;
}

static int rt5647_get_rx_delay(struct device *dev, uint32_t *delay)
{
    return 0;
}

static int rt5647_start_rx(struct device *dev)
{
    return 0;
}

static int rt5647_stop_rx(struct device *dev)
{
    return 0;
}

static int rt5647_register_rx_callback(struct device *dev,
                                       device_codec_event_callback *callback)
{
    return 0;
}

static int rt5647_audcodec_open(struct device *dev)
{
    struct rt5647_info *info = NULL;
    int ret = 0, size = 0, i = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    size = ARRAY_SIZE(rt5647_init_regs);
    for (i = 0; i < size; i++) {
        audcodec_write(rt5647_init_regs[i].reg , rt5647_init_regs[i].val);
    }
    return ret;
}

static void rt5647_audcodec_close(struct device *dev)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);
}

static int rt5647_audcodec_probe(struct device *dev)
{
    struct rt5647_info *info;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->dev = dev;
    strcpy((char*)info->name, RT5647_CODEC_NAME);

    info->dai = &rt5647_dai;
    info->controls = rt5647_controls;
    info->num_controls = ARRAY_SIZE(rt5647_controls);
    info->widgets = rt5647_widgets;
    info->num_widgets = ARRAY_SIZE(rt5647_widgets);
    info->routes = rt5647_routes;
    info->num_routes = ARRAY_SIZE(rt5647_routes);

    device_set_private(dev, info);
    return 0;
}

static void rt5647_audcodec_remove(struct device *dev)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    device_set_private(dev, NULL);
    free(info);
}

static struct device_codec_type_ops rt5647_audcodec_type_ops = {
    .get_topology_size = rt5647_get_topology_size,
    .get_topology = rt5647_get_topology,
    .get_dai_config = rt5647_get_dai_config,
    .set_dai_config = rt5647_set_dai_config,
    .get_control = rt5647_get_control,
    .set_control = rt5647_set_control,
    .enable_widget = rt5647_enable_widget,
    .disable_widget = rt5647_disable_widget,
    .get_tx_delay = rt5647_get_tx_delay,
    .start_tx = rt5647_start_tx,
    .stop_tx = rt5647_stop_tx,
    .register_tx_callback = rt5647_register_tx_callback,
    .get_rx_delay = rt5647_get_rx_delay,
    .start_rx = rt5647_start_rx,
    .stop_rx = rt5647_stop_rx,
    .register_rx_callback = rt5647_register_rx_callback,
};

static struct device_driver_ops rt5647_audcodec_ops = {
    .probe          = rt5647_audcodec_probe,
    .remove         = rt5647_audcodec_remove,
    .open           = rt5647_audcodec_open,
    .close          = rt5647_audcodec_close,
    .type_ops       = &rt5647_audcodec_type_ops,
};

struct device_driver rt5647_audcodec = {
    .type       = DEVICE_TYPE_CODEC_HW,
    .name       = "rt5647",
    .desc       = "ALC5647 Audio Codec driver",
    .ops        = &rt5647_audcodec_ops,
};
