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
#include <nuttx/device.h>
#include <nuttx/device_spi.h>

#include "up_arch.h"
#include "tsb_scm.h"

#define DW_SPI_CTRLR0   0x00
#define DW_SPI_CTRLR1   0x04
#define DW_SPI_SSIENR   0x08
#define DW_SPI_SER      0x10
#define DW_SPI_BAUDR    0x14
#define DW_SPI_TXFTLR   0x18
#define DW_SPI_RXFTLR   0x1C
#define DW_SPI_TXFLR    0x20
#define DW_SPI_RXFLR    0x24
#define DW_SPI_SR       0x28
#define DW_SPI_IMR      0x2C
#define DW_SPI_ISR      0x30
#define DW_SPI_DMACR    0x4C
#define DW_SPI_DMATDLR  0x50
#define DW_SPI_DMARDLR  0x54
#define DW_SPI_DR       0x60

/** bit for DW_SPI_CTRLR0 */
#define SPI_CTRLR0_SCPH         BIT(6)
#define SPI_CTRLR0_SCPOL        BIT(7)
#define SPI_CTRL0_TMOD_MASK     (0x03 << 8)
#define SPI_CTRLR0_DFS32_OFFSET 16
#define SPI_CTRLR0_DFS32_MASK   (0x1f << SPI_CTRLR0_DFS32_OFFSET)

/** bit for DW_SPI_SR */
#define SPI_SR_TFNF_MASK    BIT(1)
#define SPI_SR_TFE_MASK     BIT(2)
#define SPI_SR_RFNF_MASK    BIT(3)
#define SPI_SR_RFF_MASK     BIT(4)

/** bit for DW_SPI_IMR */
#define SPI_IMR_TXOIM_MASK  BIT(1)
#define SPI_IMR_RXFIM_MASK  BIT(4)

/** bit for DW_SPIISR */
#define SPI_ISR_RXFIS_MASK  BIT(4)

#define COMMAND_INTERVAL    1000 /* 1ms */
#define TOLERANCE_TIME      2    /* 2ms timeout tolerance for an operation */

/**
 * SPI device state
 */
enum tsb_spi_state {
    TSB_SPI_STATE_INVALID,
    TSB_SPI_STATE_CLOSED,
    TSB_SPI_STATE_OPEN,
    TSB_SPI_STATE_LOCKED,
};

/**
 * Current RX transition state.
 */
struct xfer_curr_info {
    /** Select bits per word for this transfer */
    uint8_t cur_bpw;

    /** Byte need to read */
    uint32_t rx_len;

    /** Currently RX buffer pointer */
    uint8_t *cur_rx;

    /** Flag for receive process checking */
    uint8_t read_done;

    /** flag for stop receive processing */
    uint8_t stop_read;
};

/**
 * Specific parameter for each Chip of tranfer operation, Vendor can modify it
 * for connected SPI device.
 */
struct device_spi_cfg chips_info[CONFIG_SPI_MAX_CHIPS] = {
    {SPI_MODE_CPHA, 32, CONFIG_SPI_MAX_FREQ, "spidev"},
    {SPI_MODE_CPOL,  8, CONFIG_SPI_MIN_FREQ, "spidev"}
};

/**
 * @brief private SPI device information
 */
struct tsb_spi_info {
    /** Driver model representation of the device */
    struct device *dev;

    /** SPI device base address */
    uint32_t reg_base;

    /** SPI device state */
    enum tsb_spi_state state;

    /** struct for currectly transfer information store */
    struct xfer_curr_info curr_xfer;

    /** struct for SPI controller capability store */
    struct master_spi_caps caps;

    /** struct for chips configuration store */
    struct device_spi_cfg *dev_cfg;

    /** Exclusive access for SPI bus */
    sem_t bus;

    /** Exclusive access for operation */
    sem_t lock;

    /** RX thread notification flag */
    sem_t start_read;

    /** info the thread should be terminated */
    uint8_t thread_stop;

    /** Handler for read thread */
    pthread_t pthread_handler;

    /** TX FIFO depth for SPI controller */
    uint32_t tx_fifo_depth;

    /** RX FIFO depth for SPI controller */
    uint32_t rx_fifo_depth;
};


/**
 * @brief Read value from register.
 *
 * This function returns register content by basic register read function.
 *
 * @param base Base address of this Controller.
 * @param addr Specific register of offset.
 *
 * @return Returns content for a specific register.
 */
static uint32_t tsb_spi_read(uint32_t base, uint32_t addr)
{
    return getreg32(base + addr);
}

/**
 * @brief Write value to register.
 *
 * This function write value to register by basic register write function.
 *
 * @param base Base address of this Controller.
 * @param addr Specific register of offset.
 * @param val The content will be write for Specific register.
 */
static void tsb_spi_write(uint32_t base, uint32_t addr, uint32_t val)
{
    putreg32(val, base + addr);
}

/**
 * @brief Lock SPI bus for exclusive access
 *
 * On SPI buses where there are multiple devices, it will be necessary to lock
 * SPI to have exclusive access to the buses for a sequence of transfers.
 * The bus should be locked before the chip is selected. After locking the SPI
 * bus, the caller should then also call the setfrequency(), setbits() , and
 * setmode() methods to make sure that the SPI is properly configured for the
 * device. If the SPI buses is being shared, then it may have been left in an
 * incompatible state.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_lock(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    /* Take the semaphore (perhaps waiting) */
    ret = sem_wait(&info->bus);
    if (ret != OK) {
        /* The sem_wait() call should fail only if we are awakened by
         * a signal.
         */
        return -get_errno();
    }
    info->state = TSB_SPI_STATE_LOCKED;

    return 0;
}

/**
 * @brief unlock SPI bus for exclusive access
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_unlock(struct device *dev)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    info->state = TSB_SPI_STATE_OPEN;
    sem_post(&info->bus);
    return 0;
}

/**
 * @brief Enable the SPI chip select pin
 *
 * The implementation of this method must include handshaking. If a device is
 * selected, it must hold off all the other attempts to select the device
 * until the device is deselected. This function should be called after lock(),
 * if the driver isn’t in lock state, it returns an error code to notify a
 * problem.
 *
 * @param dev pointer to structure of device data
 * @param devid identifier of a selected SPI slave device
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_select(struct device *dev, uint8_t devid)
{
    struct tsb_spi_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_select;
    }

    if (info->caps.csnum <= devid) {
        ret = -EINVAL;
        goto err_select;
    }

    tsb_spi_write(info->reg_base, DW_SPI_SER, (1 << devid));

err_select:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Disable the SPI chip select pin
 *
 * The implementation of this method must include handshaking. If a device is
 * selected, it must hold off all the other attempts to select the device
 * until the device is deselected. This function should be called after lock(),
 * if the driver isn’t in lock state, it returns an error code to notify a
 * problem.
 *
 * @param dev pointer to structure of device data
 * @param devid identifier of a selected SPI slave device
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_deselect(struct device *dev, uint8_t devid)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        sem_post(&info->lock);
        return -EPERM;
    }

    tsb_spi_write(info->reg_base, DW_SPI_SER, 0);

    sem_post(&info->lock);
    return 0;
}

/**
 * @brief Configure SPI clock.
 *
 * If SPI hardware doesn’t support this frequency value, this function should
 * find the nearest lower frequency in which hardware supported and then
 * configure SPI clock to this value. It will return the actual frequency
 * selected value back to the caller via parameter frequency.
 * This function should be called after lock(), if the driver is not in lock
 * state, it returns an error code to notify a problem.
 *
 * @param dev pointer to structure of device data
 * @param frequency SPI frequency requested (unit: Hz)
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setfrequency(struct device *dev, uint8_t cs,
                                uint32_t *frequency)
{
    struct tsb_spi_info *info = NULL;
    uint32_t freq;
    uint32_t div;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !frequency) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->caps.csnum <= cs) {
        ret = -EINVAL;
        goto err_freq_set;
    }

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_freq_set;
    }

    freq = *frequency;

    if (freq < CONFIG_SPI_MIN_FREQ || freq > info->dev_cfg[cs].max_speed_hz ||
        freq == 0) {
        ret = -EINVAL;
        goto err_freq_set;
    }

    div = CONFIG_SPI_MAX_FREQ / freq;

    if (div > CONFIG_SPI_MAX_DIV) {
        ret = -EINVAL;
        goto err_freq_set;
    }

    tsb_spi_write(info->reg_base, DW_SPI_BAUDR, div);

err_freq_set:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Configure SPI mode.
 *
 * To configure SPI configuration such as clock polarity and phase via the mode
 * parameter. Other possible definition of SPI mode can be found in SPI mode
 * definition. If the value of mode parameter is out of SPI mode definition or
 * this mode isn’t supported by the current hardware, this function should
 * return -EOPNOTSUPP error code.
 * This function should be called after lock(), if driver is not in lock state,
 * function returns -EPERM error code.
 *
 * @param dev pointer to structure of device data
 * @param mode SPI protocol mode requested
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setmode(struct device *dev, uint8_t cs, uint8_t mode)
{
    struct tsb_spi_info *info = NULL;
    uint32_t ctrl0 = 0;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->caps.csnum <= cs) {
        ret = -EINVAL;
        goto err_setmode;
    }

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_setmode;
    }

    /* check mode whether supported */
    if (info->dev_cfg[cs].mode & ~mode) {
        ret = -EINVAL;
        goto err_setmode;
    }

    ctrl0 = tsb_spi_read(info->reg_base, DW_SPI_CTRLR0);

    if (mode & SPI_MODE_CPHA)
        ctrl0 |= SPI_CTRLR0_SCPH;
    else
        ctrl0 &= ~SPI_CTRLR0_SCPH;

    if (mode & SPI_MODE_CPOL)
        ctrl0 |= SPI_CTRLR0_SCPOL;
    else
        ctrl0 &= ~SPI_CTRLR0_SCPOL;

    tsb_spi_write(info->reg_base, DW_SPI_CTRLR0, ctrl0);

err_setmode:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Set the number of bits per word in transmission.
 *
 * This function should be called after lock(), if driver is not in lock state,
 * this function returns -EPERM error code.
 *
 * @param dev pointer to structure of device data
 * @param nbits The number of bits requested. The nbits value range is from
 *        4 to 32.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setbits(struct device *dev, uint8_t cs, uint8_t nbits)
{
    struct tsb_spi_info *info = NULL;
    uint16_t ctrl0 = 0;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->caps.csnum <= cs) {
        ret = -EINVAL;
        goto exit_setbit;
    }

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto exit_setbit;
    }

    if (info->dev_cfg[cs].bpw != nbits) {
        ret = -EINVAL;
        goto exit_setbit;
    }

    ctrl0 = tsb_spi_read(info->reg_base, DW_SPI_CTRLR0);
    ctrl0 &= ~SPI_CTRLR0_DFS32_MASK;
    ctrl0 |= ((nbits - 1) << SPI_CTRLR0_DFS32_OFFSET);

    tsb_spi_write(info->reg_base, DW_SPI_CTRLR0, ctrl0);

    info->curr_xfer.cur_bpw = nbits;

exit_setbit:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Exchange a block of data from SPI
 *
 * Device driver uses this function to transfer and receive data from SPI bus.
 * This function should be called after lock() , if the driver is not in lock
 * state, it returns -EPERM error code.
 * The transfer structure is consists of the read/write buffer, transfer
 * length, transfer flags and callback function.
 *
 * @param dev pointer to structure of device data
 * @param transfer pointer to the spi transfer request
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_exchange(struct device *dev,
                             struct device_spi_transfer *transfer)
{
    struct tsb_spi_info *info = NULL;
    uint16_t u16_dr, time_out = 0;
    uint32_t u32_dr, sr_reg;
    uint8_t *txbuf = NULL;
    uint8_t u8_dr;
    int i = 0, ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !transfer) {
        return -EINVAL;
    }

    /* check transfer buffer */
    if (!transfer->txbuffer && !transfer->rxbuffer) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_unlock;
    }

    if (transfer->txbuffer) {
        txbuf = transfer->txbuffer;
    }

    if (transfer->rxbuffer) {
        info->curr_xfer.cur_rx = transfer->rxbuffer;
    } else {
        info->curr_xfer.cur_rx = NULL;
    }

    /* event has read request, we still need to read data because SPI data
     * exchange behavior */
    info->curr_xfer.rx_len = transfer->nwords;
    info->curr_xfer.read_done = 0;
    info->curr_xfer.stop_read = 0;
    /* enable Rx FIFO-full interrupt */
    tsb_spi_write(info->reg_base, DW_SPI_IMR,
                  (tsb_spi_read(info->reg_base, DW_SPI_IMR) |
                   SPI_IMR_RXFIM_MASK));
    /* Active read thread */
    sem_post(&info->start_read);

    sr_reg = tsb_spi_read(info->reg_base, DW_SPI_SR);

    for (i = 0; i < transfer->nwords && txbuf;) {
        if ((sr_reg & SPI_SR_TFNF_MASK) && !(sr_reg & SPI_SR_RFF_MASK)) {
            if (info->curr_xfer.cur_bpw <= 8) {
                u8_dr = (txbuf) ? *(txbuf++) : 0;
                tsb_spi_write(info->reg_base, DW_SPI_DR, u8_dr);
                i++;
            } else if (info->curr_xfer.cur_bpw <= 16) {
                u16_dr = (txbuf) ? *(uint16_t *)txbuf : 0;
                txbuf += 2;
                tsb_spi_write(info->reg_base, DW_SPI_DR, u16_dr);
                i += 2;
            } else {
                u32_dr = (txbuf) ? *(uint32_t *)txbuf : 0;
                txbuf += 4;
                tsb_spi_write(info->reg_base, DW_SPI_DR, u32_dr);
                i += 4;
            }
            /* reset timeout when each write success*/
            time_out = 0;
        } else {
            usleep(COMMAND_INTERVAL);
            time_out++;
            if (time_out > TOLERANCE_TIME) {
                /* force stop read processing if it is activated */
                info->curr_xfer.stop_read = 1;
                ret = -ETIMEDOUT;
                goto err_unlock;
            }
        }
        sr_reg = tsb_spi_read(info->reg_base, DW_SPI_SR);
    }

    /* Even this op has not read buffer, we still need to wait dummy data
     * exchange was completed.
     */
    if (!info->curr_xfer.read_done) {
        while (1) {
            /* Has read process but not complete yet */
            usleep(COMMAND_INTERVAL);
            time_out++;
            if (time_out > TOLERANCE_TIME) {
                /* force stop read process because timeout */
                info->curr_xfer.stop_read = 1;
                ret = -ETIMEDOUT;
                break;
            }
        }
    }

    tsb_spi_write(info->reg_base, DW_SPI_IMR,
                  (tsb_spi_read(info->reg_base, DW_SPI_IMR) &
                  ~SPI_IMR_RXFIM_MASK));
err_unlock:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief SPI interrupt handler
 *
 * @param irq interrupt number
 * @param context argument for interrupt handler
 * @return 0 if successful, negative error code otherwise.
 */
static int tsb_spi_irq_handler(int irq, void *context)
{
    struct tsb_spi_info *info = (struct tsb_spi_info *)context;
    uint32_t isr_reg;

    isr_reg = tsb_spi_read(info->reg_base, DW_SPI_SR);

    if (isr_reg & SPI_ISR_RXFIS_MASK) {
        info->curr_xfer.stop_read = 1;
    }

    return 0;
}

/**
 * @brief Get SPI device driver hardware capabilities information.
 *
 * This function can be called whether lock() has been called or not.
 *
 * @param dev pointer to structure of device data
 * @param caps pointer to the spi_caps structure to receive the capabilities
 *             information.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_getcaps(struct device *dev, struct master_spi_caps *caps)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !caps) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    caps->modes = info->caps.modes;
    caps->flags = info->caps.flags;
    caps->bpw = info->caps.bpw;
    caps->csnum = info->caps.csnum;
    caps->min_speed_hz = info->caps.min_speed_hz;
    caps->max_speed_hz = info->caps.max_speed_hz;
    sem_post(&info->lock);
    return 0;
}

/**
 * @brief Get SPI specific chip configured information.
 *
 * This function can be called whether lock() has been called or not.
 *
 * @param dev pointer to structure of device data
 * @param cs the specific chip number
 * @param dev_cfg pointer to the device_spi_cfg structure to receive the
 *                configuration that be set in chip.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_get_cfg(struct device *dev, uint8_t cs,
                           struct device_spi_cfg *dev_cfg)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !dev_cfg) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    dev_cfg->mode = info->dev_cfg[cs].mode;
    dev_cfg->bpw = info->dev_cfg[cs].bpw;
    dev_cfg->max_speed_hz = info->dev_cfg[cs].max_speed_hz;
    memcpy(dev_cfg->name, &info->dev_cfg[cs].name,
           sizeof(info->dev_cfg[cs].name));
    sem_post(&info->lock);
    return 0;
}

/**
 * @brief Deinitialize SPI controller
 *
 * @param info pointer to the tsb_spi_info struct.
 */
static void tsb_spi_hw_deinit(struct tsb_spi_info *info)
{
    /* Disable SPI controller */
    tsb_spi_write(info->reg_base, DW_SPI_SSIENR, 0);

    /* Release pinshare for SPI */
    tsb_release_pinshare(TSB_PIN_GPIO13 | TSB_PIN_SPIM_CS1);

    tsb_clk_disable(TSB_CLK_SPIP);
    tsb_clk_disable(TSB_CLK_SPIS);
}

/**
 * @brief Initialize SPI controller
 *
 * @param info pointer to the tsb_spi_info struct.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_hw_init(struct tsb_spi_info *info)
{
    uint32_t ctrl0;
    int ret = 0;

    /* Enable Clock */
    tsb_clk_enable(TSB_CLK_SPIP);
    tsb_clk_enable(TSB_CLK_SPIS);

    /* Reset */
    tsb_reset(TSB_CLK_SPIP);
    tsb_reset(TSB_CLK_SPIS);

    /* Disable SPI controller */
    tsb_spi_write(info->reg_base, DW_SPI_SSIENR, 0);

    /* Disable SPI all interrupts */
    tsb_spi_write(info->reg_base, DW_SPI_IMR, 0);

    /* Enable SPI controller */
    tsb_spi_write(info->reg_base, DW_SPI_SSIENR, 1);

    tsb_spi_write(info->reg_base, DW_SPI_TXFTLR, info->tx_fifo_depth);

    /* The SPI support both transmit and receiver mode */
    ctrl0 = tsb_spi_read(info->reg_base, DW_SPI_CTRLR0);
    ctrl0 &= ~SPI_CTRL0_TMOD_MASK;
    tsb_spi_write(info->reg_base, DW_SPI_CTRLR0, ctrl0);

    /* Check pinshare for SPIM pins */
    ret = tsb_request_pinshare(TSB_PIN_GPIO13 |TSB_PIN_SPIM_CS1);
    if (ret) {
        lowsyslog("SPI: cannot get ownership of SPI pins\n");
        goto err_req_pinshare;
    }

    /* Configure pin functionality for SPI */
    tsb_clr_pinshare(TSB_PIN_GPIO13);
    tsb_set_pinshare(TSB_PIN_SPIM_CS1);

err_req_pinshare:
    return ret;
}

/**
 * @brief Receive processing thread
 *
 * @param dev Pointer to structure of device data
 */
static void *tsb_spi_read_thread(void *data)
{
    struct tsb_spi_info *info = (struct tsb_spi_info *) data;
    int i = 0;
    uint8_t u8_dr;
    uint16_t u16_dr;
    uint32_t u32_dr, sr_reg;
    uint8_t *rxbuf = NULL;

    if (!info) {
        return NULL;
    }

    while (1) {
        sem_wait(&info->start_read);

        if (info->thread_stop) {
            break;
        }

        if (info->curr_xfer.cur_rx) {
            rxbuf = info->curr_xfer.cur_rx;
        }

        sr_reg = tsb_spi_read(info->reg_base, DW_SPI_SR);

        for (i = 0; i < info->curr_xfer.rx_len;) {
            /* check if read process need to force stop */
            if (info->curr_xfer.stop_read || info->thread_stop) {
                break;
            }

            /* Read receiver fifo if not empty */
            if (sr_reg & SPI_SR_RFNF_MASK) {
                if (info->curr_xfer.cur_bpw <= 8) {
                    u8_dr = tsb_spi_read(info->reg_base, DW_SPI_DR);
                    if (rxbuf)
                        *rxbuf++ = u8_dr;
                    i++;
                } else if (info->curr_xfer.cur_bpw <= 16) {
                    u16_dr = tsb_spi_read(info->reg_base, DW_SPI_DR);
                    if (rxbuf) {
                        *(uint16_t *)rxbuf = u16_dr;
                        rxbuf += 2;
                    }
                    i += 2;
                } else {
                    u32_dr = tsb_spi_read(info->reg_base, DW_SPI_DR);
                    if (rxbuf) {
                        *(uint32_t *)rxbuf = u32_dr;
                        rxbuf += 4;
                    }
                    i += 4;
                }
            } else {
                usleep(COMMAND_INTERVAL);
            }
            sr_reg = tsb_spi_read(info->reg_base, DW_SPI_SR);
        }

        /* Set Read process was completed */
        if (i == info->curr_xfer.rx_len) {
            info->curr_xfer.read_done = 1;
        }
    }

    return NULL;
}

/**
 * @brief Open SPI device
 *
 * This function is called when the caller is preparing to use this device
 * driver. This function should be called after probe () function and need to
 * check whether the driver already open or not. If driver was opened, it needs
 * to return an error code to the caller to notify the driver was opened.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_dev_open(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    int ret = 0;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_CLOSED) {
        ret = -EBUSY;
        goto err_open;
    }

    ret = tsb_spi_hw_init(info);
    if (ret) {
        goto err_open;
    }

    /* Set Capability */
    info->caps.csnum = CONFIG_SPI_MAX_CHIPS;
    info->caps.modes = (SPI_MODE_CPHA | SPI_MODE_CPOL);

    /* support both transmit and receive */
    info->caps.flags = 0;

     /* support 4 to 32 bits */
    info->caps.bpw = CONFIG_SPI_BPW_MASK;

    info->caps.min_speed_hz = CONFIG_SPI_MIN_FREQ;
    info->caps.max_speed_hz = CONFIG_SPI_MAX_FREQ;
    info->tx_fifo_depth = CONFIG_SPI_TX_DEPTH;
    info->rx_fifo_depth = CONFIG_SPI_RX_DEPTH;
    info->dev_cfg = chips_info;
    info->curr_xfer.read_done = 0;
    info->thread_stop = 0;

    ret = pthread_create(&info->pthread_handler, NULL,
                         tsb_spi_read_thread, info);
    if (ret) {
        goto err_open;
    }

    /* register SPI IRQ number */
    ret = irq_attach(TSB_IRQ_SPI, tsb_spi_irq_handler);
    if (ret != OK) {
        ret = -EIO;
        goto err_destory_thread;
    }

    up_enable_irq(TSB_IRQ_SPI);

    info->state = TSB_SPI_STATE_OPEN;

err_destory_thread:
    info->thread_stop = 1;
    sem_post(&info->start_read);
    pthread_join(info->pthread_handler, NULL);
err_open:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Close SPI device
 *
 * This function is called when the caller no longer using this driver. It
 * should release or close all resources that allocated by the open() function.
 * This function should be called after the open() function. If the device
 * is not opened yet, this function should return without any operations.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_spi_dev_close(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    irqstate_t flags;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    flags = irqsave();
    up_disable_irq(TSB_IRQ_SPI);
    irq_detach(TSB_IRQ_SPI);

    if (info->pthread_handler != (pthread_t)0) {
        info->thread_stop = 1;
        sem_post(&info->start_read);
        pthread_join(info->pthread_handler, NULL);
    }

    tsb_spi_hw_deinit(info);
    irqrestore(flags);

    info->state = TSB_SPI_STATE_CLOSED;
    sem_post(&info->lock);
}

/**
 * @brief Probe SPI device
 *
 * This function is called by the system to register the driver when the system
 * boot up. This function allocates memory for the private SPI device
 * information, and then setup the hardware resource and interrupt handler.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_dev_probe(struct device *dev)
{
    struct tsb_spi_info *info;
    struct device_resource *r;
    int ret = 0;

    if (tsb_get_rev_id() == tsb_rev_es2) {
        return -ENODEV;
    }

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    /* get register data from resource block */
    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REG, "reg_base");
    if (!r) {
        ret = -EINVAL;
        goto err_freemem;
    }

    info->reg_base = (uint32_t)r->start;
    info->dev = dev;
    info->state = TSB_SPI_STATE_CLOSED;
    device_set_private(dev, info);

    sem_init(&info->bus, 0, 1);
    sem_init(&info->lock, 0, 1);
    sem_init(&info->start_read, 0, 0);

    return 0;

err_freemem:
    free(info);
    return ret;
}

/**
 * @brief Remove SPI device
 *
 * This function is called by the system to unregister the driver. It should
 * release the hardware resource and interrupt setting, and then free memory
 * that allocated by the probe() function.
 * This function should be called after probe() function. If driver was opened,
 * this function should call close() function before releasing resources.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_spi_dev_remove(struct device *dev)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }

    info = device_get_private(dev);

    info->state = TSB_SPI_STATE_INVALID;
    sem_destroy(&info->lock);
    sem_destroy(&info->bus);
    sem_destroy(&info->start_read);
    device_set_private(dev, NULL);
    free(info);
}

static struct device_spi_type_ops tsb_spi_type_ops = {
    .lock               = tsb_spi_lock,
    .unlock             = tsb_spi_unlock,
    .select             = tsb_spi_select,
    .deselect           = tsb_spi_deselect,
    .setfrequency       = tsb_spi_setfrequency,
    .setmode            = tsb_spi_setmode,
    .setbits            = tsb_spi_setbits,
    .exchange           = tsb_spi_exchange,
    .get_master_caps    = tsb_spi_getcaps,
    .get_device_cfg     = tsb_spi_get_cfg,
};

static struct device_driver_ops tsb_spi_driver_ops = {
    .probe          = tsb_spi_dev_probe,
    .remove         = tsb_spi_dev_remove,
    .open           = tsb_spi_dev_open,
    .close          = tsb_spi_dev_close,
    .type_ops       = &tsb_spi_type_ops,
};

struct device_driver tsb_spi_driver = {
    .type       = DEVICE_TYPE_SPI_HW,
    .name       = "tsb_spi",
    .desc       = "TSB SPI Driver",
    .ops        = &tsb_spi_driver_ops,
};
