// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Spacemit
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/scatterlist.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>

#include "i2c-k1x.h"

static int spacemit_i2c_next_msg(struct spacemit_i2c_dev *spacemit_i2c);
static int spacemit_i2c_xfer_msg(struct spacemit_i2c_dev *spacemit_i2c);

static inline u32 spacemit_i2c_read_reg(struct spacemit_i2c_dev *spacemit_i2c, int reg)
{
	return readl(spacemit_i2c->mapbase + reg);
}

static inline void
spacemit_i2c_write_reg(struct spacemit_i2c_dev *spacemit_i2c, int reg, u32 val)
{
	writel(val, spacemit_i2c->mapbase + reg);
}

static void spacemit_i2c_enable(struct spacemit_i2c_dev *spacemit_i2c)
{
	spacemit_i2c_write_reg(spacemit_i2c, REG_CR,
	spacemit_i2c_read_reg(spacemit_i2c, REG_CR) | CR_IUE);
}

static void spacemit_i2c_disable(struct spacemit_i2c_dev *spacemit_i2c)
{
	spacemit_i2c->i2c_ctrl_reg_value = spacemit_i2c_read_reg(spacemit_i2c, REG_CR) & ~CR_IUE;
	spacemit_i2c_write_reg(spacemit_i2c, REG_CR, spacemit_i2c->i2c_ctrl_reg_value);
}

static void spacemit_i2c_flush_fifo_buffer(struct spacemit_i2c_dev *spacemit_i2c)
{
	/* flush REG_WFIFO_WPTR and REG_WFIFO_RPTR */
	spacemit_i2c_write_reg(spacemit_i2c, REG_WFIFO_WPTR, 0);
	spacemit_i2c_write_reg(spacemit_i2c, REG_WFIFO_RPTR, 0);

	/* flush REG_RFIFO_WPTR and REG_RFIFO_RPTR */
	spacemit_i2c_write_reg(spacemit_i2c, REG_RFIFO_WPTR, 0);
	spacemit_i2c_write_reg(spacemit_i2c, REG_RFIFO_RPTR, 0);
}

static void spacemit_i2c_controller_reset(struct spacemit_i2c_dev *spacemit_i2c)
{
	/* i2c controller reset */
	spacemit_i2c_write_reg(spacemit_i2c, REG_CR, CR_UR);
	udelay(5);
	spacemit_i2c_write_reg(spacemit_i2c, REG_CR, 0);

	/* set load counter register */
	if (spacemit_i2c->i2c_lcr)
		spacemit_i2c_write_reg(spacemit_i2c, REG_LCR, spacemit_i2c->i2c_lcr);

	/* set wait counter register */
	if (spacemit_i2c->i2c_wcr)
		spacemit_i2c_write_reg(spacemit_i2c, REG_WCR, spacemit_i2c->i2c_wcr);
}

static void spacemit_i2c_bus_reset(struct spacemit_i2c_dev *spacemit_i2c)
{
	int clk_cnt = 0;
	u32 bus_status;

	/* if bus is locked, reset unit. 0: locked */
	bus_status = spacemit_i2c_read_reg(spacemit_i2c, REG_BMR);
	if (!(bus_status & BMR_SDA) || !(bus_status & BMR_SCL)) {
		spacemit_i2c_controller_reset(spacemit_i2c);
		usleep_range(10, 20);

		/* check scl status again */
		bus_status = spacemit_i2c_read_reg(spacemit_i2c, REG_BMR);
		if (!(bus_status & BMR_SCL))
			dev_alert(spacemit_i2c->dev, "unit reset failed\n");
	}

	while (clk_cnt < 9) {
		/* check whether the SDA is still locked by slave */
		bus_status = spacemit_i2c_read_reg(spacemit_i2c, REG_BMR);
		if (bus_status & BMR_SDA)
			break;

		/* if still locked, send one clk to slave to request release */
		spacemit_i2c_write_reg(spacemit_i2c, REG_RST_CYC, 0x1);
		spacemit_i2c_write_reg(spacemit_i2c, REG_CR, CR_RSTREQ);
		usleep_range(20, 30);
		clk_cnt++;
	}

	bus_status = spacemit_i2c_read_reg(spacemit_i2c, REG_BMR);
	if (clk_cnt >= 9 && !(bus_status & BMR_SDA))
		dev_alert(spacemit_i2c->dev, "bus reset clk reaches the max 9-clocks\n");
	else
		dev_alert(spacemit_i2c->dev, "bus reset, send clk: %d\n", clk_cnt);
}

static void spacemit_i2c_reset(struct spacemit_i2c_dev *spacemit_i2c)
{
	spacemit_i2c_controller_reset(spacemit_i2c);
}

static int spacemit_i2c_recover_bus_busy(struct spacemit_i2c_dev *spacemit_i2c)
{
	int timeout;
	int cnt, ret = 0;


	timeout = 1500; /* 1500us  */

	cnt = SPACEMIT_I2C_BUS_RECOVER_TIMEOUT / timeout;

	if (likely(!(spacemit_i2c_read_reg(spacemit_i2c, REG_SR) & (SR_UB | SR_IBB))))
		return 0;

	/* wait unit and bus to recover idle */
	while (unlikely(spacemit_i2c_read_reg(spacemit_i2c, REG_SR) & (SR_UB | SR_IBB))) {
		if (cnt-- <= 0)
			break;

		usleep_range(timeout / 2, timeout);
	}

	if (unlikely(cnt <= 0)) {
		/* reset controller */
		spacemit_i2c_reset(spacemit_i2c);
		ret = -EAGAIN;
	}

	return ret;
}

static void spacemit_i2c_check_bus_release(struct spacemit_i2c_dev *spacemit_i2c)
{
	/* in case bus is not released after transfer completes */
	if (unlikely(spacemit_i2c_read_reg(spacemit_i2c, REG_SR) & SR_EBB)) {
		spacemit_i2c_bus_reset(spacemit_i2c);
		usleep_range(90, 150);
	}
}

static void spacemit_i2c_unit_init(struct spacemit_i2c_dev *spacemit_i2c)
{
	u32 cr_val = 0;

	/*
	 * Unmask interrupt bits for all xfer mode:
	 * bus error, arbitration loss detected.
	 * For transaction complete signal, we use master stop
	 * interrupt, so we don't need to unmask CR_TXDONEIE.
	 */
	cr_val |= CR_BEIE | CR_ALDIE;

	/*
		* Unmask interrupt bits for interrupt xfer mode:
		* DBR rx full.
		* For tx empty interrupt CR_DTEIE, we only
		* need to enable when trigger byte transfer to start
		* data sending.
		*/
	cr_val |= CR_DRFIE;

	/* set speed bits */
	if (spacemit_i2c->fast_mode)
		cr_val |= CR_MODE_FAST;

	/* disable response to general call */
	cr_val |= CR_GCD;

	/* enable SCL clock output */
	cr_val |= CR_SCLE;

	/* enable master stop detected */
	cr_val |= CR_MSDE | CR_MSDIE;

	spacemit_i2c_write_reg(spacemit_i2c, REG_CR, cr_val);
}

static void spacemit_i2c_trigger_byte_xfer(struct spacemit_i2c_dev *spacemit_i2c)
{
	u32 cr_val = spacemit_i2c_read_reg(spacemit_i2c, REG_CR);

	/* send start pulse */
	cr_val &= ~CR_STOP;
	cr_val |= CR_START | CR_TB | CR_DTEIE;
	spacemit_i2c_write_reg(spacemit_i2c, REG_CR, cr_val);
}

static inline void
spacemit_i2c_clear_int_status(struct spacemit_i2c_dev *spacemit_i2c, u32 mask)
{
	spacemit_i2c_write_reg(spacemit_i2c, REG_SR, mask & SPACEMIT_I2C_INT_STATUS_MASK);
}

static bool spacemit_i2c_is_last_byte_to_send(struct spacemit_i2c_dev *spacemit_i2c)
{
	return (spacemit_i2c->tx_cnt == spacemit_i2c->cur_msg->len &&
		spacemit_i2c->msg_idx == spacemit_i2c->num - 1) ? true : false;
}

static bool spacemit_i2c_is_last_byte_to_receive(struct spacemit_i2c_dev *spacemit_i2c)
{
	return (spacemit_i2c->rx_cnt == spacemit_i2c->cur_msg->len - 1 &&
		spacemit_i2c->msg_idx == spacemit_i2c->num - 1) ? true : false;
}

static void spacemit_i2c_mark_rw_flag(struct spacemit_i2c_dev *spacemit_i2c)
{
	if (spacemit_i2c->cur_msg->flags & I2C_M_RD) {
		spacemit_i2c->is_rx = true;
		spacemit_i2c->slave_addr_rw =
			((spacemit_i2c->cur_msg->addr & 0x7f) << 1) | 1;
	} else {
		spacemit_i2c->is_rx = false;
		spacemit_i2c->slave_addr_rw = (spacemit_i2c->cur_msg->addr & 0x7f) << 1;
	}
}

static void spacemit_i2c_byte_xfer_send_slave_addr(struct spacemit_i2c_dev *spacemit_i2c)
{
	spacemit_i2c->phase = SPACEMIT_I2C_XFER_SLAVE_ADDR;

	/* write slave address to DBR for interrupt mode */
	spacemit_i2c_write_reg(spacemit_i2c, REG_DBR, spacemit_i2c->slave_addr_rw);

	spacemit_i2c_trigger_byte_xfer(spacemit_i2c);
}

static int spacemit_i2c_byte_xfer(struct spacemit_i2c_dev *spacemit_i2c);
static int spacemit_i2c_byte_xfer_next_msg(struct spacemit_i2c_dev *spacemit_i2c);

static int spacemit_i2c_read(struct spacemit_i2c_dev *spacemit_i2c) {
	int ret = 0;
	u32 cr_val = spacemit_i2c_read_reg(spacemit_i2c, REG_CR);

	cr_val &= ~(CR_TB | CR_ACKNAK | CR_STOP | CR_START);
	dev_err(spacemit_i2c->dev, "->count:%ld\n", spacemit_i2c->count);
	if(spacemit_i2c->count) {
		*spacemit_i2c->msg_buf++ =
				spacemit_i2c_read_reg(spacemit_i2c, REG_DBR);
		spacemit_i2c->count --;
	}

	/* if transfer completes, ISR will handle it */
	if (spacemit_i2c->i2c_status & (SR_MSD | SR_ACKNAK))
		return 0;

	/* trigger next byte receive */
	if (spacemit_i2c->count) {
		/* send stop pulse for last byte of last msg */
		if (spacemit_i2c->count == 1 && spacemit_i2c->msg_idx == spacemit_i2c->num - 1)
			cr_val |= CR_STOP | CR_ACKNAK;

		cr_val |= CR_ALDIE | CR_TB;
		spacemit_i2c_write_reg(spacemit_i2c, REG_CR, cr_val);
	}  else if (spacemit_i2c->msg_idx < spacemit_i2c->num - 1) {
		spacemit_i2c_next_msg(spacemit_i2c);
	}

	return ret;
}

static int spacemit_i2c_byte_xfer_body(struct spacemit_i2c_dev *spacemit_i2c)
{
	int ret = 0;
	u32 cr_val = spacemit_i2c_read_reg(spacemit_i2c, REG_CR);

	cr_val &= ~(CR_TB | CR_ACKNAK | CR_STOP | CR_START);
	spacemit_i2c->phase = SPACEMIT_I2C_XFER_BODY;

	if (spacemit_i2c->i2c_status & SR_IRF) { /* i2c receive full */
		/* if current is transmit mode, ignore this signal */
		if (!spacemit_i2c->is_rx)
			return 0;
		dev_err(spacemit_i2c->dev, "receiver full\n");
		
		if (spacemit_i2c->rx_cnt < spacemit_i2c->cur_msg->len) {
			*spacemit_i2c->msg_buf++ =
				spacemit_i2c_read_reg(spacemit_i2c, REG_DBR);
			spacemit_i2c->rx_cnt++;
		}
		/* if transfer completes, ISR will handle it */
		if (spacemit_i2c->i2c_status & (SR_MSD | SR_ACKNAK))
			return 0;

		/* trigger next byte receive */
		if (spacemit_i2c->rx_cnt < spacemit_i2c->cur_msg->len) {
			/* send stop pulse for last byte of last msg */
			if (spacemit_i2c_is_last_byte_to_receive(spacemit_i2c))
				cr_val |= CR_STOP | CR_ACKNAK;

			cr_val |= CR_ALDIE | CR_TB;
			spacemit_i2c_write_reg(spacemit_i2c, REG_CR, cr_val);
		} else if (spacemit_i2c->msg_idx < spacemit_i2c->num - 1) {
			ret = spacemit_i2c_byte_xfer_next_msg(spacemit_i2c);
		} else {
			/*
			 * For this branch, we do nothing, here the receive
			 * transfer is already done, the master stop interrupt
			 * should be generated to complete this transaction.
			*/
		}
	} else if (spacemit_i2c->i2c_status & SR_ITE) { /* i2c transmit empty */
		/* MSD comes with ITE */
		if (spacemit_i2c->i2c_status & SR_MSD)
			return ret;
		dev_err(spacemit_i2c->dev, "transmit empty\n");
		if (spacemit_i2c->i2c_status & SR_RWM) { /* receive mode */
			dev_err(spacemit_i2c->dev, "transmit empty: receive mode\n");
			/* if current is transmit mode, ignore this signal */
			if (!spacemit_i2c->is_rx)
				return 0;

			if (spacemit_i2c_is_last_byte_to_receive(spacemit_i2c)) 
				cr_val |= CR_STOP | CR_ACKNAK;

			/* trigger next byte receive */
			cr_val |= CR_ALDIE | CR_TB;

			/*
			 * Mask transmit empty interrupt to avoid useless tx
			 * interrupt signal after switch to receive mode, the
			 * next expected is receive full interrupt signal.
			 */
			cr_val &= ~CR_DTEIE;
			spacemit_i2c_write_reg(spacemit_i2c, REG_CR, cr_val);
		} else { /* transmit mode */
			dev_err(spacemit_i2c->dev, "transmit empty: transmit mode\n");
			/* if current is receive mode, ignore this signal */
			if (spacemit_i2c->is_rx)
				return 0;
			dev_err(spacemit_i2c->dev, 
					"tx_cnt: %ld, cur_msg->len:%d, msg_idx:%d, num:%d\n",
			 		spacemit_i2c->tx_cnt,
					spacemit_i2c->cur_msg->len,
					spacemit_i2c->msg_idx,
					spacemit_i2c->num);
			if (spacemit_i2c->tx_cnt < spacemit_i2c->cur_msg->len) {
				dev_err(spacemit_i2c->dev, "send: %x\n", *spacemit_i2c->msg_buf);
				spacemit_i2c_write_reg(spacemit_i2c, REG_DBR,
						*spacemit_i2c->msg_buf++);
				spacemit_i2c->tx_cnt++;
				
				/* send stop pulse for last byte of last msg */
				if (spacemit_i2c_is_last_byte_to_send(spacemit_i2c)) {
					dev_err(spacemit_i2c->dev, "send a stop\n");
					cr_val |= CR_STOP;
				}
					

				cr_val |= CR_ALDIE | CR_TB;
				spacemit_i2c_write_reg(spacemit_i2c, REG_CR, cr_val);
			} else if (spacemit_i2c->msg_idx < spacemit_i2c->num - 1) {
				dev_err(spacemit_i2c->dev, "next msg\n");
				ret = spacemit_i2c_byte_xfer_next_msg(spacemit_i2c);
			} else {
				/*
				 * For this branch, we do nothing, here the
				 * sending transfer is already done, the master
				 * stop interrupt should be generated to
				 * complete this transaction.
				*/
			}
		}
	}

	return ret;
}

static int spacemit_i2c_next_msg(struct spacemit_i2c_dev *spacemit_i2c) {
	if (spacemit_i2c->msg_idx == spacemit_i2c->num - 1)
		return 0;

	spacemit_i2c->msg_idx++;
	spacemit_i2c->cur_msg = spacemit_i2c->msgs + spacemit_i2c->msg_idx;
	spacemit_i2c->msg_buf = spacemit_i2c->cur_msg->buf;
	spacemit_i2c->rx_cnt = 0;
	spacemit_i2c->tx_cnt = 0;
	spacemit_i2c->i2c_err = 0;
	spacemit_i2c->i2c_status = 0;
	spacemit_i2c->phase = SPACEMIT_I2C_XFER_IDLE;
	spacemit_i2c->count = spacemit_i2c->cur_msg->len;

	spacemit_i2c_mark_rw_flag(spacemit_i2c);

	return spacemit_i2c_xfer_msg(spacemit_i2c);
}

static int spacemit_i2c_byte_xfer_next_msg(struct spacemit_i2c_dev *spacemit_i2c)
{
	if (spacemit_i2c->msg_idx == spacemit_i2c->num - 1)
		return 0;

	spacemit_i2c->msg_idx++;
	spacemit_i2c->cur_msg = spacemit_i2c->msgs + spacemit_i2c->msg_idx;
	spacemit_i2c->msg_buf = spacemit_i2c->cur_msg->buf;
	spacemit_i2c->rx_cnt = 0;
	spacemit_i2c->tx_cnt = 0;
	spacemit_i2c->i2c_err = 0;
	spacemit_i2c->i2c_status = 0;
	spacemit_i2c->phase = SPACEMIT_I2C_XFER_IDLE;
	spacemit_i2c->count = spacemit_i2c->cur_msg->len;

	spacemit_i2c_mark_rw_flag(spacemit_i2c);

	return spacemit_i2c_byte_xfer(spacemit_i2c);
}

static int spacemit_i2c_xfer_msg(struct spacemit_i2c_dev *spacemit_i2c) {
	/* i2c error occurs */
	if (unlikely(spacemit_i2c->i2c_err))
		return -1;
	
	spacemit_i2c->count = spacemit_i2c->cur_msg->len;
	spacemit_i2c_byte_xfer_send_slave_addr(spacemit_i2c);

	return 0;
}

static int spacemit_i2c_byte_xfer(struct spacemit_i2c_dev *spacemit_i2c)
{
	int ret = 0;

	/* i2c error occurs */
	if (unlikely(spacemit_i2c->i2c_err))
		return -1;

	if (spacemit_i2c->phase == SPACEMIT_I2C_XFER_IDLE) {
		spacemit_i2c_byte_xfer_send_slave_addr(spacemit_i2c);
	} else {
		ret = spacemit_i2c_byte_xfer_body(spacemit_i2c);
	}

	return ret;
}

static int spacemit_i2c_handle_err(struct spacemit_i2c_dev *spacemit_i2c)
{
	if (unlikely(spacemit_i2c->i2c_err)) {
		dev_dbg(spacemit_i2c->dev, "i2c error status: 0x%08x\n",
				spacemit_i2c->i2c_status);
		if (spacemit_i2c->i2c_err & (SR_BED  | SR_ALD))
			spacemit_i2c_reset(spacemit_i2c);

		/* try transfer again */
		if (spacemit_i2c->i2c_err & (SR_RXOV | SR_ALD)) {
			spacemit_i2c_flush_fifo_buffer(spacemit_i2c);
			return -EAGAIN;
		}
		return (spacemit_i2c->i2c_status & SR_ACKNAK) ? -ENXIO : -EIO;
	}

	return 0;
}

static irqreturn_t spacemit_i2c_int_handler(int irq, void *devid)
{
	struct spacemit_i2c_dev *spacemit_i2c = devid;
	u32 status, ctrl;
	int ret = 0;

	/* record i2c status */
	status = spacemit_i2c_read_reg(spacemit_i2c, REG_SR);
	spacemit_i2c->i2c_status = status;

	/* check if a valid interrupt status */
	if(!status) {
		/* nothing need be done */
		return IRQ_HANDLED;
	}

	/* bus error, rx overrun, arbitration lost */
	spacemit_i2c->i2c_err = status & (SR_BED | SR_RXOV | SR_ALD);

	/* clear interrupt status bits[31:18]*/
	spacemit_i2c_clear_int_status(spacemit_i2c, status);

	/* i2c error happens */
	if (unlikely(spacemit_i2c->i2c_err))
		goto err_out;

	/* process interrupt mode */
	if (likely(spacemit_i2c->xfer_mode == SPACEMIT_I2C_MODE_INTERRUPT)) {
		// dev_err(spacemit_i2c->dev, "call byte_xfer from int\n");
		// ret = spacemit_i2c_byte_xfer(spacemit_i2c);
		if (spacemit_i2c->i2c_status & SR_IRF) {
			dev_err(spacemit_i2c->dev, "call read from int\n");
			ret = spacemit_i2c_read(spacemit_i2c);
			// ret = spacemit_i2c_byte_xfer(spacemit_i2c);
		} else {
			dev_err(spacemit_i2c->dev, "call byte_xfer from int\n");
			ret = spacemit_i2c_byte_xfer(spacemit_i2c);
		}
	}

	dev_err(spacemit_i2c->dev, "err: %d, ret:%d, status & MSD: %d\n",
			spacemit_i2c->i2c_err, 
			ret,
			status & SR_MSD);
		

err_out:
	/*
	 * send transaction complete signal:
	 * error happens, detect master stop
	 */
	if (likely(spacemit_i2c->i2c_err || (ret < 0) || (status & SR_MSD))) {
		/*
		 * Here the transaction is already done, we don't need any
		 * other interrupt signals from now, in case any interrupt
		 * happens before spacemit_i2c_xfer to disable irq and i2c unit,
		 * we mask all the interrupt signals and clear the interrupt
		 * status.
		*/
		ctrl = spacemit_i2c_read_reg(spacemit_i2c, REG_CR);
		ctrl &= ~SPACEMIT_I2C_INT_CTRL_MASK;
		spacemit_i2c_write_reg(spacemit_i2c, REG_CR, ctrl);

		spacemit_i2c_clear_int_status(spacemit_i2c, SPACEMIT_I2C_INT_STATUS_MASK);

		complete(&spacemit_i2c->complete);
	}

	return IRQ_HANDLED;
}

static void spacemit_i2c_choose_xfer_mode(struct spacemit_i2c_dev *spacemit_i2c)
{
	unsigned long timeout;
	int idx = 0, cnt = 0, freq;

	while (idx < spacemit_i2c->num) {
		cnt += (spacemit_i2c->msgs + idx)->len + 1;

		/*
		 * Some SMBus transactions require that
		 * we receive the transacttion length as the first read byte.
		 * force to use I2C_MODE_INTERRUPT
		 */
		if ((spacemit_i2c->msgs + idx)->flags & I2C_M_RECV_LEN) {
			cnt += I2C_SMBUS_BLOCK_MAX + 2;
		}
		idx++;
	}

	spacemit_i2c->xfer_mode = SPACEMIT_I2C_MODE_INTERRUPT;
	
	if (likely(spacemit_i2c->fast_mode))
		freq = 400000;
	else
		freq = 100000;

	timeout = cnt * 9 * USEC_PER_SEC / freq;
	spacemit_i2c->timeout = usecs_to_jiffies(timeout + 500000);	
}

static void spacemit_i2c_init_xfer_params(struct spacemit_i2c_dev *spacemit_i2c)
{
	/* initialize transfer parameters */
	spacemit_i2c->msg_idx = 0;
	spacemit_i2c->cur_msg = spacemit_i2c->msgs;
	spacemit_i2c->msg_buf = spacemit_i2c->cur_msg->buf;
	spacemit_i2c->rx_cnt = 0;
	spacemit_i2c->tx_cnt = 0;
	spacemit_i2c->i2c_err = 0;
	spacemit_i2c->i2c_status = 0;
	spacemit_i2c->phase = SPACEMIT_I2C_XFER_IDLE;
}

static int
spacemit_i2c_xfer(struct i2c_adapter *adapt, struct i2c_msg msgs[], int num)
{
	struct spacemit_i2c_dev *spacemit_i2c = i2c_get_adapdata(adapt);
	int ret = 0, xfer_try = 0;
	unsigned long time_left;

	mutex_lock(&spacemit_i2c->mtx);
	spacemit_i2c->msgs = msgs;
	spacemit_i2c->num = num;

xfer_retry:
	/* if unit keeps the last control status, don't need to do reset */
	if (unlikely(spacemit_i2c_read_reg(spacemit_i2c, REG_CR) != spacemit_i2c->i2c_ctrl_reg_value))
		/* i2c controller & bus reset */
		spacemit_i2c_reset(spacemit_i2c);

	/* choose transfer mode */
	spacemit_i2c_choose_xfer_mode(spacemit_i2c);

	/* i2c unit init */
	spacemit_i2c_unit_init(spacemit_i2c);

	/* clear all interrupt status */
	spacemit_i2c_clear_int_status(spacemit_i2c, SPACEMIT_I2C_INT_STATUS_MASK);

	spacemit_i2c_init_xfer_params(spacemit_i2c);

	spacemit_i2c_mark_rw_flag(spacemit_i2c);

	reinit_completion(&spacemit_i2c->complete);

	spacemit_i2c_enable(spacemit_i2c);
	enable_irq(spacemit_i2c->irq);

	/* i2c wait for bus busy */
	ret = spacemit_i2c_recover_bus_busy(spacemit_i2c);
	if (unlikely(ret))
		goto err_recover;

	/* i2c msg transmit */
	if (likely(spacemit_i2c->xfer_mode == SPACEMIT_I2C_MODE_INTERRUPT)) {
		dev_err(spacemit_i2c->dev, "call byte_xfer from xfer\n");
		// ret = spacemit_i2c_byte_xfer(spacemit_i2c);
		ret = spacemit_i2c_xfer_msg(spacemit_i2c);
	}
		

	if (unlikely(ret < 0)) {
		dev_dbg(spacemit_i2c->dev, "i2c transfer error\n");
		/* timeout error should not be overrided, and the transfer
		 * error will be confirmed by err handle function latter,
		 * the reset should be invalid argument error. */
		if (ret != -ETIMEDOUT)
			ret = -EINVAL;
		goto err_xfer;
	}

	if (likely(spacemit_i2c->xfer_mode == SPACEMIT_I2C_MODE_INTERRUPT)) {
		time_left = wait_for_completion_timeout(&spacemit_i2c->complete,
							spacemit_i2c->timeout);
		if (unlikely(time_left == 0)) {
			dev_alert(spacemit_i2c->dev, "msg completion timeout\n");
			spacemit_i2c_bus_reset(spacemit_i2c);
			spacemit_i2c_reset(spacemit_i2c);
			ret = -ETIMEDOUT;
			goto err_xfer;
		}
	}

err_xfer:
	if (likely(!ret))
		spacemit_i2c_check_bus_release(spacemit_i2c);

err_recover:
	disable_irq(spacemit_i2c->irq);

	/* disable spacemit i2c */
	spacemit_i2c_disable(spacemit_i2c);

	/* process i2c error */
	if (unlikely(spacemit_i2c->i2c_err))
		ret = spacemit_i2c_handle_err(spacemit_i2c);

	xfer_try++;
	/* retry i2c transfer 3 times for timeout and bus busy */
	if (unlikely((ret == -ETIMEDOUT || ret == -EAGAIN) &&
		xfer_try <= spacemit_i2c->drv_retries)) {
		dev_alert(spacemit_i2c->dev, "i2c transfer retry %d, ret %d mode %d err 0x%x\n",
				xfer_try, ret, spacemit_i2c->xfer_mode, spacemit_i2c->i2c_err);
		usleep_range(150, 200);
		ret = 0;
		goto xfer_retry;
	}

	mutex_unlock(&spacemit_i2c->mtx);

	return ret < 0 ? ret : num;
}

static u32 spacemit_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm spacemit_i2c_algrtm = {
	.master_xfer	= spacemit_i2c_xfer,
	.functionality	= spacemit_i2c_func,
};

static int
spacemit_i2c_parse_dt(struct platform_device *pdev, struct spacemit_i2c_dev *spacemit_i2c)
{
	struct device_node *dnode = pdev->dev.of_node;
	int ret;

	/* enable fast speed mode */
	spacemit_i2c->fast_mode = of_property_read_bool(dnode, "spacemit,i2c-fast-mode");

	ret = of_property_read_u32(dnode, "spacemit,i2c-lcr", &spacemit_i2c->i2c_lcr);
	if (ret) {
		dev_err(spacemit_i2c->dev, "failed to get i2c lcr\n");
		return ret;
	}

	ret = of_property_read_u32(dnode, "spacemit,i2c-wcr", &spacemit_i2c->i2c_wcr);
	if (ret) {
		dev_err(spacemit_i2c->dev, "failed to get i2c wcr\n");
		return ret;
	}

	/*
	 * adapter device id:
	 * assigned in dt node or alias name, or automatically allocated
	 * in i2c_add_numbered_adapter()
	 */
	ret = of_property_read_u32(dnode, "spacemit,adapter-id", &pdev->id);
	if (ret)
		pdev->id = -1;

	/* apb clock: 26MHz or 52MHz */
	ret = of_property_read_u32(dnode, "spacemit,apb_clock", &spacemit_i2c->apb_clock);
	if (ret) {
		dev_err(spacemit_i2c->dev, "failed to get apb clock\n");
		return ret;
	} else if ((spacemit_i2c->apb_clock != SPACEMIT_I2C_APB_CLOCK_26M) &&
			(spacemit_i2c->apb_clock != SPACEMIT_I2C_APB_CLOCK_52M)) {
		dev_err(spacemit_i2c->dev, "the apb clock should be 26M or 52M\n");
		return -EINVAL;
	}

	return 0;
}

static int spacemit_i2c_probe(struct platform_device *pdev)
{
	struct spacemit_i2c_dev *spacemit_i2c;
	struct device_node *dnode = pdev->dev.of_node;
	int ret = 0;

	/* allocate memory */
	spacemit_i2c = devm_kzalloc(&pdev->dev,
				sizeof(struct spacemit_i2c_dev),
				GFP_KERNEL);
	if (!spacemit_i2c) {
		ret =  -ENOMEM;
		goto err_out;
	}

	spacemit_i2c->dev = &pdev->dev;
	platform_set_drvdata(pdev, spacemit_i2c);
	mutex_init(&spacemit_i2c->mtx);

	spacemit_i2c->resets = devm_reset_control_get_optional(&pdev->dev, NULL);
	if(IS_ERR(spacemit_i2c->resets)) {
		dev_err(&pdev->dev, "failed to get resets\n");
		goto err_out;
	}
	/* reset the i2c controller */
	reset_control_assert(spacemit_i2c->resets);
	udelay(200);
	reset_control_deassert(spacemit_i2c->resets);

	ret = spacemit_i2c_parse_dt(pdev, spacemit_i2c);
	if (ret)
		goto err_out;

	ret = of_address_to_resource(dnode, 0, &spacemit_i2c->resrc);
	if (ret) {
		dev_err(&pdev->dev, "failed to get resource\n");
		ret =  -ENODEV;
		goto err_out;
	}

	spacemit_i2c->mapbase = devm_ioremap_resource(spacemit_i2c->dev, &spacemit_i2c->resrc);
	if (IS_ERR(spacemit_i2c->mapbase)) {
		dev_err(&pdev->dev, "failed to do ioremap\n");
		ret =  PTR_ERR(spacemit_i2c->mapbase);
		goto err_out;
	}

	spacemit_i2c->irq = platform_get_irq(pdev, 0);
	if (spacemit_i2c->irq < 0) {
		dev_err(spacemit_i2c->dev, "failed to get irq resource\n");
		ret = spacemit_i2c->irq;
		goto err_out;
	}

	ret = devm_request_irq(spacemit_i2c->dev, spacemit_i2c->irq, spacemit_i2c_int_handler,
			IRQF_NO_SUSPEND | IRQF_ONESHOT,
			dev_name(spacemit_i2c->dev), spacemit_i2c);
	if (ret) {
		dev_err(spacemit_i2c->dev, "failed to request irq\n");
		goto err_out;
	}
	disable_irq(spacemit_i2c->irq);

	spacemit_i2c->clk = devm_clk_get(spacemit_i2c->dev, NULL);
	if (IS_ERR(spacemit_i2c->clk)) {
		dev_err(spacemit_i2c->dev, "failed to get clock\n");
		ret = PTR_ERR(spacemit_i2c->clk);
		goto err_out;
	}
	clk_prepare_enable(spacemit_i2c->clk);

	i2c_set_adapdata(&spacemit_i2c->adapt, spacemit_i2c);
	spacemit_i2c->adapt.owner = THIS_MODULE;
	spacemit_i2c->adapt.algo = &spacemit_i2c_algrtm;
	spacemit_i2c->adapt.dev.parent = spacemit_i2c->dev;
	spacemit_i2c->adapt.nr = pdev->id;
	/* retries used by i2c framework: 3 times */
	spacemit_i2c->adapt.retries = 3;
	/*
	 * retries used by i2c driver: 3 times
	 * this is for the very low occasionally PMIC i2c access failure.
	 */
	spacemit_i2c->drv_retries = 3;
	spacemit_i2c->adapt.dev.of_node = dnode;
	spacemit_i2c->adapt.algo_data = spacemit_i2c;
	strscpy(spacemit_i2c->adapt.name, "spacemit-i2c-adapter",
		sizeof(spacemit_i2c->adapt.name));

	init_completion(&spacemit_i2c->complete);

	ret = i2c_add_numbered_adapter(&spacemit_i2c->adapt);
	if (ret) {
		dev_err(spacemit_i2c->dev, "failed to add i2c adapter\n");
		goto err_clk;
	}

	dev_dbg(spacemit_i2c->dev, "driver probe success");
	return 0;

err_clk:
	clk_disable_unprepare(spacemit_i2c->clk);
err_out:
	return ret;
}

static int spacemit_i2c_remove(struct platform_device *pdev)
{
	struct spacemit_i2c_dev *spacemit_i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&spacemit_i2c->adapt);

	mutex_destroy(&spacemit_i2c->mtx);

	reset_control_assert(spacemit_i2c->resets);

	clk_disable_unprepare(spacemit_i2c->clk);

	dev_dbg(spacemit_i2c->dev, "driver removed\n");
	return 0;
}

static const struct of_device_id spacemit_i2c_dt_match[] = {
	{
		.compatible = "spacemit,k1x-i2c",
	},
	{}
};

MODULE_DEVICE_TABLE(of, spacemit_i2c_dt_match);

static struct platform_driver spacemit_i2c_driver = {
	.probe  = spacemit_i2c_probe,
	.remove = spacemit_i2c_remove,
	.driver = {
		.name		= "i2c-spacemit-k1x",
		.of_match_table	= spacemit_i2c_dt_match,
	},
};

module_platform_driver(spacemit_i2c_driver);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("i2c driver for k1 of spacemit SoCs");
