/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (C) 2024 Troy Mitchell <troymitchell988@gmail.com>
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/uaccess.h>

#define SPACEMIT_I2C_REG_LCR_VALUE		0x82c469f
#define SPACEMIT_I2C_REG_WCR_VALUE		0x142a

/* spacemit i2c registers */
enum {
	REG_CR = 0x0,		/* Control Register */
	REG_SR = 0x4,		/* Status Register */
	REG_SAR = 0x8,		/* Slave Address Register */
	REG_DBR = 0xc,		/* Data Buffer Register */
	REG_LCR = 0x10,		/* Load Count Register */
	REG_WCR = 0x14,		/* Wait Count Register */
	REG_RST_CYC = 0x18,	/* Bus reset cycle counter */
	REG_BMR = 0x1c,		/* Bus monitor register */
	REG_WFIFO = 0x20,	/* Write FIFO Register */
	REG_WFIFO_WPTR = 0x24,	/* Write FIFO Write Pointer Register */
	REG_WFIFO_RPTR = 0x28,	/* Write FIFO Read Pointer Register */
	REG_RFIFO = 0x2c,	/* Read FIFO Register */
	REG_RFIFO_WPTR = 0x30,	/* Read FIFO Write Pointer Register */
	REG_RFIFO_RPTR = 0x34,	/* Read FIFO Read Pointer Register */
};

/* register REG_CR fields */
enum {
	CR_START = BIT(0),	/* start bit */
	CR_STOP = BIT(1),	/* stop bit */
	CR_ACKNAK = BIT(2),	/* send ACK(0) or NAK(1) */
	CR_TB = BIT(3),		/* transfer byte bit */
	CR_TXBEGIN = BIT(4),	/* transaction begin */
	CR_FIFOEN = BIT(5),	/* enable FIFO mode */
	CR_GPIOEN = BIT(6),	/* enable GPIO mode for SCL in HS */
	CR_DMAEN = BIT(7),	/* enable DMA for TX and RX FIFOs */
	CR_MODE_FAST = BIT(8),	/* bus mode (master operation) */
	CR_MODE_HIGH = BIT(9),	/* bus mode (master operation) */
	CR_UR = BIT(10),	/* unit reset */
	CR_RSTREQ = BIT(11),	/* i2c bus reset request */
	CR_MA = BIT(12),	/* master abort */
	CR_SCLE = BIT(13),	/* master clock enable */
	CR_IUE = BIT(14),	/* unit enable */
	CR_HS_STRETCH = BIT(16),	/* I2C hs stretch */
	CR_ALDIE = BIT(18),	/* enable arbitration interrupt */
	CR_DTEIE = BIT(19),	/* enable tx interrupts */
	CR_DRFIE = BIT(20),	/* enable rx interrupts */
	CR_GCD = BIT(21),	/* general call disable */
	CR_BEIE = BIT(22),	/* enable bus error ints */
	CR_SADIE = BIT(23),	/* slave address detected int enable */
	CR_SSDIE = BIT(24),	/* slave STOP detected int enable */
	CR_MSDIE = BIT(25),	/* master STOP detected int enable */
	CR_MSDE = BIT(26),	/* master STOP detected enable */
	CR_TXDONEIE = BIT(27),	/* transaction done int enable */
	CR_TXEIE = BIT(28),	/* transmit FIFO empty int enable */
	CR_RXHFIE = BIT(29),	/* receive FIFO half-full int enable */
	CR_RXFIE = BIT(30),	/* receive FIFO full int enable */
	CR_RXOVIE = BIT(31),	/* receive FIFO overrun int enable */
};

/* register REG_SR fields */
enum {
	SR_RWM = BIT(13),	/* read/write mode */
	SR_ACKNAK = BIT(14),	/* ACK/NACK status */
	SR_UB = BIT(15),	/* unit busy */
	SR_IBB = BIT(16),	/* i2c bus busy */
	SR_EBB = BIT(17),	/* early bus busy */
	SR_ALD = BIT(18),	/* arbitration loss detected */
	SR_ITE = BIT(19),	/* tx buffer empty */
	SR_IRF = BIT(20),	/* rx buffer full */
	SR_GCAD = BIT(21),	/* general call address detected */
	SR_BED = BIT(22),	/* bus error no ACK/NAK */
	SR_SAD = BIT(23),	/* slave address detected */
	SR_SSD = BIT(24),	/* slave stop detected */
	SR_MSD = BIT(26),	/* master stop detected */
	SR_TXDONE = BIT(27),	/* transaction done */
	SR_TXE = BIT(28),	/* tx FIFO empty */
	SR_RXHF = BIT(29),	/* rx FIFO half-full */
	SR_RXF = BIT(30),	/* rx FIFO full */
	SR_RXOV = BIT(31),	/* RX FIFO overrun */
};

/* register REG_LCR fields */
enum {
	LCR_SLV = 0x000001FF,	/* SLV: bit[8:0] */
	LCR_FLV = 0x0003FE00,	/* FLV: bit[17:9] */
	LCR_HLVH = 0x07FC0000,	/* HLVH: bit[26:18] */
	LCR_HLVL = 0xF8000000,	/* HLVL: bit[31:27] */
};

/* register REG_WCR fields */
enum {
	WCR_COUNT = 0x0000001F,	/* COUNT: bit[4:0] */
	WCR_COUNT1 = 0x000003E0,	/* HS_COUNT1: bit[9:5] */
	WCR_COUNT2 = 0x00007C00,	/* HS_COUNT2: bit[14:10] */
};

/* register REG_BMR fields */
enum {
	BMR_SDA = BIT(0),	/* SDA line level */
	BMR_SCL = BIT(1),	/* SCL line level */
};

/* register REG_WFIFO fields */
enum {
	WFIFO_DATA_MSK = 0x000000FF,	/* data: bit[7:0] */
	WFIFO_CTRL_MSK = 0x000003E0,	/* control: bit[11:8] */
	WFIFO_CTRL_START = BIT(8),	/* start bit */
	WFIFO_CTRL_STOP = BIT(9),	/* stop bit */
	WFIFO_CTRL_ACKNAK = BIT(10),	/* send ACK(0) or NAK(1) */
	WFIFO_CTRL_TB = BIT(11),	/* transfer byte bit */
};

/* status register init value */
enum {
	SPACEMIT_I2C_INT_STATUS_MASK = 0xfffc0000,	/* SR bits[31:18] */
	SPACEMIT_I2C_INT_CTRL_MASK = (CR_ALDIE | CR_DTEIE | CR_DRFIE |
				      CR_BEIE | CR_TXDONEIE | CR_TXEIE |
				      CR_RXHFIE | CR_RXFIE | CR_RXOVIE |
				      CR_MSDIE),
};

/* i2c bus recover timeout: us */
#define SPACEMIT_I2C_BUS_RECOVER_TIMEOUT	(100000)

#define SPACEMIT_I2C_APB_CLOCK_26M		(26000000)
#define SPACEMIT_I2C_APB_CLOCK_52M		(52000000)

/* i2c-spacemit driver's main struct */
struct spacemit_i2c_dev {
	struct device *dev;
	struct i2c_adapter adapt;
	struct i2c_msg *msgs;
	int num;
	struct resource resrc;
	struct mutex mtx;

	/* virtual base address mapped for register */
	void __iomem *mapbase;

	struct reset_control *resets;
	struct clk *clk;
	int irq;

	/* for control reset param  */
	u32 lcr;
	u32 wcr;

	struct i2c_msg *cur_msg;
	int msg_idx;
	u8 *msg_buf;
	size_t count;

	struct completion complete;
	u32 timeout;
	u32 i2c_ctrl_reg_value;
	u32 i2c_status;
	u32 i2c_err;
};

static inline u32
spacemit_i2c_read_reg(struct spacemit_i2c_dev *spacemit_i2c, int reg)
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
			       spacemit_i2c_read_reg(spacemit_i2c,
						     REG_CR) | CR_IUE);
}

static void spacemit_i2c_disable(struct spacemit_i2c_dev *spacemit_i2c)
{
	spacemit_i2c->i2c_ctrl_reg_value =
	    spacemit_i2c_read_reg(spacemit_i2c, REG_CR) & ~CR_IUE;
	spacemit_i2c_write_reg(spacemit_i2c, REG_CR,
			       spacemit_i2c->i2c_ctrl_reg_value);
}

static void spacemit_i2c_flush_fifo_buffer(struct spacemit_i2c_dev
					   *spacemit_i2c)
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
	spacemit_i2c_write_reg(spacemit_i2c, REG_LCR,
			       SPACEMIT_I2C_REG_WCR_VALUE);

	/* set wait counter register */
	spacemit_i2c_write_reg(spacemit_i2c, REG_WCR,
			       SPACEMIT_I2C_REG_LCR_VALUE);
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
		dev_alert(spacemit_i2c->dev,
			  "bus reset clk reaches the max 9-clocks\n");
	else
		dev_alert(spacemit_i2c->dev, "bus reset, send clk: %d\n",
			  clk_cnt);
}

static void spacemit_i2c_reset(struct spacemit_i2c_dev *spacemit_i2c)
{
	spacemit_i2c_controller_reset(spacemit_i2c);
}

static int spacemit_i2c_recover_bus_busy(struct spacemit_i2c_dev *spacemit_i2c)
{
	int ret = 0;
	u32 val;

	if (likely
	    (!(spacemit_i2c_read_reg(spacemit_i2c, REG_SR) & (SR_UB | SR_IBB))))
		return 0;

	ret = readl_poll_timeout(spacemit_i2c->mapbase + REG_SR, val, !(val & (SR_UB | SR_IBB)), 1500, SPACEMIT_I2C_BUS_RECOVER_TIMEOUT);
	if (unlikely(ret)) {
		spacemit_i2c_reset(spacemit_i2c);
		ret = -EAGAIN;
	}

	return ret;
}

static void spacemit_i2c_check_bus_release(struct spacemit_i2c_dev
					   *spacemit_i2c)
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

	/* set speed bits: default fast mode */
	cr_val |= CR_MODE_FAST;

	/* disable response to general call */
	cr_val |= CR_GCD;

	/* enable SCL clock output */
	cr_val |= CR_SCLE;

	/* enable master stop detected */
	cr_val |= CR_MSDE | CR_MSDIE;

	spacemit_i2c_write_reg(spacemit_i2c, REG_CR, cr_val);
}

static void spacemit_i2c_trigger_byte_xfer(struct spacemit_i2c_dev
					   *spacemit_i2c)
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
	spacemit_i2c_write_reg(spacemit_i2c, REG_SR,
			       mask & SPACEMIT_I2C_INT_STATUS_MASK);
}

static void
spacemit_i2c_byte_xfer_send_slave_addr(struct spacemit_i2c_dev *spacemit_i2c)
{
	u32 slave_addr_rw;

	if (spacemit_i2c->cur_msg->flags & I2C_M_RD)
		slave_addr_rw = ((spacemit_i2c->cur_msg->addr & 0x7f) << 1) | 1;
	else
		slave_addr_rw = (spacemit_i2c->cur_msg->addr & 0x7f) << 1;

	/* write slave address to DBR for interrupt mode */
	spacemit_i2c_write_reg(spacemit_i2c, REG_DBR, slave_addr_rw);

	spacemit_i2c_trigger_byte_xfer(spacemit_i2c);
}

static int spacemit_i2c_xfer_msg(struct spacemit_i2c_dev *spacemit_i2c)
{
	/* i2c error occurs */
	if (unlikely(spacemit_i2c->i2c_err))
		return -1;

	spacemit_i2c->count = spacemit_i2c->cur_msg->len;
	spacemit_i2c_byte_xfer_send_slave_addr(spacemit_i2c);

	return 0;
}

static int spacemit_i2c_xfer_next_msg(struct spacemit_i2c_dev *spacemit_i2c)
{
	if (spacemit_i2c->msg_idx == spacemit_i2c->num - 1)
		return 0;

	spacemit_i2c->msg_idx++;
	spacemit_i2c->cur_msg = spacemit_i2c->msgs + spacemit_i2c->msg_idx;
	spacemit_i2c->msg_buf = spacemit_i2c->cur_msg->buf;
	spacemit_i2c->i2c_err = 0;
	spacemit_i2c->i2c_status = 0;
	spacemit_i2c->count = spacemit_i2c->cur_msg->len;

	return spacemit_i2c_xfer_msg(spacemit_i2c);
}

static int spacemit_i2c_ready_read(struct spacemit_i2c_dev *spacemit_i2c,
				   u32 cr_val)
{
	/* send stop pulse for last byte of last msg */
	if (spacemit_i2c->count == 1
	    && spacemit_i2c->msg_idx == spacemit_i2c->num - 1) {
		cr_val |= CR_STOP | CR_ACKNAK;
	}

	/* trigger next byte receive */
	cr_val |= CR_ALDIE | CR_TB;

	/*
	 * Mask transmit empty interrupt to avoid useless tx
	 * interrupt signal after switch to receive mode, the
	 * next expected is receive full interrupt signal.
	 */
	cr_val &= ~CR_DTEIE;
	spacemit_i2c_write_reg(spacemit_i2c, REG_CR, cr_val);

	return 0;
}

static int spacemit_i2c_read(struct spacemit_i2c_dev *spacemit_i2c, u32 cr_val)
{
	int ret = 0;

	if (spacemit_i2c->count) {
		*spacemit_i2c->msg_buf++ =
		    spacemit_i2c_read_reg(spacemit_i2c, REG_DBR);
		spacemit_i2c->count--;
	}

	/* if transfer completes, ISR will handle it */
	if (spacemit_i2c->i2c_status & (SR_MSD | SR_ACKNAK))
		return 0;

	/* trigger next byte receive */
	if (spacemit_i2c->count) {
		/* send stop pulse for last byte of last msg */
		if (spacemit_i2c->count == 1
		    && spacemit_i2c->msg_idx == spacemit_i2c->num - 1)
			cr_val |= CR_STOP | CR_ACKNAK;

		cr_val |= CR_ALDIE | CR_TB;
		spacemit_i2c_write_reg(spacemit_i2c, REG_CR, cr_val);
	} else if (spacemit_i2c->msg_idx < spacemit_i2c->num - 1) {
		ret = spacemit_i2c_xfer_next_msg(spacemit_i2c);
	}

	return ret;
}

static int spacemit_i2c_write(struct spacemit_i2c_dev *spacemit_i2c, u32 cr_val)
{
	int ret = 0;

	/* MSD comes with ITE */
	if (spacemit_i2c->i2c_status & SR_MSD)
		return ret;

	if (spacemit_i2c->count) {
		spacemit_i2c_write_reg(spacemit_i2c, REG_DBR,
				       *spacemit_i2c->msg_buf++);

		spacemit_i2c->count--;

		/* send stop pulse for last byte of last msg */
		if (!spacemit_i2c->count
		    && spacemit_i2c->msg_idx == spacemit_i2c->num - 1)
			cr_val |= CR_STOP;

		cr_val |= CR_ALDIE | CR_TB;
		spacemit_i2c_write_reg(spacemit_i2c, REG_CR, cr_val);
	} else if (spacemit_i2c->msg_idx < spacemit_i2c->num - 1) {
		ret = spacemit_i2c_xfer_next_msg(spacemit_i2c);
	}

	return ret;
}

static int spacemit_i2c_handle_err(struct spacemit_i2c_dev *spacemit_i2c)
{
	if (unlikely(spacemit_i2c->i2c_err)) {
		dev_dbg(spacemit_i2c->dev, "i2c error status: 0x%08x\n",
			spacemit_i2c->i2c_status);
		if (spacemit_i2c->i2c_err & (SR_BED | SR_ALD))
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
	u32 status, ctrl, cr_val;
	int ret = 0;

	status = spacemit_i2c_read_reg(spacemit_i2c, REG_SR);
	spacemit_i2c->i2c_status = status;

	/* check if a valid interrupt status */
	if (!status)
		return IRQ_HANDLED;

	/* bus error, rx overrun, arbitration lost */
	spacemit_i2c->i2c_err = status & (SR_BED | SR_RXOV | SR_ALD);

	/* clear interrupt status bits[31:18] */
	spacemit_i2c_clear_int_status(spacemit_i2c, status);

	if (unlikely(spacemit_i2c->i2c_err))
		goto err_out;

	cr_val = spacemit_i2c_read_reg(spacemit_i2c, REG_CR);

	cr_val &= ~(CR_TB | CR_ACKNAK | CR_STOP | CR_START);

	/* rx not empty */
	if (spacemit_i2c->i2c_status & SR_IRF)
		ret = spacemit_i2c_read(spacemit_i2c, cr_val);
	/* transmited slave addr with read flag */
	else if ((spacemit_i2c->i2c_status & SR_ITE)
		 && (spacemit_i2c->i2c_status & SR_RWM))
		ret = spacemit_i2c_ready_read(spacemit_i2c, cr_val);
	/* tx empty */
	else if (spacemit_i2c->i2c_status & SR_ITE)
		ret = spacemit_i2c_write(spacemit_i2c, cr_val);

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

		spacemit_i2c_clear_int_status(spacemit_i2c,
					      SPACEMIT_I2C_INT_STATUS_MASK);

		complete(&spacemit_i2c->complete);
	}

	return IRQ_HANDLED;
}

static void spacemit_i2c_calc_timeout(struct spacemit_i2c_dev *spacemit_i2c)
{
	unsigned long timeout;
	int idx = 0, cnt = 0, freq;

	while (idx < spacemit_i2c->num) {
		cnt += (spacemit_i2c->msgs + idx)->len + 1;

		idx++;
	}

	/* fast mode */
	freq = 400000;

	timeout = cnt * 9 * USEC_PER_SEC / freq;
	spacemit_i2c->timeout = usecs_to_jiffies(timeout + 500000);
}

static void spacemit_i2c_init_xfer_params(struct spacemit_i2c_dev *spacemit_i2c)
{
	/* initialize transfer parameters */
	spacemit_i2c->msg_idx = 0;
	spacemit_i2c->cur_msg = spacemit_i2c->msgs;
	spacemit_i2c->msg_buf = spacemit_i2c->cur_msg->buf;
	spacemit_i2c->i2c_err = 0;
	spacemit_i2c->i2c_status = 0;
}

static int spacemit_i2c_xfer_core(struct spacemit_i2c_dev *spacemit_i2c)
{
	int ret = 0;
	unsigned long time_left;

	/* if unit keeps the last control status, don't need to do reset */
	if (unlikely
	    (spacemit_i2c_read_reg(spacemit_i2c, REG_CR) !=
	     spacemit_i2c->i2c_ctrl_reg_value))
		/* i2c controller & bus reset */
		spacemit_i2c_reset(spacemit_i2c);

	spacemit_i2c_calc_timeout(spacemit_i2c);

	spacemit_i2c_unit_init(spacemit_i2c);

	/* clear all interrupt status */
	spacemit_i2c_clear_int_status(spacemit_i2c,
				      SPACEMIT_I2C_INT_STATUS_MASK);

	spacemit_i2c_init_xfer_params(spacemit_i2c);

	reinit_completion(&spacemit_i2c->complete);

	spacemit_i2c_enable(spacemit_i2c);
	enable_irq(spacemit_i2c->irq);

	/* i2c wait for bus busy */
	ret = spacemit_i2c_recover_bus_busy(spacemit_i2c);
	if (unlikely(ret))
		return ret;

	ret = spacemit_i2c_xfer_msg(spacemit_i2c);

	if (unlikely(ret < 0)) {
		dev_dbg(spacemit_i2c->dev, "i2c transfer error\n");
		/* timeout error should not be overridden, and the transfer
		 * error will be confirmed by err handle function latter,
		 * the reset should be invalid argument error.
		 */
		if (ret != -ETIMEDOUT)
			ret = -EINVAL;
		return ret;
	}

	time_left = wait_for_completion_timeout(&spacemit_i2c->complete,
						spacemit_i2c->timeout);
	if (unlikely(time_left == 0)) {
		dev_alert(spacemit_i2c->dev, "msg completion timeout\n");
		spacemit_i2c_bus_reset(spacemit_i2c);
		spacemit_i2c_reset(spacemit_i2c);
		ret = -ETIMEDOUT;
		return ret;
	}

	return ret;
}

static int
spacemit_i2c_xfer(struct i2c_adapter *adapt, struct i2c_msg msgs[], int num)
{
	struct spacemit_i2c_dev *spacemit_i2c = i2c_get_adapdata(adapt);
	int ret = 0, xfer_try = 0;

	mutex_lock(&spacemit_i2c->mtx);
	spacemit_i2c->msgs = msgs;
	spacemit_i2c->num = num;

xfer_retry:
	ret = spacemit_i2c_xfer_core(spacemit_i2c);
	if (unlikely((ret == -ETIMEDOUT || ret == -EAGAIN)))
		goto err_recover;

	if (likely(!ret))
		spacemit_i2c_check_bus_release(spacemit_i2c);

err_recover:
	disable_irq(spacemit_i2c->irq);

	spacemit_i2c_disable(spacemit_i2c);

	if (unlikely(spacemit_i2c->i2c_err))
		ret = spacemit_i2c_handle_err(spacemit_i2c);

	xfer_try++;

	/* retry i2c transfer 3 times for timeout and bus busy */
	if (unlikely((ret == -ETIMEDOUT || ret == -EAGAIN) &&
		     xfer_try <= spacemit_i2c->adapt.retries)) {
		dev_alert(spacemit_i2c->dev,
			  "i2c transfer retry %d, ret %d err 0x%x\n", xfer_try,
			  ret, spacemit_i2c->i2c_err);
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
	.master_xfer = spacemit_i2c_xfer,
	.functionality = spacemit_i2c_func,
};

static int spacemit_i2c_probe(struct platform_device *pdev)
{
	struct spacemit_i2c_dev *spacemit_i2c;
	struct device_node *of_node = pdev->dev.of_node;
	int ret = 0;

	spacemit_i2c = devm_kzalloc(&pdev->dev,
				    sizeof(struct spacemit_i2c_dev),
				    GFP_KERNEL);
	if (!spacemit_i2c)
		return -ENOMEM;

	dev_err(&pdev->dev, "pdev->id: %d\n", pdev->id);
	spacemit_i2c->dev = &pdev->dev;
	mutex_init(&spacemit_i2c->mtx);

	ret = of_address_to_resource(of_node, 0, &spacemit_i2c->resrc);
	if (ret) {
		dev_err(&pdev->dev, "failed to get resource\n");
		return -ENODEV;
	}

	spacemit_i2c->mapbase =
	    devm_ioremap_resource(spacemit_i2c->dev, &spacemit_i2c->resrc);
	if (IS_ERR(spacemit_i2c->mapbase)) {
		dev_err(&pdev->dev, "failed to do ioremap\n");
		ret = PTR_ERR(spacemit_i2c->mapbase);
		return ret;
	}

	spacemit_i2c->resets = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(spacemit_i2c->resets)) {
		dev_err(&pdev->dev, "failed to get resets\n");
		ret = PTR_ERR(spacemit_i2c->resets);
		return ret;
	}

	/* reset the i2c controller */
	reset_control_assert(spacemit_i2c->resets);
	udelay(200);
	reset_control_deassert(spacemit_i2c->resets);

	spacemit_i2c->irq = platform_get_irq(pdev, 0);
	if (spacemit_i2c->irq < 0) {
		dev_err(spacemit_i2c->dev, "failed to get irq resource\n");
		ret = spacemit_i2c->irq;
		return ret;
	}

	ret =
	    devm_request_irq(spacemit_i2c->dev, spacemit_i2c->irq,
			     spacemit_i2c_int_handler,
			     IRQF_NO_SUSPEND | IRQF_ONESHOT,
			     dev_name(spacemit_i2c->dev), spacemit_i2c);
	if (ret) {
		dev_err(spacemit_i2c->dev, "failed to request irq\n");
		return ret;
	}
	disable_irq(spacemit_i2c->irq);

	spacemit_i2c->clk = devm_clk_get(spacemit_i2c->dev, NULL);
	if (IS_ERR(spacemit_i2c->clk)) {
		dev_err(spacemit_i2c->dev, "failed to get clock\n");
		ret = PTR_ERR(spacemit_i2c->clk);
		return ret;
	}
	clk_prepare_enable(spacemit_i2c->clk);

	i2c_set_adapdata(&spacemit_i2c->adapt, spacemit_i2c);
	spacemit_i2c->adapt.owner = THIS_MODULE;
	spacemit_i2c->adapt.algo = &spacemit_i2c_algrtm;
	spacemit_i2c->adapt.dev.parent = spacemit_i2c->dev;
	spacemit_i2c->adapt.nr = pdev->id;

	/* this is for the very low occasionally PMIC i2c access failure. */
	spacemit_i2c->adapt.retries = 3;

	spacemit_i2c->adapt.dev.of_node = of_node;
	spacemit_i2c->adapt.algo_data = spacemit_i2c;
	strscpy(spacemit_i2c->adapt.name, "spacemit-i2c-adapter",
		sizeof(spacemit_i2c->adapt.name));

	init_completion(&spacemit_i2c->complete);

	ret = i2c_add_numbered_adapter(&spacemit_i2c->adapt);
	if (ret) {
		dev_err(spacemit_i2c->dev, "failed to add i2c adapter\n");
		goto err_clk;
	}

	platform_set_drvdata(pdev, spacemit_i2c);
	dev_dbg(spacemit_i2c->dev, "driver probe success");
	return 0;

err_clk:
	clk_disable_unprepare(spacemit_i2c->clk);
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
	 .compatible = "spacemit,k1-i2c",
	  },
	{ }
};

MODULE_DEVICE_TABLE(of, spacemit_i2c_dt_match);

static struct platform_driver spacemit_i2c_driver = {
	.probe = spacemit_i2c_probe,
	.remove = spacemit_i2c_remove,
	.driver = {
		   .name = "i2c-spacemit-k1",
		   .of_match_table = spacemit_i2c_dt_match,
		    },
};

module_platform_driver(spacemit_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("i2c driver for k1 of spacemit SoCs");
