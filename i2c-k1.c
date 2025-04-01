// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2025 Troy Mitchell <troymitchell988@gmail.com>
 */

 #include <linux/clk.h>
 #include <linux/i2c.h>
 #include <linux/iopoll.h>
 #include <linux/module.h>
 #include <linux/of_address.h>
 #include <linux/platform_device.h>

#define DEBUG(i2c, x, ...) dev_err(i2c, x, ##__VA_ARGS__)

#define I2C_FIFO		1

/* spacemit i2c registers */
#define SPACEMIT_ICR		 0x0		/* Control register */
#define SPACEMIT_ISR		 0x4		/* Status register */
#define SPACEMIT_IDBR		 0xc		/* Data buffer register */
#define SPACEMIT_IBMR		 0x1c		/* Bus monitor register */
#define SPACEMIT_IWFIFO		 0x20		/* Write FIFO Register */
#define SPACEMIT_IWFIFO_WPTR  	 0x24		/* Write FIFO Write Pointer Register */
#define SPACEMIT_IWFIFO_RPTR  	 0x28		/* Write FIFO Read Pointer Register */
#define SPACEMIT_IRFIFO       	 0x2c		/* Read FIFO Register */
#define SPACEMIT_IRFIFO_WPTR  	 0x30		/* Read FIFO Write Pointer Register */
#define SPACEMIT_IRFIFO_RPTR  	 0x34		/* Read FIFO Read Pointer Register */

/* SPACEMIT_ICR register fields */
#define SPACEMIT_CR_START        BIT(0)		/* start bit */
#define SPACEMIT_CR_STOP         BIT(1)		/* stop bit */
#define SPACEMIT_CR_ACKNAK       BIT(2)		/* send ACK(0) or NAK(1) */
#define SPACEMIT_CR_TB           BIT(3)		/* transfer byte bit */
#define SPACEMIT_CR_TXBEGIN      BIT(4)		/* transaction begin */
#define SPACEMIT_CR_FIFOEN       BIT(5)		/* enable FIFO mode */
/* Bits 6-7 are reserved */
#define SPACEMIT_CR_MODE_FAST    BIT(8)		/* bus mode (master operation) */
/* Bit 9 is reserved */
#define SPACEMIT_CR_UR           BIT(10)	/* unit reset */
/* Bits 11-12 are reserved */
#define SPACEMIT_CR_SCLE         BIT(13)	/* master clock enable */
#define SPACEMIT_CR_IUE          BIT(14)	/* unit enable */
/* Bits 15-17 are reserved */
#define SPACEMIT_CR_ALDIE        BIT(18)	/* enable arbitration interrupt */
#define SPACEMIT_CR_DTEIE        BIT(19)	/* enable TX interrupts */
#define SPACEMIT_CR_DRFIE        BIT(20)	/* enable RX interrupts */
#define SPACEMIT_CR_GCD          BIT(21)	/* general call disable */
#define SPACEMIT_CR_BEIE         BIT(22)	/* enable bus error ints */
/* Bits 23-24 are reserved */
#define SPACEMIT_CR_MSDIE        BIT(25)	/* master STOP detected int enable */
#define SPACEMIT_CR_MSDE         BIT(26)	/* master STOP detected enable */
#define SPACEMIT_CR_TXDONEIE     BIT(27)	/* transaction done int enable */
#define SPACEMIT_CR_TXEIE        BIT(28)	/* transmit FIFO empty int enable */
#define SPACEMIT_CR_RXHFIE       BIT(29)	/* receive FIFO half-full int enable */
#define SPACEMIT_CR_RXFIE        BIT(30)	/* receive FIFO full int enable */
#define SPACEMIT_CR_RXOVIE       BIT(31)	/* receive FIFO overrun int enable */

#define SPACEMIT_I2C_INT_CTRL_MASK	(SPACEMIT_CR_ALDIE | SPACEMIT_CR_DTEIE | \
					 SPACEMIT_CR_DRFIE | SPACEMIT_CR_BEIE | \
					 SPACEMIT_CR_TXDONEIE | SPACEMIT_CR_TXEIE | \
					 SPACEMIT_CR_RXHFIE | SPACEMIT_CR_RXFIE | \
					 SPACEMIT_CR_RXOVIE | SPACEMIT_CR_MSDIE | \
					 SPACEMIT_CR_FIFOEN)

/* SPACEMIT_ISR register fields */
/* Bits 0-13 are reserved */
#define SPACEMIT_SR_ACKNAK       BIT(14)	/* ACK/NACK status */
#define SPACEMIT_SR_UB           BIT(15)	/* unit busy */
#define SPACEMIT_SR_IBB          BIT(16)	/* i2c bus busy */
#define SPACEMIT_SR_EBB          BIT(17)	/* early bus busy */
#define SPACEMIT_SR_ALD          BIT(18)	/* arbitration loss detected */
#define SPACEMIT_SR_ITE          BIT(19)	/* TX buffer empty */
#define SPACEMIT_SR_IRF          BIT(20)	/* RX buffer full */
#define SPACEMIT_SR_GCAD         BIT(21)	/* general call address detected */
#define SPACEMIT_SR_BED          BIT(22)	/* bus error no ACK/NAK */
#define SPACEMIT_SR_SAD          BIT(23)	/* slave address detected */
#define SPACEMIT_SR_SSD          BIT(24)	/* slave stop detected */
/* Bit 25 is reserved */
#define SPACEMIT_SR_MSD          BIT(26)	/* master stop detected */
#define SPACEMIT_SR_TXDONE       BIT(27)	/* transaction done */
#define SPACEMIT_SR_TXE          BIT(28)	/* TX FIFO empty */
#define SPACEMIT_SR_RXHF         BIT(29)	/* RX FIFO half-full */
#define SPACEMIT_SR_RXF          BIT(30)	/* RX FIFO full */
#define SPACEMIT_SR_RXOV         BIT(31)	/* RX FIFO overrun */

#define SPACEMIT_I2C_INT_STATUS_MASK	(SPACEMIT_SR_RXOV | SPACEMIT_SR_RXF | SPACEMIT_SR_RXHF | \
					SPACEMIT_SR_TXE | SPACEMIT_SR_TXDONE | SPACEMIT_SR_MSD | \
					SPACEMIT_SR_SSD | SPACEMIT_SR_SAD | SPACEMIT_SR_BED | \
					SPACEMIT_SR_GCAD | SPACEMIT_SR_IRF | SPACEMIT_SR_ITE | \
					SPACEMIT_SR_ALD)

/* SPACEMIT_IBMR register fields */
#define SPACEMIT_BMR_SDA         BIT(0)		/* SDA line level */
#define SPACEMIT_BMR_SCL         BIT(1)		/* SCL line level */

/* SPACEMIT_IWFIFO register fields */
#define SPACEMIT_WFIFO_DATA_MSK      0x000000FF  /* data: bit[7:0] */
#define SPACEMIT_WFIFO_CTRL_MSK      0x000003E0  /* control: bit[11:8] */
#define SPACEMIT_WFIFO_CTRL_START    BIT(8)      /* start bit */
#define SPACEMIT_WFIFO_CTRL_STOP     BIT(9)      /* stop bit */
#define SPACEMIT_WFIFO_CTRL_ACKNAK   BIT(10)     /* send ACK(0) or NAK(1) */
#define SPACEMIT_WFIFO_CTRL_TB       BIT(11)     /* transfer byte bit */

/* i2c bus recover timeout: us */
#define SPACEMIT_I2C_BUS_BUSY_TIMEOUT		100000

#define SPACEMIT_I2C_MAX_STANDARD_MODE_FREQ	100000	/* Hz */
#define SPACEMIT_I2C_MAX_FAST_MODE_FREQ		400000	/* Hz */

#define SPACEMIT_SR_ERR	(SPACEMIT_SR_BED | SPACEMIT_SR_RXOV | SPACEMIT_SR_ALD)

#define SPACEMIT_I2C_TX_FIFO_DEPTH		8
#define SPACEMIT_I2C_RX_FIFO_DEPTH		16

enum spacemit_i2c_state {
	SPACEMIT_STATE_IDLE,
	SPACEMIT_STATE_START,
	SPACEMIT_STATE_READ,
	SPACEMIT_STATE_WRITE,
};

/* i2c-spacemit driver's main struct */
struct spacemit_i2c_dev {
	struct device *dev;
	struct i2c_adapter adapt;

	/* hardware resources */
	void __iomem *base;
	int irq;
	u32 clock_freq;

	struct i2c_msg *msgs;
	u32 msg_num;

	/* index of the current message being processed */
	u32 msg_idx;
	u8 *msg_buf;
	/* the number of unprocessed bytes remaining in the current message  */
	u32 unprocessed;

	enum spacemit_i2c_state state;
	bool read;
	struct completion complete;
	u32 status;
};

static void spacemit_i2c_enable(struct spacemit_i2c_dev *i2c)
{
	u32 val;

	val = readl(i2c->base + SPACEMIT_ICR);
	val |= SPACEMIT_CR_IUE;
	writel(val, i2c->base + SPACEMIT_ICR);
}

static void spacemit_i2c_disable(struct spacemit_i2c_dev *i2c)
{
	u32 val;

	val = readl(i2c->base + SPACEMIT_ICR);
	val &= ~SPACEMIT_CR_IUE;
	writel(val, i2c->base + SPACEMIT_ICR);
}

static void spacemit_i2c_reset(struct spacemit_i2c_dev *i2c)
{
	writel(SPACEMIT_CR_UR, i2c->base + SPACEMIT_ICR);
	udelay(5);
	writel(0, i2c->base + SPACEMIT_ICR);
}

static int spacemit_i2c_handle_err(struct spacemit_i2c_dev *i2c)
{
	dev_dbg(i2c->dev, "i2c error status: 0x%08x\n", i2c->status);

	if (i2c->status & (SPACEMIT_SR_BED | SPACEMIT_SR_ALD)) {
		spacemit_i2c_reset(i2c);
		return -EAGAIN;
	}

	return i2c->status & SPACEMIT_SR_ACKNAK ? -ENXIO : -EIO;
}

static void spacemit_i2c_conditionally_reset_bus(struct spacemit_i2c_dev *i2c)
{
	u32 status;

	/* if bus is locked, reset unit. 0: locked */
	status = readl(i2c->base + SPACEMIT_IBMR);
	if ((status & SPACEMIT_BMR_SDA) && (status & SPACEMIT_BMR_SCL))
		return;

	spacemit_i2c_reset(i2c);
	usleep_range(10, 20);

	/* check scl status again */
	status = readl(i2c->base + SPACEMIT_IBMR);
	if (!(status & SPACEMIT_BMR_SCL))
		dev_warn_ratelimited(i2c->dev, "unit reset failed\n");
}

static int spacemit_i2c_wait_bus_idle(struct spacemit_i2c_dev *i2c)
{
	int ret;
	u32 val;

	val = readl(i2c->base + SPACEMIT_ISR);
	if (!(val & (SPACEMIT_SR_UB | SPACEMIT_SR_IBB)))
		return 0;

	ret = readl_poll_timeout(i2c->base + SPACEMIT_ISR,
				 val, !(val & (SPACEMIT_SR_UB | SPACEMIT_SR_IBB)),
				 1500, SPACEMIT_I2C_BUS_BUSY_TIMEOUT);
	if (ret)
		spacemit_i2c_reset(i2c);

	return ret;
}

static void spacemit_i2c_check_bus_release(struct spacemit_i2c_dev *i2c)
{
	/* in case bus is not released after transfer completes */
	if (readl(i2c->base + SPACEMIT_ISR) & SPACEMIT_SR_EBB) {
		spacemit_i2c_conditionally_reset_bus(i2c);
		usleep_range(90, 150);
	}
}

static void spacemit_i2c_init(struct spacemit_i2c_dev *i2c)
{
	u32 val;

	/*
	 * Unmask interrupt bits for all xfer mode:
	 * bus error, arbitration loss detected.
	 * For transaction complete signal, we use master stop
	 * interrupt, so we don't need to unmask SPACEMIT_CR_TXDONEIE.
	 */
	val = SPACEMIT_CR_BEIE | SPACEMIT_CR_ALDIE;

	/*
	 * Unmask interrupt bits for interrupt xfer mode:
	 * When IDBR receives a byte, an interrupt is triggered.
	 *
	 * For the tx empty interrupt, it will be enabled in the
	 * i2c_start function.
	 * Otherwise, it will cause an erroneous empty interrupt before i2c_start.
	 */
	val |= SPACEMIT_CR_DRFIE;

	if (i2c->clock_freq == SPACEMIT_I2C_MAX_FAST_MODE_FREQ)
		val |= SPACEMIT_CR_MODE_FAST;

	/* disable response to general call */
	val |= SPACEMIT_CR_GCD;

	/* enable SCL clock output */
	val |= SPACEMIT_CR_SCLE;

	/* enable master stop detected */
	val |= SPACEMIT_CR_MSDE | SPACEMIT_CR_MSDIE;

#ifdef I2C_FIFO
	val |= SPACEMIT_CR_FIFOEN;
	val |= SPACEMIT_CR_RXHFIE;
#endif

	writel(val, i2c->base + SPACEMIT_ICR);
}

static inline void
spacemit_i2c_clear_int_status(struct spacemit_i2c_dev *i2c, u32 mask)
{
	writel(mask & SPACEMIT_I2C_INT_STATUS_MASK, i2c->base + SPACEMIT_ISR);
}

static void spacemit_i2c_start(struct spacemit_i2c_dev *i2c)
{
	u32 target_addr_rw, val;
	struct i2c_msg *cur_msg = i2c->msgs + i2c->msg_idx;

	DEBUG(i2c->dev, "i2c start");

	i2c->read = !!(cur_msg->flags & I2C_M_RD);

	i2c->state = SPACEMIT_STATE_START;

	target_addr_rw = (cur_msg->addr & 0x7f) << 1;
	if (cur_msg->flags & I2C_M_RD)
		target_addr_rw |= 1;

#if I2C_FIFO
	writel(target_addr_rw | SPACEMIT_WFIFO_CTRL_START | SPACEMIT_WFIFO_CTRL_TB, i2c->base + SPACEMIT_IWFIFO);
	val = readl(i2c->base + SPACEMIT_ICR);
	val |= SPACEMIT_CR_TXBEGIN;
	val |= SPACEMIT_CR_TXEIE;
	writel(val, i2c->base + SPACEMIT_ICR);
#else
	writel(target_addr_rw, i2c->base + SPACEMIT_IDBR);

	/* send start pulse */
	val = readl(i2c->base + SPACEMIT_ICR);
	val &= ~SPACEMIT_CR_STOP;
	val |= SPACEMIT_CR_START | SPACEMIT_CR_TB | SPACEMIT_CR_DTEIE;
	writel(val, i2c->base + SPACEMIT_ICR);
#endif
}

static void spacemit_i2c_stop(struct spacemit_i2c_dev *i2c)
{
	u32 val;

	DEBUG(i2c->dev, "i2c stop");
#ifdef I2C_FIFO
	val = *i2c->msg_buf;
	val |= SPACEMIT_WFIFO_CTRL_TB | SPACEMIT_WFIFO_CTRL_STOP;
	if (i2c->read)
		val |= SPACEMIT_WFIFO_CTRL_ACKNAK;
	writel(val, i2c->base + SPACEMIT_IWFIFO);
	/* TODO: if we don't add delay here, the i2c hadrware will not detect master stop signal */
	udelay(500);
	if (i2c->read)
		*i2c->msg_buf = readl(i2c->base + SPACEMIT_IRFIFO);
	i2c->unprocessed --;
#else
	val = readl(i2c->base + SPACEMIT_ICR);
	val |= SPACEMIT_CR_STOP | SPACEMIT_CR_ALDIE | SPACEMIT_CR_TB;

	if (i2c->read)
		val |= SPACEMIT_CR_ACKNAK;

	writel(val, i2c->base + SPACEMIT_ICR);
#endif
}

static int spacemit_i2c_xfer_msg(struct spacemit_i2c_dev *i2c)
{
	unsigned long time_left;
	struct i2c_msg *msg;

	for (i2c->msg_idx = 0; i2c->msg_idx < i2c->msg_num; i2c->msg_idx++) {
		DEBUG(i2c->dev, "next msg");
		msg = &i2c->msgs[i2c->msg_idx];
		i2c->msg_buf = msg->buf;
		i2c->unprocessed = msg->len;
		i2c->status = 0;

		reinit_completion(&i2c->complete);

		spacemit_i2c_start(i2c);

		time_left = wait_for_completion_timeout(&i2c->complete,
							i2c->adapt.timeout);
		if (!time_left) {
			dev_err(i2c->dev, "msg completion timeout\n");
			spacemit_i2c_conditionally_reset_bus(i2c);
			spacemit_i2c_reset(i2c);
			return -ETIMEDOUT;
		}

		if (i2c->status & SPACEMIT_SR_ERR)
			return spacemit_i2c_handle_err(i2c);
	}

	return 0;
}

static bool spacemit_i2c_is_last_msg(struct spacemit_i2c_dev *i2c)
{
	if (i2c->msg_idx != i2c->msg_num - 1)
		return false;

#ifdef I2C_FIFO
	return i2c->unprocessed == 1;
#else
	if (i2c->read)
		return i2c->unprocessed == 1;

	return !i2c->unprocessed;
#endif
}

static void spacemit_i2c_fill_transmit_buf(struct spacemit_i2c_dev *i2c)
{
#if I2C_FIFO
	int fill = 0, i;
	size_t unprocessed;
	u16 len;
	u32 data_buf[SPACEMIT_I2C_TX_FIFO_DEPTH * 2];
	u32 data;
	u32 val;

	unprocessed = i2c->unprocessed;
	if (i2c->msg_idx == i2c->msg_num - 1)
		unprocessed -= 1;

	len = min_t(size_t,
		    unprocessed,
		    SPACEMIT_I2C_TX_FIFO_DEPTH - fill);

	DEBUG(i2c->dev, "fill len: %d", len);
	len += fill;

	for (; fill < len; fill ++) {
		data = *(i2c->msg_buf++);
		i2c->unprocessed --;
		data |= SPACEMIT_WFIFO_CTRL_TB;
		data_buf[fill] = data;
	}

	for (i = 0; i < fill; i++) {
		/*dev_err(i2c->dev, "write: %x\n", data_buf[i]);*/
		writel(data_buf[i], i2c->base + SPACEMIT_IWFIFO);
		u32 count = 0;
		/*count = spacemit_i2c_read_reg(i2c, IWFIFO_WPTR);*/
		/*dev_err(i2c->dev, "write count: %d\n", count & 0xf);*/
	}
	val = readl(i2c->base + SPACEMIT_ICR);
	val |= SPACEMIT_CR_TXEIE;
	writel(val, i2c->base + SPACEMIT_ICR);
#else
	spacemit_i2c_write_reg(i2c, IDBR, *i2c->msg_buf++);
	i2c->unprocessed--;
#endif	
}

static void spacemit_i2c_handle_write(struct spacemit_i2c_dev *i2c)
{
	/* if transfer completes, SPACEMIT_ISR will handle it */
	if (i2c->status & SPACEMIT_SR_MSD)
		return;

	if (i2c->unprocessed) {
		/*writel(*i2c->msg_buf++, i2c->base + SPACEMIT_IDBR);*/
		/*i2c->unprocessed--;*/
		spacemit_i2c_fill_transmit_buf(i2c);
		return;
	}

	/* SPACEMIT_STATE_IDLE avoids trigger next byte */
	i2c->state = SPACEMIT_STATE_IDLE;

	complete(&i2c->complete);
}

static void spacemit_i2c_prepare_read(struct spacemit_i2c_dev *i2c)
{
#if I2C_FIFO
	u32 data_buf[SPACEMIT_I2C_RX_FIFO_DEPTH];
	u16 len;
	u32 unprocessed;
	int fill = 0, i;
	u32 data;
	char *msg_buf = i2c->msg_buf;
/*
	if (i2c->unprocessed == i2c->cur_msg->len) {
		data = i2c->slave_addr_rw;
		data |= WFIFO_CTRL_TB | WFIFO_CTRL_START;
		data_buf[fill++] = data;
	}
*/
	unprocessed = i2c->unprocessed;
	if (i2c->msg_idx == i2c->msg_num - 1)
		unprocessed -= 1;

	len = min_t(size_t,
		    unprocessed,
		    SPACEMIT_I2C_TX_FIFO_DEPTH - fill);

	/*if(len == i2c->unprocessed && len != 1)*/
		/*len -= 1;*/

	/*dev_err(i2c->dev, "fill len: %d\n", len);*/
	len += fill;

	for (; fill < len; fill ++) {
		data = *(msg_buf++);
		data |= SPACEMIT_WFIFO_CTRL_TB;
		data_buf[fill] = data;
	}

	for (i = 0; i < fill; i++) {
		/*dev_err(i2c->dev, "write: %x\n", data_buf[i]);*/
		writel(data_buf[i], i2c->base + SPACEMIT_IWFIFO);
		/*u32 count = 0;*/
		/*count = spacemit_i2c_read_reg(i2c, IWFIFO_WPTR);*/
		/*dev_err(i2c->dev, "write count: %d\n", count & 0xf);*/
	}

	udelay(500);

	while (len > 0) {
		*(i2c->msg_buf++) = readl(i2c->base + SPACEMIT_IRFIFO);
		i2c->unprocessed --;
		len --;
		/*dev_err(i2c->dev, "prepare read: %x\n", *(i2c->msg_buf - 1));*/
	}
#else
	*i2c->msg_buf++ = spacemit_i2c_read_reg(i2c, IDBR);
	i2c->unprocessed--;
#endif
}

static void spacemit_i2c_handle_read(struct spacemit_i2c_dev *i2c)
{
	if (i2c->unprocessed)
		spacemit_i2c_prepare_read(i2c);

	/* if transfer completes, SPACEMIT_ISR will handle it */
	if (i2c->status & (SPACEMIT_SR_MSD | SPACEMIT_SR_ACKNAK))
		return;

	/* it has to append stop bit in icr that read last byte */
	if (i2c->unprocessed)
		return;

	/* SPACEMIT_STATE_IDLE avoids trigger next byte */
	i2c->state = SPACEMIT_STATE_IDLE;
	complete(&i2c->complete);
}

static void spacemit_i2c_handle_start(struct spacemit_i2c_dev *i2c)
{
	u32 val;

	i2c->state = i2c->read ? SPACEMIT_STATE_READ : SPACEMIT_STATE_WRITE;
	if (i2c->state == SPACEMIT_STATE_WRITE) {
		spacemit_i2c_handle_write(i2c);
		return;
	}

	/* We have to enable TXEIE to trigger interrupt */
	val = readl(i2c->base + SPACEMIT_ICR);
	val |= SPACEMIT_CR_TXEIE;
	writel(val, i2c->base + SPACEMIT_ICR);
}

static void spacemit_i2c_err_check(struct spacemit_i2c_dev *i2c)
{
	u32 val;

	/*
	 * Send transaction complete signal:
	 * error happens, detect master stop
	 */
	if (!(i2c->status & (SPACEMIT_SR_ERR | SPACEMIT_SR_MSD)))
		return;

	DEBUG(i2c->dev, "done");
	/*
	 * Here the transaction is already done, we don't need any
	 * other interrupt signals from now, in case any interrupt
	 * happens before spacemit_i2c_xfer to disable irq and i2c unit,
	 * we mask all the interrupt signals and clear the interrupt
	 * status.
	 */
	val = readl(i2c->base + SPACEMIT_ICR);
	val &= ~SPACEMIT_I2C_INT_CTRL_MASK;
	writel(val, i2c->base + SPACEMIT_ICR);

	spacemit_i2c_clear_int_status(i2c, SPACEMIT_I2C_INT_STATUS_MASK);

	i2c->state = SPACEMIT_STATE_IDLE;
	complete(&i2c->complete);
}

static irqreturn_t spacemit_i2c_irq_handler(int irq, void *devid)
{
	struct spacemit_i2c_dev *i2c = devid;
	u32 status, val;

	status = readl(i2c->base + SPACEMIT_ISR);
	if (!status)
		return IRQ_HANDLED;

	i2c->status = status;

	spacemit_i2c_clear_int_status(i2c, status);

	if (i2c->status & SPACEMIT_SR_ERR)
		goto err_out;

	val = readl(i2c->base + SPACEMIT_ICR);
	val &= ~(SPACEMIT_CR_TXEIE | SPACEMIT_CR_TB | SPACEMIT_CR_ACKNAK | SPACEMIT_CR_STOP | SPACEMIT_CR_START);
	writel(val, i2c->base + SPACEMIT_ICR);

	switch (i2c->state) {
	case SPACEMIT_STATE_START:
		DEBUG(i2c->dev, "handle start");
		spacemit_i2c_handle_start(i2c);
		break;
	case SPACEMIT_STATE_READ:
		DEBUG(i2c->dev, "handle read");
		spacemit_i2c_handle_read(i2c);
		break;
	case SPACEMIT_STATE_WRITE:
		DEBUG(i2c->dev, "handle write");
		spacemit_i2c_handle_write(i2c);
		break;
	default:
		break;
	}

	DEBUG(i2c->dev, "i2c->state: %d", i2c->state);
	if (i2c->state != SPACEMIT_STATE_IDLE) {
		if (spacemit_i2c_is_last_msg(i2c)) {
			/* trigger next byte with stop */
			DEBUG(i2c->dev, "before stop: i2c->state: %d, i2c->idx: %d, i2c->unprocessed:%d", i2c->state, i2c->msg_idx, i2c->unprocessed);
			spacemit_i2c_stop(i2c);
		} else {
			/* trigger next byte */
			/*val |= SPACEMIT_CR_ALDIE | SPACEMIT_CR_TB;*/
			/*writel(val, i2c->base + SPACEMIT_ICR);*/
		}
	}

err_out:
	spacemit_i2c_err_check(i2c);
	return IRQ_HANDLED;
}

static void spacemit_i2c_calc_timeout(struct spacemit_i2c_dev *i2c)
{
	unsigned long timeout;
	int idx = 0, cnt = 0;

	for (; idx < i2c->msg_num; idx++)
		cnt += (i2c->msgs + idx)->len + 1;

	/*
	 * Multiply by 9 because each byte in I2C transmission requires
	 * 9 clock cycles: 8 bits of data plus 1 ACK/NACK bit.
	 */
	timeout = cnt * 9 * USEC_PER_SEC / i2c->clock_freq;

	i2c->adapt.timeout = usecs_to_jiffies(timeout + USEC_PER_SEC / 10) / i2c->msg_num;
}

static int spacemit_i2c_xfer(struct i2c_adapter *adapt, struct i2c_msg *msgs, int num)
{
	struct spacemit_i2c_dev *i2c = i2c_get_adapdata(adapt);
	int ret;

	i2c->msgs = msgs;
	i2c->msg_num = num;

	spacemit_i2c_calc_timeout(i2c);

	spacemit_i2c_init(i2c);

	spacemit_i2c_enable(i2c);

	ret = spacemit_i2c_wait_bus_idle(i2c);
	if (!ret)
		spacemit_i2c_xfer_msg(i2c);
	else if (ret < 0)
		dev_dbg(i2c->dev, "i2c transfer error: %d\n", ret);
	else
		spacemit_i2c_check_bus_release(i2c);

	spacemit_i2c_disable(i2c);

	if (ret == -ETIMEDOUT || ret == -EAGAIN)
		dev_err(i2c->dev, "i2c transfer failed, ret %d err 0x%lx\n",
			  ret, i2c->status & SPACEMIT_SR_ERR);

	return ret < 0 ? ret : num;
}

static u32 spacemit_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm spacemit_i2c_algo = {
	.master_xfer = spacemit_i2c_xfer,
	.functionality = spacemit_i2c_func,
};

static int spacemit_i2c_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct device *dev = &pdev->dev;
	struct device_node *of_node = pdev->dev.of_node;
	struct spacemit_i2c_dev *i2c;
	int ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	of_property_read_u32(of_node, "clock-frequency", &i2c->clock_freq);
	if (ret && ret != -EINVAL)
		dev_warn(dev, "failed to read clock-frequency property: %d\n", ret);

	/* For now, this driver doesn't support high-speed. */
	if (!i2c->clock_freq || i2c->clock_freq > SPACEMIT_I2C_MAX_FAST_MODE_FREQ) {
		dev_warn(dev, "unsupported clock frequency %u; using %u\n",
			 i2c->clock_freq, SPACEMIT_I2C_MAX_FAST_MODE_FREQ);
		i2c->clock_freq = SPACEMIT_I2C_MAX_FAST_MODE_FREQ;
	} else if (i2c->clock_freq < SPACEMIT_I2C_MAX_STANDARD_MODE_FREQ) {
		dev_warn(dev, "unsupported clock frequency %u; using %u\n",
			 i2c->clock_freq,  SPACEMIT_I2C_MAX_STANDARD_MODE_FREQ);
		i2c->clock_freq = SPACEMIT_I2C_MAX_STANDARD_MODE_FREQ;
	}

	i2c->dev = &pdev->dev;

	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->base))
		return dev_err_probe(dev, PTR_ERR(i2c->base), "failed to do ioremap");

	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq < 0)
		return dev_err_probe(dev, i2c->irq, "failed to get irq resource");

	ret = devm_request_irq(i2c->dev, i2c->irq, spacemit_i2c_irq_handler,
			       IRQF_NO_SUSPEND | IRQF_ONESHOT, dev_name(i2c->dev), i2c);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq");
	
	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to enable clock");

	/*clk = devm_clk_get_enabled(dev, "func");*/
	/*if (IS_ERR(clk))*/
		/*return dev_err_probe(dev, PTR_ERR(clk), "failed to enable func clock");*/

	/*clk = devm_clk_get_enabled(dev, "bus");*/
	/*if (IS_ERR(clk))*/
		/*return dev_err_probe(dev, PTR_ERR(clk), "failed to enable bus clock");*/

	spacemit_i2c_reset(i2c);

	i2c_set_adapdata(&i2c->adapt, i2c);
	i2c->adapt.owner = THIS_MODULE;
	i2c->adapt.algo = &spacemit_i2c_algo;
	i2c->adapt.dev.parent = i2c->dev;
	i2c->adapt.nr = pdev->id;

	i2c->adapt.dev.of_node = of_node;

	strscpy(i2c->adapt.name, "spacemit-i2c-adapter", sizeof(i2c->adapt.name));

	init_completion(&i2c->complete);

	platform_set_drvdata(pdev, i2c);

	ret = i2c_add_numbered_adapter(&i2c->adapt);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to add i2c adapter");

	return 0;
}

static int spacemit_i2c_remove(struct platform_device *pdev)
{
	struct spacemit_i2c_dev *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adapt);

	return 0;
}

static const struct of_device_id spacemit_i2c_of_match[] = {
	{ .compatible = "spacemit,k1-i2c", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spacemit_i2c_of_match);

static struct platform_driver spacemit_i2c_driver = {
	.probe = spacemit_i2c_probe,
	.remove = spacemit_i2c_remove,
	.driver = {
		.name = "i2c-k1",
		.of_match_table = spacemit_i2c_of_match,
	},
};
module_platform_driver(spacemit_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bus driver for SpacemiT K1 SoC");
