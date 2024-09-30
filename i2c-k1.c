// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Troy Mitchell <troymitchell988@gmail.com>
 */

 #include <linux/clk.h>
 #include <linux/i2c.h>
 #include <linux/iopoll.h>
 #include <linux/module.h>
 #include <linux/of_address.h>
 #include <linux/platform_device.h>
 #include <linux/reset.h>

/* spacemit i2c registers */
#define ICR          0x0		/* Control Register */
#define ISR          0x4		/* Status Register */
#define ISAR         0x8		/* Slave Address Register */
#define IDBR         0xc		/* Data Buffer Register */
#define ILCR         0x10		/* Load Count Register */
#define IWCR         0x14		/* Wait Count Register */
#define IRST_CYC     0x18		/* Bus reset cycle counter */
#define IBMR         0x1c		/* Bus monitor register */
#define IWFIFO       0x20		/* Write FIFO Register */
#define IWFIFO_WPTR  0x24		/* Write FIFO Write Pointer Register */
#define IWFIFO_RPTR  0x28		/* Write FIFO Read Pointer Register */
#define IRFIFO       0x2c		/* Read FIFO Register */
#define IRFIFO_WPTR  0x30		/* Read FIFO Write Pointer Register */
#define IRFIFO_RPTR  0x34		/* Read FIFO Read Pointer Register */

/* register ICR fields */
#define CR_START        BIT(0)		/* start bit */
#define CR_STOP         BIT(1)		/* stop bit */
#define CR_ACKNAK       BIT(2)		/* send ACK(0) or NAK(1) */
#define CR_TB           BIT(3)		/* transfer byte bit */
#define CR_TXBEGIN      BIT(4)		/* transaction begin */
#define CR_FIFOEN       BIT(5)		/* enable FIFO mode */
#define CR_GPIOEN       BIT(6)		/* enable GPIO mode for SCL in HS */
#define CR_DMAEN        BIT(7)		/* enable DMA for TX and RX FIFOs */
#define CR_MODE_FAST    BIT(8)		/* bus mode (master operation) */
#define CR_MODE_HIGH    BIT(9)		/* bus mode (master operation) */
#define CR_UR           BIT(10)		/* unit reset */
#define CR_RSTREQ       BIT(11)		/* i2c bus reset request */
#define CR_MA           BIT(12)		/* master abort */
#define CR_SCLE         BIT(13)		/* master clock enable */
#define CR_IUE          BIT(14)		/* unit enable */
#define CR_HS_STRETCH   BIT(16)		/* I2C hs stretch */
#define CR_ALDIE        BIT(18)		/* enable arbitration interrupt */
#define CR_DTEIE        BIT(19)		/* enable tx interrupts */
#define CR_DRFIE        BIT(20)		/* enable rx interrupts */
#define CR_GCD          BIT(21)		/* general call disable */
#define CR_BEIE         BIT(22)		/* enable bus error ints */
#define CR_SADIE        BIT(23)		/* slave address detected int enable */
#define CR_SSDIE        BIT(24)		/* slave STOP detected int enable */
#define CR_MSDIE        BIT(25)		/* master STOP detected int enable */
#define CR_MSDE         BIT(26)		/* master STOP detected enable */
#define CR_TXDONEIE     BIT(27)		/* transaction done int enable */
#define CR_TXEIE        BIT(28)		/* transmit FIFO empty int enable */
#define CR_RXHFIE       BIT(29)		/* receive FIFO half-full int enable */
#define CR_RXFIE        BIT(30)		/* receive FIFO full int enable */
#define CR_RXOVIE       BIT(31)		/* receive FIFO overrun int enable */

/* register ISR fields */
#define SR_RWM          BIT(13)		/* read/write mode */
#define SR_ACKNAK       BIT(14)		/* ACK/NACK status */
#define SR_UB           BIT(15)		/* unit busy */
#define SR_IBB          BIT(16)		/* i2c bus busy */
#define SR_EBB          BIT(17)		/* early bus busy */
#define SR_ALD          BIT(18)		/* arbitration loss detected */
#define SR_ITE          BIT(19)		/* tx buffer empty */
#define SR_IRF          BIT(20)		/* rx buffer full */
#define SR_GCAD         BIT(21)		/* general call address detected */
#define SR_BED          BIT(22)		/* bus error no ACK/NAK */
#define SR_SAD          BIT(23)		/* slave address detected */
#define SR_SSD          BIT(24)		/* slave stop detected */
#define SR_MSD          BIT(26)		/* master stop detected */
#define SR_TXDONE       BIT(27)		/* transaction done */
#define SR_TXE          BIT(28)		/* tx FIFO empty */
#define SR_RXHF         BIT(29)		/* rx FIFO half-full */
#define SR_RXF          BIT(30)		/* rx FIFO full */
#define SR_RXOV         BIT(31)		/* RX FIFO overrun */

/* register ILCR fields */
#define LCR_SLV         0x000001FF	/* SLV: bit[8:0] */
#define LCR_FLV         0x0003FE00	/* FLV: bit[17:9] */
#define LCR_HLVH        0x07FC0000	/* HLVH: bit[26:18] */
#define LCR_HLVL        0xF8000000	/* HLVL: bit[31:27] */

/* register IWCR fields */
#define WCR_COUNT       0x0000001F	/* COUNT: bit[4:0] */
#define WCR_COUNT1      0x000003E0	/* HS_COUNT1: bit[9:5] */
#define WCR_COUNT2      0x00007C00	/* HS_COUNT2: bit[14:10] */

/* register IBMR fields */
#define BMR_SDA         BIT(0)		/* SDA line level */
#define BMR_SCL         BIT(1)		/* SCL line level */

/* register IWFIFO fields */
#define WFIFO_DATA_MSK      0x000000FF  /* data: bit[7:0] */
#define WFIFO_CTRL_MSK      0x000003E0  /* control: bit[11:8] */
#define WFIFO_CTRL_START    BIT(8)      /* start bit */
#define WFIFO_CTRL_STOP     BIT(9)      /* stop bit */
#define WFIFO_CTRL_ACKNAK   BIT(10)     /* send ACK(0) or NAK(1) */
#define WFIFO_CTRL_TB       BIT(11)     /* transfer byte bit */

/* status register init value */
#define I2C_INT_STATUS_MASK    0xfffc0000  /* SR bits[31:18] */
#define I2C_INT_CTRL_MASK      (CR_ALDIE | CR_DTEIE | CR_DRFIE | \
				CR_BEIE | CR_TXDONEIE | CR_TXEIE | \
				CR_RXHFIE | CR_RXFIE | CR_RXOVIE | \
				CR_MSDIE)

/* i2c bus recover timeout: us */
#define I2C_BUS_RECOVER_TIMEOUT		(100000)

#define I2C_FAST_MODE_FREQ		(400000)

enum spacemit_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
};

enum spacemit_i2c_dir {
	DIR_WRITE,
	DIR_READ
};

/* i2c-spacemit driver's main struct */
struct spacemit_i2c_dev {
	struct device *dev;
	struct i2c_adapter adapt;

	/* hardware resources */
	void __iomem *base;
	struct reset_control *resets;
	int irq;

	struct i2c_msg *msgs;
	int msg_num;
	struct i2c_msg *cur_msg;
	int msg_idx;
	u8 *msg_buf;
	size_t unprocessed;

	enum spacemit_i2c_state state;
	enum spacemit_i2c_dir dir;
	struct completion complete;
	u32 status;
	u32 err;
};

static int spacemit_i2c_handle_err(struct spacemit_i2c_dev *i2c);

static inline u32 spacemit_i2c_read_reg(struct spacemit_i2c_dev *i2c, int reg)
{
	return readl(i2c->base + reg);
}

static inline void
spacemit_i2c_write_reg(struct spacemit_i2c_dev *i2c, int reg, u32 val)
{
	writel(val, i2c->base + reg);
}

static void spacemit_i2c_enable(struct spacemit_i2c_dev *i2c)
{
	u32 val;

	val = spacemit_i2c_read_reg(i2c, ICR);
	spacemit_i2c_write_reg(i2c, ICR, val | CR_IUE);
}

static void spacemit_i2c_disable(struct spacemit_i2c_dev *i2c)
{
	u32 val;

	val = spacemit_i2c_read_reg(i2c, ICR);
	val &= ~CR_IUE;
	spacemit_i2c_write_reg(i2c, ICR, val);
}

static void spacemit_i2c_reset(struct spacemit_i2c_dev *i2c)
{
	spacemit_i2c_write_reg(i2c, ICR, CR_UR);
	udelay(5);
	spacemit_i2c_write_reg(i2c, ICR, 0);
}

static void spacemit_i2c_bus_reset(struct spacemit_i2c_dev *i2c)
{
	u32 status;

	/* if bus is locked, reset unit. 0: locked */
	status = spacemit_i2c_read_reg(i2c, IBMR);

	if (!(status & BMR_SDA) || !(status & BMR_SCL)) {
		spacemit_i2c_reset(i2c);
		usleep_range(10, 20);

		/* check scl status again */
		status = spacemit_i2c_read_reg(i2c, IBMR);

		if (!(status & BMR_SCL))
			dev_alert(i2c->dev, "unit reset failed\n");
	}
}

static int spacemit_i2c_recover_bus_busy(struct spacemit_i2c_dev *i2c)
{
	int ret = 0;
	u32 val;

	if (likely(!(spacemit_i2c_read_reg(i2c, ISR) & (SR_UB | SR_IBB))))
		return 0;

	ret = readl_poll_timeout(i2c->base + ISR,
				 val,
				 !(val & (SR_UB | SR_IBB)),
				 1500,
				 I2C_BUS_RECOVER_TIMEOUT);

	if (unlikely(ret)) {
		spacemit_i2c_reset(i2c);
		ret = -EAGAIN;
	}

	return ret;
}

static void spacemit_i2c_check_bus_release(struct spacemit_i2c_dev *i2c)
{
	/* in case bus is not released after transfer completes */
	if (unlikely(spacemit_i2c_read_reg(i2c, ISR) & SR_EBB)) {
		spacemit_i2c_bus_reset(i2c);
		usleep_range(90, 150);
	}
}

static void spacemit_i2c_init(struct spacemit_i2c_dev *i2c)
{
	u32 val = 0;

	/*
	 * Unmask interrupt bits for all xfer mode:
	 * bus error, arbitration loss detected.
	 * For transaction complete signal, we use master stop
	 * interrupt, so we don't need to unmask CR_TXDONEIE.
	 */
	val |= CR_BEIE | CR_ALDIE;

	/*
	 * Unmask interrupt bits for interrupt xfer mode:
	 * DBR rx full.
	 * For tx empty interrupt CR_DTEIE, we only
	 * need to enable when trigger byte transfer to start
	 * data sending.
	 */
	val |= CR_DRFIE;

	/* set speed bits: default fast mode */
	val |= CR_MODE_FAST;

	/* disable response to general call */
	val |= CR_GCD;

	/* enable SCL clock output */
	val |= CR_SCLE;

	/* enable master stop detected */
	val |= CR_MSDE | CR_MSDIE;

	spacemit_i2c_write_reg(i2c, ICR, val);
}

static inline void
spacemit_i2c_clear_int_status(struct spacemit_i2c_dev *i2c, u32 mask)
{
	spacemit_i2c_write_reg(i2c, ISR, mask & I2C_INT_STATUS_MASK);
}

static void
spacemit_i2c_start(struct spacemit_i2c_dev *i2c)
{
	u32 slave_addr_rw, val;

	i2c->dir = i2c->cur_msg->flags & I2C_M_RD;
	i2c->state = STATE_START;

	if (i2c->cur_msg->flags & I2C_M_RD)
		slave_addr_rw = ((i2c->cur_msg->addr & 0x7f) << 1) | 1;
	else
		slave_addr_rw = (i2c->cur_msg->addr & 0x7f) << 1;

	spacemit_i2c_write_reg(i2c, IDBR, slave_addr_rw);

	val = spacemit_i2c_read_reg(i2c, ICR);

	/* send start pulse */
	val &= ~CR_STOP;
	val |= CR_START | CR_TB | CR_DTEIE;
	spacemit_i2c_write_reg(i2c, ICR, val);
}

static void spacemit_i2c_stop(struct spacemit_i2c_dev *i2c)
{
	u32 val;

	val = spacemit_i2c_read_reg(i2c, ICR);

	val |= CR_STOP | CR_ALDIE | CR_TB;

	if (i2c->dir == DIR_READ)
		val |= CR_ACKNAK;

	spacemit_i2c_write_reg(i2c, ICR, val);
}

static int spacemit_i2c_xfer_msg(struct spacemit_i2c_dev *i2c)
{
	unsigned long time_left;

	if (unlikely(i2c->err))
		return -1;

	for (i2c->msg_idx = 0; i2c->msg_idx < i2c->msg_num; i2c->msg_idx++) {
		i2c->cur_msg = i2c->msgs + i2c->msg_idx;
		i2c->msg_buf = i2c->cur_msg->buf;
		i2c->err = 0;
		i2c->status = 0;
		i2c->unprocessed = i2c->cur_msg->len;

		spacemit_i2c_start(i2c);

		time_left = wait_for_completion_timeout(&i2c->complete,
							i2c->adapt.timeout);

		if (unlikely(time_left == 0)) {
			dev_alert(i2c->dev, "msg completion timeout\n");
			spacemit_i2c_bus_reset(i2c);
			spacemit_i2c_reset(i2c);
			return -ETIMEDOUT;
		}

		if (unlikely(i2c->err))
			return spacemit_i2c_handle_err(i2c);

		init_completion(&i2c->complete);
	}

	return 0;
}

static int spacemit_i2c_is_last_msg(struct spacemit_i2c_dev *i2c)
{
	if (i2c->dir == DIR_READ)
		return (i2c->unprocessed == 1 && i2c->msg_idx == i2c->msg_num - 1) ? 1 : 0;
	else if (i2c->dir == DIR_WRITE)
		return (!i2c->unprocessed && i2c->msg_idx == i2c->msg_num - 1) ? 1 : 0;
	return 0;
}

static void spacemit_i2c_handle_write(struct spacemit_i2c_dev *i2c)
{
	/* if transfer completes, ISR will handle it */
	if (i2c->status & SR_MSD)
		return;

	if (i2c->unprocessed) {
		spacemit_i2c_write_reg(i2c, IDBR, *i2c->msg_buf++);
		i2c->unprocessed--;
		return;
	}

	/* STATE_IDLE avoids trigger next byte */
	i2c->state = STATE_IDLE;
	complete(&i2c->complete);
}

static void spacemit_i2c_handle_read(struct spacemit_i2c_dev *i2c)
{
	if (i2c->unprocessed) {
		*i2c->msg_buf++ = spacemit_i2c_read_reg(i2c, IDBR);
		i2c->unprocessed--;
	}

	/* if transfer completes, ISR will handle it */
	if (i2c->status & (SR_MSD | SR_ACKNAK))
		return;

	/* it has to append stop bit in icr that read last byte */
	if (i2c->unprocessed)
		return;

	/* STATE_IDLE avoids trigger next byte */
	i2c->state = STATE_IDLE;
	complete(&i2c->complete);
}

static void spacemit_i2c_handle_start(struct spacemit_i2c_dev *i2c)
{
	if (i2c->dir == DIR_READ) {
		i2c->state = STATE_READ;
	} else if (i2c->dir == DIR_WRITE) {
		i2c->state = STATE_WRITE;
		spacemit_i2c_handle_write(i2c);
	}
}

static int spacemit_i2c_handle_err(struct spacemit_i2c_dev *i2c)
{
	if (!i2c->err)
		return 0;

	dev_dbg(i2c->dev, "i2c error status: 0x%08x\n",
		i2c->status);

	if (i2c->err & (SR_BED | SR_ALD))
		spacemit_i2c_reset(i2c);

	/* try transfer again */
	if (i2c->err & (SR_RXOV | SR_ALD))
		return -EAGAIN;

	return (i2c->status & SR_ACKNAK) ? -ENXIO : -EIO;
}

static irqreturn_t spacemit_i2c_irq_handler(int irq, void *devid)
{
	struct spacemit_i2c_dev *i2c = devid;
	u32 status, ctrl, val;

	status = spacemit_i2c_read_reg(i2c, ISR);

	/* check if a valid interrupt status */
	if (!status)
		return IRQ_HANDLED;

	i2c->status = status;

	/* bus error, rx overrun, arbitration lost */
	i2c->err = status & (SR_BED | SR_RXOV | SR_ALD);

	/* clear interrupt status bits[31:18] */
	spacemit_i2c_clear_int_status(i2c, status);

	if (unlikely(i2c->err))
		goto err_out;

	val = spacemit_i2c_read_reg(i2c, ICR);

	val &= ~(CR_TB | CR_ACKNAK | CR_STOP | CR_START);

	switch (i2c->state) {
	case STATE_START:
		spacemit_i2c_handle_start(i2c);
		break;
	case STATE_READ:
		spacemit_i2c_handle_read(i2c);
		break;
	case STATE_WRITE:
		spacemit_i2c_handle_write(i2c);
		break;
	default:
		break;
	}

	if (i2c->state != STATE_IDLE) {
		if (spacemit_i2c_is_last_msg(i2c)) {
			/* trigger next byte with stop */
			spacemit_i2c_stop(i2c);
		} else {
			/* trigger next byte */
			val |= CR_ALDIE | CR_TB;
			spacemit_i2c_write_reg(i2c, ICR, val);
		}
	}

err_out:
	/*
	 * send transaction complete signal:
	 * error happens, detect master stop
	 */
	if (likely(i2c->err || (status & SR_MSD))) {
		/*
		 * Here the transaction is already done, we don't need any
		 * other interrupt signals from now, in case any interrupt
		 * happens before spacemit_i2c_xfer to disable irq and i2c unit,
		 * we mask all the interrupt signals and clear the interrupt
		 * status.
		 */
		ctrl = spacemit_i2c_read_reg(i2c, ICR);
		ctrl &= ~I2C_INT_CTRL_MASK;
		spacemit_i2c_write_reg(i2c, ICR, ctrl);

		spacemit_i2c_clear_int_status(i2c, I2C_INT_STATUS_MASK);

		i2c->state = STATE_IDLE;
		complete(&i2c->complete);
	}

	return IRQ_HANDLED;
}

static void spacemit_i2c_calc_timeout(struct spacemit_i2c_dev *i2c)
{
	unsigned long timeout;
	int idx = 0, cnt = 0, freq;

	while (idx < i2c->msg_num) {
		cnt += (i2c->msgs + idx)->len + 1;

		idx++;
	}

	freq = I2C_FAST_MODE_FREQ;

	timeout = cnt * 9 * USEC_PER_SEC / freq;

	i2c->adapt.timeout = usecs_to_jiffies(timeout + USEC_PER_SEC / 2) / i2c->msg_num;
}

static inline int spacemit_i2c_xfer_core(struct spacemit_i2c_dev *i2c)
{
	int ret = 0;

	spacemit_i2c_reset(i2c);

	spacemit_i2c_calc_timeout(i2c);

	spacemit_i2c_init(i2c);

	reinit_completion(&i2c->complete);

	spacemit_i2c_enable(i2c);
	enable_irq(i2c->irq);

	/* i2c wait for bus busy */
	ret = spacemit_i2c_recover_bus_busy(i2c);

	if (unlikely(ret))
		return ret;

	ret = spacemit_i2c_xfer_msg(i2c);

	if (unlikely(ret < 0)) {
		dev_dbg(i2c->dev, "i2c transfer error\n");
		/* timeout error should not be overridden, and the transfer
		 * error will be confirmed by err handle function latter,
		 * the reset should be invalid argument error.
		 */
		if (ret != -ETIMEDOUT)
			ret = -EINVAL;
	}

	return ret;
}

static int
spacemit_i2c_xfer(struct i2c_adapter *adapt, struct i2c_msg msgs[], int num)
{
	struct spacemit_i2c_dev *i2c = i2c_get_adapdata(adapt);
	int ret;

	i2c->msgs = msgs;
	i2c->msg_num = num;

	ret = spacemit_i2c_xfer_core(i2c);

	if (likely(!ret))
		spacemit_i2c_check_bus_release(i2c);

	disable_irq(i2c->irq);

	spacemit_i2c_disable(i2c);

	if (unlikely((ret == -ETIMEDOUT || ret == -EAGAIN)))
		dev_alert(i2c->dev,
			  "i2c transfer failed, ret %d err 0x%x\n", ret, i2c->err);

	return ret < 0 ? ret : num;
}

static u32 spacemit_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm spacemit_i2c_algo = {
	.xfer = spacemit_i2c_xfer,
	.functionality = spacemit_i2c_func,
};

static int spacemit_i2c_probe(struct platform_device *pdev)
{
	struct spacemit_i2c_dev *i2c;
	struct device_node *of_node = pdev->dev.of_node;
	struct clk *clk;
	int ret = 0;

	i2c = devm_kzalloc(&pdev->dev,
			   sizeof(struct spacemit_i2c_dev),
			   GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->dev = &pdev->dev;

	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (!i2c->base) {
		ret = PTR_ERR(i2c->base);
		return dev_err_probe(&pdev->dev, ret, "failed to do ioremap");
	}

	i2c->resets = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(i2c->resets)) {
		ret = PTR_ERR(i2c->resets);
		return dev_err_probe(&pdev->dev, ret, "failed to get resets");
	}

	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq < 0) {
		ret = i2c->irq;
		return dev_err_probe(&pdev->dev, ret, "failed to get irq resource");
	}

	ret = devm_request_irq(i2c->dev, i2c->irq,
			       spacemit_i2c_irq_handler,
			       IRQF_NO_SUSPEND | IRQF_ONESHOT,
			       dev_name(i2c->dev), i2c);

	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to request irq");

	disable_irq(i2c->irq);

	clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		return dev_err_probe(&pdev->dev, ret, "failed to enable clock");
	}

	reset_control_assert(i2c->resets);
	usleep_range(200, 300);
	reset_control_deassert(i2c->resets);

	i2c_set_adapdata(&i2c->adapt, i2c);
	i2c->adapt.owner = THIS_MODULE;
	i2c->adapt.algo = &spacemit_i2c_algo;
	i2c->adapt.dev.parent = i2c->dev;
	i2c->adapt.nr = pdev->id;

	i2c->adapt.dev.of_node = of_node;
	i2c->adapt.algo_data = i2c;

	strscpy(i2c->adapt.name, "spacemit-i2c-adapter", sizeof(i2c->adapt.name));

	init_completion(&i2c->complete);

	ret = i2c_add_numbered_adapter(&i2c->adapt);

	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to add i2c adapter");

	platform_set_drvdata(pdev, i2c);

	return 0;
}

static int spacemit_i2c_remove(struct platform_device *pdev)
{
	struct spacemit_i2c_dev *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adapt);

	return 0;
}

static const struct of_device_id spacemit_i2c_dt_match[] = {
	{ .compatible = "spacemit,k1-i2c", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, spacemit_i2c_dt_match);

static struct platform_driver spacemit_i2c_driver = {
	.probe = spacemit_i2c_probe,
	.remove = spacemit_i2c_remove,
	.driver = {
		.name = "i2c-k1",
		.of_match_table = spacemit_i2c_dt_match,
	},
};

module_platform_driver(spacemit_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bus driver for SpacemiT K1 SoC");
