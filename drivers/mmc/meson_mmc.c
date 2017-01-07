/*
 * (C) Copyright 2016 Carlo Caione <ca...@caione.org>
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */

#include <common.h>
#include <fdtdec.h>
#include <malloc.h>
#include <mmc.h>
#include <asm/io.h>
#include <asm/arch/sd_emmc.h>
#include <dm/device.h>

static inline void *get_regbase(const struct mmc *mmc)
{
	struct meson_mmc_platdata *pdata = mmc->priv;

	return pdata->regbase;
}

static inline uint32_t meson_read(struct mmc *mmc, int offset)
{
	return readl(get_regbase(mmc) + offset);
}

static inline void meson_write(struct mmc *mmc, uint32_t val, int offset)
{
	writel(val, get_regbase(mmc) + offset);
}

static void meson_mmc_config_clock(struct mmc *mmc)
{
	uint32_t meson_mmc_clk = 0;
	unsigned int clk, clk_src, clk_div;

	if (mmc->clock > 12000000) {
		clk = SD_EMMC_CLKSRC_DIV2;
		clk_src = 1;
	} else {
		clk = SD_EMMC_CLKSRC_24M;
		clk_src = 0;
	}
	clk_div = clk / mmc->clock;

	if (mmc->clock < mmc->cfg->f_min)
		mmc->clock = mmc->cfg->f_min;
	if (mmc->clock > mmc->cfg->f_max)
		mmc->clock = mmc->cfg->f_max;

	/* Keep the clock always on */
	meson_mmc_clk |= (1 << CLK_ALWAYS_ON);

	/* 180 phase */
	meson_mmc_clk |= (2 << CLK_CO_PHASE);

	/* clock settings */
	meson_mmc_clk |= (clk_src << CLK_SRC);
	meson_mmc_clk |= (clk_div << CLK_DIV);

	meson_write(mmc, meson_mmc_clk, MESON_SD_EMMC_CLOCK);
}

static int meson_dm_mmc_set_ios(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	uint32_t meson_mmc_cfg = 0;

	meson_mmc_config_clock(mmc);

	meson_mmc_cfg = meson_read(mmc, MESON_SD_EMMC_CFG);

	/* 1-bit mode */
	meson_mmc_cfg &= ~CFG_BUS_WIDTH_MASK;
	meson_mmc_cfg |= (mmc->bus_width / 4 << CFG_BUS_WIDTH);

	/* 512 bytes block length */
	meson_mmc_cfg &= ~CFG_BL_LEN_MASK;
	meson_mmc_cfg |= (9 << CFG_BL_LEN);

	/* Response timeout 256 clk */
	meson_mmc_cfg &= ~CFG_RESP_TIMEOUT_MASK;
	meson_mmc_cfg |= (7 << CFG_RESP_TIMEOUT);

	/* Command-command gap 1024 clk */
	meson_mmc_cfg &= ~CFG_RC_CC_MASK;
	meson_mmc_cfg |= (4 << CFG_RC_CC);

	meson_write(mmc, meson_mmc_cfg, MESON_SD_EMMC_CFG);

	return 0;
}

static void meson_mmc_setup_cmd(struct mmc *mmc, struct mmc_data *data,
				struct mmc_cmd *cmd)
{
	uint32_t meson_mmc_cmd = 0;

	meson_mmc_cmd = ((0x80 | cmd->cmdidx) << CMD_CFG_CMD_INDEX);

	if (cmd->resp_type & MMC_RSP_PRESENT) {
		if (cmd->resp_type & MMC_RSP_136)
			meson_mmc_cmd |= CMD_CFG_RESP_128;

		if (cmd->resp_type & MMC_RSP_BUSY)
			meson_mmc_cmd |= CMD_CFG_R1B;

		if (!(cmd->resp_type & MMC_RSP_CRC))
			meson_mmc_cmd |= CMD_CFG_RESP_NOCRC;
	} else {
		meson_mmc_cmd |= CMD_CFG_NO_RESP;
	}

	if (data) {
		meson_mmc_cmd |= CMD_CFG_DATA_IO;

		if (data->flags == MMC_DATA_WRITE)
			meson_mmc_cmd |= CMD_CFG_DATA_WR;

		if (data->blocks > 1) {
			meson_mmc_cmd |= CMD_CFG_BLOCK_MODE;
			meson_mmc_cmd |= data->blocks;
		} else {
			meson_mmc_cmd |= (data->blocksize & CMD_CFG_LENGTH_MASK);
		}
	}

	meson_mmc_cmd |= CMD_CFG_OWNER;
	meson_mmc_cmd |= CMD_CFG_END_OF_CHAIN;

	meson_write(mmc, meson_mmc_cmd, MESON_SD_EMMC_CMD_CFG);
}

static void meson_mmc_setup_addr(struct mmc *mmc, struct mmc_data *data)
{
	struct meson_mmc_platdata *pdata = mmc->priv;
	unsigned int data_size = 0;
	uint32_t meson_mmc_data_addr = 0;

	if (data) {
		data_size = data->blocks * data->blocksize;

		if (data->flags == MMC_DATA_READ) {
			if (data_size < 0x200) {
				meson_mmc_data_addr =
					(ulong) (get_regbase(mmc) + MESON_SD_EMMC_PING);
				meson_mmc_data_addr |= ADDR_USE_PING_BUF;
			} else {
				invalidate_dcache_range((ulong) data->dest,
							(ulong) (data->dest + data_size));
				meson_mmc_data_addr = (ulong) data->dest;
			}
		} else if (data->flags == MMC_DATA_WRITE) {
			pdata->w_buf = calloc(data_size, sizeof(char));
			memcpy(pdata->w_buf, data->src, data_size);
			flush_dcache_range((ulong) pdata->w_buf,
					  (ulong) (pdata->w_buf + data_size));
			meson_mmc_data_addr = (ulong) pdata->w_buf;
		}
	}

	meson_write(mmc, meson_mmc_data_addr, MESON_SD_EMMC_CMD_DAT);
}

static void meson_mmc_read_response(struct mmc *mmc, struct mmc_data *data,
				    struct mmc_cmd *cmd)
{
	unsigned int data_size = 0;

	if (data) {
		data_size = data->blocks * data->blocksize;
		if ((data_size < 0x200) && (data->flags == MMC_DATA_READ))
			memcpy(data->dest,
			       get_regbase(mmc) + MESON_SD_EMMC_PING,
			       data_size);
	}

	if (cmd->resp_type & MMC_RSP_136) {
		cmd->response[0] = meson_read(mmc, MESON_SD_EMMC_CMD_RSP3);
		cmd->response[1] = meson_read(mmc, MESON_SD_EMMC_CMD_RSP2);
		cmd->response[2] = meson_read(mmc, MESON_SD_EMMC_CMD_RSP1);
		cmd->response[3] = meson_read(mmc, MESON_SD_EMMC_CMD_RSP);
	} else {
		cmd->response[0] = meson_read(mmc, MESON_SD_EMMC_CMD_RSP);
	}
}

static int meson_dm_mmc_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
				 struct mmc_data *data)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	struct meson_mmc_platdata *pdata = mmc->priv;
	uint32_t meson_mmc_irq = 0;
	int ret = 0;

	meson_mmc_setup_cmd(mmc, data, cmd);
	meson_mmc_setup_addr(mmc, data);

	meson_write(mmc, SD_IRQ_ALL, MESON_SD_EMMC_STATUS);
	meson_write(mmc, cmd->cmdarg, MESON_SD_EMMC_CMD_ARG);

	do {
		meson_mmc_irq = meson_read(mmc, MESON_SD_EMMC_STATUS);
	} while(!(meson_mmc_irq & STATUS_END_OF_CHAIN));

	if (meson_mmc_irq & STATUS_RESP_TIMEOUT)
		ret = -ETIMEDOUT;
	else if (meson_mmc_irq & STATUS_ERR_MASK)
		ret = -EIO;

	meson_mmc_read_response(mmc, data, cmd);

	if (data && data->flags == MMC_DATA_WRITE)
		free(pdata->w_buf);

	return ret;
}

static const struct dm_mmc_ops meson_dm_mmc_ops = {
	.send_cmd = meson_dm_mmc_send_cmd,
	.set_ios = meson_dm_mmc_set_ios,
};

static int meson_mmc_ofdata_to_platdata(struct udevice *dev)
{
	struct meson_mmc_platdata *pdata = dev->platdata;
	fdt_addr_t addr;

	addr = dev_get_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	pdata->regbase = (void *)addr;

	return 0;
}

static int meson_mmc_probe(struct udevice *dev)
{
	struct meson_mmc_platdata *pdata = dev_get_platdata(dev);
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct mmc_config *cfg = &pdata->cfg;

	cfg->voltages = MMC_VDD_33_34 | MMC_VDD_32_33 |
			MMC_VDD_31_32 | MMC_VDD_165_195;
	cfg->host_caps = MMC_MODE_8BIT | MMC_MODE_4BIT |
			MMC_MODE_HS_52MHz | MMC_MODE_HS;
	cfg->f_min = 400000;
	cfg->f_max = 50000000;
	cfg->b_max = 256;
	cfg->name = dev->name;

	pdata->mmc.priv = pdata;
	pdata->mmc.clock = 400000;
	upriv->mmc = &pdata->mmc;

	return 0;
}

int meson_mmc_bind(struct udevice *dev)
{
	struct meson_mmc_platdata *pdata = dev_get_platdata(dev);

	return mmc_bind(dev, &pdata->mmc, &pdata->cfg);
}

static const struct udevice_id meson_mmc_match[] = {
	{ .compatible = "amlogic,meson-mmc" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(meson_mmc) = {
	.name = "meson_mmc",
	.id = UCLASS_MMC,
	.of_match = meson_mmc_match,
	.ops = &meson_dm_mmc_ops,
	.probe = meson_mmc_probe,
	.bind = meson_mmc_bind,
	.ofdata_to_platdata = meson_mmc_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct meson_mmc_platdata),
};
