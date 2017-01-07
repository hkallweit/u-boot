/*
 * (C) Copyright 2016 Carlo Caione <ca...@caione.org>
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */

#ifndef __SD_EMMC_H__
#define __SD_EMMC_H__

#include <mmc.h>

#define SDIO_PORT_A			0
#define SDIO_PORT_B			1
#define SDIO_PORT_C			2

#define SD_EMMC_BASE_A			0xd0070000
#define SD_EMMC_BASE_B			0xd0072000
#define SD_EMMC_BASE_C			0xd0074000

#define SD_IRQ_ALL			0x3fff

#define SD_EMMC_CLKSRC_24M		24000000
#define SD_EMMC_CLKSRC_DIV2		1000000000

#define CLK_DIV				0
#define CLK_SRC				6
#define CLK_CO_PHASE			8
#define CLK_ALWAYS_ON			24

#define ADDR_USE_PING_BUF		BIT(1)

#define CFG_BUS_WIDTH			0
#define CFG_BUS_WIDTH_MASK		(0x3 << 0)
#define CFG_BL_LEN			4
#define CFG_BL_LEN_MASK			(0xf << 4)
#define CFG_RESP_TIMEOUT		8
#define CFG_RESP_TIMEOUT_MASK		(0xf << 8)
#define CFG_RC_CC			12
#define CFG_RC_CC_MASK			(0xf << 12)

#define STATUS_ERR_MASK			GENMASK(12, 0)
#define STATUS_RXD_ERR_MASK		0xff
#define STATUS_TXD_ERR			BIT(8)
#define STATUS_DESC_ERR			BIT(9)
#define STATUS_RESP_ERR			BIT(10)
#define STATUS_RESP_TIMEOUT		BIT(11)
#define STATUS_DESC_TIMEOUT		BIT(12)
#define STATUS_END_OF_CHAIN		BIT(13)

#define CMD_CFG_LENGTH_MASK		0x1ff
#define CMD_CFG_CMD_INDEX		24
#define CMD_CFG_BLOCK_MODE		BIT(9)
#define CMD_CFG_R1B			BIT(10)
#define CMD_CFG_END_OF_CHAIN		BIT(11)
#define CMD_CFG_NO_RESP			BIT(16)
#define CMD_CFG_DATA_IO			BIT(18)
#define CMD_CFG_DATA_WR			BIT(19)
#define CMD_CFG_RESP_NOCRC		BIT(20)
#define CMD_CFG_RESP_128		BIT(21)
#define CMD_CFG_OWNER			BIT(31)

#define MESON_SD_EMMC_CLOCK		0x000
#define MESON_SD_EMMC_CFG		0x044
#define MESON_SD_EMMC_STATUS		0x048
#define MESON_SD_EMMC_CMD_CFG		0x050
#define MESON_SD_EMMC_CMD_ARG		0x054
#define MESON_SD_EMMC_CMD_DAT		0x058
#define MESON_SD_EMMC_CMD_RSP		0x05c
#define MESON_SD_EMMC_CMD_RSP1		0x060
#define MESON_SD_EMMC_CMD_RSP2		0x064
#define MESON_SD_EMMC_CMD_RSP3		0x068
#define MESON_SD_EMMC_PING		0x400

struct meson_mmc_platdata {
	struct mmc_config cfg;
	struct mmc mmc;
	void *regbase;
	char *w_buf;
};

#endif
