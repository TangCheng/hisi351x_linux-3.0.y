/******************************************************************************
*    NAND Flash Controller V301 Device Driver
*    Copyright (c) 2009-2010 by Hisilicon.
*    All rights reserved.
* ***
*
******************************************************************************/
#include <linux/io.h>
#include "hinfc301.h"

/******************************************************************************/

#define PERI_CRG30                                     (0x00B8)
#define PERI_CRG30_CLK_EN                              (1U << 8)
#define PERI_CRG30_CLK_SEL_99M                         (1U << 16)
/******************************************************************************/

void hinfc301_controller_enable(struct hinfc_host *host, int enable)
{
	unsigned int reg_val = readl(host->sysreg + PERI_CRG30);
	if (enable)
		reg_val |= (PERI_CRG30_CLK_EN | PERI_CRG30_CLK_SEL_99M);
	else
		reg_val &= ~PERI_CRG30_CLK_EN;

	writel(reg_val, (host->sysreg + PERI_CRG30));
}
