/*  arch/arm/mach-lpc313x/cgu.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  CGU driver for LPC313x & LPC315x.
 *
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/cpufreq.h>
#include <mach/hardware.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <mach/cgu.h>
#include <asm/io.h>
#include <asm/div64.h>

/***********************************************************************
* CGU driver package data
***********************************************************************/
static u32 g_clkin_freq[CGU_FIN_SELECT_MAX];

/***********************************************************************
* CGU driver private functions
**********************************************************************/
/***********************************************************************
* calculate int(log2(i))+1
*
* Processing:
*     This algorithm is used to calculate the above said formula.
*     This is used to calculate the madd and msub width in frac div
*     registers.Reasonably fast.
**********************************************************************/
static u32 cgu_fdiv_num_bits(unsigned int i)
{
	u32 x = 0, y = 16;

	for (; y > 0; y = y >> 1) {
		if (i >> y) {
			x += y;
			i = i >> y;
		}
	}

	if (i)
		x++;

	return x;
}

static inline u32 f_mult_m_div_n(u32 f_in, u32 m, u32 n)
{
	u32 temp;
	union {
		unsigned long word[2];
		unsigned long long longword;
	} work;

	/* extract low 16 bits ,multiply */
	work.word[0]=(f_in & 0xFFFF) * m;
	/* extract high 16 bits ,multiply */
	work.word[1]=(f_in >> 16)    * m;
	/* low half of base_clk[1] add to high half of base_clk[0] */
	temp = (work.word[1] &0xFFFF) << 16;
	work.word[0] += temp;
	/* put high half of base_clk[1] in low half */
	work.word[1] >>= 16; 

	/* detect and correct  overflow from adding bottom 16 bits of high word to top of low word */
	if (work.word[0] < temp) {
		work.word[1] ++;
	}
	/* do division ignoring remainder */
	do_div(work.longword, n);
	return work.word[0];
}

/***********************************************************************
*     Decode m value from mdec reg value.
**********************************************************************/
static unsigned int pl550_m(int x)
{
	int m = 1;

	if ((x<0) || (x>0x40000))
		return 0;
	if (x == 0x18003)
		return 1;
	if (x == 0x10003)
		return 2;

	while (x!=0x4000) {
		int new = (x & 1)^((x >> 1) & 1);
		x = (x >> 1) | (new << 14);
		m++;
		if (m > 0x8000) 
			return 0;
	}
	return m+1;
}

/***********************************************************************
*     Decode n value from ndec reg value.
**********************************************************************/
static unsigned int pl550_n( int x)
{
	int n = 1;

	if ((x<0) || (x>0x400)) 
		return 0;
	if ( x == 0x302)
		return 1;
	if ( x == 0x202)
		return 2;

	while (x != 0x80) {
		int new = (((x&1) ^ ((x>>2)&1)) ^ ((x>>3)&1)) ^ ((x>>4)&1);
		x = (x>>1) | (new<<7);
		n++;
		if (n > 255)
			return 0;
	}

	return n+1;
}

/***********************************************************************
*     Decode p value from pdec reg value.
**********************************************************************/
static unsigned int pl550_p(int x)
{
	int p = 1;

	if ((x<0) || (x>0x62))
		return 0;
	if (x==0x62)
		return 1;
	if (x==0x42)
		return 2;

	while (x!=0x10) {
		int new = (x&1)^((x>>2)&1);
		x = (x>>1)|(new<<4);
		p++;
		if (p>31)
			return 0;
	}

	return p+1;
}

/***********************************************************************
*     Finds ESR index corresponding to the requested clock Id.
**********************************************************************/
u32 cgu_clkid2esrid(CGU_CLOCK_ID_T clkid)
{
	u32 esrIndex = (u32)clkid;

	switch (clkid)
	{
	case CGU_SB_I2SRX_BCK0_ID:
	case CGU_SB_I2SRX_BCK1_ID:
	case CGU_SB_SYSCLK_O_ID:
		/* invalid esr index. No ESR register for these clocks */
		esrIndex = CGU_INVALID_ID;
		break;

	case CGU_SB_SPI_CLK_ID:
	case CGU_SB_SPI_CLK_GATED_ID:
		esrIndex = esrIndex - 2;
		break;
	default:
		/* do nothing */
		break;
	}

	return esrIndex;
}
/***********************************************************************
*     Finds BCR index corresponding to the requested domain Id.
**********************************************************************/
u32 cgu_DomainId2bcrid(CGU_DOMAIN_ID_T domainid)
{
	u32 bcridx = CGU_INVALID_ID;
	switch (domainid)
	{
	case CGU_SB_SYS_BASE_ID:
	case CGU_SB_AHB0_APB0_BASE_ID:
	case CGU_SB_AHB0_APB1_BASE_ID:
	case CGU_SB_AHB0_APB2_BASE_ID:
		bcridx = domainid;
		break;
	case CGU_SB_CLK1024FS_BASE_ID:
		bcridx = CGU_SB_NR_BCR - 1;
		break;
	default:
		bcridx = CGU_INVALID_ID;
		break;
	}
	return bcridx;
}
/***********************************************************************
*     Finds domain index and fractional divider index for the requested
*	   clock.
**********************************************************************/
void cgu_ClkId2DomainId(CGU_CLOCK_ID_T clkid, CGU_DOMAIN_ID_T* pDomainId,
                        u32* pSubdomainId)
{
	u32 esrIndex, esrReg;
	u32 fracdiv_base = CGU_INVALID_ID;

	/*    1. Get the domain ID */

	if (clkid <= CGU_SYS_LAST) {
		*pDomainId = CGU_SB_SYS_BASE_ID;
		fracdiv_base = CGU_SB_BASE0_FDIV_LOW_ID;

	} else if (clkid <= CGU_AHB0APB0_LAST) {
		*pDomainId = CGU_SB_AHB0_APB0_BASE_ID;
		fracdiv_base = CGU_SB_BASE1_FDIV_LOW_ID;

	} else 	if (clkid <= CGU_AHB0APB1_LAST) {
		*pDomainId = CGU_SB_AHB0_APB1_BASE_ID;
		fracdiv_base = CGU_SB_BASE2_FDIV_LOW_ID;

	} else 	if (clkid <= CGU_AHB0APB2_LAST) {
		*pDomainId = CGU_SB_AHB0_APB2_BASE_ID;
		fracdiv_base = CGU_SB_BASE3_FDIV_LOW_ID;

	} else 	if (clkid <= CGU_AHB0APB3_LAST) {
		*pDomainId = CGU_SB_AHB0_APB3_BASE_ID;
		fracdiv_base = CGU_SB_BASE4_FDIV_LOW_ID;

	} else 	if (clkid == CGU_PCM_LAST) {
		*pDomainId = CGU_SB_IPINT_BASE_ID;
		fracdiv_base = CGU_SB_BASE5_FDIV_LOW_ID;

	} else 	if (clkid == CGU_UART_LAST) {
		*pDomainId = CGU_SB_UARTCLK_BASE_ID;
		fracdiv_base = CGU_SB_BASE6_FDIV_LOW_ID;

	} else 	if (clkid <= CGU_CLK1024FS_LAST) {
		*pDomainId = CGU_SB_CLK1024FS_BASE_ID;
		fracdiv_base = CGU_SB_BASE7_FDIV_LOW_ID;

	} else 	if (clkid == CGU_I2SRX_BCK0_LAST) {
		*pDomainId = CGU_SB_I2SRX_BCK0_BASE_ID;
		fracdiv_base = CGU_INVALID_ID;

	} else 	if (clkid == CGU_I2SRX_BCK1_LAST) {
		*pDomainId = CGU_SB_I2SRX_BCK1_BASE_ID;
		fracdiv_base = CGU_INVALID_ID;

	} else 	if (clkid <= CGU_SPI_LAST) {
		*pDomainId = CGU_SB_SPI_CLK_BASE_ID;
		fracdiv_base = CGU_SB_BASE10_FDIV_LOW_ID;

	} else {
		*pDomainId = CGU_SB_SYSCLK_O_BASE_ID;
		fracdiv_base = CGU_INVALID_ID;
	}

	*pSubdomainId = CGU_INVALID_ID;

	/* read the clocks ESR to get the fractional divider */
	esrIndex = cgu_clkid2esrid(clkid);

	if (CGU_INVALID_ID != esrIndex) {
		/* read the clocks ESR to get the fractional divider */
		esrReg = CGU_SB->clk_esr[esrIndex];

		/* A clock may not be connected to any sub-domain and it might be
		connected directly to domain. This is also a valid combination. So,
		errror should not be returned */
		if (esrReg & CGU_SB_ESR_ENABLE) {
			*pSubdomainId = CGU_SB_ESR_SEL_GET(esrReg) + fracdiv_base;
		}
	}

}

/***********************************************************************
* Configure the selected fractional divider
*********************************************************************/
/* frac divider config function */
u32 cgu_fdiv_config(u32 fdId, CGU_FDIV_SETUP_T fdivCfg, u32 enable)
{
	u32 conf, maddw, msubw, maxw, fdWidth;
	int madd, msub;

	/* calculating program values to see if they fit in fractional divider*/
	madd = fdivCfg.m - fdivCfg.n;
	msub = -fdivCfg.n;

	/* Find required bit width of madd & msub:*/
	maddw = cgu_fdiv_num_bits((u32)madd);
	msubw = cgu_fdiv_num_bits((u32)fdivCfg.n);
	maxw = (maddw > msubw) ? maddw : msubw;
	fdWidth = CGU_SB_BASE0_FDIV0_W;

	if (fdId == CGU_SB_BASE7_FDIV_LOW_ID) {
		/* for Frac divider 17 the bit width is 13 */
		fdWidth = CGU_SB_BASE7_FDIV0_W;
	}

	/* Calculate Configure parameter:*/
	conf = ((((1 << fdWidth) - 1) &
		(msub << (fdWidth - maxw))) <<
		(fdWidth + CGU_SB_FDC_MADD_POS)) |
		(madd << (fdWidth - maxw + CGU_SB_FDC_MADD_POS));

	/* check whther 50% duty cycle is needed for this divider*/
	if (fdivCfg.stretch)
		conf |= CGU_SB_FDC_STRETCH;
	/* check whehter to enable the divider immediately */
	if (enable)
		conf |= CGU_SB_FDC_RUN;

	/* finally configure the divider*/
	CGU_SB->base_fdc[fdId] = conf;

	return conf;
}

/***********************************************************************
* CGU driver public functions
***********************************************************************/



/***********************************************************************
* Get frequency of requested base domain clock.
**********************************************************************/
u32 cgu_get_base_freq(CGU_DOMAIN_ID_T baseid)
{
	/* get base frequency for the domain */
	return g_clkin_freq[CGU_SB_SSR_FS_GET(CGU_SB->base_ssr[baseid])];
}

/***********************************************************************
* Set frequency of requested base domain.
**********************************************************************/
void cgu_set_base_freq(CGU_DOMAIN_ID_T baseid, u32 fin_sel)
{
	u32 baseSCR;

	/* Switch configuration register*/
	baseSCR = CGU_SB->base_scr[baseid] & ~CGU_SB_SCR_FS_MASK;
	/* If fs1 is currently enabled set refId to fs2 and enable fs2*/
	if (CGU_SB->base_ssr[baseid] & CGU_SB_SCR_EN1) {
		/* check if the selcted frequency is same as requested. If not switch.*/
		if (CGU_SB->base_fs1[baseid] != fin_sel) {
			CGU_SB->base_fs2[baseid] = fin_sel;

			/* Don't touch stop bit in SCR register*/
			CGU_SB->base_scr[baseid] = baseSCR | CGU_SB_SCR_EN2;
		}
	} else {
		/* check if the selcted frequency is same as requested. If not switch.*/
		if (CGU_SB->base_fs2[baseid] != fin_sel) {
			CGU_SB->base_fs1[baseid] = fin_sel;

			/* Don't touch stop bit in SCR register*/
			CGU_SB->base_scr[baseid] = baseSCR | CGU_SB_SCR_EN1;
		}
	}
}

/***********************************************************************
* Configure the selected HPLL
* CGU_FIN_SELECT_HPPLL0 : Is used as Audio PLL
* CGU_FIN_SELECT_HPPLL1 : Is used as System PLL
**********************************************************************/
void cgu_hpll_config(CGU_HPLL_ID_T pllid, CGU_HPLL_SETUP_T* pllsetup)
{
	CGU_HP_CFG_REGS* hppll;
	u32 switched_domains = 0;
	CGU_DOMAIN_ID_T domainId;

	/**********************************************************
	* switch domains connected to HPLL to FFAST automatically
	***********************************************************/
	for (domainId = CGU_SB_BASE_FIRST; domainId < CGU_SB_NR_BASE; domainId++) {
		if (CGU_SB_SSR_FS_GET(CGU_SB->base_ssr[domainId]) ==
			(CGU_FIN_SELECT_HPPLL0 + pllid)) {
			/* switch reference clock in to FFAST */
			cgu_set_base_freq(domainId, CGU_FIN_SELECT_FFAST);
			/* store the domain id to switch back to HPLL */
			switched_domains |= _BIT(domainId);
		}
	}

	/* get PLL regs */
	hppll = &CGU_CFG->hp[pllid];

	/* disable clock, disable skew enable, power down pll,
	* (dis/en)able post divider, (dis/en)able pre-divider,
	* disable free running mode, disable bandsel,
	* enable up limmiter, disable bypass
	*/
	hppll->mode = CGU_HPLL_MODE_PD;

	/* check if pllsetup is valid if not just switch off pll */
	if (pllsetup != NULL) {

		/* Select fin */
		hppll->fin_select = pllsetup->fin_select;

		/* M divider */
		hppll->mdec = pllsetup->mdec & CGU_HPLL_MDEC_MASK;

		/* N divider */
		hppll->ndec = pllsetup->ndec & CGU_HPLL_NDEC_MSK;

		/* P divider */
		hppll->pdec = pllsetup->pdec & CGU_HPLL_PDEC_MSK;

		/* Set bandwidth */
		hppll->selr = pllsetup->selr;
		hppll->seli = pllsetup->seli;
		hppll->selp = pllsetup->selp;

		/* Power up pll */
		hppll->mode = (pllsetup->mode & ~CGU_HPLL_MODE_PD) | CGU_HPLL_MODE_CLKEN;

		/* store the estimated freq in driver data for future clk calcs */
		g_clkin_freq[CGU_FIN_SELECT_HPPLL0 + pllid] = pllsetup->freq;

		/* wait for PLL to lock */
		while ((hppll->status & CGU_HPLL_STATUS_LOCK) == 0);

		/**********************************************************
		* switch domains back to HPLL
		***********************************************************/
		for (domainId = CGU_SB_BASE_FIRST; domainId < CGU_SB_NR_BASE; domainId++) {
			if (switched_domains & _BIT(domainId)) {
				/* switch reference clock in to HPLL */
				cgu_set_base_freq(domainId, CGU_FIN_SELECT_HPPLL0 + pllid);
			}
		}
	}

}

/***********************************************************************
* Set external enable feature for the requested clock
**********************************************************************/
void cgu_clk_set_exten(CGU_CLOCK_ID_T clkid, u32 enable)
{
	switch (clkid)
	{
	case CGU_SB_OTP_PCLK_ID:
	case CGU_SB_PCM_APB_PCLK_ID:
	case CGU_SB_EVENT_ROUTER_PCLK_ID:
	case CGU_SB_ADC_PCLK_ID:
	case CGU_SB_IOCONF_PCLK_ID:
	case CGU_SB_CGU_PCLK_ID:
	case CGU_SB_SYSCREG_PCLK_ID:
	case CGU_SB_DMA_CLK_GATED_ID:
	case CGU_SB_SPI_PCLK_GATED_ID:
	case CGU_SB_SPI_CLK_GATED_ID:
	case CGU_SB_PCM_CLK_IP_ID:
	case CGU_SB_PWM_PCLK_REGS_ID:
		if (enable)
			CGU_SB->clk_pcr[clkid] |= CGU_SB_PCR_EXTEN_EN;
		else
			CGU_SB->clk_pcr[clkid] &= ~CGU_SB_PCR_EXTEN_EN;
		break;
		/* force disable for the following clocks */
	case CGU_SB_I2C0_PCLK_ID:
	case CGU_SB_I2C1_PCLK_ID:
	case CGU_SB_WDOG_PCLK_ID:
	case CGU_SB_UART_APB_CLK_ID:
	case CGU_SB_LCD_PCLK_ID:
		CGU_SB->clk_pcr[clkid] &= ~CGU_SB_PCR_EXTEN_EN;
		break;
	default:
		break;
	}
}

/***********************************************************************
*
* Function: cgu_get_clk_freq
*
* Purpose:
*
* Processing:
*     .
*
* Parameters:
*     i : Number
*
* Outputs: None
*
* Returns:
*
* Notes: None
*
**********************************************************************/
u32 cgu_get_clk_freq(CGU_CLOCK_ID_T clkid)
{
	u32 freq = 0;
	CGU_DOMAIN_ID_T domainId;
	u32 subDomainId;
	int n, m;
	u32 fdcVal;

	/* get domain and frac div info for the clock */
	cgu_ClkId2DomainId(clkid, &domainId, &subDomainId);

	/* get base frequency for the domain */
	freq = g_clkin_freq[CGU_SB_SSR_FS_GET(CGU_SB->base_ssr[domainId])];

	/* direct connection  has no fraction divider*/
	if (subDomainId == CGU_INVALID_ID)
		return freq;

	/* read frac div control register value */
	fdcVal = CGU_SB->base_fdc[subDomainId];

	if (fdcVal & CGU_SB_FDC_RUN)  { /* Is the fracdiv enabled ?*/
		/* Yes, so reverse calculation of madd and msub */
		int msub, madd;

		if (subDomainId != CGU_SB_BASE7_FDIV_LOW_ID) {
			msub = CGU_SB_FDC_MSUB_GET(fdcVal);
			madd = CGU_SB_FDC_MADD_GET(fdcVal);
		} else {
			msub = CGU_SB_FDC17_MSUB_GET(fdcVal);
			madd = CGU_SB_FDC17_MADD_GET(fdcVal);
		}

		/* remove trailing zeros */
		while (!(msub & 1)  && !(madd & 1)) {
			madd = madd >> 1;
			msub = msub >> 1;
		}
		/* compute m and n values */
		n = - msub;
		m = madd + n;
		/* check m and n are non-zero values */
		if ((n == 0) || (m == 0)) {
			return 0;
		}
		/* calculate the frequency based on m and n values */
		freq = (freq * n) / m ;
	}
	/* else There is no fractional divider in the clocks path */

	//printk(KERN_INFO "CGU: Get clock id:%d freq:%d\n", clkid, freq);

	return  freq;
}

/***********************************************************************
* Get frequency of requested clock.
**********************************************************************/
void cgu_set_subdomain_freq(CGU_CLOCK_ID_T clkid, CGU_FDIV_SETUP_T fdiv_cfg)
{
	CGU_DOMAIN_ID_T domainId;
	u32 subDomainId, base_freq, bcrId;

	/* get domain and frac div info for the clock */
	cgu_ClkId2DomainId(clkid, &domainId, &subDomainId);

	/* direct connection  has no fraction divider*/
	if (subDomainId != CGU_INVALID_ID) {
		/* store base freq */
		base_freq = CGU_SB_SSR_FS_GET(CGU_SB->base_ssr[domainId]);
		/* switch domain to FFAST */
		cgu_set_base_freq(domainId, CGU_FIN_SELECT_FFAST);
		/* check if the domain has a BCR*/
		bcrId = cgu_DomainId2bcrid(domainId);
		/* disable all BCRs */
		if (bcrId != CGU_INVALID_ID) {
			CGU_SB->base_bcr[bcrId] = 0;
		}
		/* change fractional divider */
		cgu_fdiv_config(subDomainId, fdiv_cfg, 1);
		/* enable BCRs */
		if (bcrId != CGU_INVALID_ID) {
			CGU_SB->base_bcr[bcrId] = CGU_SB_BCR_FD_RUN;
		}
		/* switch domain to original base frequency */
		cgu_set_base_freq(domainId, base_freq);
	}
}

/***********************************************************************
* Get frequency of requested PLL clock.
**********************************************************************/
u32 cgu_get_pll_freq(CGU_HPLL_ID_T pll_id, u32 infreq)
{
	u32 mdec;     /* 17 bits */
	u32 ndec;     /* 10 bits */
	u32 pdec;     /*  7 bits */
	u32 mode;
	u32 M = 0;
	u32 N = 0;
	u32 P = 0;
	u32 ofreq = 0;


	mdec = CGU_CFG->hp[pll_id].mdec;  /* dec val feedback divider */
	ndec = CGU_CFG->hp[pll_id].ndec;  /* dec val pre-divider */
	pdec = CGU_CFG->hp[pll_id].pdec;  /* dec val post-divider */
	mode = CGU_CFG->hp[pll_id].mode;

	/* calculate clock setting */
	M = pl550_m( mdec);
	N = pl550_n( ndec);
	P = pl550_p( pdec);

	//printk(KERN_INFO "%s decode pll%d m/n/p values\n  (0x%x,0x%x,0x%x decoded to 0x%x,0x%x,0x%x)\n",
	//	__FUNCTION__, pll_id, mdec, ndec, pdec, M, N, P);

	switch (mode) {

	case 0x01: /* 1d */
		if ((M==0)||(N==0)||(P==0)) {
			return 0;
		}

		ofreq = f_mult_m_div_n(infreq,M,N*P); // (P * N)/M = out
		break;

	case 0x09: /* 1c */
		if ((M == 0) || (N == 0)) {
			printk(KERN_WARNING "%s: decode pll bad m/n values\n",__FUNCTION__);
			return -1;
		}

		ofreq = f_mult_m_div_n(infreq,2*M,N);// N/(2*M)  =out
		break;

	case 0x11: /* 1b */
		if ((M == 0)||(P == 0)) {
			printk(KERN_WARNING "%s: decode pll bad m/p values\n",__FUNCTION__);
			return -1;
		}

		ofreq = f_mult_m_div_n(infreq,M,P); // P/M  =out
		break;

	case 0x19: /* 1a */
		if (M==0) {
			printk(KERN_WARNING "%s: decode pll bad m value\n",__FUNCTION__);
			return -1;
		}

		ofreq = 2 * infreq * M;		//  1/(2*M) =out
		break;



	default:
		if(mode & 4) /* if this bit is set then the PLL is powered down */
			return 0;
		else {
			printk(KERN_WARNING "%s: decode pll unknown mode %x\n",__FUNCTION__, mode);
			return -1;
		}
	}

	return ofreq;
}

#if defined (CONFIG_DEBUG_FS)
/*
 * The debugfs stuff below is mostly optimized away when
 * CONFIG_DEBUG_FS is not set.
 */
static int lpc313x_cgu_clocks_show(struct seq_file *s, void *v)
{
	u32 clk_id = CGU_SYS_FIRST;
	u32 end_id = (CGU_SYSCLK_O_LAST + 1);
	char* str[2] = { "OFF", " ON"}; 

	while (clk_id < end_id) {
		seq_printf (s, "clock[%02d] %s(PSR)/%s(PCR) : %d\r\n", clk_id, 
			str[(CGU_SB->clk_psr[clk_id] & 0x1)], 
			str[(CGU_SB->clk_pcr[clk_id] & 0x1)], 
			cgu_get_clk_freq(clk_id));
		clk_id++;
	}

	return 0;
}

static int lpc313x_cgu_clocks_open(struct inode *inode, struct file *file)
{
	return single_open(file, lpc313x_cgu_clocks_show, inode->i_private);
}

static const struct file_operations lpc313x_cgu_clocks_fops = {
	.owner		= THIS_MODULE,
	.open		= lpc313x_cgu_clocks_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void lpc313x_cgu_init_debugfs(void)
{
	struct dentry		*node;

	node = debugfs_create_file("cgu_clks", S_IRUSR, NULL, NULL,
			&lpc313x_cgu_clocks_fops);
	if (IS_ERR(node))
		printk("cgu_init: failed to initialize debugfs for CGU\n");

	return;
}
#else
static void lpc313x_cgu_init_debugfs(void) {}
#endif
/***********************************************************************
* Initialize CGU data structure with PLL frequency passed by the boot 
* loader.
**********************************************************************/
int __init cgu_init(char *str)
{
	int i, j;
	u32 flags;
	/* disable all non-essential clocks, enabel main clocks and wakeup
	 * enables.
	 */
	for(i = 0; i < CGU_SB_NR_CLK; i++) {

		if (i < 32) {
			flags = CGU_WKE_CLKS_0_31;
			j = 0;
		} else	if (i < 64) {
			flags = CGU_WKE_CLKS_32_63;
			j = 32;
		} else if (i < 96) {
			flags = CGU_WKE_CLKS_64_92;
			j = 64;
		}

		if (flags & _BIT((i - j))) {
			CGU_SB->clk_pcr[i] |= CGU_SB_PCR_WAKE_EN |
					CGU_SB_PCR_RUN | CGU_SB_PCR_AUTO;
		} else {
			CGU_SB->clk_pcr[i] &= ~(CGU_SB_PCR_WAKE_EN | CGU_SB_PCR_RUN);
		}
	}
	g_clkin_freq[0] = FFAST_CLOCK;
	g_clkin_freq[1] = 0;
	g_clkin_freq[2] = 0;
	g_clkin_freq[3] = 0;
	g_clkin_freq[4] = 0;
	g_clkin_freq[5] = cgu_get_pll_freq(CGU_HPLL0_ID, FFAST_CLOCK);
	g_clkin_freq[6] = cgu_get_pll_freq(CGU_HPLL1_ID, FFAST_CLOCK);
 	printk(/*KERN_INFO*/ "cgu_init pll set at %d\n", g_clkin_freq[6]);
	
	lpc313x_cgu_init_debugfs();

	return 0;
}


EXPORT_SYMBOL(cgu_get_base_freq);
EXPORT_SYMBOL(cgu_set_base_freq);
EXPORT_SYMBOL(cgu_get_clk_freq);
EXPORT_SYMBOL(cgu_get_pll_freq);
EXPORT_SYMBOL(cgu_set_subdomain_freq);
EXPORT_SYMBOL(cgu_hpll_config);
//EXPORT_SYMBOL(cgu_clk_set_exten);

