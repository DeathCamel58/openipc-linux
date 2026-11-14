// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3520D V100 SATA PHY driver
 *
 * Copyright (c) 2026 Dylan Corrales <deathcamel58@gmail.com>
 *
 * The PHY is configured through registers inside the AHCI register window
 * (PHY control, OOB control and the per-port FIFOTH and PHYCTL registers) and
 * has its own reset line in PERI_CRG45. The AHCI clocks and controller resets
 * must already be on when the PHY is initialised, which ahci_platform
 * guarantees by enabling them before it initialises its PHYs.
 *
 * Register and field names follow the Hi3520D V100 register description (SATA
 * block). The tuning values are the ones of the vendor BSP (CONFIG_HI_SATA,
 * 3 Gbit/s mode) for this SoC.
 *
 * Differences from the vendor sequence (hi_sata_init(), the data sheet's
 * SATA initialization): the controller, TX, RX and alive resets are released
 * by ahci_platform before the PHY is programmed (the vendor releases them
 * last), DIS_CLK is not set in CTLH while it is written, and the clock
 * selection bits of PERI_CRG45 are not touched.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>

/* "ctrl" window, offsets from SATA_TM (0x50 in the AHCI window) */
#define SATA_PHY0_CTLL			0x04
#define SATA_PHY0_CTLH			0x08
#define SATA_PHY1_CTLL			0x10
#define SATA_PHY1_CTLH			0x14
#define SATA_OOB_CTL			0x1c

/* "ports" window, per port, 0x80 apart */
#define SATA_PORT_STRIDE		0x80
#define SATA_PORT_FIFOTH		0x44
#define SATA_PORT_PHYCTL		0x74

/* SATA_PHYn_CTLL: analog PHY parameters */
/* reserved in the register description, the vendor value has 2 here */
#define SATA_PHY_CTLL_RSVD		GENMASK(31, 30)
#define SATA_PHY_CTLL_MPLL_NCY		GENMASK(29, 25)
#define SATA_PHY_CTLL_MPLL_NCY5		GENMASK(24, 23)
#define SATA_PHY_CTLL_MPLL_INT_CTL	GENMASK(22, 20)
#define SATA_PHY_CTLL_MPLL_PROP_CTL	GENMASK(19, 17)
#define SATA_PHY_CTLL_TX_LVL		GENMASK(16, 12)
#define SATA_PHY_CTLL_LOS_LVL		GENMASK(11, 7)
#define SATA_PHY_CTLL_ACJT_LVL		GENMASK(6, 2)
#define SATA_PHY_CTLL_FAST_TECH		BIT(1)
#define SATA_PHY_CTLL_PDDQ_H		BIT(0)

/* SATA_PHYn_CTLH */
#define SATA_PHY_CTLH_USE_REFCLK_ALT	BIT(13)
#define SATA_PHY_CTLH_MPLL_SS_EN	BIT(10)
#define SATA_PHY_CTLH_CKO_WORD_CON	GENMASK(9, 7)
#define SATA_PHY_CTLH_CKO_ALIVE_CON	GENMASK(6, 5)
#define SATA_PHY_CTLH_WIDE_XFACE	BIT(0)

/* 0x00002521 */
#define SATA_PHY_CTLH_VAL						\
	(SATA_PHY_CTLH_USE_REFCLK_ALT | SATA_PHY_CTLH_MPLL_SS_EN |	\
	 FIELD_PREP_CONST(SATA_PHY_CTLH_CKO_WORD_CON, 2) |		\
	 FIELD_PREP_CONST(SATA_PHY_CTLH_CKO_ALIVE_CON, 1) |		\
	 SATA_PHY_CTLH_WIDE_XFACE)

/* SATA_OOB_CTL: COMWAKE/COMINIT burst spacing limits */
#define SATA_OOB_CTL_VALID		BIT(31)
#define SATA_OOB_CTL_MIN_COMWAKE	GENMASK(30, 24)
#define SATA_OOB_CTL_MAX_COMWAKE	GENMASK(23, 16)
#define SATA_OOB_CTL_MIN_COMINIT	GENMASK(15, 8)
#define SATA_OOB_CTL_MAX_COMINIT	GENMASK(7, 0)

/* 0x84060c15 */
#define SATA_OOB_CTL_VAL						\
	(SATA_OOB_CTL_VALID |						\
	 FIELD_PREP_CONST(SATA_OOB_CTL_MIN_COMWAKE, 0x04) |		\
	 FIELD_PREP_CONST(SATA_OOB_CTL_MAX_COMWAKE, 0x06) |		\
	 FIELD_PREP_CONST(SATA_OOB_CTL_MIN_COMINIT, 0x0c) |		\
	 FIELD_PREP_CONST(SATA_OOB_CTL_MAX_COMINIT, 0x15))

/*
 * SATA_PORT_FIFOTH: bits 13:9 bypass hardware bugs of the controller (data
 * transmit control, record, PM reset, fetch and a Seagate R_ERR issue; the
 * vendor driver sets them on every port, the data sheet does not describe
 * them), the rest are the receive FIFO flow control thresholds.
 */
#define SATA_PORT_FIFOTH_BYPASS_ALL	GENMASK(13, 9)
#define SATA_PORT_FIFOTH_DMAC_RXFIFO_TH	GENMASK(8, 4)
#define SATA_PORT_FIFOTH_LINK_RXFIFO_TH	GENMASK(2, 0)

/* 0x00003f24 */
#define SATA_PORT_FIFOTH_VAL						\
	(SATA_PORT_FIFOTH_BYPASS_ALL |					\
	 FIELD_PREP_CONST(SATA_PORT_FIFOTH_DMAC_RXFIFO_TH, 0x12) |	\
	 FIELD_PREP_CONST(SATA_PORT_FIFOTH_LINK_RXFIFO_TH, 4))

/* SATA_PORT_PHYCTL: the fields the 3 Gbit/s tuning sets */
#define SATA_PORT_PHYCTL_CALIBRATED	BIT(27)
#define SATA_PORT_PHYCTL_SPD_CHG_ACK	BIT(26)
#define SATA_PORT_PHYCTL_DP_RDY		BIT(25)
#define SATA_PORT_PHYCTL_GEN2_EN	BIT(22)
#define SATA_PORT_PHYCTL_LOS_CTL	GENMASK(21, 20)
#define SATA_PORT_PHYCTL_RX_DPLL_MODE	GENMASK(19, 17)
#define SATA_PORT_PHYCTL_RX_EQ_VAL	GENMASK(16, 14)
#define SATA_PORT_PHYCTL_RX_TERM_EN	BIT(13)
#define SATA_PORT_PHYCTL_TX_EDGERATE	GENMASK(11, 10)
#define SATA_PORT_PHYCTL_TX_CKO_EN	BIT(9)
#define SATA_PORT_PHYCTL_RX_ALIGN_EN	BIT(8)
#define SATA_PORT_PHYCTL_TX_ATTEN	GENMASK(6, 4)
#define SATA_PORT_PHYCTL_TX_BOOST	GENMASK(3, 0)

/* Size of the "ctrl" window: up to and including SATA_OOB_CTL */
#define SATA_CTRL_WINDOW_MIN_SIZE	(SATA_OOB_CTL + sizeof(u32))

/* The controller implements two ports (ports implemented 0x3), the PHY registers cover two */
#define SATA_MAX_PORTS			2

/*
 * Settle time after each PHY reset change. The vendor driver (hi_sata_init()
 * in drivers/ata/ahci_sys.c) waits the same time; no data sheet minimum is
 * known.
 */
#define SATA_PHY_RESET_DELAY_MS		20

/**
 * struct hi3520dv100_sata_phy_cfg - PHY tuning values of one SoC
 * @ctll:	SATA_PHYn_CTLL value
 * @portctl:	SATA_PORT_PHYCTL value
 *
 * The analog tuning cannot be read back from the hardware (the registers
 * reset to other values), so it is match data. The number of ports comes from
 * the size of the "ports" register window in the device tree, the other
 * values (CTLH, OOB, FIFOTH) are the same constants in the vendor drivers of
 * all the SoCs checked. A new SoC with this register layout needs one more
 * instance of this structure and a compatible.
 *
 * The values are the vendor's 3 Gbit/s ones; they were tested on one board
 * with one SSD. The vendor's 1.5 Gbit/s variant (CTLL 0x880ec788: LOS_LVL
 * 0xf; PHYCTL 0x0e276716: no GEN2_EN, TX_ATTEN 1) is not used; a slower link
 * can be forced with libata.force=1.5Gbps.
 */
struct hi3520dv100_sata_phy_cfg {
	u32 ctll;
	u32 portctl;
};

/* 0x880ec888 and 0x0e676706 */
static const struct hi3520dv100_sata_phy_cfg hi3520dv100_sata_phy_cfg = {
	.ctll = FIELD_PREP_CONST(SATA_PHY_CTLL_RSVD, 2) |
		FIELD_PREP_CONST(SATA_PHY_CTLL_MPLL_NCY, 4) |
		FIELD_PREP_CONST(SATA_PHY_CTLL_MPLL_PROP_CTL, 7) |
		FIELD_PREP_CONST(SATA_PHY_CTLL_TX_LVL, 0x0c) |
		FIELD_PREP_CONST(SATA_PHY_CTLL_LOS_LVL, 0x11) |
		FIELD_PREP_CONST(SATA_PHY_CTLL_ACJT_LVL, 2),
	.portctl = SATA_PORT_PHYCTL_CALIBRATED | SATA_PORT_PHYCTL_SPD_CHG_ACK |
		   SATA_PORT_PHYCTL_DP_RDY | SATA_PORT_PHYCTL_GEN2_EN |
		   FIELD_PREP_CONST(SATA_PORT_PHYCTL_LOS_CTL, 2) |
		   FIELD_PREP_CONST(SATA_PORT_PHYCTL_RX_DPLL_MODE, 3) |
		   FIELD_PREP_CONST(SATA_PORT_PHYCTL_RX_EQ_VAL, 5) |
		   SATA_PORT_PHYCTL_RX_TERM_EN |
		   FIELD_PREP_CONST(SATA_PORT_PHYCTL_TX_EDGERATE, 1) |
		   SATA_PORT_PHYCTL_TX_CKO_EN | SATA_PORT_PHYCTL_RX_ALIGN_EN |
		   FIELD_PREP_CONST(SATA_PORT_PHYCTL_TX_BOOST, 6),
};

struct hi3520dv100_sata_phy {
	void __iomem *ctrl;
	void __iomem *ports;
	unsigned int nports;
	struct reset_control *rst;
	const struct hi3520dv100_sata_phy_cfg *cfg;
};

static int hi3520dv100_sata_phy_init(struct phy *phy)
{
	struct hi3520dv100_sata_phy *priv = phy_get_drvdata(phy);
	unsigned int i;
	int ret;

	ret = reset_control_deassert(priv->rst);
	if (ret)
		return ret;
	msleep(SATA_PHY_RESET_DELAY_MS);

	/*
	 * The vendor driver writes both control register sets. The data sheet
	 * lists only the first one and the second reads back as 0 on the
	 * board, so its effect is unknown; it is written for the second port
	 * the way the vendor does.
	 */
	writel(priv->cfg->ctll, priv->ctrl + SATA_PHY0_CTLL);
	writel(SATA_PHY_CTLH_VAL, priv->ctrl + SATA_PHY0_CTLH);
	if (priv->nports > 1) {
		writel(priv->cfg->ctll, priv->ctrl + SATA_PHY1_CTLL);
		writel(SATA_PHY_CTLH_VAL, priv->ctrl + SATA_PHY1_CTLH);
	}
	writel(SATA_OOB_CTL_VAL, priv->ctrl + SATA_OOB_CTL);

	for (i = 0; i < priv->nports; i++) {
		void __iomem *port = priv->ports + i * SATA_PORT_STRIDE;

		writel(SATA_PORT_FIFOTH_VAL, port + SATA_PORT_FIFOTH);
		writel(priv->cfg->portctl, port + SATA_PORT_PHYCTL);
	}

	/* Pulse the PHY reset so that it samples the new configuration */
	ret = reset_control_assert(priv->rst);
	if (ret)
		return ret;
	msleep(SATA_PHY_RESET_DELAY_MS);
	ret = reset_control_deassert(priv->rst);
	if (ret)
		return ret;
	msleep(SATA_PHY_RESET_DELAY_MS);

	return 0;
}

static int hi3520dv100_sata_phy_exit(struct phy *phy)
{
	struct hi3520dv100_sata_phy *priv = phy_get_drvdata(phy);

	return reset_control_assert(priv->rst);
}

static const struct phy_ops hi3520dv100_sata_phy_ops = {
	.init	= hi3520dv100_sata_phy_init,
	.exit	= hi3520dv100_sata_phy_exit,
	.owner	= THIS_MODULE,
};

static void __iomem *hi3520dv100_sata_phy_map(struct platform_device *pdev,
					      const char *name,
					      resource_size_t min_size,
					      resource_size_t *size)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return ERR_PTR(dev_err_probe(dev, -EINVAL,
					     "missing %s registers\n", name));
	if (resource_size(res) < min_size)
		return ERR_PTR(dev_err_probe(dev, -EINVAL,
					     "%s window too small\n", name));

	/*
	 * The registers belong to the AHCI window, which the AHCI driver has
	 * requested: map them without claiming the region.
	 */
	base = devm_ioremap(dev, res->start, resource_size(res));
	if (!base)
		return ERR_PTR(-ENOMEM);
	*size = resource_size(res);

	return base;
}

static int hi3520dv100_sata_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct hi3520dv100_sata_phy *priv;
	resource_size_t size;
	struct phy *phy;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->cfg = device_get_match_data(dev);
	if (!priv->cfg)
		return -EINVAL;

	priv->ctrl = hi3520dv100_sata_phy_map(pdev, "ctrl",
					      SATA_CTRL_WINDOW_MIN_SIZE, &size);
	if (IS_ERR(priv->ctrl))
		return PTR_ERR(priv->ctrl);

	priv->ports = hi3520dv100_sata_phy_map(pdev, "ports", SATA_PORT_STRIDE,
					       &size);
	if (IS_ERR(priv->ports))
		return PTR_ERR(priv->ports);
	priv->nports = size / SATA_PORT_STRIDE;
	if (priv->nports > SATA_MAX_PORTS)
		return dev_err_probe(dev, -EINVAL, "ports window too large\n");

	priv->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->rst))
		return dev_err_probe(dev, PTR_ERR(priv->rst),
				     "cannot get the PHY reset\n");

	phy = devm_phy_create(dev, NULL, &hi3520dv100_sata_phy_ops);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy), "cannot create the PHY\n");

	phy_set_drvdata(phy, priv);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id hi3520dv100_sata_phy_of_match[] = {
	{ .compatible = "hisilicon,hi3520dv100-sata-phy",
	  .data = &hi3520dv100_sata_phy_cfg },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3520dv100_sata_phy_of_match);

static struct platform_driver hi3520dv100_sata_phy_driver = {
	.probe	= hi3520dv100_sata_phy_probe,
	.driver = {
		.name = "hi3520dv100-sata-phy",
		/* the PHY is used by the AHCI driver: no unbind under it */
		.suppress_bind_attrs = true,
		.of_match_table = hi3520dv100_sata_phy_of_match,
	},
};
module_platform_driver(hi3520dv100_sata_phy_driver);

MODULE_DESCRIPTION("HiSilicon Hi3520D V100 SATA PHY driver");
MODULE_AUTHOR("Dylan Corrales <deathcamel58@gmail.com>");
MODULE_LICENSE("GPL");
