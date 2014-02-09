/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

//#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"

#define MX6Q_SABRELITE_SD3_CD		IMX_GPIO_NR(7, 0)
#define MX6Q_SABRELITE_SD3_WP		IMX_GPIO_NR(7, 1)
#define MX6Q_SABRELITE_SD4_CD		IMX_GPIO_NR(2, 6)
#define MX6Q_SABRELITE_SD4_WP		IMX_GPIO_NR(2, 7)
#define MX6Q_SABRELITE_ECSPI1_CS1	IMX_GPIO_NR(3, 19)
#define MX6Q_SABRELITE_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define MX6Q_SABRELITE_CAP_TCH_INT1	IMX_GPIO_NR(1, 9)
#define MX6Q_SABRELITE_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
#define MX6Q_SABRELITE_CAN1_STBY	IMX_GPIO_NR(1, 2)
#define MX6Q_SABRELITE_CAN1_EN		IMX_GPIO_NR(1, 4)
#define MX6Q_SABRELITE_MENU_KEY		IMX_GPIO_NR(2, 1)
#define MX6Q_SABRELITE_BACK_KEY		IMX_GPIO_NR(2, 2)
#define MX6Q_SABRELITE_ONOFF_KEY	IMX_GPIO_NR(2, 3)
#define MX6Q_SABRELITE_HOME_KEY		IMX_GPIO_NR(2, 4)
#define MX6Q_SABRELITE_VOL_UP_KEY	IMX_GPIO_NR(7, 13)
#define MX6Q_SABRELITE_VOL_DOWN_KEY	IMX_GPIO_NR(4, 5)
#define MX6Q_SABRELITE_CSI0_RST		IMX_GPIO_NR(1, 8)
#define MX6Q_SABRELITE_CSI0_PWN		IMX_GPIO_NR(1, 6)

//NEW CODE by PRABA
/* Praba : GPIO for main camera */
#define MX6Q_SABRELITE_CSIO_RSTB        IMX_GPIO_NR(1,1)
#define MX6Q_SABRELITE_CSIO_VSYNC       IMX_GPIO_NR(5,21)
#define MX6Q_SABRELITE_CSIO_PWDN        IMX_GPIO_NR(1,2)
#define MX6Q_SABRELITE_CSIO_HSYNC       IMX_GPIO_NR(5,19)
#define MX6Q_SABRELITE_CSIO_DAT19       IMX_GPIO_NR(6,5)
#define MX6Q_SABRELITE_CSIO_DAT18       IMX_GPIO_NR(6,4)
#define MX6Q_SABRELITE_CSIO_DAT17       IMX_GPIO_NR(6,3)
#define MX6Q_SABRELITE_CSIO_DAT16       IMX_GPIO_NR(6,2)
#define MX6Q_SABRELITE_CSIO_DAT15       IMX_GPIO_NR(6,1)
#define MX6Q_SABRELITE_CSIO_DAT14       IMX_GPIO_NR(6,0)
#define MX6Q_SABRELITE_CSIO_DAT13       IMX_GPIO_NR(5,31)
#define MX6Q_SABRELITE_CSIO_DAT12       IMX_GPIO_NR(6,30)
#define MX6Q_SABRELITE_CSIO_PIXCLK      IMX_GPIO_NR(5,18)


#define MX6Q_SABRELITE_GPIO_MCLK        IMX_GPIO_NR(1,0)        /*  MCLK SAME FOR CAMERA,AUDIO */


/*  Praba : GPIO for 2nd camera - mipi interface */
#define MX6Q_SABRELITE_MIPI_CAM_MCLK    IMX_GPIO_NR(1,4) /* GPIO_4 */
#define MX6Q_SABRELITE_MIPICSI_PWN      IMX_GPIO_NR(1,3) /* GPIO_3 */
#define MX6Q_SABRELITE_MIPICSI_RST      IMX_GPIO_NR(1,5) /* GPIO_5 */

/*Praba : GPIO for I2C */

/* GPIO for I2C3 */
#define MX6Q_SABRELITE_I2C3_SCL          IMX_GPIO_NR(3,17) /* EIM_D17 */
#define MX6Q_SABRELITE_I2C3_SDA          IMX_GPIO_NR(3,18) /* EIM_D18 */
/* GPIO for I2C2*/
#define MX6Q_SABRELITE_I2C2_SCL          IMX_GPIO_NR(4,13) /* KEY_ROW3 */
#define MX6Q_SABRELITE_I2C2_SDA          IMX_GPIO_NR(4,12) /* KEY_COL3 */
/* GPIO for I2C1 */
#define MX6Q_SABRELITE_I2C1_SCL          IMX_GPIO_NR(5,27) /* CSI0_DAT9 */
#define MX6Q_SABRELITE_I2C1_SDA          IMX_GPIO_NR(5,26) /* CSI0_DAT8 */



/* Praba : GPIO for PMIC interrupt */
#define MX6Q_SABRELITE_PFUZE_INT        IMX_GPIO_NR(3,30) 


/* Praba : Audio GPIO pins */
#define MX6Q_SABRELITE_AUD4_RXD		IMX_GPIO_NR(1,15)
#define MX6Q_SABRELITE_AUD4_TXFS	IMX_GPIO_NR(1,14)
#define MX6Q_SABRELITE_AUD4_TXD		IMX_GPIO_NR(1,13)
#define MX6Q_SABRELITE_AUD4_TXC		IMX_GPIO_NR(1,12)

/* Praba : GPRS GPIO PINS */
#define MX6Q_SABRELITE_AUD3_TXC        IMX_GPIO_NR(5,22)
#define MX6Q_SABRELITE_AUD3_TXD        IMX_GPIO_NR(5,23)
#define MX6Q_SABRELITE_AUD3_TXFS       IMX_GPIO_NR(5,24)
#define MX6Q_SABRELITE_AUD3_RXD        IMX_GPIO_NR(5,25)

#define MX6Q_SABRELITE_POWER_ON_N      IMX_GPIO_NR(1,6)
#define MX6Q_SABRELITE_SYSTEM_RESET_N  IMX_GPIO_NR(1,7)
#define MX6Q_SABRELITE_W_DISABLE_N     IMX_GPIO_NR(1,8)
#define MX6Q_SABRELITE_WAKE_N	       IMX_GPIO_NR(1,9)

/* Praba : I2C EXPANDER GPIO PINS*/
#define MX6Q_SABRELITE_SD2_CMD         IMX_GPIO_NR(1,11)

//End of new code by praba

#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9

#define MX6Q_SABRELITE_SD3_WP_PADCFG	(PAD_CTL_PKE | PAD_CTL_PUE |	\
		PAD_CTL_PUS_22K_UP | PAD_CTL_SPEED_MED |	\
		PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define CAMERA_CONFIG
#define LCD_CONFIG
#define I2C_CONFIG
#define PFUZE100 
#define MIPI_CONFIG
#define AUDIO_CONFIG
#define GPRS_CONFIG
#define I2C_EXPN_CONFIG
#define SGTL15000_CONFIG 0

#ifdef PFUZE100
extern int mx6q_sabrelite_init_pfuze100(u32 int_gpio);
#endif

void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk;
static struct clk *clko;
extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern bool enet_to_gpio_6;
static int caam_enabled;

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);

static iomux_v3_cfg_t mx6q_sabrelite_pads[] = {
	/* AUDMUX */                         
	MX6Q_PAD_SD2_DAT0__AUDMUX_AUD4_RXD,
	MX6Q_PAD_SD2_DAT3__AUDMUX_AUD4_TXC,
	MX6Q_PAD_SD2_DAT2__AUDMUX_AUD4_TXD,
	MX6Q_PAD_SD2_DAT1__AUDMUX_AUD4_TXFS,

	/* CAN1  */
	MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,
	MX6Q_PAD_KEY_COL2__CAN1_TXCAN,
	MX6Q_PAD_GPIO_2__GPIO_1_2,		/* STNDBY */
	MX6Q_PAD_GPIO_7__GPIO_1_7,		/* NERR */
	MX6Q_PAD_GPIO_4__GPIO_1_4,		/* Enable */

	/* CCM  */
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk */    
	MX6Q_PAD_GPIO_3__CCM_CLKO2,		/* J5 - Camera MCLK */

	/* ECSPI1 */
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D19__GPIO_3_19,	/*SS1*/

	/* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6Q_PAD_ENET_TX_EN__GPIO_1_28,		/* Micrel RGMII Phy Interrupt */
	MX6Q_PAD_EIM_D23__GPIO_3_23,		/* RGMII reset */

	/* GPIO1 */
	MX6Q_PAD_ENET_RX_ER__GPIO_1_24,		/* J9 - Microphone Detect */

	/* GPIO2 */
	MX6Q_PAD_NANDF_D1__GPIO_2_1,	/* J14 - Menu Button */
	MX6Q_PAD_NANDF_D2__GPIO_2_2,	/* J14 - Back Button */
	MX6Q_PAD_NANDF_D3__GPIO_2_3,	/* J14 - Search Button */
	MX6Q_PAD_NANDF_D4__GPIO_2_4,	/* J14 - Home Button */
	MX6Q_PAD_EIM_A22__GPIO_2_16,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A21__GPIO_2_17,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A20__GPIO_2_18,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A19__GPIO_2_19,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A18__GPIO_2_20,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A17__GPIO_2_21,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A16__GPIO_2_22,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_RW__GPIO_2_26,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_LBA__GPIO_2_27,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB0__GPIO_2_28,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB1__GPIO_2_29,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB3__GPIO_2_31,	/* J12 - Boot Mode Select */

	/* GPIO3 */
	MX6Q_PAD_EIM_DA0__GPIO_3_0,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA1__GPIO_3_1,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA2__GPIO_3_2,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA3__GPIO_3_3,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA4__GPIO_3_4,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA5__GPIO_3_5,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA6__GPIO_3_6,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA7__GPIO_3_7,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA8__GPIO_3_8,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA9__GPIO_3_9,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA10__GPIO_3_10,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA11__GPIO_3_11,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA12__GPIO_3_12,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA13__GPIO_3_13,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA14__GPIO_3_14,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA15__GPIO_3_15,	/* J12 - Boot Mode Select */

	/* GPIO4 */
	MX6Q_PAD_GPIO_19__GPIO_4_5,	/* J14 - Volume Down */

	/* GPIO5 */
	MX6Q_PAD_EIM_WAIT__GPIO_5_0,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A24__GPIO_5_4,	/* J12 - Boot Mode Select */

	/* GPIO6 */
	MX6Q_PAD_EIM_A23__GPIO_6_6,	/* J12 - Boot Mode Select */

	/* GPIO7 */
	MX6Q_PAD_GPIO_17__GPIO_7_12,	/* USB Hub Reset */
	MX6Q_PAD_GPIO_18__GPIO_7_13,	/* J14 - Volume Up */


	/* I2C1, SGTL5000 */
/* OVIYA added */
#ifdef I2C_CONFIG
	MX6Q_PAD_CSI0_DAT9__I2C1_SCL,    /* GPIO5[27] */
	MX6Q_PAD_CSI0_DAT8__I2C1_SDA,	/* GPIO5[26] */
#else
	MX6Q_PAD_EIM_D21__I2C1_SCL,	/* GPIO3[21] */
	MX6Q_PAD_EIM_D28__I2C1_SDA,	/* GPIO3[28] */
#endif


	/* I2C2 Camera, MIPI */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,    /* GPIO4[12] */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,    /* GPIO4[13] */


	/* I2C3 */ /* OVIYA - ADDED*/
#ifdef I2C_CONFIG
	MX6Q_PAD_EIM_D17__I2C3_SCL,	/* GPIO3[17] */
	MX6Q_PAD_EIM_D18__I2C3_SDA,	/* GPIO3[18] */
#else
	MX6Q_PAD_GPIO_5__I2C3_SCL,	/* GPIO1[5] - J7 - Display card */
#ifdef CONFIG_FEC_1588
	MX6Q_PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT,
#else
	MX6Q_PAD_GPIO_16__I2C3_SDA,	/* GPIO7[11] - J15 - RGB connector */
#endif
#endif

	/* DISPLAY */
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,		/* DE */
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,		/* HSync */
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,		/* VSync */
	MX6Q_PAD_DI0_PIN4__IPU1_DI0_PIN4,		/* Contrast */
	MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6Q_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6Q_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6Q_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6Q_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
	MX6Q_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
	MX6Q_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
	MX6Q_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
	MX6Q_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,
	MX6Q_PAD_GPIO_7__GPIO_1_7,		/* J7 - Display Connector GP */
	MX6Q_PAD_GPIO_9__GPIO_1_9,		/* J7 - Display Connector GP */
	MX6Q_PAD_NANDF_D0__GPIO_2_0,		/* J6 - LVDS Display contrast */


	/* PWM1 */
	MX6Q_PAD_SD1_DAT3__PWM1_PWMO,		/* GPIO1[21] */

	/* PWM2 */
	MX6Q_PAD_SD1_DAT2__PWM2_PWMO,		/* GPIO1[19] */

	/* PWM3 */
	MX6Q_PAD_SD1_DAT1__PWM3_PWMO,		/* GPIO1[17] */

	/* PWM4 */
	MX6Q_PAD_SD1_CMD__PWM4_PWMO,		/* GPIO1[18] */

	/* UART1  */
	MX6Q_PAD_SD3_DAT7__UART1_TXD,
	MX6Q_PAD_SD3_DAT6__UART1_RXD,

	/* UART2 for debug */
	MX6Q_PAD_EIM_D26__UART2_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,

	/* USBOTG ID pin */
	MX6Q_PAD_GPIO_1__USBOTG_ID,

	/* USB OC pin */
	MX6Q_PAD_KEY_COL4__USBOH3_USBOTG_OC,
	MX6Q_PAD_EIM_D30__USBOH3_USBH1_OC,

	/* USDHC3 */
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6Q_PAD_SD3_DAT5__GPIO_7_0,		/* J18 - SD3_CD */
	NEW_PAD_CTRL(MX6Q_PAD_SD3_DAT4__GPIO_7_1, MX6Q_SABRELITE_SD3_WP_PADCFG),

	/* USDHC4 */
	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6Q_PAD_NANDF_D6__GPIO_2_6,		/* J20 - SD4_CD */
	MX6Q_PAD_NANDF_D7__GPIO_2_7,		/* SD4_WP */
};

static iomux_v3_cfg_t mx6q_sabrelite_csi0_sensor_pads[] = {
	/* IPU1 Camera */
  
	MX6Q_PAD_CSI0_DAT8__IPU1_CSI0_D_8,    // Oviya : not used
	MX6Q_PAD_CSI0_DAT9__IPU1_CSI0_D_9,    // Oviya : not used
	MX6Q_PAD_CSI0_DAT10__IPU1_CSI0_D_10,  // Oviya : not used
	MX6Q_PAD_CSI0_DAT11__IPU1_CSI0_D_11,  // Oviya : not used

	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6Q_PAD_CSI0_DATA_EN__IPU1_CSI0_DATA_EN,
	MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,
	MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6Q_PAD_GPIO_8__GPIO_1_8,		/* J5 - Camera Reset */ // oviya :not used
	MX6Q_PAD_SD1_DAT0__GPIO_1_16,		/* J5 - Camera GP */    // Oviya not used
	MX6Q_PAD_NANDF_D5__GPIO_2_5,		/* J16 - MIPI GP */     // Oviya not used
	MX6Q_PAD_NANDF_WP_B__GPIO_6_9,		/* J16 - MIPI GP */     // Oviya not used


//      PRABA - CODE - BEGINS
    
//      ADDED BY PRABA - changes in camera 1
#ifdef CAMERA_CONFIG
      MX6Q_PAD_GPIO_1__GPIO_1_1,             // CAMERA RESET - J4  
      MX6Q_PAD_GPIO_2__GPIO_1_2,             // CAMERA PWDN  - J4 
#endif 

};

// oviya : added based on sabresd mipi pads
#if 0
static iomux_v3_cfg_t mx6q_sabrelite_mipi_sensor_pads[] = {
        MX6Q_PAD_GPIO_0__CCM_CLKO,              /* camera clk */

        MX6Q_PAD_SD1_DAT2__GPIO_1_19,           /* camera PWDN */
        MX6Q_PAD_SD1_CLK__GPIO_1_20,            /* camera RESET */
};
#endif
/* Praba : MIPI sensor pads*/
#ifdef MIPI_CONFIG                       
static iomux_v3_cfg_t mx6q_sabrelite_mipi_sensor_pads[] = {
        // second camera//
        MX6Q_PAD_GPIO_4__GPIO_1_4,            //CAM2 - MCLK
        MX6Q_PAD_GPIO_3__GPIO_1_3,            //CAM2 - PWDN - J4
        MX6Q_PAD_GPIO_5__GPIO_1_5,            //CAM2 - RST- J4  
        
};
#endif

/* AUDIO sensor pads */


     /* Praba : These pads are already defined in line 183 */
#if 0
static iomux_v3_cfg_t mx6q_sabrelite_audio_sensor_pads[] = {
	MX6Q_PAD_GPIO_0__AUDIO_CLK,             // AUDIO - MASTER CLK (MCLK)
	MX6Q_PAD_SD2_DAT3__AUDIO_BCLK,          // AUDIO -TXC   
        MX6Q_PAD_SD2_DAT1__AUDIO_DACLRC,  	// AUDIO -TXFS
	MX6Q_PAD_SD2_DAT2__AUDIO_DACDAT,   	// AUDIO -TXD
	MX6Q_PAD_SD2_DAT0__AUDIO_ADCDAT,   	// AUDIO - RXD
};
#endif	

/* GPRS SL808X ensor pads */

#ifdef GPRS_CONFIG 
static iomux_v3_cfg_t mx6q_sabrelite_gprs_sensor_pads[] = {
	/* GPIO PINS */
	MX6Q_PAD_GPIO_7__GPIO_1_7,	/* GPRS_SYSTEM_RESET_N, */
	MX6Q_PAD_GPIO_8__GPIO_1_8,	/* GPRS_W_DISABLE_N, */
	MX6Q_PAD_GPIO_9__GPIO_1_9,	/* GPRS_WAKE_N,  */
	MX6Q_PAD_GPIO_6__GPIO_1_6,	/* GPRS_POWER_ON_N, */
	/* AUDIO PINS */	
	MX6Q_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC,
	MX6Q_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD,
	MX6Q_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS,
	MX6Q_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD,

	/* EXT-USIM PINS  */
//	MX6Q_PAD_SIMRTS__EXT_USIM_RESET,
//	MX6Q_PAD_SIMIO__EXT_USIM_DATA,
//	MX6Q_PAD_SIMCLK__EXT_USIM_CLK,
//	MX6Q_PAD_SIMVCC__EXT_VREG_USIM,
//	MX6Q_PAD_SIMIO__EXT_USIM_DATA,
	/* UART PINS */
//	MX6Q_PAD_GPRS_UART1_RXD,
//	MX6Q_PAD_GPRS_UART1_TXD,
//	MX6Q_PAD_GPRS_UART1_RTS_N,
//	MX6Q_PAD_GPRS_UART1_CTS_N,
	/*USB PINS */
//	MX6Q_PAD_GPRS_USB_D+,
//	MX6Q_PAD_GPRS_USB_D-,
	/* SUPPLY PINS  */
//	MX6Q_PAD_GPRS_VREF_1V8,
//	MX6Q_PAD_GPRS_VCC_3V3_1,
//	MX6Q_PAD_GPRS_VCC_3V3_2,


//	MX6Q_PAD_GPRS_LED_FLASH,
};
#endif

/* I2C EXPANDER */ 
#ifdef I2C_EXPN_CONFIG 
static iomux_v3_cfg_t mx6q_sabrelite_i2c_expander_pads[] = {
	MX6Q_PAD_SD2_CMD__GPIO_1_11,     /*I2C EXPANDER INT */
};
#endif


#ifdef LCD_CONFIG
static iomux_v3_cfg_t mx6q_sabrelite_lcd_pads[] = {
	MX6Q_PAD_SD2_CLK__GPIO_1_10,     /*lcd reset */
	
};
#endif
// PRABA - CODE - ENDS 

static iomux_v3_cfg_t mx6q_sabrelite_hdmi_ddc_pads[] = {
	MX6Q_PAD_KEY_COL3__HDMI_TX_DDC_SCL, /* HDMI DDC SCL */
	MX6Q_PAD_KEY_ROW3__HDMI_TX_DDC_SDA, /* HDMI DDC SDA */
};

static iomux_v3_cfg_t mx6q_sabrelite_i2c2_pads[] = {
	MX6Q_PAD_KEY_COL3__I2C2_SCL,	/* I2C2 SCL */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,	/* I2C2 SDA */
};

#define MX6Q_USDHC_PAD_SETTING(id, speed)	\
mx6q_sd##id##_##speed##mhz[] = {		\
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
}

static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 200);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 200);

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

static int plt_sd_pad_change(unsigned int index, int clock)
{
	/* LOW speed is the default state of SD pads */
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd_pads_50mhz = NULL;

	u32 sd_pads_200mhz_cnt;
	u32 sd_pads_100mhz_cnt;
	u32 sd_pads_50mhz_cnt;

	switch (index) {
	case 2:
		sd_pads_200mhz = mx6q_sd3_200mhz;
		sd_pads_100mhz = mx6q_sd3_100mhz;
		sd_pads_50mhz = mx6q_sd3_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd3_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd3_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd3_50mhz);
		break;
	case 3:
		sd_pads_200mhz = mx6q_sd4_200mhz;
		sd_pads_100mhz = mx6q_sd4_100mhz;
		sd_pads_50mhz = mx6q_sd4_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd4_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd4_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd4_50mhz);
		break;
	default:
		printk(KERN_ERR "no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_200mhz,
							sd_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_100mhz,
							sd_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_50mhz,
							sd_pads_50mhz_cnt);
	}
}

static const struct esdhc_platform_data mx6q_sabrelite_sd3_data __initconst = {
	.cd_gpio = MX6Q_SABRELITE_SD3_CD,
	.wp_gpio = MX6Q_SABRELITE_SD3_WP,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd_pad_change,
};

static const struct esdhc_platform_data mx6q_sabrelite_sd4_data __initconst = {
	.cd_gpio = MX6Q_SABRELITE_SD4_CD,
	.wp_gpio = MX6Q_SABRELITE_SD4_WP,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd_pad_change,
};

static const struct anatop_thermal_platform_data
	mx6q_sabrelite_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_sabrelite_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
}

static int mx6q_sabrelite_fec_phy_init(struct phy_device *phydev)
{
	/* prefer master mode, disable 1000 Base-T capable */
	phy_write(phydev, 0x9, 0x1c00);

	/* min rx data delay */
	phy_write(phydev, 0x0b, 0x8105);
	phy_write(phydev, 0x0c, 0x0000);

	/* max rx/tx clock delay, min rx/tx control delay */
	phy_write(phydev, 0x0b, 0x8104);
	phy_write(phydev, 0x0c, 0xf0f0);
	phy_write(phydev, 0x0b, 0x104);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_sabrelite_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
	.gpio_irq = MX6_ENET_IRQ,
};

static int mx6q_sabrelite_spi_cs[] = {
	MX6Q_SABRELITE_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_sabrelite_spi_data __initconst = {
	.chipselect     = mx6q_sabrelite_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sabrelite_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_sabrelite_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data imx6_sabrelite__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_sabrelite_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_sabrelite_spi_nor_partitions),
	.type = "sst25vf016b",
};
#endif

static struct spi_board_info imx6_sabrelite_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &imx6_sabrelite__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_sabrelite_spi_nor_device,
				ARRAY_SIZE(imx6_sabrelite_spi_nor_device));
}

/* Praba : codes related to sgtl5000 - commented */
#if 0 
static struct mxc_audio_platform_data mx6_sabrelite_audio_data;        


static int mx6_sabrelite_sgtl5000_init(void)
{
	struct clk *clko;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 16000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error:SGTL5000 mclk freq %d out of range!\n", rate);
		clk_put(clko);
		return -1;
	}

	mx6_sabrelite_audio_data.sysclk = rate;
	clk_set_rate(clko, rate);
	clk_enable(clko);
	return 0;
}

static struct imx_ssi_platform_data mx6_sabrelite_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_sabrelite_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 4,
	.init = mx6_sabrelite_sgtl5000_init,
	.hp_gpio = -1,
};


static struct platform_device mx6_sabrelite_audio_device = {
	.name = "imx-sgtl5000",
};
#endif


static struct imxi2c_platform_data mx6q_sabrelite_i2c_data = {
	.bitrate = 100000,
};


/* OV560 MIPI CAMERA ADDED BY OVIYA */
#ifdef MIPI_CONFIG
static void mx6q_mipi_powerdown(int powerdown)
{
        if (powerdown)
                gpio_set_value(MX6Q_SABRELITE_MIPICSI_PWN, 1);
        else
                gpio_set_value(MX6Q_SABRELITE_MIPICSI_PWN, 0);

        msleep(2);
}


static void mx6q_mipi_sensor_io_init(void)
{
     //   if (cpu_is_mx6q())
                mxc_iomux_v3_setup_multiple_pads(mx6q_sabrelite_mipi_sensor_pads,
                        ARRAY_SIZE(mx6q_sabrelite_mipi_sensor_pads));
       // else if (cpu_is_mx6dl())
         //       mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_mipi_sensor_pads,
           //             ARRAY_SIZE(mx6dl_sabresd_mipi_sensor_pads));

        /* Camera reset */
        gpio_request(MX6Q_SABRELITE_MIPICSI_RST, "cam-reset");
        gpio_direction_output(MX6Q_SABRELITE_MIPICSI_RST, 1);

        /* Camera power down */
        gpio_request(MX6Q_SABRELITE_MIPICSI_PWN, "cam-pwdn");
        gpio_direction_output(MX6Q_SABRELITE_MIPICSI_PWN, 1);
        msleep(5);
        gpio_set_value(MX6Q_SABRELITE_MIPICSI_PWN, 0);
        msleep(5);
        gpio_set_value(MX6Q_SABRELITE_MIPICSI_RST, 0);
        msleep(1);
        gpio_set_value(MX6Q_SABRELITE_MIPICSI_RST, 1);
        msleep(5);
        gpio_set_value(MX6Q_SABRELITE_MIPICSI_PWN, 1);
        /*for mx6dl, mipi virtual channel 1 connect to csi 1*/
 //       if (cpu_is_mx6dl())
                mxc_iomux_set_gpr_register(13, 3, 3, 1);
}
static struct fsl_mxc_camera_platform_data mipi_csi2_data = {
        .mclk = 24000000,
        .mclk_source = 0,
        .csi = 1,
        .io_init = mx6q_mipi_sensor_io_init,
        .pwdn = mx6q_mipi_powerdown,
};
#endif 
static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
#if 0
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
#endif
#ifdef AUDIO_CONFIG
	{
		I2C_BOARD_INFO("wm8960", 0x1a),         // Praba Added
	},
#endif
#ifdef LCD_CONFIG
	{
		I2C_BOARD_INFO("lcd-comm", 0x7c),     // Praba added
	},
#endif
#ifdef BATTEREY_CONFIG
	{
		I2C_BOARD_INFO("bq24192", 0x6b),          // Praba added
	},
#endif
#ifdef MIPI_CONFIG
	{
                I2C_BOARD_INFO("ov5640_mipi", 0x3c),
                .platform_data = (void *)&mipi_csi2_data,
	},
#endif
#ifdef I2C_EXPN_CONFIG
       	{
                I2C_BOARD_INFO("pca9535", 0x41),     // Praba added // 0x41 - read mode // 0x40 - write mode
	},
#endif
  	
};



/** ADDED BY PRABAKARAN **/

/*BEGINING OF TEST CODE*/
static void gpio_test(void)
{

	printk("Oviya GPIO TEST BY PRABAKARAN---------------------\n");
	
/* CSI0 PWDN GPIO*/
	gpio_request(MX6Q_SABRELITE_CSIO_PWDN, "cam1-pwdn");
	gpio_direction_output(MX6Q_SABRELITE_CSIO_PWDN, 1);
	msleep(3000);
	gpio_set_value(MX6Q_SABRELITE_CSIO_PWDN, 0);
/* CSI0 RST GPIO*/
	gpio_request(MX6Q_SABRELITE_CSIO_RSTB, "cam1-rst");
	gpio_direction_output(MX6Q_SABRELITE_CSIO_RSTB, 1);
	msleep(3000);
	gpio_set_value(MX6Q_SABRELITE_CSIO_RSTB, 0);

 }
/*END OF TEST CODE*/
#ifdef CAMERA_CONFIG 
static void mx6q_csi0_cam_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(MX6Q_SABRELITE_CSI0_PWN, 1);
	else
		gpio_set_value(MX6Q_SABRELITE_CSI0_PWN, 0);

	msleep(2);
}

static void mx6q_csi0_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_sabrelite_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_sabrelite_csi0_sensor_pads));
#if 0
	/* Camera power down */
	gpio_request(MX6Q_SABRELITE_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(MX6Q_SABRELITE_CSI0_PWN, 1);
	msleep(1);
	gpio_set_value(MX6Q_SABRELITE_CSI0_PWN, 0);
#endif
	/* Camera reset */
	gpio_request(MX6Q_SABRELITE_CSI0_RST, "cam-reset");
	gpio_direction_output(MX6Q_SABRELITE_CSI0_RST, 1);

	gpio_set_value(MX6Q_SABRELITE_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(MX6Q_SABRELITE_CSI0_RST, 1);

	/* For MX6Q GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 */
	mxc_iomux_set_gpr_register(1, 19, 1, 1);
}

static struct fsl_mxc_camera_platform_data camera_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.pwdn = mx6q_csi0_cam_powerdown,
};
#endif

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
#ifdef CAMERA_CONFIG
	{
		I2C_BOARD_INFO("ov564x", 0x3c),
		.platform_data = (void *)&camera_data,
	},
#endif
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(MX6Q_SABRELITE_CAP_TCH_INT1),
	},
};

static void imx6q_sabrelite_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6Q_SABRELITE_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6Q_SABRELITE_USB_OTG_PWR, 0);
}

static void __init imx6q_sabrelite_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(MX6Q_SABRELITE_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6Q_SABRELITE_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(MX6Q_SABRELITE_USB_OTG_PWR, 0);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_sabrelite_usbotg_vbus);
}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_sabrelite_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_sabrelite_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_sabrelite_sata_data = {
	.init = mx6q_sabrelite_sata_init,
	.exit = mx6q_sabrelite_sata_exit,
};
#endif

static struct gpio mx6q_sabrelite_flexcan_gpios[] = {
	{ MX6Q_SABRELITE_CAN1_EN, GPIOF_OUT_INIT_LOW, "flexcan1-en" },
	{ MX6Q_SABRELITE_CAN1_STBY, GPIOF_OUT_INIT_LOW, "flexcan1-stby" },
};

static void mx6q_sabrelite_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(MX6Q_SABRELITE_CAN1_EN, 1);
		gpio_set_value(MX6Q_SABRELITE_CAN1_STBY, 1);
	} else {
		gpio_set_value(MX6Q_SABRELITE_CAN1_EN, 0);
		gpio_set_value(MX6Q_SABRELITE_CAN1_STBY, 0);
	}
}

static const struct flexcan_platform_data
	mx6q_sabrelite_flexcan0_pdata __initconst = {
	.transceiver_switch = mx6q_sabrelite_flexcan0_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data sabrelite_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-SVGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if ((mx6q_revision() > IMX_CHIP_REVISION_1_1))
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x sbarelite board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_sabrelite_hdmi_ddc_pads,
		ARRAY_SIZE(mx6q_sabrelite_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_sabrelite_i2c2_pads,
		ARRAY_SIZE(mx6q_sabrelite_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 1,
	.sec_disp_id = 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko2_clk",
	}, {
	.rev = 4,
	.csi_clk[0] = "clko2_clk",
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};


struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = SZ_128M,
};


static void sabrelite_suspend_enter(void)
{
	/* suspend preparation */
}

static void sabrelite_suspend_exit(void)
{
	/* resume restore */
}
static const struct pm_platform_data mx6q_sabrelite_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = sabrelite_suspend_enter,
	.suspend_exit = sabrelite_suspend_exit,
};

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
}

static struct gpio_keys_button sabrelite_buttons[] = {
	GPIO_BUTTON(MX6Q_SABRELITE_ONOFF_KEY, KEY_POWER, 1, "key-power", 1),
	GPIO_BUTTON(MX6Q_SABRELITE_MENU_KEY, KEY_MENU, 1, "key-memu", 0),
	GPIO_BUTTON(MX6Q_SABRELITE_HOME_KEY, KEY_HOME, 1, "key-home", 0),
	GPIO_BUTTON(MX6Q_SABRELITE_BACK_KEY, KEY_BACK, 1, "key-back", 0),
	GPIO_BUTTON(MX6Q_SABRELITE_VOL_UP_KEY, KEY_VOLUMEUP, 1, "volume-up", 0), 
	GPIO_BUTTON(MX6Q_SABRELITE_VOL_DOWN_KEY, KEY_VOLUMEDOWN, 1, "volume-down", 0),
};

static struct gpio_keys_platform_data sabrelite_button_data = {
	.buttons	= sabrelite_buttons,
	.nbuttons	= ARRAY_SIZE(sabrelite_buttons),
};

static struct platform_device sabrelite_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &sabrelite_button_data,
	}
};

static void __init sabrelite_add_device_buttons(void)
{
	platform_device_register(&sabrelite_button_device);
}
#else
static void __init sabrelite_add_device_buttons(void) {}
#endif

static struct regulator_consumer_supply sabrelite_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sabrelite_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sabrelite_vmmc_consumers),
	.consumer_supplies = sabrelite_vmmc_consumers,
};

static struct fixed_voltage_config sabrelite_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sabrelite_vmmc_init,
};

static struct platform_device sabrelite_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &sabrelite_vmmc_reg_config,
	},
};

#if 0
#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "0-000a",
};

static struct regulator_init_data sgtl5000_sabrelite_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vdda,
};

static struct regulator_init_data sgtl5000_sabrelite_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vddio,
};

static struct regulator_init_data sgtl5000_sabrelite_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 2500000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 0,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vddd_reg_initdata,
};


static struct platform_device sgtl5000_sabrelite_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_sabrelite_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_sabrelite_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int imx6q_init_audio(void)
{
	mxc_register_device(&mx6_sabrelite_audio_device,
			    &mx6_sabrelite_audio_data);
	imx6q_add_imx_ssi(1, &mx6_sabrelite_ssi_pdata);

#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_sabrelite_vdda_reg_devices);
	platform_device_register(&sgtl5000_sabrelite_vddio_reg_devices);
	platform_device_register(&sgtl5000_sabrelite_vddd_reg_devices);
#endif
	return 0;
}
#endif // SGTL15000_CONFIG


/* AUDIO CHIP WM8960 Adeed by Praba - AUDIO WM8960 */

#ifdef AUDIO_CONFIG

static struct imx_ssi_platform_data mx6_sabrelite_ssi_pdata = {      /*Praba : already available in sgtl5000 */
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_sabrelite_audio_wm8960_device = {
	.name = "wm8960-codec",
};

static struct mxc_audio_platform_data wm8960_data;

static int wm8960_clk_enable(int enable)
{
	if (enable)
		clk_enable(clko);
	else
		clk_disable(clko);
	return 0;
}

static int mxc_wm8960_init(void)
{
	
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)){
		pr_err("can't get CLKO clock. \n");
		return PTR_ERR(clko);
	}
	/* both audio codec and camera use CLKO clk*/
	rate = clk_round_rate(clko, 16000000);                     /* Praba : foud to 11.2896Mhz used in most cases*/
	clk_set_rate(clko, rate);

	wm8960_data.sysclk = rate;

	return 0;
}
#if 0
static struct wm8960_pdata wm8960_config_data = {
	.gpio_init = {
		[2] = WM8960_GPIO_FN_DMICCLK,                      /* Praba : DMICCLK,DMICDAT not available in wm8960*/
		[4] = 0x8000 | WM8960_GPIO_FN_DMICDAT,
	},
};
#endif
static struct mxc_audio_platform_data wm8960_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
//	.hp_gpio = SABRELITE_HEADPHONE_DET,                       /* Praba : assigned to (only 1 gpio available) */
	.hp_active_low = 1,
//	.mic_gpio = SABRELITE_MICROPHONE_DET,                     /* Praba : assigned to (only 1 gpio available ) */
	.mic_active_low = 1,
	.init = mxc_wm8960_init,
	.clock_enable = wm8960_clk_enable,
};

static struct regulator_consumer_supply sabrelite_wm8960_consumers[] = {
	REGULATOR_SUPPLY("SPKVDD1", "0-001a"),
	REGULATOR_SUPPLY("SPKVDD2", "0-001a"),
};

static struct regulator_init_data sabrelite_wm8960_init = {
	.constraints = {
		.name = "SPKVDD",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
	},
	.num_consumer_supplies =  ARRAY_SIZE(sabrelite_wm8960_consumers),
	.consumer_supplies = sabrelite_wm8960_consumers,
};

static struct fixed_voltage_config sabrelite_wm8960_reg_config = {
	.supply_name = "SPKVDD",
	.microvolts = 4200000,                                          /* Praba : range -0.3 to 7.0 v , 3.3 in schematic*/
	.gpio = -1 /*SABRELITE_CODEC_PWR_EN*/,                                /* Praba : set with ADCLRC/GPIO_1 */
	.enable_high = 1,
	.enabled_at_boot = 1,
	.init_data = &sabrelite_wm8960_init,
};

static struct platform_device sabrelite_wm8960_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 4,                                                      
	.dev	= {
		.platform_data = &sabrelite_wm8960_reg_config,
	},
};

static int __init imx6q_init_audio_wm8960(void)
{
	platform_device_register(&sabrelite_wm8960_reg_devices);
	mxc_register_device(&mx6_sabrelite_audio_wm8960_device,&wm8960_data);
	imx6q_add_imx_ssi(1,&mx6_sabrelite_ssi_pdata);
	mxc_wm8960_init();
	
	return 0;
}

#endif // AUDIO_CONFIG



static struct platform_pwm_backlight_data mx6_sabrelite_pwm_backlight_data = {
	.pwm_id = 3,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data sabrelite_dvfscore_data = {
	.reg_id = "cpu_vddgp",
	.soc_id = "cpu_vddsoc",
	.pu_id = "cpu_vddvpu",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 0,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

/*!
 * Board specific initialization.
 */
static void __init mx6_sabrelite_board_init(void)
{
	printk("OVIYA LINUX KERNEL --------------------\n");
	int i;
	int ret;
	struct clk *clko2;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	mxc_iomux_v3_setup_multiple_pads(mx6q_sabrelite_pads,
					ARRAY_SIZE(mx6q_sabrelite_pads));

/*Gpio test added by praba*/
	gpio_test();
	
	if (enet_to_gpio_6) {
		iomux_v3_cfg_t enet_gpio_pad =
			MX6Q_PAD_GPIO_6__ENET_IRQ_TO_GPIO_6;
		mxc_iomux_v3_setup_pad(enet_gpio_pad);
	} else {
		/* J5 - Camera GP */
		iomux_v3_cfg_t camera_gpio_pad =
			MX6Q_PAD_GPIO_6__GPIO_1_6;
		mxc_iomux_v3_setup_pad(camera_gpio_pad);
	}



#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = sabrelite_dvfscore_data.reg_id;
	soc_reg_id = sabrelite_dvfscore_data.soc_id;
	pu_reg_id = sabrelite_dvfscore_data.pu_id;
	mx6q_sabrelite_init_uart();
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	imx6q_add_ipuv3(1, &ipu_data[1]);

	for (i = 0; i < ARRAY_SIZE(sabrelite_fb_data); i++)
		imx6q_add_ipuv3fb(i, &sabrelite_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}


	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_imx_i2c(0, &mx6q_sabrelite_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_sabrelite_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_sabrelite_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

// PMIC - added by Praba

#ifdef PFUZE100
       ret = gpio_request(MX6Q_SABRELITE_PFUZE_INT,"pFUZE-int") ;
       if(ret){
             printk(KERN_ERR"request pFUZE-init error !!\n");
             return;
       } else {
             gpio_direction_input(MX6Q_SABRELITE_PFUZE_INT);
             mx6q_sabrelite_init_pfuze100(MX6Q_SABRELITE_PFUZE_INT);
       }
#endif

#ifdef AUDIO_CONFIG
//	strcpy(mxc_i2c0_board_info[0].type, "wm8960");
//	mxc_i2c0_board_info[0].platform_data = &wm8960_config_data;
#endif
	/* SPI */
	imx6q_add_ecspi(0, &mx6q_sabrelite_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabrelite_anatop_thermal_data);
	if (enet_to_gpio_6)
		/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
		mxc_iomux_set_specialbits_register(
			IOMUX_OBSRV_MUX1_OFFSET,
			OBSRV_MUX1_ENET_IRQ,
			OBSRV_MUX1_MASK);
	else
		fec_data.gpio_irq = -1;

	imx6_init_fec(fec_data);
	imx6q_add_pm_imx(0, &mx6q_sabrelite_pm_data);
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_sabrelite_sd4_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabrelite_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabrelite_init_usb();

	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_sabrelite_sata_data);
#else
		mx6q_sabrelite_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	#ifdef SGTL1500_CONFIG
 	imx6q_init_audio();                                                       //commented by praba
	#endif
	#ifdef AUDIO_CONFIG
	imx6q_init_audio_wm8960();
	#endif
	platform_device_register(&sabrelite_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");         
	imx6q_add_asrc(&imx_asrc_data);

	/* release USB Hub reset */
	gpio_set_value(MX6Q_SABRELITE_USB_HUB_RESET, 1);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(3, &mx6_sabrelite_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&sabrelite_dvfscore_data);

	sabrelite_add_device_buttons();

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	ret = gpio_request_array(mx6q_sabrelite_flexcan_gpios,
			ARRAY_SIZE(mx6q_sabrelite_flexcan_gpios));
	if (ret)
		pr_err("failed to request flexcan1-gpios: %d\n", ret);
	else
		imx6q_add_flexcan0(&mx6q_sabrelite_flexcan0_pdata);

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);
	imx6q_add_busfreq();

	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);


}

extern void __iomem *twd_base;
static void __init mx6_sabrelite_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART2_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_sabrelite_timer = {
	.init   = mx6_sabrelite_timer_init,
};

static void __init mx6q_sabrelite_reserve(void)
{
	phys_addr_t phys;
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}

}

/*
 * initialize __mach_desc_MX6Q_SABRELITE data structure.
 */
MACHINE_START(MX6Q_SABRELITE, "Freescale i.MX 6Quad Sabre-Lite Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_sabrelite_board_init,
	.timer = &mx6_sabrelite_timer,
	.reserve = mx6q_sabrelite_reserve,
MACHINE_END
