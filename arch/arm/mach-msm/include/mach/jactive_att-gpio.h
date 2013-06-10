/*
 * jactive_att-gpio.h
 *
 * header file supporting gpio functions for Samsung device
 *
 * COPYRIGHT(C) Samsung Electronics Co., Ltd. 2006-2011 All Right Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define PM8921_GPIO_BASE        NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)
#define PM8921_MPP_BASE         (PM8921_GPIO_BASE + PM8921_NR_GPIOS)
#define PM8921_MPP_PM_TO_SYS(pm_mpp)    (pm_mpp - 1 + PM8921_MPP_BASE)
#define PM8921_IRQ_BASE         (NR_MSM_IRQS + NR_GPIO_IRQS)
/* APQ8064 GPIO */
#define GPIO_S_LED_I2C_SDA		 6
#define GPIO_S_LED_I2C_SCL		 7
#define GPIO_USB_I2C_SDA		10
#define GPIO_USB_I2C_SCL		11
#ifdef CONFIG_MFD_MAX77693
#define GPIO_IF_PMIC_SDA                22
#define GPIO_IF_PMIC_SCL                23
#define GPIO_IF_PMIC_IRQ                55
#endif
#define GPIO_FUELGAUGE_I2C_SDA          0
#define GPIO_FUELGAUGE_I2C_SCL          1
#define GPIO_FUEL_INT                   29
#define GPIO_FUELGAUGE_I2C_SDA_OLD      14
#define GPIO_FUELGAUGE_I2C_SCL_OLD      15
#define GPIO_FUEL_INT_04A               6 /* REV 0.4A, PM GPIO 6 */
#define GPIO_VIB_PWM                    32
#define GPIO_KEY_MENU		PM8921_GPIO_PM_TO_SYS(4)
#define GPIO_KEY_BACK		PM8921_GPIO_PM_TO_SYS(5)
#define GPIO_KEY_VOLUME_UP	PM8921_GPIO_PM_TO_SYS(35)
#define GPIO_KEY_VOLUME_DOWN	PM8921_GPIO_PM_TO_SYS(37)
#define GPIO_KEY_HOME		PM8921_GPIO_PM_TO_SYS(30)
#define GPIO_NFC_IRQ		PM8921_GPIO_PM_TO_SYS(29)
#define GPIO_NFC_FIRMWARE	PM8921_GPIO_PM_TO_SYS(25)
#define GPIO_NFC_EN		PM8921_GPIO_PM_TO_SYS(11)
#define GPIO_EAR_DET			81
#define GPIO_EAR_GND_DET	        28  /* PM8921 GPIO 28 */
#define GPIO_SHORT_SENDEND		56
#define GPIO_EAR_BIAS_DISCHARGE		45
#define GPIO_MCU_AP_INT		PM8921_GPIO_PM_TO_SYS(20)
#define GPIO_MCU_AP_INT_2	PM8921_GPIO_PM_TO_SYS(21)
#define GPIO_AP_MCU_INT		PM8921_GPIO_PM_TO_SYS(22)
#define GPIO_MCU_NRST		PM8921_GPIO_PM_TO_SYS(27)
#define GPIO_MCU_CHG			69
#ifdef CONFIG_SEC_FPGA
#define GPIO_FPGA_CRESET_B              31
#define GPIO_FPGA_CDONE                 38
#define GPIO_FPGA_SPI_EN                43
#define GPIO_FPGA_MAIN_CLK              34
#endif
#if defined(CONFIG_VIDEO_MHL_V2)
#define GPIO_MHL_RST                    1
#define GPIO_MHL_SDA                    36
#define GPIO_MHL_SCL                    37
#define GPIO_MHL_INT                    77
#define GPIO_MHL_VSIL                    PM8921_GPIO_PM_TO_SYS(32)
#endif
#define GPIO_CAM_IO_EN			PM8921_GPIO_PM_TO_SYS(32) 
#define GPIO_MAIN_CAM_MCLK      5
#define GPIO_CAM1_RST_N			PM8921_GPIO_PM_TO_SYS(6)
#define GPIO_I2C_CLK_AF			70
#define GPIO_I2C_DATA_AF		71

/* PM8921 GPIO */
#define PMIC_GPIO_VPS_SOUND_EN		9
#define PMIC_MAIN_MICBIAS_EN		13
#define PMIC_GPIO_EAR_MICBIAS_EN        14
#define PMIC_GPIO_2MIC_RST              16
#define PMIC_GPIO_SPK_EN                18
#define PMIC_GPIO_VPS_EN                19
#define PMIC_GPIO_FSA8048_EN		28
#define PMIC_GPIO_IRDA_WAKE             -1
#define PMIC_GPIO_IRDA_IRQ		3
#define PMIC_GPIO_2MIC_WAKE             34
#define PMIC_GPIO_CODEC_RESET_N         38
#ifdef CONFIG_SEC_FPGA
#define PMIC_GPIO_FPGA_CRESET_B		9
#define PMIC_GPIO_FPGA_RST_N            36
#define PMIC_GPIO_FPGA_SPI_CLK          40
#define PMIC_GPIO_FPGA_SPI_SI           41
#endif
#define PMIC_MPP_FPGA_SPI_CLK            9
#define PMIC_MPP_FPGA_SPI_SI            10
#ifdef CONFIG_SENSORS_HALL
#define PMIC_GPIO_HALL_SENSOR_INT	7
#endif

/* gpio for changed list */
enum {
	BOARD_REV00,
	BOARD_REV01,
	BOARD_REV02,
	BOARD_REV03,
	BOARD_REV04,
	BOARD_REV05,
	BOARD_REV06,
	BOARD_REV07,
	BOARD_REV08,
	BOARD_REV09,
	BOARD_REV10,
	BOARD_REV11,
	BOARD_REV12,
	BOARD_REV13,
	BOARD_REV14,
	BOARD_REV15,
	BOARD_REV16,
	BOARD_REV17,
	BOARD_REV18,
	BOARD_REV19,
	BOARD_REV20,
	BOARD_REV21,
	BOARD_REV22,
	BOARD_REV23,
	BOARD_REV24,
	BOARD_REV25,
	GPIO_REV_MAX,
};

/* GPIO array Index */
enum {
	GPIO_FPGA_EN,
	GPIO_BARCODE_SDA,
	GPIO_BARCODE_SCL,
	GPIO_IRDA_SDA,
	GPIO_IRDA_SCL,
};

enum {
	FPGA_VSIL_A_1P2_EN = 0,
	FPGA_GPIO_01,
	FPGA_GPIO_02,
	FPGA_GPIO_WLAN_EN,
	FPGA_GPIO_BT_EN,
	FPGA_GPIO_BT_WAKE,
	FPGA_GPIO_06,
	FPGA_GPIO_07,
	FPGA_GPIO_MHL_RST,
	FPGA_GPIO_VPS_SOUND_EN,
};
