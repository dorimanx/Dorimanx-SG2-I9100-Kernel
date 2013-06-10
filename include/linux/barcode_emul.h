/*
 * Copyright (C) 2012 Samsung Electronics
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef _BARCODE_EMUL_H_
#define _BARCODE_EMUL_H_

#define FPGA_GPIO_BASE	300
#define NR_FPGA_GPIO	16
#define ICE_FPGA_GPIO_TO_SYS(fpga_gpio)  (fpga_gpio + FPGA_GPIO_BASE)

enum { ICE_12M = 0, ICE_19M, ICE_GPIOX, ICE_I2C, ICE_24M };

extern int ice_gpiox_set(int num, int val);
extern int ice_gpiox_get(int num);

struct barcode_emul_platform_data {
	int spi_clk;
	int spi_si;
	int spi_en;
	int cresetb;
	int rst_n;
	int cdone;
	int fw_type;
#if defined CONFIG_IR_REMOCON_FPGA
	int irda_wake;
	int irda_irq;
#endif
	void (*poweron) (int);
	void (*gpioconfig) (void);
	int	(*clock_en) (int);
#if defined CONFIG_IR_REMOCON_FPGA
	void (*ir_remote_init) (void);
	void (*ir_wake_en)(bool onoff);
	void(*ir_vdd_onoff)(bool onoff);
	void(*ir_led_poweron)(int);
#endif
#if defined(CONFIG_MACH_JF_DCM)
	int(*get_fpga_felica_flag)(void);
#endif	
};

#define GPIO_LEVEL_LOW        0
#define GPIO_LEVEL_HIGH       1

#define BEAMING_OFF           0
#define BEAMING_ON            1

#define STOP_BEAMING       0x00

#endif /* _BARCODE_EMUL_H_ */
