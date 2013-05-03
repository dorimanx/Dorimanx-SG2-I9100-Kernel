/*
 * =================================================================
 *
 *       Filename:  smart_mtp_se6e8fa.c
 *
 *    Description:  Smart dimming algorithm implementation
 *
 *        Author: jb09.kim
 *        Company:  Samsung Electronics
 *
 * ================================================================
 */
/*
<one line to give the program's name and a brief idea of what it does.>
Copyright (C) 2012, Samsung Electronics. All rights reserved.

*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/

#include "smart_mtp_se6e8fa.h"
#include "smart_mtp_2p2_gamma.h"

/*
#define SMART_DIMMING_DEBUG
*/

static char max_lux_table[GAMMA_SET_MAX];
static char min_lux_table[GAMMA_SET_MAX];

/*
*	To support different center cell gamma setting
*/
static char V255_300CD_R_MSB;
static char V255_300CD_R_LSB;

static char V255_300CD_G_MSB;
static char V255_300CD_G_LSB;

static char V255_300CD_B_MSB;
static char V255_300CD_B_LSB;

static char V203_300CD_R;
static char V203_300CD_G;
static char V203_300CD_B;

static char V151_300CD_R;
static char V151_300CD_G;
static char V151_300CD_B;

static char V87_300CD_R;
static char V87_300CD_G;
static char V87_300CD_B;

static char V51_300CD_R;
static char V51_300CD_G;
static char V51_300CD_B;

static char V35_300CD_R;
static char V35_300CD_G;
static char V35_300CD_B;

static char V23_300CD_R;
static char V23_300CD_G;
static char V23_300CD_B;

static char V11_300CD_R;
static char V11_300CD_G;
static char V11_300CD_B;

static char V3_300CD_R;
static char V3_300CD_G;
static char V3_300CD_B;

static char VT_300CD_R;
static char VT_300CD_G;
static char VT_300CD_B;

static int color_mods[5][9] = {
	{  0,  0,  5, -18, -16, -10, 0, 0,  3 },
	{  0,  0,  2,  -9,  -8,  -5, 0, 0,  1 },
	{  0,  0,  0,   0,   0,   0, 0, 0,  0 },
	{  0,  0, -2,   9,   8,   5, 0, 0, -1 },
	{  0,  0, -5,  18,  16,  10, 0, 0, -3 }
};
static unsigned int panelval = 2;

static int char_to_int(char data1)
{
	int cal_data;

	if (data1 & 0x80) {
		cal_data = data1 & 0x7F;
		cal_data *= -1;
	} else
		cal_data = data1;

	return cal_data;
}

void panel_load_colors(unsigned int val, struct SMART_DIM *pSmart)
{
	if (val < 5) {
		panelval = val;
		smart_dimming_init(pSmart);
	}
}

static int char_to_int_v255(char data1, char data2)
{
	int cal_data;

	if (data1)
		cal_data = data2 * -1;
	else
		cal_data = data2;

	return cal_data;
}

#define v255_coefficient 72
#define v255_denominator 860
static int v255_adjustment(struct SMART_DIM *pSmart)
{
	unsigned long long result_1, result_2, result_3, result_4;
	int add_mtp;
	int LSB;
	int v255_value;

	v255_value = (V255_300CD_R_MSB << 8) | (V255_300CD_R_LSB);
	LSB = char_to_int_v255(pSmart->MTP.R_OFFSET.OFFSET_255_MSB,
				pSmart->MTP.R_OFFSET.OFFSET_255_LSB);
	add_mtp = LSB + v255_value;
	result_1 = result_2 = (v255_coefficient+add_mtp + color_mods[panelval][0]) << BIT_SHIFT;
	do_div(result_2, v255_denominator);
	result_3 = (S6E8FA_VREG0_REF * result_2) >> BIT_SHIFT;
	result_4 = S6E8FA_VREG0_REF - result_3;
	pSmart->RGB_OUTPUT.R_VOLTAGE.level_255 = result_4;
	pSmart->RGB_OUTPUT.R_VOLTAGE.level_0 = S6E8FA_VREG0_REF;

	v255_value = (V255_300CD_G_MSB << 8) | (V255_300CD_G_LSB);
	LSB = char_to_int_v255(pSmart->MTP.G_OFFSET.OFFSET_255_MSB,
				pSmart->MTP.G_OFFSET.OFFSET_255_LSB);
	add_mtp = LSB + v255_value;
	result_1 = result_2 = (v255_coefficient+add_mtp + color_mods[panelval][1]) << BIT_SHIFT;
	do_div(result_2, v255_denominator);
	result_3 = (S6E8FA_VREG0_REF * result_2) >> BIT_SHIFT;
	result_4 = S6E8FA_VREG0_REF - result_3;
	pSmart->RGB_OUTPUT.G_VOLTAGE.level_255 = result_4;
	pSmart->RGB_OUTPUT.G_VOLTAGE.level_0 = S6E8FA_VREG0_REF;

	v255_value = (V255_300CD_B_MSB << 8) | (V255_300CD_B_LSB);
	LSB = char_to_int_v255(pSmart->MTP.B_OFFSET.OFFSET_255_MSB,
				pSmart->MTP.B_OFFSET.OFFSET_255_LSB);
	add_mtp = LSB + v255_value;
	result_1 = result_2 = (v255_coefficient+add_mtp + color_mods[panelval][2]) << BIT_SHIFT;
	do_div(result_2, v255_denominator);
	result_3 = (S6E8FA_VREG0_REF * result_2) >> BIT_SHIFT;
	result_4 = S6E8FA_VREG0_REF - result_3;
	pSmart->RGB_OUTPUT.B_VOLTAGE.level_255 = result_4;
	pSmart->RGB_OUTPUT.B_VOLTAGE.level_0 = S6E8FA_VREG0_REF;

#ifdef SMART_DIMMING_DEBUG
	pr_info("%s V255 RED:%d GREEN:%d BLUE:%d\n", __func__,
			pSmart->RGB_OUTPUT.R_VOLTAGE.level_255,
			pSmart->RGB_OUTPUT.G_VOLTAGE.level_255,
			pSmart->RGB_OUTPUT.B_VOLTAGE.level_255);
#endif

	return 0;
}

static void v255_hexa(int *index, struct SMART_DIM *pSmart, char *str)
{
	unsigned long long result_1, result_2, result_3;

	result_1 = S6E8FA_VREG0_REF -
		(pSmart->GRAY.TABLE[index[V255_INDEX]].R_Gray);
	result_2 = result_1 * v255_denominator;
	do_div(result_2, S6E8FA_VREG0_REF);
	result_3 = result_2  - v255_coefficient;
	str[0] = (result_3 & 0xff00) >> 8;
	str[1] = result_3 & 0xff;

	result_1 = S6E8FA_VREG0_REF -
		(pSmart->GRAY.TABLE[index[V255_INDEX]].G_Gray);
	result_2 = result_1 * v255_denominator;
	do_div(result_2, S6E8FA_VREG0_REF);
	result_3 = result_2  - v255_coefficient;
	str[2] = (result_3 & 0xff00) >> 8;
	str[3] = result_3 & 0xff;

	result_1 = S6E8FA_VREG0_REF -
			(pSmart->GRAY.TABLE[index[V255_INDEX]].B_Gray);
	result_2 = result_1 * v255_denominator;
	do_div(result_2, S6E8FA_VREG0_REF);
	result_3 = result_2  - v255_coefficient;
	str[4] = (result_3 & 0xff00) >> 8;
	str[5] = result_3 & 0xff;

}

static int vt_coefficient[] = {
	0, 12, 24, 36, 48,
	60, 72, 84, 96, 108,
	138, 148, 158, 168,
	178, 186,
};
#define vt_denominator 860
static int vt_adjustment(struct SMART_DIM *pSmart)
{
	unsigned long long result_1, result_2, result_3, result_4;
	int add_mtp;
	int LSB;

	LSB = char_to_int(pSmart->MTP.R_OFFSET.OFFSET_1);
	add_mtp = LSB + VT_300CD_R;
	result_1 = result_2 = (vt_coefficient[LSB] + color_mods[panelval][3]) << BIT_SHIFT;
	do_div(result_2, vt_denominator);
	result_3 = (S6E8FA_VREG0_REF * result_2) >> BIT_SHIFT;
	result_4 = S6E8FA_VREG0_REF - result_3;
	pSmart->GRAY.VT_TABLE.R_Gray = result_4;

	LSB = char_to_int(pSmart->MTP.G_OFFSET.OFFSET_1);
	add_mtp = LSB + VT_300CD_G;
	result_1 = result_2 = (vt_coefficient[LSB]  + color_mods[panelval][4]) << BIT_SHIFT;
	do_div(result_2, vt_denominator);
	result_3 = (S6E8FA_VREG0_REF * result_2) >> BIT_SHIFT;
	result_4 = S6E8FA_VREG0_REF - result_3;
	pSmart->GRAY.VT_TABLE.G_Gray = result_4;

	LSB = char_to_int(pSmart->MTP.B_OFFSET.OFFSET_1);
	add_mtp = LSB + VT_300CD_B;
	result_1 = result_2 = (vt_coefficient[LSB]  + color_mods[panelval][5]) << BIT_SHIFT;
	do_div(result_2, vt_denominator);
	result_3 = (S6E8FA_VREG0_REF * result_2) >> BIT_SHIFT;
	result_4 = S6E8FA_VREG0_REF - result_3;
	pSmart->GRAY.VT_TABLE.B_Gray = result_4;

#ifdef SMART_DIMMING_DEBUG
	pr_info("%s VT RED:%d GREEN:%d BLUE:%d\n", __func__,
			pSmart->GRAY.VT_TABLE.R_Gray,
			pSmart->GRAY.VT_TABLE.G_Gray,
			pSmart->GRAY.VT_TABLE.B_Gray);
#endif

	return 0;

}

static void vt_hexa(int *index, struct SMART_DIM *pSmart, char *str)
{
	str[30] = VT_300CD_R;
	str[31] = VT_300CD_G;
	str[32] = VT_300CD_B;
}

#define v203_coefficient 64
#define v203_denominator 320
static int v203_adjustment(struct SMART_DIM *pSmart)
{
	unsigned long long result_1, result_2, result_3, result_4;
	int add_mtp;
	int LSB;

	LSB = char_to_int(pSmart->MTP.R_OFFSET.OFFSET_203);
	add_mtp = LSB + V203_300CD_R;
	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
				- (pSmart->RGB_OUTPUT.R_VOLTAGE.level_255);
	result_2 = (v203_coefficient + add_mtp + color_mods[panelval][6]) << BIT_SHIFT;
	do_div(result_2, v203_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.R_Gray) - result_3;
	pSmart->RGB_OUTPUT.R_VOLTAGE.level_203 = result_4;

	LSB = char_to_int(pSmart->MTP.G_OFFSET.OFFSET_203);
	add_mtp = LSB + V203_300CD_G;
	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
				- (pSmart->RGB_OUTPUT.G_VOLTAGE.level_255);
	result_2 = (v203_coefficient + add_mtp + color_mods[panelval][7]) << BIT_SHIFT;
	do_div(result_2, v203_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.G_Gray) - result_3;
	pSmart->RGB_OUTPUT.G_VOLTAGE.level_203 = result_4;

	LSB = char_to_int(pSmart->MTP.B_OFFSET.OFFSET_203);
	add_mtp = LSB + V203_300CD_B;
	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
				- (pSmart->RGB_OUTPUT.B_VOLTAGE.level_255);
	result_2 = (v203_coefficient + add_mtp + color_mods[panelval][8]) << BIT_SHIFT;
	do_div(result_2, v203_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.B_Gray) - result_3;
	pSmart->RGB_OUTPUT.B_VOLTAGE.level_203 = result_4;

#ifdef SMART_DIMMING_DEBUG
	pr_info("%s V203 RED:%d GREEN:%d BLUE:%d\n", __func__,
			pSmart->RGB_OUTPUT.R_VOLTAGE.level_203,
			pSmart->RGB_OUTPUT.G_VOLTAGE.level_203,
			pSmart->RGB_OUTPUT.B_VOLTAGE.level_203);
#endif

	return 0;

}

static void v203_hexa(int *index, struct SMART_DIM *pSmart, char *str)
{
	unsigned long long result_1, result_2, result_3;

	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V203_INDEX]].R_Gray);
	result_2 = result_1 * v203_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V255_INDEX]].R_Gray);
	do_div(result_2, result_3);
	str[6] = (result_2  - v203_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V203_INDEX]].G_Gray);
	result_2 = result_1 * v203_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V255_INDEX]].G_Gray);
	do_div(result_2, result_3);
	str[7] = (result_2  - v203_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V203_INDEX]].B_Gray);
	result_2 = result_1 * v203_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V255_INDEX]].B_Gray);
	do_div(result_2, result_3);
	str[8] = (result_2  - v203_coefficient) & 0xff;

}

#define v151_coefficient 64
#define v151_denominator 320
static int v151_adjustment(struct SMART_DIM *pSmart)
{
	unsigned long long result_1, result_2, result_3, result_4;
	int add_mtp;
	int LSB;

	LSB = char_to_int(pSmart->MTP.R_OFFSET.OFFSET_151);
	add_mtp = LSB + V151_300CD_R;
	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->RGB_OUTPUT.R_VOLTAGE.level_203);
	result_2 = (v151_coefficient + add_mtp + color_mods[panelval][6]) << BIT_SHIFT;
	do_div(result_2, v151_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.R_Gray) - result_3;
	pSmart->RGB_OUTPUT.R_VOLTAGE.level_151 = result_4;

	LSB = char_to_int(pSmart->MTP.G_OFFSET.OFFSET_151);
	add_mtp = LSB + V151_300CD_G;
	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->RGB_OUTPUT.G_VOLTAGE.level_203);
	result_2 = (v151_coefficient + add_mtp + color_mods[panelval][7]) << BIT_SHIFT;
	do_div(result_2, v151_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.G_Gray) - result_3;
	pSmart->RGB_OUTPUT.G_VOLTAGE.level_151 = result_4;

	LSB = char_to_int(pSmart->MTP.B_OFFSET.OFFSET_151);
	add_mtp = LSB + V151_300CD_B;
	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->RGB_OUTPUT.B_VOLTAGE.level_203);
	result_2 = (v151_coefficient + add_mtp + color_mods[panelval][8]) << BIT_SHIFT;
	do_div(result_2, v151_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.B_Gray) - result_3;
	pSmart->RGB_OUTPUT.B_VOLTAGE.level_151 = result_4;

#ifdef SMART_DIMMING_DEBUG
	pr_info("%s V151 RED:%d GREEN:%d BLUE:%d\n", __func__,
			pSmart->RGB_OUTPUT.R_VOLTAGE.level_151,
			pSmart->RGB_OUTPUT.G_VOLTAGE.level_151,
			pSmart->RGB_OUTPUT.B_VOLTAGE.level_151);
#endif

	return 0;

}

static void v151_hexa(int *index, struct SMART_DIM *pSmart, char *str)
{
	unsigned long long result_1, result_2, result_3;

	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V151_INDEX]].R_Gray);
	result_2 = result_1 * v151_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V203_INDEX]].R_Gray);
	do_div(result_2, result_3);
	str[9] = (result_2  - v151_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V151_INDEX]].G_Gray);
	result_2 = result_1 * v151_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V203_INDEX]].G_Gray);
	do_div(result_2, result_3);
	str[10] = (result_2  - v151_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V151_INDEX]].B_Gray);
	result_2 = result_1 * v151_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V203_INDEX]].B_Gray);
	do_div(result_2, result_3);
	str[11] = (result_2  - v151_coefficient) & 0xff;
}

#define v87_coefficient 64
#define v87_denominator 320
static int v87_adjustment(struct SMART_DIM *pSmart)
{
	unsigned long long result_1, result_2, result_3, result_4;
	int add_mtp;
	int LSB;

	LSB = char_to_int(pSmart->MTP.R_OFFSET.OFFSET_87);
	add_mtp = LSB + V87_300CD_R;
	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->RGB_OUTPUT.R_VOLTAGE.level_151);
	result_2 = (v87_coefficient + add_mtp + color_mods[panelval][6]) << BIT_SHIFT;
	do_div(result_2, v87_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.R_Gray) - result_3;
	pSmart->RGB_OUTPUT.R_VOLTAGE.level_87 = result_4;

	LSB = char_to_int(pSmart->MTP.G_OFFSET.OFFSET_87);
	add_mtp = LSB + V87_300CD_G;
	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->RGB_OUTPUT.G_VOLTAGE.level_151);
	result_2 = (v87_coefficient + add_mtp + color_mods[panelval][7]) << BIT_SHIFT;
	do_div(result_2, v87_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.G_Gray) - result_3;
	pSmart->RGB_OUTPUT.G_VOLTAGE.level_87 = result_4;

	LSB = char_to_int(pSmart->MTP.B_OFFSET.OFFSET_87);
	add_mtp = LSB + V87_300CD_B;
	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->RGB_OUTPUT.B_VOLTAGE.level_151);
	result_2 = (v87_coefficient + add_mtp + color_mods[panelval][8]) << BIT_SHIFT;
	do_div(result_2, v87_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.B_Gray) - result_3;
	pSmart->RGB_OUTPUT.B_VOLTAGE.level_87 = result_4;

#ifdef SMART_DIMMING_DEBUG
	pr_info("%s V87 RED:%d GREEN:%d BLUE:%d\n", __func__,
			pSmart->RGB_OUTPUT.R_VOLTAGE.level_87,
			pSmart->RGB_OUTPUT.G_VOLTAGE.level_87,
			pSmart->RGB_OUTPUT.B_VOLTAGE.level_87);
#endif

	return 0;

}

static void v87_hexa(int *index, struct SMART_DIM *pSmart, char *str)
{
	unsigned long long result_1, result_2, result_3;

	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V87_INDEX]].R_Gray);
	result_2 = result_1 * v87_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V151_INDEX]].R_Gray);
	do_div(result_2, result_3);
	str[12] = (result_2  - v87_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V87_INDEX]].G_Gray);
	result_2 = result_1 * v87_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V151_INDEX]].G_Gray);
	do_div(result_2, result_3);
	str[13] = (result_2  - v87_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V87_INDEX]].B_Gray);
	result_2 = result_1 * v87_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V151_INDEX]].B_Gray);
	do_div(result_2, result_3);
	str[14] = (result_2  - v87_coefficient) & 0xff;
}

#define v51_coefficient 64
#define v51_denominator 320
static int v51_adjustment(struct SMART_DIM *pSmart)
{
	unsigned long long result_1, result_2, result_3, result_4;
	int add_mtp;
	int LSB;

	LSB = char_to_int(pSmart->MTP.R_OFFSET.OFFSET_51);
	add_mtp = LSB + V51_300CD_R;
	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->RGB_OUTPUT.R_VOLTAGE.level_87);
	result_2 = (v51_coefficient + add_mtp + color_mods[panelval][6]) << BIT_SHIFT;
	do_div(result_2, v51_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.R_Gray) - result_3;
	pSmart->RGB_OUTPUT.R_VOLTAGE.level_51 = result_4;

	LSB = char_to_int(pSmart->MTP.G_OFFSET.OFFSET_51);
	add_mtp = LSB + V51_300CD_G;
	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->RGB_OUTPUT.G_VOLTAGE.level_87);
	result_2 = (v51_coefficient + add_mtp + color_mods[panelval][7]) << BIT_SHIFT;
	do_div(result_2, v51_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.G_Gray) - result_3;
	pSmart->RGB_OUTPUT.G_VOLTAGE.level_51 = result_4;

	LSB = char_to_int(pSmart->MTP.B_OFFSET.OFFSET_51);
	add_mtp = LSB + V51_300CD_B;
	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->RGB_OUTPUT.B_VOLTAGE.level_87);
	result_2 = (v51_coefficient + add_mtp + color_mods[panelval][8]) << BIT_SHIFT;
	do_div(result_2, v51_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.B_Gray) - result_3;
	pSmart->RGB_OUTPUT.B_VOLTAGE.level_51 = result_4;

#ifdef SMART_DIMMING_DEBUG
	pr_info("%s V51 RED:%d GREEN:%d BLUE:%d\n", __func__,
			pSmart->RGB_OUTPUT.R_VOLTAGE.level_51,
			pSmart->RGB_OUTPUT.G_VOLTAGE.level_51,
			pSmart->RGB_OUTPUT.B_VOLTAGE.level_51);
#endif

	return 0;

}

static void v51_hexa(int *index, struct SMART_DIM *pSmart, char *str)
{
	unsigned long long result_1, result_2, result_3;

	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V51_INDEX]].R_Gray);
	result_2 = result_1 * v51_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V87_INDEX]].R_Gray);
	do_div(result_2, result_3);
	str[15] = (result_2  - v51_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V51_INDEX]].G_Gray);
	result_2 = result_1 * v51_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V87_INDEX]].G_Gray);
	do_div(result_2, result_3);
	str[16] = (result_2  - v51_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V51_INDEX]].B_Gray);
	result_2 = result_1 * v51_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V87_INDEX]].B_Gray);
	do_div(result_2, result_3);
	str[17] = (result_2  - v51_coefficient) & 0xff;

}

#define v35_coefficient 64
#define v35_denominator 320
static int v35_adjustment(struct SMART_DIM *pSmart)
{
	unsigned long long result_1, result_2, result_3, result_4;
	int add_mtp;
	int LSB;

	LSB = char_to_int(pSmart->MTP.R_OFFSET.OFFSET_35);
	add_mtp = LSB + V35_300CD_R;
	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->RGB_OUTPUT.R_VOLTAGE.level_51);
	result_2 = (v35_coefficient + add_mtp + color_mods[panelval][6]) << BIT_SHIFT;
	do_div(result_2, v35_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.R_Gray) - result_3;
	pSmart->RGB_OUTPUT.R_VOLTAGE.level_35 = result_4;

	LSB = char_to_int(pSmart->MTP.G_OFFSET.OFFSET_35);
	add_mtp = LSB + V35_300CD_G;
	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->RGB_OUTPUT.G_VOLTAGE.level_51);
	result_2 = (v35_coefficient + add_mtp + color_mods[panelval][7]) << BIT_SHIFT;
	do_div(result_2, v35_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.G_Gray) - result_3;
	pSmart->RGB_OUTPUT.G_VOLTAGE.level_35 = result_4;

	LSB = char_to_int(pSmart->MTP.B_OFFSET.OFFSET_35);
	add_mtp = LSB + V35_300CD_B;
	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->RGB_OUTPUT.B_VOLTAGE.level_51);
	result_2 = (v35_coefficient + add_mtp + color_mods[panelval][8]) << BIT_SHIFT;
	do_div(result_2, v35_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.B_Gray) - result_3;
	pSmart->RGB_OUTPUT.B_VOLTAGE.level_35 = result_4;

#ifdef SMART_DIMMING_DEBUG
	pr_info("%s V35 RED:%d GREEN:%d BLUE:%d\n", __func__,
			pSmart->RGB_OUTPUT.R_VOLTAGE.level_35,
			pSmart->RGB_OUTPUT.G_VOLTAGE.level_35,
			pSmart->RGB_OUTPUT.B_VOLTAGE.level_35);
#endif

	return 0;

}

static void v35_hexa(int *index, struct SMART_DIM *pSmart, char *str)
{
	unsigned long long result_1, result_2, result_3;

	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V35_INDEX]].R_Gray);
	result_2 = result_1 * v35_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V51_INDEX]].R_Gray);
	do_div(result_2, result_3);
	str[18] = (result_2  - v35_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V35_INDEX]].G_Gray);
	result_2 = result_1 * v35_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V51_INDEX]].G_Gray);
	do_div(result_2, result_3);
	str[19] = (result_2  - v35_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V35_INDEX]].B_Gray);
	result_2 = result_1 * v35_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V51_INDEX]].B_Gray);
	do_div(result_2, result_3);
	str[20] = (result_2  - v35_coefficient) & 0xff;

}

#define v23_coefficient 64
#define v23_denominator 320
static int v23_adjustment(struct SMART_DIM *pSmart)
{
	unsigned long long result_1, result_2, result_3, result_4;
	int add_mtp;
	int LSB;

	LSB = char_to_int(pSmart->MTP.R_OFFSET.OFFSET_23);
	add_mtp = LSB + V23_300CD_R;
	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->RGB_OUTPUT.R_VOLTAGE.level_35);
	result_2 = (v23_coefficient + add_mtp + color_mods[panelval][6]) << BIT_SHIFT;
	do_div(result_2, v23_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.R_Gray) - result_3;
	pSmart->RGB_OUTPUT.R_VOLTAGE.level_23 = result_4;

	LSB = char_to_int(pSmart->MTP.G_OFFSET.OFFSET_23);
	add_mtp = LSB + V23_300CD_G;
	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->RGB_OUTPUT.G_VOLTAGE.level_35);
	result_2 = (v23_coefficient + add_mtp + color_mods[panelval][7]) << BIT_SHIFT;
	do_div(result_2, v23_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.G_Gray) - result_3;
	pSmart->RGB_OUTPUT.G_VOLTAGE.level_23 = result_4;

	LSB = char_to_int(pSmart->MTP.B_OFFSET.OFFSET_23);
	add_mtp = LSB + V23_300CD_B;
	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->RGB_OUTPUT.B_VOLTAGE.level_35);
	result_2 = (v23_coefficient + add_mtp + color_mods[panelval][8]) << BIT_SHIFT;
	do_div(result_2, v23_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.B_Gray) - result_3;
	pSmart->RGB_OUTPUT.B_VOLTAGE.level_23 = result_4;

#ifdef SMART_DIMMING_DEBUG
	pr_info("%s V23 RED:%d GREEN:%d BLUE:%d\n", __func__,
			pSmart->RGB_OUTPUT.R_VOLTAGE.level_23,
			pSmart->RGB_OUTPUT.G_VOLTAGE.level_23,
			pSmart->RGB_OUTPUT.B_VOLTAGE.level_23);
#endif

	return 0;

}

static void v23_hexa(int *index, struct SMART_DIM *pSmart, char *str)
{
	unsigned long long result_1, result_2, result_3;

	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V23_INDEX]].R_Gray);
	result_2 = result_1 * v23_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V35_INDEX]].R_Gray);
	do_div(result_2, result_3);
	str[21] = (result_2  - v23_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V23_INDEX]].G_Gray);
	result_2 = result_1 * v23_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V35_INDEX]].G_Gray);
	do_div(result_2, result_3);
	str[22] = (result_2  - v23_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V23_INDEX]].B_Gray);
	result_2 = result_1 * v23_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V35_INDEX]].B_Gray);
	do_div(result_2, result_3);
	str[23] = (result_2  - v23_coefficient) & 0xff;

}

#define v11_coefficient 64
#define v11_denominator 320
static int v11_adjustment(struct SMART_DIM *pSmart)
{
	unsigned long long result_1, result_2, result_3, result_4;
	int add_mtp;
	int LSB;

	LSB = char_to_int(pSmart->MTP.R_OFFSET.OFFSET_11);
	add_mtp = LSB + V11_300CD_R;
	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->RGB_OUTPUT.R_VOLTAGE.level_23);
	result_2 = (v11_coefficient + add_mtp + color_mods[panelval][6]) << BIT_SHIFT;
	do_div(result_2, v11_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.R_Gray) - result_3;
	pSmart->RGB_OUTPUT.R_VOLTAGE.level_11 = result_4;

	LSB = char_to_int(pSmart->MTP.G_OFFSET.OFFSET_11);
	add_mtp = LSB + V11_300CD_G;
	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->RGB_OUTPUT.G_VOLTAGE.level_23);
	result_2 = (v11_coefficient + add_mtp + color_mods[panelval][7]) << BIT_SHIFT;
	do_div(result_2, v11_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.G_Gray) - result_3;
	pSmart->RGB_OUTPUT.G_VOLTAGE.level_11 = result_4;

	LSB = char_to_int(pSmart->MTP.B_OFFSET.OFFSET_11);
	add_mtp = LSB + V11_300CD_B;
	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->RGB_OUTPUT.B_VOLTAGE.level_23);
	result_2 = (v11_coefficient + add_mtp + color_mods[panelval][8]) << BIT_SHIFT;
	do_div(result_2, v11_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (pSmart->GRAY.VT_TABLE.B_Gray) - result_3;
	pSmart->RGB_OUTPUT.B_VOLTAGE.level_11 = result_4;

#ifdef SMART_DIMMING_DEBUG
	pr_info("%s V11 RED:%d GREEN:%d BLUE:%d\n", __func__,
			pSmart->RGB_OUTPUT.R_VOLTAGE.level_11,
			pSmart->RGB_OUTPUT.G_VOLTAGE.level_11,
			pSmart->RGB_OUTPUT.B_VOLTAGE.level_11);
#endif

	return 0;

}

static void v11_hexa(int *index, struct SMART_DIM *pSmart, char *str)
{
	unsigned long long result_1, result_2, result_3;

	result_1 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V11_INDEX]].R_Gray);
	result_2 = result_1 * v11_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.R_Gray)
			- (pSmart->GRAY.TABLE[index[V23_INDEX]].R_Gray);
	do_div(result_2, result_3);
	str[24] = (result_2  - v11_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V11_INDEX]].G_Gray);
	result_2 = result_1 * v11_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.G_Gray)
			- (pSmart->GRAY.TABLE[index[V23_INDEX]].G_Gray);
	do_div(result_2, result_3);
	str[25] = (result_2  - v11_coefficient) & 0xff;

	result_1 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V11_INDEX]].B_Gray);
	result_2 = result_1 * v11_denominator;
	result_3 = (pSmart->GRAY.VT_TABLE.B_Gray)
			- (pSmart->GRAY.TABLE[index[V23_INDEX]].B_Gray);
	do_div(result_2, result_3);
	str[26] = (result_2  - v11_coefficient) & 0xff;

}

#define v3_coefficient 64
#define v3_denominator 320
static int v3_adjustment(struct SMART_DIM *pSmart)
{
	unsigned long long result_1, result_2, result_3, result_4;
	int add_mtp;
	int LSB;

	LSB = char_to_int(pSmart->MTP.R_OFFSET.OFFSET_3);
	add_mtp = LSB + V3_300CD_R;
	result_1 = (S6E8FA_VREG0_REF)
			- (pSmart->RGB_OUTPUT.R_VOLTAGE.level_11);
	result_2 = (v3_coefficient + add_mtp + color_mods[panelval][6]) << BIT_SHIFT;
	do_div(result_2, v3_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (S6E8FA_VREG0_REF) - result_3;
	pSmart->RGB_OUTPUT.R_VOLTAGE.level_3 = result_4;

	LSB = char_to_int(pSmart->MTP.G_OFFSET.OFFSET_3);
	add_mtp = LSB + V3_300CD_G;
	result_1 = (S6E8FA_VREG0_REF)
			- (pSmart->RGB_OUTPUT.G_VOLTAGE.level_11);
	result_2 = (v3_coefficient + add_mtp + color_mods[panelval][7]) << BIT_SHIFT;
	do_div(result_2, v3_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (S6E8FA_VREG0_REF) - result_3;
	pSmart->RGB_OUTPUT.G_VOLTAGE.level_3 = result_4;

	LSB = char_to_int(pSmart->MTP.B_OFFSET.OFFSET_3);
	add_mtp = LSB + V3_300CD_B;
	result_1 = (S6E8FA_VREG0_REF)
			- (pSmart->RGB_OUTPUT.B_VOLTAGE.level_11);
	result_2 = (v3_coefficient + add_mtp + color_mods[panelval][8]) << BIT_SHIFT;
	do_div(result_2, v3_denominator);
	result_3 = (result_1 * result_2) >> BIT_SHIFT;
	result_4 = (S6E8FA_VREG0_REF) - result_3;
	pSmart->RGB_OUTPUT.B_VOLTAGE.level_3 = result_4;

#ifdef SMART_DIMMING_DEBUG
	pr_info("%s V3 RED:%d GREEN:%d BLUE:%d\n", __func__,
			pSmart->RGB_OUTPUT.R_VOLTAGE.level_3,
			pSmart->RGB_OUTPUT.G_VOLTAGE.level_3,
			pSmart->RGB_OUTPUT.B_VOLTAGE.level_3);
#endif

	return 0;

}

static void v3_hexa(int *index, struct SMART_DIM *pSmart, char *str)
{
	unsigned long long result_1, result_2, result_3;

	result_1 = (S6E8FA_VREG0_REF)
			- (pSmart->GRAY.TABLE[index[V3_INDEX]].R_Gray);
	result_2 = result_1 * v3_denominator;
	result_3 = (S6E8FA_VREG0_REF)
			- (pSmart->GRAY.TABLE[index[V11_INDEX]].R_Gray);
	do_div(result_2, result_3);
	str[27] = (result_2  - v3_coefficient) & 0xff;

	result_1 = (S6E8FA_VREG0_REF)
			- (pSmart->GRAY.TABLE[index[V3_INDEX]].G_Gray);
	result_2 = result_1 * v3_denominator;
	result_3 = (S6E8FA_VREG0_REF)
			- (pSmart->GRAY.TABLE[index[V11_INDEX]].G_Gray);
	do_div(result_2, result_3);
	str[28] = (result_2  - v3_coefficient) & 0xff;

	result_1 = (S6E8FA_VREG0_REF)
			- (pSmart->GRAY.TABLE[index[V3_INDEX]].B_Gray);
	result_2 = result_1 * v3_denominator;
	result_3 = (S6E8FA_VREG0_REF)
			- (pSmart->GRAY.TABLE[index[V11_INDEX]].B_Gray);
	do_div(result_2, result_3);
	str[29] = (result_2  - v3_coefficient) & 0xff;

}


/*V0,V1,V3,V11,V23,V35,V51,V87,V151,V203,V255*/
int S6E8FA_ARRAY[S6E8FA_MAX] = {0, 1, 3, 11, 23, 35, 51, 87, 151, 203, 255};

#define V0toV3_Coefficient 2
#define V0toV3_Multiple 1
#define V0toV3_denominator 3

#define V3toV11_Coefficient 7
#define V3toV11_Multiple 1
#define V3toV11_denominator 8

#define V11toV23_Coefficient 11
#define V11toV23_Multiple 1
#define V11toV23_denominator 12

#define V23toV35_Coefficient 11
#define V23toV35_Multiple 1
#define V23toV35_denominator 12

#define V35toV51_Coefficient 15
#define V35toV51_Multiple 1
#define V35toV51_denominator 16

#define V51toV87_Coefficient 35
#define V51toV87_Multiple 1
#define V51toV87_denominator 36

#define V87toV151_Coefficient 63
#define V87toV151_Multiple 1
#define V87toV151_denominator 64

#define V151toV203_Coefficient 51
#define V151toV203_Multiple 1
#define V151toV203_denominator 52

#define V203toV255_Coefficient 51
#define V203toV255_Multiple 1
#define V203toV255_denominator 52

static int cal_gray_scale_linear(int up, int low, int coeff,
int mul, int deno, int cnt)
{
	unsigned long long result_1, result_2, result_3, result_4;

	result_1 = up - low;
	result_2 = (result_1 * (coeff - (cnt * mul))) << BIT_SHIFT;
	do_div(result_2, deno);
	result_3 = result_2 >> BIT_SHIFT;
	result_4 = low + result_3;

	return (int)result_4;
}

static int generate_gray_scale(struct SMART_DIM *pSmart)
{
	int cnt = 0, cal_cnt = 0;
	int array_index = 0;
	struct GRAY_VOLTAGE *ptable = (struct GRAY_VOLTAGE *)
						(&(pSmart->GRAY.TABLE));

	for (cnt = 0; cnt < S6E8FA_MAX; cnt++) {
		pSmart->GRAY.TABLE[S6E8FA_ARRAY[cnt]].R_Gray =
			((int *)&(pSmart->RGB_OUTPUT.R_VOLTAGE))[cnt];

		pSmart->GRAY.TABLE[S6E8FA_ARRAY[cnt]].G_Gray =
			((int *)&(pSmart->RGB_OUTPUT.G_VOLTAGE))[cnt];

		pSmart->GRAY.TABLE[S6E8FA_ARRAY[cnt]].B_Gray =
			((int *)&(pSmart->RGB_OUTPUT.B_VOLTAGE))[cnt];
	}

	/*
		below codes use hard coded value.
		So it is possible to modify on each model.
		V0,V1,V3,V11,V23,V35,V51,V87,V151,V203,V255
	*/
	for (cnt = 0; cnt < S6E8FA_GRAY_SCALE_MAX; cnt++) {

		if (cnt == S6E8FA_ARRAY[0]) {
			/* 0 */
			array_index = 0;
			cal_cnt = 0;
		} else if ((cnt > S6E8FA_ARRAY[0]) &&
			(cnt < S6E8FA_ARRAY[2])) {
			/* 1 ~ 2 */
			array_index = 2;

			pSmart->GRAY.TABLE[cnt].R_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-2]].R_Gray,
			ptable[S6E8FA_ARRAY[array_index]].R_Gray,
			V0toV3_Coefficient, V0toV3_Multiple,
			V0toV3_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].G_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-2]].G_Gray,
			ptable[S6E8FA_ARRAY[array_index]].G_Gray,
			V0toV3_Coefficient, V0toV3_Multiple,
			V0toV3_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].B_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-2]].B_Gray,
			ptable[S6E8FA_ARRAY[array_index]].B_Gray,
			V0toV3_Coefficient, V0toV3_Multiple,
			V0toV3_denominator , cal_cnt);

			cal_cnt++;
		} else if (cnt == S6E8FA_ARRAY[2]) {
			/* 3 */
			cal_cnt = 0;
		} else if ((cnt > S6E8FA_ARRAY[2]) &&
			(cnt < S6E8FA_ARRAY[3])) {
			/* 4 ~ 10 */
			array_index = 3;

			pSmart->GRAY.TABLE[cnt].R_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].R_Gray,
			ptable[S6E8FA_ARRAY[array_index]].R_Gray,
			V3toV11_Coefficient, V3toV11_Multiple,
			V3toV11_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].G_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].G_Gray,
			ptable[S6E8FA_ARRAY[array_index]].G_Gray,
			V3toV11_Coefficient, V3toV11_Multiple,
			V3toV11_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].B_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].B_Gray,
			ptable[S6E8FA_ARRAY[array_index]].B_Gray,
			V3toV11_Coefficient, V3toV11_Multiple,
			V3toV11_denominator , cal_cnt);

			cal_cnt++;
		} else if (cnt == S6E8FA_ARRAY[3]) {
			/* 11 */
			cal_cnt = 0;
		} else if ((cnt > S6E8FA_ARRAY[3]) &&
			(cnt < S6E8FA_ARRAY[4])) {
			/* 12 ~ 22 */
			array_index = 4;

			pSmart->GRAY.TABLE[cnt].R_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].R_Gray,
			ptable[S6E8FA_ARRAY[array_index]].R_Gray,
			V11toV23_Coefficient, V11toV23_Multiple,
			V11toV23_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].G_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].G_Gray,
			ptable[S6E8FA_ARRAY[array_index]].G_Gray,
			V11toV23_Coefficient, V11toV23_Multiple,
			V11toV23_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].B_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].B_Gray,
			ptable[S6E8FA_ARRAY[array_index]].B_Gray,
			V11toV23_Coefficient, V11toV23_Multiple,
			V11toV23_denominator , cal_cnt);

			cal_cnt++;
		}  else if (cnt == S6E8FA_ARRAY[4]) {
			/* 23 */
			cal_cnt = 0;
		} else if ((cnt > S6E8FA_ARRAY[4]) &&
			(cnt < S6E8FA_ARRAY[5])) {
			/* 24 ~ 34 */
			array_index = 5;

			pSmart->GRAY.TABLE[cnt].R_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].R_Gray,
			ptable[S6E8FA_ARRAY[array_index]].R_Gray,
			V23toV35_Coefficient, V23toV35_Multiple,
			V23toV35_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].G_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].G_Gray,
			ptable[S6E8FA_ARRAY[array_index]].G_Gray,
			V23toV35_Coefficient, V23toV35_Multiple,
			V23toV35_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].B_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].B_Gray,
			ptable[S6E8FA_ARRAY[array_index]].B_Gray,
			V23toV35_Coefficient, V23toV35_Multiple,
			V23toV35_denominator , cal_cnt);

			cal_cnt++;
		} else if (cnt == S6E8FA_ARRAY[5]) {
			/* 35 */
			cal_cnt = 0;
		} else if ((cnt > S6E8FA_ARRAY[5]) &&
			(cnt < S6E8FA_ARRAY[6])) {
			/* 36 ~ 50 */
			array_index = 6;

			pSmart->GRAY.TABLE[cnt].R_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].R_Gray,
			ptable[S6E8FA_ARRAY[array_index]].R_Gray,
			V35toV51_Coefficient, V35toV51_Multiple,
			V35toV51_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].G_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].G_Gray,
			ptable[S6E8FA_ARRAY[array_index]].G_Gray,
			V35toV51_Coefficient, V35toV51_Multiple,
			V35toV51_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].B_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].B_Gray,
			ptable[S6E8FA_ARRAY[array_index]].B_Gray,
			V35toV51_Coefficient, V35toV51_Multiple,
			V35toV51_denominator, cal_cnt);
			cal_cnt++;

		} else if (cnt == S6E8FA_ARRAY[6]) {
			/* 51 */
			cal_cnt = 0;
		} else if ((cnt > S6E8FA_ARRAY[6]) &&
			(cnt < S6E8FA_ARRAY[7])) {
			/* 52 ~ 86 */
			array_index = 7;

			pSmart->GRAY.TABLE[cnt].R_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].R_Gray,
			ptable[S6E8FA_ARRAY[array_index]].R_Gray,
			V51toV87_Coefficient, V51toV87_Multiple,
			V51toV87_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].G_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].G_Gray,
			ptable[S6E8FA_ARRAY[array_index]].G_Gray,
			V51toV87_Coefficient, V51toV87_Multiple,
			V51toV87_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].B_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].B_Gray,
			ptable[S6E8FA_ARRAY[array_index]].B_Gray,
			V51toV87_Coefficient, V51toV87_Multiple,
			V51toV87_denominator, cal_cnt);
			cal_cnt++;

		} else if (cnt == S6E8FA_ARRAY[7]) {
			/* 87 */
			cal_cnt = 0;
		} else if ((cnt > S6E8FA_ARRAY[7]) &&
			(cnt < S6E8FA_ARRAY[8])) {
			/* 88 ~ 150 */
			array_index = 8;

			pSmart->GRAY.TABLE[cnt].R_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].R_Gray,
			ptable[S6E8FA_ARRAY[array_index]].R_Gray,
			V87toV151_Coefficient, V87toV151_Multiple,
			V87toV151_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].G_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].G_Gray,
			ptable[S6E8FA_ARRAY[array_index]].G_Gray,
			V87toV151_Coefficient, V87toV151_Multiple,
			V87toV151_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].B_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].B_Gray,
			ptable[S6E8FA_ARRAY[array_index]].B_Gray,
			V87toV151_Coefficient, V87toV151_Multiple,
			V87toV151_denominator, cal_cnt);

			cal_cnt++;
		} else if (cnt == S6E8FA_ARRAY[8]) {
			/* 151 */
			cal_cnt = 0;
		} else if ((cnt > S6E8FA_ARRAY[8]) &&
			(cnt < S6E8FA_ARRAY[9])) {
			/* 152 ~ 202 */
			array_index = 9;

			pSmart->GRAY.TABLE[cnt].R_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].R_Gray,
			ptable[S6E8FA_ARRAY[array_index]].R_Gray,
			V151toV203_Coefficient, V151toV203_Multiple,
			V151toV203_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].G_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].G_Gray,
			ptable[S6E8FA_ARRAY[array_index]].G_Gray,
			V151toV203_Coefficient, V151toV203_Multiple,
			V151toV203_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].B_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].B_Gray,
			ptable[S6E8FA_ARRAY[array_index]].B_Gray,
			V151toV203_Coefficient, V151toV203_Multiple,
			V151toV203_denominator, cal_cnt);

			cal_cnt++;
		} else if (cnt == S6E8FA_ARRAY[9]) {
			/* 203 */
			cal_cnt = 0;
		} else if ((cnt > S6E8FA_ARRAY[9]) &&
			(cnt < S6E8FA_ARRAY[10])) {
			/* 204 ~ 254 */
			array_index = 10;

			pSmart->GRAY.TABLE[cnt].R_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].R_Gray,
			ptable[S6E8FA_ARRAY[array_index]].R_Gray,
			V203toV255_Coefficient, V203toV255_Multiple,
			V203toV255_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].G_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].G_Gray,
			ptable[S6E8FA_ARRAY[array_index]].G_Gray,
			V203toV255_Coefficient, V203toV255_Multiple,
			V203toV255_denominator, cal_cnt);

			pSmart->GRAY.TABLE[cnt].B_Gray = cal_gray_scale_linear(
			ptable[S6E8FA_ARRAY[array_index-1]].B_Gray,
			ptable[S6E8FA_ARRAY[array_index]].B_Gray,
			V203toV255_Coefficient, V203toV255_Multiple,
			V203toV255_denominator, cal_cnt);

			cal_cnt++;
		 } else {
			if (cnt == S6E8FA_ARRAY[10]) {
				pr_info("%s end\n", __func__);
			} else {
				pr_err("%s fail cnt:%d\n", __func__, cnt);
				return -EINVAL;
			}
		}

	}

#ifdef SMART_DIMMING_DEBUG
		for (cnt = 0; cnt < S6E8FA_GRAY_SCALE_MAX; cnt++) {
			pr_info("%s %8d %8d %8d %d\n", __func__,
				pSmart->GRAY.TABLE[cnt].R_Gray,
				pSmart->GRAY.TABLE[cnt].G_Gray,
				pSmart->GRAY.TABLE[cnt].B_Gray, cnt);
		}
#endif
	return 0;
}

char offset_cal(int offset,  char value)
{
	unsigned char real_value;

	if (value < 0 )
		real_value = value * -1;
	else
		real_value = value;

	if (real_value - offset < 0)
		return 0;
	else if (real_value - offset > 255)
		return 0xFF;
	else
		return real_value - offset;
}

void mtp_offset_substraction(struct SMART_DIM *pSmart, char *str)
{
	int level_255_temp = 0;
	int level_255_temp_MSB = 0;
	int MTP_V255;

	/*subtration MTP_OFFSET value from generated gamma table*/
	level_255_temp = (str[0] << 8) | str[1] ;
	MTP_V255 = char_to_int_v255(pSmart->MTP.R_OFFSET.OFFSET_255_MSB,
				pSmart->MTP.R_OFFSET.OFFSET_255_LSB);
	level_255_temp -=  MTP_V255;
	level_255_temp_MSB = level_255_temp / 256;
	str[0] = level_255_temp_MSB & 0xff;
	str[1] = level_255_temp & 0xff;

	level_255_temp = (str[2] << 8) | str[3] ;
	MTP_V255 = char_to_int_v255(pSmart->MTP.G_OFFSET.OFFSET_255_MSB,
				pSmart->MTP.G_OFFSET.OFFSET_255_LSB);
	level_255_temp -=  MTP_V255;
	level_255_temp_MSB = level_255_temp / 256;
	str[2] = level_255_temp_MSB & 0xff;
	str[3] = level_255_temp & 0xff;

	level_255_temp = (str[4] << 8) | str[5] ;
	MTP_V255 = char_to_int_v255(pSmart->MTP.B_OFFSET.OFFSET_255_MSB,
				pSmart->MTP.B_OFFSET.OFFSET_255_LSB);
	level_255_temp -=  MTP_V255;
	level_255_temp_MSB = level_255_temp / 256;
	str[4] = level_255_temp_MSB & 0xff;
	str[5] = level_255_temp & 0xff;

	str[6] = offset_cal(char_to_int(pSmart->MTP.R_OFFSET.OFFSET_203), str[6]);
	str[7] = offset_cal(char_to_int(pSmart->MTP.G_OFFSET.OFFSET_203), str[7]);
	str[8] = offset_cal(char_to_int(pSmart->MTP.B_OFFSET.OFFSET_203), str[8]);

	str[9] = offset_cal(char_to_int(pSmart->MTP.R_OFFSET.OFFSET_151), str[9]);
	str[10] = offset_cal(char_to_int(pSmart->MTP.G_OFFSET.OFFSET_151), str[10]);
	str[11] = offset_cal(char_to_int(pSmart->MTP.B_OFFSET.OFFSET_151), str[11]);

	str[12] = offset_cal(char_to_int(pSmart->MTP.R_OFFSET.OFFSET_87), str[12]);
	str[13] = offset_cal(char_to_int(pSmart->MTP.G_OFFSET.OFFSET_87), str[13]);
	str[14] = offset_cal(char_to_int(pSmart->MTP.B_OFFSET.OFFSET_87), str[14]);

	str[15] = offset_cal(char_to_int(pSmart->MTP.R_OFFSET.OFFSET_51), str[15]);
	str[16] = offset_cal(char_to_int(pSmart->MTP.G_OFFSET.OFFSET_51), str[16]);
	str[17] = offset_cal(char_to_int(pSmart->MTP.B_OFFSET.OFFSET_51), str[17]);

	str[18] = offset_cal(char_to_int(pSmart->MTP.R_OFFSET.OFFSET_35), str[18]);
	str[19] = offset_cal(char_to_int(pSmart->MTP.G_OFFSET.OFFSET_35), str[19]);
	str[20] = offset_cal(char_to_int(pSmart->MTP.B_OFFSET.OFFSET_35), str[20]);

	str[21] = offset_cal(char_to_int(pSmart->MTP.R_OFFSET.OFFSET_23), str[21]);
	str[22] = offset_cal(char_to_int(pSmart->MTP.G_OFFSET.OFFSET_23), str[22]);
	str[23] = offset_cal(char_to_int(pSmart->MTP.B_OFFSET.OFFSET_23), str[23]);

	str[24] = offset_cal(char_to_int(pSmart->MTP.R_OFFSET.OFFSET_11), str[24]);
	str[25] = offset_cal(char_to_int(pSmart->MTP.G_OFFSET.OFFSET_11), str[25]);
	str[26] = offset_cal(char_to_int(pSmart->MTP.B_OFFSET.OFFSET_11), str[26]);

	str[27] = offset_cal(char_to_int(pSmart->MTP.R_OFFSET.OFFSET_3), str[27]);
	str[28] = offset_cal(char_to_int(pSmart->MTP.G_OFFSET.OFFSET_3), str[28]);
	str[29] = offset_cal(char_to_int(pSmart->MTP.B_OFFSET.OFFSET_3), str[29]);
}



static int searching_function(long long candela, int *index, int gamma_curve)
{
	long long delta_1 = 0, delta_2 = 0;
	int cnt;

	/*
	*	This searching_functin should be changed with improved
		searcing algorithm to reduce searching time.
	*/
	*index = -1;

	for (cnt = 0; cnt < (S6E8FA_GRAY_SCALE_MAX-1); cnt++) {
		if (gamma_curve == GAMMA_CURVE_1P9) {
			delta_1 = candela - curve_1p9[cnt];
			delta_2 = candela - curve_1p9[cnt+1];
		} else {
			delta_1 = candela - curve_2p2[cnt];
			delta_2 = candela - curve_2p2[cnt+1];
		}

		if (delta_2 < 0) {
			*index = (delta_1 + delta_2) <= 0 ? cnt : cnt+1;
			break;
		}

		if (delta_1 == 0) {
			*index = cnt;
			break;
		}

		if (delta_2 == 0) {
			*index = cnt+1;
			break;
		}
	}

	if (*index == -1)
		return -EINVAL;
	else
		return 0;
}


/* -1 means V1 */
#define S6E8FA_TABLE_MAX  (S6E8FA_MAX-1)
void(*Make_hexa[S6E8FA_TABLE_MAX])(int*, struct SMART_DIM*, char*) = {
	v255_hexa,
	v203_hexa,
	v151_hexa,
	v87_hexa,
	v51_hexa,
	v35_hexa,
	v23_hexa,
	v11_hexa,
	v3_hexa,
	vt_hexa,
};

#if defined(AID_OPERATION)
/*
*	Because of AID operation & display quality.
*
*	only smart dimmg range : 300CD ~ 190CD
*	AOR fix range : 180CD ~ 110CD  AOR 40%
*	AOR adjust range : 100CD ~ 10CD
*/
#define AOR_FIX_CD 180
#define AOR_ADJUST_CD 110

static int aor_fix_bl_leve_vt888[] = {
	170,	/* 110CD */
	184,	/* 120CD */
	199,	/* 130CD */
	213,	/* 140CD */
	227,	/* 150CD */
	241,	/* 160CD */
	256,	/* 170CD */
	270,	/* 180CD */
};

static int adding_180cd_110cd_vt888[][3] = {
/*	  R  G  B	*/
	{ 0, -1, 0},	/* V255 */
	{ 0,  0, 0},	/* V87 */
	{ 0,  0, 0},	/* V51 */
	{ -2,  0, -2},	/* V35 */
	{ -6,  0, -4},	/* V25 */
	{ -25,  0, -3},	/* V11 */
};

static int adding_compensation_vt888[][24] = {

	{
	/*	  R  G  B	100CD */
		 0, 0, 0,		/* V255 */
		 0, 0, 0,		/* V203 */
		 0, 0, 0,		/* V151 */
		 0, 0, 0,		/* V87 */
		 0, 0, 0,		/* V51 */
		 0, 0, 0,		/* V35 */
		 0, 0, 0,		/* V23 */
		 0, 0, 0,		/* V11 */
	},
	{
	/*	  R  G  B	90CD */
		 0, -1, 0,	/* V255 */
		 0, 0, 0,		/* V203 */
		 0, 0, 0,		/* V151 */
		 0, 0, 0,		/* V87 */
		 -2, 0, -2,	/* V51 */
		 -3, 0, -1,	/* V35 */
		 -3, 0, -1,	/* V23 */
		 -25, 0, 0,	/* V11 */
	},
	{
	/*	  R  G  B	80CD */
		 0, -1, 0,	/* V255 */
		 0, 0, 0,		/* V203 */
		 0, 0, 0,		/* V151 */
		 -1, 0, -1,	/* V87 */
		 -2, 0, -2,	/* V51 */
		 -4, 0, -2,	/* V35 */
		 -7, 0, -4,	/* V23 */
		 -48, 0, 0,	/* V11 */
	},
	{
	/*	  R  G  B	70CD */
		 0, -1, 0,	/* V255 */
		 0, 0, 0,		/* V203 */
		 0, 0, 0,		/* V151 */
		 -1, 0, -2,	/* V87 */
		 -3, 0, -2,	/* V51 */
		 -6, 0, -3,	/* V35 */
		 -12, 0, -6,	/* V23 */
		 -60, 0, 0,	/* V11 */
	},
	{
	/*	  R  G  B	60 CD*/
		 0, -1, 0,	/* V255 */
		 0, 0, 0,		/* V203 */
		 0, 0, 0,		/* V151 */
		 -2, 0, -2,	/* V87 */
		 -4, 0, -3,	/* V51 */
		 -8, 0, -5,	/* V35 */
		 -19, 0, -7,	/* V23 */
		 -60, 0, 0,	/* V11 */
	},
	{
	/*	  R  G  B	50 CD*/
		 0, -1, 0,	/* V255 */
		 0, 0, 0,		/* V203 */
		 0, 0, 0,		/* V151 */
		 -2, 0, -2,	/* V87 */
		 -5, 0, -4,	/* V51 */
		 -10, 0, -7,	/* V35 */
		 -27, 0, 7,	/* V23 */
		 -60, 0, 0,	/* V11 */
	},
	{
	/*	  R  G  B	40CD */
		 0, -1, 0,	/* V255 */
		 0, 0, 0,		/* V203 */
		 0, 0, 0,		/* V151 */
		 -2, 1, -2,	/* V87 */
		 -6, 1, -4,	/* V51 */
		 -12, 1, -8,	/* V35 */
		 -38, 2, -7,	/* V23 */
		 -60, 0, 1,	/* V11 */
	},
	{
	/*	  R  G  B	30CD */
		 -1, -1, 0,	/* V255 */
		 0, 0, 0,		/* V203 */
		 -1, 0, 0,	/* V151 */
		 -2, 1, -2,	/* V87 */
		 -8, 2, -4,	/* V51 */
		 -15, 3, -9,	/* V35 */
		 -53, 3, -8,	/* V23 */
		 -60, 0, 7,	/* V11 */
	},
	{
	/*	  R  G  B	20CD */
		 -1, -1, 0,	/* V255 */
		 0, 0, 0,		/* V203 */
		 -1, 0, 0,	/* V151 */
		 -5, 3, -3,	/* V87 */
		 -13, 4, -5,	/* V51 */
		 -19, 5, 10,	/* V35 */
		 -60, 3, -5,	/* V23 */
		 -60, 0, 23,	/* V11 */
	},
	{
		/*	  R  G  B	10CD */
		 -1, -1, 0,	/* V255 */
		 0, 1, 0,		/* V203 */
		 -2, 3, 0,	/* V151 */
		 -9, 4, -4,	/* V87 */
		 -28, 5, -9,	/* V51 */
		 -60, 7, -11,	/* V35 */
		 -60, 0, 5,	/* V23 */
		 -60, 0, 48,	/* V11 */
	}
};

static void gamma_init_vt888(struct SMART_DIM *pSmart, char *str, int size)
{
	long long candela_level[S6E8FA_TABLE_MAX] = {-1, };
	int bl_index[S6E8FA_TABLE_MAX] = {-1, };

	long long temp_cal_data = 0;
	int bl_level, aor_bl_level;
	int level_255_temp = 0;
	int level_255_temp_MSB = 0;
	int point_index;
	int cnt;

	/*calculate candela level */
	if (pSmart->brightness_level > AOR_FIX_CD) {
		/* 300CD ~ 190CD */
		bl_level = pSmart->brightness_level;
	} else if ((pSmart->brightness_level <= AOR_FIX_CD) &&
				(pSmart->brightness_level >= AOR_ADJUST_CD)) {
		/* 180CD ~ 110CD */
		aor_bl_level = (pSmart->brightness_level / 10) - 11;
		bl_level = aor_fix_bl_leve_vt888[aor_bl_level];
	} else {
		/* 100CD ~ 10CD */
		bl_level = AOR_ADJUST_CD;
	}

	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
		point_index = S6E8FA_ARRAY[cnt+1];
		temp_cal_data =
		((long long)(candela_coeff_2p2[point_index])) *
		((long long)(bl_level));
		candela_level[cnt] = temp_cal_data;
	}


#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n candela_1:%llu  candela_3:%llu  candela_11:%llu ",
		candela_level[0], candela_level[1], candela_level[2]);
	printk(KERN_INFO "candela_23:%llu  candela_35:%llu  candela_51:%llu ",
		candela_level[3], candela_level[4], candela_level[5]);
	printk(KERN_INFO "candela_87:%llu  candela_151:%llu  candela_203:%llu ",
		candela_level[6], candela_level[7], candela_level[8]);
	printk(KERN_INFO "candela_255:%llu\n", candela_level[9]);
#endif

	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
		if (searching_function(candela_level[cnt],
			&(bl_index[cnt]), GAMMA_CURVE_2P2)) {
			pr_info("%s searching functioin error cnt:%d\n",
				__func__, cnt);
		}
	}

	/*
	*	210CD ~ 190CD compensation
	*	V3 level + 1
	*/
	if ((pSmart->brightness_level >= 190) &&
					(pSmart->brightness_level <= 210))
		bl_index[1] += 1;

	/*
	*	130CD ~ 10CD compensation
	*	V3 level + 1
	*/
	if ((pSmart->brightness_level >= 10) &&
					(pSmart->brightness_level <= 130))
		bl_index[1] += 1;

#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n bl_index_1:%d  bl_index_3:%d  bl_index_11:%d",
		bl_index[0], bl_index[1], bl_index[2]);
	printk(KERN_INFO "bl_index_23:%d bl_index_35:%d  bl_index_51:%d",
		bl_index[3], bl_index[4], bl_index[5]);
	printk(KERN_INFO "bl_index_87:%d  bl_index_151:%d bl_index_203:%d",
		bl_index[5], bl_index[7], bl_index[8]);
	printk(KERN_INFO "bl_index_255:%d\n", bl_index[9]);
#endif

	/*Generate Gamma table*/
	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++)
		(void)Make_hexa[cnt](bl_index , pSmart, str);

	/*
	*	180CD ~ 110CD compensation
	*/
	if ((pSmart->brightness_level <= 180) &&
				(pSmart->brightness_level >= 110)) {
		/* V255 */
		level_255_temp = (str[0] << 8) | str[1] ;
		level_255_temp +=  adding_180cd_110cd_vt888[0][0];
		level_255_temp_MSB = level_255_temp / 256;
		str[0] = level_255_temp_MSB & 0xff;
		str[1] = level_255_temp & 0xff;

		level_255_temp = (str[2] << 8) | str[3] ;
		level_255_temp +=  adding_180cd_110cd_vt888[0][1];
		level_255_temp_MSB = level_255_temp / 256;
		str[2] = level_255_temp_MSB & 0xff;
		str[3] = level_255_temp & 0xff;

		level_255_temp = (str[4] << 8) | str[5] ;
		level_255_temp +=  adding_180cd_110cd_vt888[0][2];
		level_255_temp_MSB = level_255_temp / 256;
		str[4] = level_255_temp_MSB & 0xff;
		str[5] = level_255_temp & 0xff;

		/* V87 */
		str[12] += adding_180cd_110cd_vt888[2][0];
		str[13] += adding_180cd_110cd_vt888[2][1];
		str[14] += adding_180cd_110cd_vt888[2][2];

		/* V51 */
		str[15] += adding_180cd_110cd_vt888[2][0];
		str[16] += adding_180cd_110cd_vt888[2][1];
		str[17] += adding_180cd_110cd_vt888[2][2];

		/* V35 */
		str[18] += adding_180cd_110cd_vt888[3][0];
		str[19] += adding_180cd_110cd_vt888[3][1];
		str[20] += adding_180cd_110cd_vt888[3][2];

		/* V23 */
		str[21] += adding_180cd_110cd_vt888[4][0];
		str[22] += adding_180cd_110cd_vt888[4][1];
		str[23] += adding_180cd_110cd_vt888[4][2];

		/* V11 */
		str[24] += adding_180cd_110cd_vt888[5][0];
		str[25] += adding_180cd_110cd_vt888[5][1];
		str[26] += adding_180cd_110cd_vt888[5][2];
	}

	if (pSmart->brightness_level <= 100) {
		point_index = 10 - (pSmart->brightness_level / 10);

		level_255_temp = (str[0] << 8) | str[1] ;
		level_255_temp +=  adding_compensation_vt888[point_index][0];
		level_255_temp_MSB = level_255_temp / 256;
		str[0] = level_255_temp_MSB & 0xff;
		str[1] = level_255_temp & 0xff;

		level_255_temp = (str[2] << 8) | str[3] ;
		level_255_temp +=  adding_compensation_vt888[point_index][1];
		level_255_temp_MSB = level_255_temp / 256;
		str[2] = level_255_temp_MSB & 0xff;
		str[3] = level_255_temp & 0xff;

		level_255_temp = (str[4] << 8) | str[5] ;
		level_255_temp +=  adding_compensation_vt888[point_index][2];
		level_255_temp_MSB = level_255_temp / 256;
		str[4] = level_255_temp_MSB & 0xff;
		str[5] = level_255_temp & 0xff;

		for (cnt = 3; cnt < 24; cnt++) {
			str[cnt + 3] +=
				adding_compensation_vt888[point_index][cnt];
		}
	}

	/*subtration MTP_OFFSET value from generated gamma table*/
	mtp_offset_substraction(pSmart, str);
}

static int aor_fix_bl_leve_vt232[] = {
	180,	/* 110CD */
	194,	/* 120CD */
	208,	/* 130CD */
	222,	/* 140CD */
	235,	/* 150CD */
	249,	/* 160CD */
	263,	/* 170CD */
	277,	/* 180CD */
};

static int adding_180cd_110cd_vt232[][12] = {
	{
		/*	  R  G  B	180CD */
		0, 0, 0,		/* V51 */
		0, 0, 0,		/* V35 */
		-2, 0, -3,	/* V23 */
		-6, 2, -6,	/* V11 */
	},
	{
		/*	  R  G  B	170CD */
		0, 0, 0,		/* V51 */
		0, 0, 0,		/* V35 */
		-2, 0, -3,	/* V23 */
		-6, 2, -6,	/* V11 */
	},
	{
		/*	  R  G  B	160CD */
		0, 0, 0,		/* V51 */
		0, 0, 0,		/* V35 */
		-2, 0, -3,	/* V23 */
		-6, 2, -6,	/* V11 */
	},
	{
		/*	  R  G  B	150CD */
		0, 0, 0,		/* V51 */
		0, 0, 0,		/* V35 */
		-2, 0, -3,	/* V23 */
		-6, 2, -6,	/* V11 */
	},
	{
		/*	  R  G  B	140CD */
		0, 0, -1,		/* V51 */
		0, 0, 0,		/* V35 */
		-3, 0, -4,	/* V23 */
		-10, 3, -10,	/* V11 */
	},
	{
		/*	  R  G  B	130CD */
		0, 0, -2,		/* V51 */
		0, 0, 0,		/* V35 */
		-3, 0, -4,	/* V23 */
		-12, 3, -12,	/* V11 */
	},
	{
		/*	  R  G  B	120CD */
		0, 0, -2,		/* V51 */
		0, 0, 0,		/* V35 */
		-3, 0, -4,	/* V23 */
		-12, 3, -12,	/* V11 */
	},
	{
		/*	  R  G  B	110CD */
		0, 0, -2,		/* V51 */
		0, 0, 0,		/* V35 */
		-3, 0, -4,	/* V23 */
		-12, 3, -12,	/* V11 */
	},
};

static int adding_compensation_vt232[][24] = {
	{
		/*	  R  G  B	100CD */
		0, 0, 0,		/* V255 */
		0, 0, 0,		/* V203 */
		0, 0, 0,		/* V151 */
		-2, 0, -1,		/* V87 */
		-1, 0, 0,		/* V51 */
		-2, 0, 0,		/* V35 */
		0, 0, -4,		/* V23 */
		-26, 0, -11,		/* V11 */
	},
	{
		/*	  R  G  B	90CD */
		0, -1, 0,		/* V255 */
		-1, 0, -1,	/* V203 */
		-1, 0, -1,	/* V151 */
		-1, 0, -1,	/* V87 */
		-2, 0, 0,		/* V51 */
		-2, 0, 0,		/* V35 */
		-2, 0, -6,	/* V23 */
		-18, 0, -17,	/* V11 */
	},
	{
		/*	  R  G  B	80CD */
		0, -1, 0,		/* V255 */
		-1, 0, -1,	/* V203 */
		-1, 0, -1,	/* V151 */
		-1, 0, -1,	/* V87 */
		-3, 0, -1,	/* V51 */
		-3, 0, -1,	/* V35 */
		-3, 0, -6,	/* V23 */
		-21, 0, -18,	/* V11 */
	},
	{
		/*	  R  G  B	70CD */
		0, -1, 0,		/* V255 */
		-1, 0, -1,	/* V203 */
		-1, 0, -1,	/* V151 */
		-2, 0, -1,	/* V87 */
		-1, 0, -1,	/* V51 */
		-5, 0, -2,	/* V35 */
		-4, 1, -8,	/* V23 */
		-28, 0, -23,	/* V11 */
	},
	{
		/*	  R  G  B	60 CD*/
		0, -1, 0,		/* V255 */
		-1, 0, -1,	/* V203 */
		-1, 0, -1,	/* V151 */
		-2, 0, -2,	/* V87 */
		-3, 0, -1,	/* V51 */
		-8, 0, -6,	/* V35 */
		-9, 6, -12,	/* V23 */
		-4, 4, -11,	/* V11 */
	},
	{
		/*	  R  G  B	50 CD*/
		0, -1, 0,		/* V255 */
		-1, 0, -1,	/* V203 */
		-1, 0, -1,	/* V151 */
		-1, 0, -2,	/* V87 */
		-4, 0, -3,	/* V51 */
		-9, 0, -6,	/* V35 */
		-8, 1, -11,	/* V23 */
		-50, 0, -27,	/* V11 */
	},
	{
		/*	  R  G  B	40CD */
		0, -1, 0,		/* V255 */
		0, 0, -1,		/* V203 */
		-2, 0, -1,	/* V151 */
		-2, 0, -3,	/* V87 */
		-6, 0, -5,	/* V51 */
		-9, 1, -6,	/* V35 */
		-9, 2, -13,	/* V23 */
		-20, 0, -18,	/* V11 */
	},
	{
		/*	  R  G  B	30CD */
		-1, -1, 0,	/* V255 */
		0, 0, -1,		/* V203 */
		-2, 0, -2,	/* V151 */
		-3, 0, -4,	/* V87 */
		-8, 1, -5,	/* V51 */
		-10, 1, -9,	/* V35 */
		-16, 2, -19,	/* V23 */
		-2, 0, -4,	/* V11 */
	},
	{
		/*	  R  G  B	20CD */
		-1, -1, 0,	/* V255 */
		0, 0, -1,		/* V203 */
		-2, 0, -2,	/* V151 */
		-5, 1, -5,	/* V87 */
		-8, 3, -6,	/* V51 */
		-6, 3, -6,	/* V35 */
		-12, 6, -12,	/* V23 */
		0, 2, -3,		/* V11 */
	},
	{
		/*	  R  G  B	10CD */
		-1, 0, 0,		/* V255 */
		-3, 0, -2,	/* V203 */
		-3, 1, -2,		/* V151 */
		-6, 3, -8,	/* V87 */
		-11, 4, -9,	/* V51 */
		-8, 6, -8,	/* V35 */
		-7, 10, -5,	/* V23 */
		-13, 0, 5,	/* V11 */
	}
};

static void gamma_init_vt232(struct SMART_DIM *pSmart, char *str, int size)
{
	long long candela_level[S6E8FA_TABLE_MAX] = {-1, };
	int bl_index[S6E8FA_TABLE_MAX] = {-1, };

	long long temp_cal_data = 0;
	int bl_level, aor_bl_level;
	int level_255_temp = 0;
	int level_255_temp_MSB = 0;
	int point_index;
	int cnt;

	/*calculate candela level */
	if (pSmart->brightness_level > AOR_FIX_CD) {
		/* 300CD ~ 190CD */
		bl_level = pSmart->brightness_level;
	} else if ((pSmart->brightness_level <= AOR_FIX_CD) &&
				(pSmart->brightness_level >= AOR_ADJUST_CD)) {
		/* 180CD ~ 110CD */
		aor_bl_level = (pSmart->brightness_level / 10) - 11;
		bl_level = aor_fix_bl_leve_vt232[aor_bl_level];
	} else {
		/* 100CD ~ 10CD */
		bl_level = AOR_ADJUST_CD;
	}

	if ((pSmart->brightness_level <= 290) &&
					(pSmart->brightness_level >= 190)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p25[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 180) &&
					(pSmart->brightness_level >= 80)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p2[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 70) &&
					(pSmart->brightness_level >= 60)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p15[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 50) &&
					(pSmart->brightness_level >= 30)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p1[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if (pSmart->brightness_level == 20) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p0[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if (pSmart->brightness_level == 10) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p9[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p2[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}
	}


#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n candela_1:%llu  candela_3:%llu  candela_11:%llu ",
		candela_level[0], candela_level[1], candela_level[2]);
	printk(KERN_INFO "candela_23:%llu  candela_35:%llu  candela_51:%llu ",
		candela_level[3], candela_level[4], candela_level[5]);
	printk(KERN_INFO "candela_87:%llu  candela_151:%llu  candela_203:%llu ",
		candela_level[6], candela_level[7], candela_level[8]);
	printk(KERN_INFO "candela_255:%llu\n", candela_level[9]);
#endif

	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
		if (searching_function(candela_level[cnt],
			&(bl_index[cnt]), GAMMA_CURVE_2P2)) {
			pr_info("%s searching functioin error cnt:%d\n",
			__func__, cnt);
		}
	}

	/*
	*	290CD ~ 10CD compensation
	*/
	if (pSmart->brightness_level == 290)
		bl_index[5] += 1; /* V51 */
	else if (pSmart->brightness_level == 280)
		bl_index[4] += 1; /* V35 */
	else if (pSmart->brightness_level == 260) {
		bl_index[1] += 1; /*V3*/
		bl_index[4] += 1; /* V35 */
	} else if ((pSmart->brightness_level <= 250) &&
					(pSmart->brightness_level >= 190))
		bl_index[1] += 1; /*V3*/
	else if ((pSmart->brightness_level <= 130) &&
					(pSmart->brightness_level >= 110))
		bl_index[1] += 1; /*V3*/
	else if (pSmart->brightness_level == 100) {
		bl_index[1] += 1; /*V3*/
		bl_index[2] += 1; /*V11*/
	} else if (pSmart->brightness_level == 90) {
		bl_index[1] += 1; /* V3 */
		bl_index[2] += 2; /* V11 */
		bl_index[3] += 1; /* V23 */
	} else if (pSmart->brightness_level == 80) {
		bl_index[1] += 1; /* V3 */
		bl_index[2] += 3; /* V11 */
		bl_index[3] += 2; /* V23 */
		bl_index[4] += 1; /* V35 */
	} else if (pSmart->brightness_level == 70) {
		bl_index[1] += 1; /* V3 */
		bl_index[2] += 4; /* V11 */
		bl_index[3] += 2; /* V23 */
		bl_index[4] += 1; /* V35 */
	} else if (pSmart->brightness_level == 60) {
		bl_index[1] += 1; /* V3 */
		bl_index[2] += 4; /* V11 */
		bl_index[3] += 3; /* V23 */
		bl_index[4] += 1; /* V35 */
	} else if (pSmart->brightness_level == 50) {
		bl_index[1] += 1; /* V3 */
		bl_index[2] += 4; /* V11 */
		bl_index[3] += 3; /* V23 */
		bl_index[4] += 1; /* V35 */
	} else if (pSmart->brightness_level == 40) {
		bl_index[1] += 1; /* V3 */
		bl_index[2] += 5; /* V11 */
		bl_index[3] += 4; /* V23 */
		bl_index[4] += 2; /* V35 */
		bl_index[5] += 1; /* V51 */
	} else if (pSmart->brightness_level == 30) {
		bl_index[1] += 1; /* V3 */
		bl_index[2] += 7; /* V11 */
		bl_index[3] += 6; /* V23 */
		bl_index[4] += 4; /* V35 */
		bl_index[5] += 2; /* V51 */
	} else if (pSmart->brightness_level == 20) {
		bl_index[2] += 8; /* V11 */
		bl_index[3] += 7; /* V23 */
		bl_index[4] += 5; /* V35 */
		bl_index[5] += 3; /* V51 */
		bl_index[6] += 1; /* V87 */
	} else if (pSmart->brightness_level == 10) {
		bl_index[2] += 10; /* V11 */
		bl_index[3] += 9; /* V23 */
		bl_index[4] += 8; /* V35 */
		bl_index[5] += 6; /* V51 */
		bl_index[6] += 3; /* V87 */
	} else
		pr_debug("%s nothing", __func__);

#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n bl_index_1:%d  bl_index_3:%d  bl_index_11:%d",
		bl_index[0], bl_index[1], bl_index[2]);
	printk(KERN_INFO "bl_index_23:%d bl_index_35:%d  bl_index_51:%d",
		bl_index[3], bl_index[4], bl_index[5]);
	printk(KERN_INFO "bl_index_87:%d  bl_index_151:%d bl_index_203:%d",
		bl_index[6], bl_index[7], bl_index[8]);
	printk(KERN_INFO "bl_index_255:%d\n", bl_index[9]);
#endif
	/*Generate Gamma table*/
	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++)
		(void)Make_hexa[cnt](bl_index , pSmart, str);

	/*
	*	180CD ~ 110CD compensation
	*/
	if ((pSmart->brightness_level <= 180) &&
				(pSmart->brightness_level >= 110)) {
		point_index = 18 - (pSmart->brightness_level / 10);

		for (cnt = 15; cnt < 27; cnt++) {
			str[cnt] +=
				adding_180cd_110cd_vt232[point_index][cnt - 15];
		}
	}

	if (pSmart->brightness_level <= 100) {
		point_index = 10 - (pSmart->brightness_level / 10);

		level_255_temp = (str[0] << 8) | str[1] ;
		level_255_temp +=  adding_compensation_vt232[point_index][0];
		level_255_temp_MSB = level_255_temp / 256;
		str[0] = level_255_temp_MSB & 0xff;
		str[1] = level_255_temp & 0xff;

		level_255_temp = (str[2] << 8) | str[3] ;
		level_255_temp +=  adding_compensation_vt232[point_index][1];
		level_255_temp_MSB = level_255_temp / 256;
		str[2] = level_255_temp_MSB & 0xff;
		str[3] = level_255_temp & 0xff;

		level_255_temp = (str[4] << 8) | str[5] ;
		level_255_temp +=  adding_compensation_vt232[point_index][2];
		level_255_temp_MSB = level_255_temp / 256;
		str[4] = level_255_temp_MSB & 0xff;
		str[5] = level_255_temp & 0xff;

		for (cnt = 3; cnt < 24; cnt++) {
			str[cnt + 3] +=
				adding_compensation_vt232[point_index][cnt];
		}
	}

	/*subtration MTP_OFFSET value from generated gamma table*/
	mtp_offset_substraction(pSmart, str);
}


#define CCG6_MAX_TABLE 53
static int ccg6_candela_table[][2] = {
{10, 0,},
{11, 1,},
{12, 2,},
{13, 3,},
{14, 4,},
{15, 5,},
{16, 6,},
{17, 7,},
{19, 8,},
{20, 9,},
{21, 10,},
{22, 11,},
{24, 12,},
{25, 13,},
{27, 14,},
{29, 15,},
{30, 16,},
{32, 17,},
{34, 18,},
{37, 19,},
{39, 20,},
{41, 21,},
{44, 22,},
{47, 23,},
{50, 24,},
{53, 25,},
{56, 26,},
{60, 27,},
{64, 28,},
{68, 29,},
{72, 30,},
{77, 31,},
{82, 32,},
{87, 33,},
{93, 34,},
{98, 35,},
{105, 36,},
{111, 37,},
{119, 38,},
{126, 39,},
{134, 40,},
{143, 41,},
{152, 42,},
{162, 43,},
{172, 44,},
{183, 45,},
{195, 46,},
{207, 47,},
{220, 48,},
{234, 49,},
{249, 50,},
{265, 51,},
{282, 52,},
{300, 53,},
};

static int gradation_offset[][9] = {
/*	V255 V203 V151 V87 V51 V35 V23 V11 V3 */	
	{0, 2, 3, 6, 7, 9, 10, 12, 15,},
	{0, 2, 3, 6, 7, 9, 10, 12, 15,},
	{0, 2, 3, 6, 7, 8, 9, 11, 14,},
	{0, 2, 3, 5, 6, 8, 9, 11, 14,},
	{0, 2, 3, 5, 6, 7, 8, 10, 13,},
	{0, 2, 3, 5, 6, 7, 8, 10, 13,},
	{0, 2, 3, 5, 6, 6, 7, 9, 12,},
	{0, 2, 3, 4, 5, 6, 7, 9, 11,},
	{0, 2, 3, 4, 5, 5, 6, 8, 10,},
	{0, 2, 3, 4, 5, 5, 6, 8, 10,},
	{0, 1, 2, 3, 4, 4, 5, 8, 9,},
	{0, 1, 2, 3, 4, 4, 5, 7, 9,},
	{0, 1, 2, 3, 4, 4, 5, 7, 8,},
	{0, 1, 2, 3, 4, 4, 5, 7, 8,},
	{0, 1, 2, 3, 3, 4, 5, 6, 8,},
	{0, 1, 2, 3, 3, 4, 5, 6, 7,},
	{0, 1, 2, 3, 3, 4, 5, 6, 7,},
	{0, 1, 2, 3, 3, 4, 5, 5, 7,},
	{0, 1, 2, 3, 2, 4, 5, 5, 7,},
	{0, 1, 1, 2, 2, 3, 4, 4, 6,},
	{0, 1, 1, 2, 2, 3, 4, 4, 6,},
	{0, 1, 1, 2, 2, 3, 4, 4, 6,},
	{0, 1, 1, 2, 3, 3, 4, 4, 6,},
	{0, 1, 1, 1, 3, 3, 4, 4, 6,},
	{0, 1, 1, 1, 3, 3, 4, 4, 6,},
	{0, 1, 1, 1, 3, 3, 3, 3, 5,},
	{0, 1, 1, 1, 3, 3, 3, 3, 5,},
	{0, 1, 1, 1, 3, 3, 3, 3, 5,},
	{0, 1, 1, 1, 2, 2, 3, 2, 5,},
	{0, 2, 1, 2, 2, 2, 3, 2, 4,},
	{0, 2, 1, 2, 2, 2, 3, 2, 4,},
	{0, 2, 1, 2, 2, 1, 2, 2, 3,},
	{0, 2, 1, 2, 2, 1, 2, 2, 3,},
	{0, 2, 2, 1, 3, 1, 2, 2, 2,},
	{0, 2, 2, 1, 2, 1, 2, 2, 2,},
	{0, 1, 2, 1, 2, 1, 2, 2, 2,},
	{0, 1, 2, 1, 2, 1, 1, 1, 1,},
	{0, 1, 2, 1, 1, 1, 1, 1, 1,},
	{0, 1, 2, 1, 1, 0, 1, 1, 1,},
	{0, 0, 0, 0, 0, 0, 1, 1, 1,},
	{0, 0, 0, 0, 0, 0, 0, 1, 1,},
	{0, 0, 0, 0, 0, 0, 0, 1, 1,},
	{0, 0, 0, 0, 0, 0, 0, 1, 1,},
	{0, 0, 0, 0, 0, 0, 0, 0, 1,},
	{0, 0, 0, 0, 0, 0, 0, 0, 1,},
	{0, 0, 0, 0, 0, 0, 0, 0, 0,},
	{0, 0, 0, 0, 1, 1, 1, 0, 0,},
	{0, 0, 0, 0, 0, 1, 1, 0, 0,},
	{0, 0, 0, 0, 0, 0, 0, 0, 0,},
	{0, 0, 0, 0, 0, 0, 0, 0, 0,},
	{0, 0, 0, 0, 0, 0, 0, 0, 0,},
	{0, 0, 0, 0, 0, 0, 0, 0, 0,},
	{0, 0, 0, 0, 0, 0, 0, 0, 0,},
	{0, 0, 0, 0, 0, 0, 0, 0, 0,},
};

#define RGB_COMPENSATION 24
static int rgb_offset[][RGB_COMPENSATION] = {
/*	R255 G255 B255 R203 G203 B203 R151 G151 B151
	R87 G87 B87 R51 G51 B51 R35 G35 B35 
	R23 G23 B23 R11 G11 B11
*/
{-1, 0, 0, 0, 0, -2, -2, 0, -3, -3, 1, -4, -5,
3, -5, -3, 3, -3, -5, 4, -6, -13, 8, -13,},
{-1, 0, 0, 0, 0, -2, -2, 0, -3, -3, 1, -4, -5,
3, -5, -3, 3, -3, -5, 4, -6, -13, 8, -13,},
{-1, 0, 0, 0, 0, -1, -2, 0, -3, -3, 1, -4, -5,
3, -5, -3, 3, -3, -5, 4, -6, -12, 8, -13,},
{0, 0, 0, 0, 0, -1, -2, 0, -2, -2, 1, -3, -4,
3, -4, -3, 3, -3, -5, 4, -6, -12, 8, -13,},
{0, 0, 0, 0, 0, -1, -2, 0, -2, -2, 1, -3, -4,
2, -4, -3, 2, -4, -5, 4, -6, -11, 8, -13,},
{0, 0, 0, 0, 0, 0, -2, 0, -2, -2, 1, -3, -4,
2, -4, -4, 2, -4, -5, 3, -6, -11, 8, -13,},
{0, 0, 0, 0, 0, 0, -2, 0, -2, -2, 1, -3, -4,
2, -4, -4, 2, -4, -5, 3, -6, -10, 8, -13,},
{0, 0, 0, 0, 0, 0, -2, 0, -2, -1, 1, -2, -3,
2, -3, -4, 2, -4, -5, 3, -6, -10, 8, -13,},
{0, 0, 0, 0, 0, 0, -2, 0, -2, -1, 1, -2, -3,
2, -3, -4, 2, -4, -5, 3, -6, -9, 8, -13,},
{0, 0, 0, 0, 0, 0, -2, 0, -2, -1, 1, -2, -3,
2, -3, -4, 2, -4, -5, 3, -6, -9, 8, -13,},
{0, 0, 0, 0, 0, 0, -2, 0, -2, -1, 1, -2, -3,
2, -3, -3, 2, -3, -5, 3, -5, -9, 6, -13,},
{0, 0, 0, 0, 0, 0, -2, 0, -2, -1, 1, -2, -3,
2, -3, -3, 2, -3, -5, 3, -5, -9, 6, -12,},
{0, 0, 0, 0, 0, 0, -1, 0, -2, -1, 1, -2, -3,
2, -3, -3, 2, -2, -5, 3, -5, -9, 6, -12,},
{0, 0, 1, -1, 0, -1, -1, 0, -2, -2, 1, -2, -2,
1, -2, -2, 2, -2, -4, 3, -5, -9, 6, -11,},
{0, 0, 1, -1, 0, -1, -1, 0, -2, -2, 1, -2, -2,
1, -2, -2, 2, -2, -4, 2, -4, -9, 6, -11,},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 1, -2, -2,
1, -2, -1, 2, -1, -4, 2, -4, -9, 6, -10,},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 1, -2, -2,
1, -2, -1, 2, -1, -4, 2, -4, -9, 6, -9,},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 1, -2, -2,
1, -2, -1, 2, -1, -4, 2, -4, -9, 6, -9,},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 1, -2, -2,
1, -2, -1, 2, -2, -3, 2, -3, -9, 5, -9,},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 0, -2, -2,
1, -1, -2, 1, -2, -3, 2, -3, -8, 5, -8,},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 0, -2, -2,
1, -1, -2, 1, -2, -2, 2, -2, -8, 5, -8,},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 0, -2, -2,
1, -1, -2, 1, -2, -2, 2, -2, -8, 5, -8,},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -1, 0, -2, -2,
1, -1, -2, 1, -1, -2, 1, -2, -7, 5, -8,},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -2, -2, 0,
-2, -2, 1, -1, -2, 1, -2, -7, 4, -8,},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -1, -2, 0,
-2, -1, 1, -1, -2, 1, -2, -7, 4, -8,},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -1, -2, 0,
-2, -1, 1, -1, -3, 1, -2, -6, 4, -8,},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -1, -1, 0,
-1, -1, 0, -2, -3, 0, -2, -5, 3, -8,},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -1, -1, 0,
-1, -1, 0, -2, -4, 0, -2, -4, 3, -8,},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -1, -1, 0,
-1, -1, 0, -2, -4, 0, -2, -4, 3, -8,},
{0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, -1, -2, 0, -1,
-1, 0, -2, -4, 0, -2, -6, 3, -7,},
{0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, -1, -2, 0, -1,
-1, 0, -2, -2, 0, -2, -6, 3, -7,},
{0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0,
-1, 0, -2, -2, 0, -3, -6, 1, -7,},
{0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0,
-1, 0, -2, -2, 0, -3, -6, 1, -7,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0,
-1, 0, -1, -2, 0, -2, -5, 1, -6,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0,
-1, 0, -1, -2, 0, -2, -5, 1, -6,},
{0, 0, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0,
-2, 0, -1, -1, 0, -1, -4, 0, -5,},
{0, 0, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0,
-2, 0, -1, -1, 0, -1, -3, 0, -5,},
{1, 0, 2, 0, 0, 0, 0, 1, 0, -1, 0, -1, 0, 0, -2,
0, 0, 0, -3, 0, -2, -5, 3, -7,},
{1, 0, 2, 1, 0, 0, 0, 1, 0, -1, 0, -1, -2, 0, -2,
0, 0, 0, -1, 0, -1, -4, 3, -7,},
{1, 0, 2, 1, 0, 0, 0, 1, 0, -1, 0, -1, -2, 0, -2,
0, 0, 0, -1, 0, -1, -4, 3, -7,},
{1, 0, 2, 1, 0, 0, 0, 1, 0, 0, 0, 0, -2, 0, -2, 0,
0, 0, -1, 0, -1, -4, 2, -6,},
{1, 0, 2, 1, 0, 0, 0, 1, 0, 0, 0, 0, -2, 0, -2, 0,
0, 0, -1, 0, -1, -3, 2, -5,},
{1, 0, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0, -2, 0, -2, 0,
0, 0, -1, 0, 0, -3, 2, -5,},
{1, 0, 2, 1, 0, 0, 0, 1, 0, 0, 0, 0, -2, 0, -2, 0,
0, 0, -1, 0, 0, -2, 2, -4,},
{1, 0, 2, 1, 0, 0, 0, 1, 0, -1, 0, -1, -2, 0, -2,
0, 0, 0, -1, 0, 0, -2, 2, -4,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,},
};

int find_cadela_table(int brightness)
{
	int loop;
	int err = -1;

	for(loop = 0; loop <= CCG6_MAX_TABLE; loop++)
		if (ccg6_candela_table[loop][0] == brightness)
			return ccg6_candela_table[loop][1];

	return err;
}

static void gamma_init_evt1(
				struct SMART_DIM *pSmart, char *str, int size)
{
	long long candela_level[S6E8FA_TABLE_MAX] = {-1, };
	int bl_index[S6E8FA_TABLE_MAX] = {-1, };

	long long temp_cal_data = 0;
	int bl_level;

	int level_255_temp_MSB = 0;
	int level_V255 = 0;

	int point_index;
	int cnt;
	int table_index;

	/*calculate candela level */
	if (pSmart->brightness_level > AOR_FIX_CD) {
		/* 300CD ~ 190CD */
		bl_level = pSmart->brightness_level;
	} else if ((pSmart->brightness_level <= AOR_FIX_CD) &&
				(pSmart->brightness_level >= AOR_ADJUST_CD)) {
		/* 180CD ~ 110CD */
		if (pSmart->brightness_level == 111)
			bl_level = 180;
		else if (pSmart->brightness_level == 119)
			bl_level = 190;
		else if (pSmart->brightness_level == 126)
			bl_level = 200;
		else if (pSmart->brightness_level == 134)
			bl_level = 211;
		else if (pSmart->brightness_level == 143)
			bl_level = 222;
		else if (pSmart->brightness_level == 152)
			bl_level = 235;
		else if (pSmart->brightness_level == 162)
			bl_level = 249;
		else
			bl_level = 263;
	} else {
		/* 100CD ~ 10CD */
		bl_level = AOR_ADJUST_CD;
	}

	if ((pSmart->brightness_level <= 300) &&
					(pSmart->brightness_level >= 195)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p85[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 183) &&
					(pSmart->brightness_level >= 111)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p8[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 105) &&
					(pSmart->brightness_level >= 87)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p85[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 82) &&
					(pSmart->brightness_level >= 68)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p8[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 64) &&
					(pSmart->brightness_level >= 44)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p75[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 41) &&
					(pSmart->brightness_level >= 24)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p7[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 22) &&
					(pSmart->brightness_level >= 13)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p65[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 12) &&
					(pSmart->brightness_level >= 10)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p6[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p2[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}
	}


#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n candela_1:%llu  candela_3:%llu  candela_11:%llu ",
		candela_level[0], candela_level[1], candela_level[2]);
	printk(KERN_INFO "candela_23:%llu  candela_35:%llu  candela_51:%llu ",
		candela_level[3], candela_level[4], candela_level[5]);
	printk(KERN_INFO "candela_87:%llu  candela_151:%llu  candela_203:%llu ",
		candela_level[6], candela_level[7], candela_level[8]);
	printk(KERN_INFO "candela_255:%llu\n", candela_level[9]);
#endif

	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
		if (searching_function(candela_level[cnt],
			&(bl_index[cnt]), GAMMA_CURVE_1P9)) {
			pr_info("%s searching functioin error cnt:%d\n",
			__func__, cnt);
		}
	}

	/*
	*	Candela compensation
	*/
	for (cnt = 1; cnt < S6E8FA_TABLE_MAX; cnt++) {
		table_index = find_cadela_table(pSmart->brightness_level);

		if (table_index == -1) {
			table_index = CCG6_MAX_TABLE;
			pr_info("%s fail candela table_index cnt : %d brightness %d",
				__func__, cnt, pSmart->brightness_level);
		}

		bl_index[S6E8FA_TABLE_MAX - cnt] +=
			gradation_offset[table_index][cnt - 1];

		/* THERE IS M-GRAY0 target */
		if (bl_index[S6E8FA_TABLE_MAX - cnt] == 0)
			bl_index[S6E8FA_TABLE_MAX - cnt] = 1;
	}

#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n bl_index_1:%d  bl_index_3:%d  bl_index_11:%d",
		bl_index[0], bl_index[1], bl_index[2]);
	printk(KERN_INFO "bl_index_23:%d bl_index_35:%d  bl_index_51:%d",
		bl_index[3], bl_index[4], bl_index[5]);
	printk(KERN_INFO "bl_index_87:%d  bl_index_151:%d bl_index_203:%d",
		bl_index[6], bl_index[7], bl_index[8]);
	printk(KERN_INFO "bl_index_255:%d\n", bl_index[9]);
#endif
	/*Generate Gamma table*/
	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++)
		(void)Make_hexa[cnt](bl_index , pSmart, str);

	/*
	*	RGB compensation
	*/
	for (cnt = 0; cnt < RGB_COMPENSATION; cnt++) {
		table_index = find_cadela_table(pSmart->brightness_level);

		if (table_index == -1) {
			table_index = CCG6_MAX_TABLE;
			pr_info("%s fail RGB table_index cnt : %d brightness %d",
				__func__, cnt, pSmart->brightness_level);
		}

		if (cnt < 3) {
			level_V255 = str[cnt * 2] << 8 | str[(cnt * 2) + 1];
			level_V255 +=
				rgb_offset[table_index][cnt];
			level_255_temp_MSB = level_V255 / 256;

			str[cnt * 2] = level_255_temp_MSB & 0xff;
			str[(cnt * 2) + 1] = level_V255 & 0xff;
		} else {
			str[cnt+3] += rgb_offset[table_index][cnt];
		}
	}
	/*subtration MTP_OFFSET value from generated gamma table*/
	mtp_offset_substraction(pSmart, str);
}

static int gradation_offset_evt1_second[][9] = {
/*	V255 V203 V151 V87 V51 V35 V23 V11 V3 */	
	{0, 1, 2, 5, 8, 10, 12, 15, 19},
	{0, 1, 2, 5, 8, 10, 11, 14, 18},
	{0, 1, 2, 4, 7, 9, 10, 13, 17},
	{0, 2, 3, 5, 8, 9, 10, 14, 18},
	{0, 2, 3, 5, 8, 9, 10, 14, 17},
	{0, 2, 3, 4, 7, 8, 9, 13, 16},
	{0, 2, 3, 4, 7, 8, 9, 13, 16},
	{0, 2, 3, 4, 7, 8, 9, 13, 15},
	{0, 1, 2, 3, 6, 7, 8, 12, 14},
	{0, 1, 2, 3, 6, 7, 8, 11, 13},
	{0, 1, 2, 2, 6, 6, 7, 10, 12},
	{0, 1, 2, 2, 6, 6, 7, 10, 12},
	{0, 1, 2, 3, 6, 7, 8, 10, 11},
	{0, 1, 2, 3, 6, 7, 8, 9, 10},
	{0, 1, 2, 3, 6, 7, 8, 9, 10},
	{0, 1, 1, 2, 5, 6, 7, 8, 9},
	{0, 1, 1, 2, 5, 6, 7, 8, 9},
	{0, 1, 1, 2, 5, 5, 6, 7, 8},
	{0, 1, 1, 2, 5, 5, 6, 7, 8},
	{0, 1, 1, 2, 5, 5, 6, 7, 8},
	{0, 1, 1, 2, 4, 4, 5, 6, 7},
	{0, 1, 1, 2, 4, 4, 5, 6, 7},
	{0, 1, 1, 2, 5, 4, 5, 6, 7},
	{0, 1, 1, 2, 6, 4, 5, 5, 6},
	{0, 1, 1, 1, 6, 4, 5, 5, 6},
	{0, 1, 1, 1, 6, 4, 4, 4, 6},
	{0, 1, 1, 1, 5, 4, 4, 4, 5},
	{0, 1, 1, 1, 5, 3, 4, 4, 5},
	{0, 1, 1, 2, 5, 4, 4, 4, 5},
	{0, 2, 1, 2, 5, 4, 4, 3, 4},
	{0, 2, 1, 2, 5, 4, 4, 3, 4},
	{0, 2, 1, 2, 5, 4, 4, 2, 4},
	{0, 2, 0, 1, 5, 3, 3, 2, 4},
	{0, 2, 2, 1, 6, 3, 3, 3, 4},
	{0, 2, 2, 1, 6, 3, 3, 3, 3},
	{0, 1, 1, 1, 6, 3, 3, 2, 3},
	{0, 1, 0, 0, 5, 2, 2, 2, 3},
	{0, 1, 2, 2, 4, 3, 2, 2, 4},
	{0, 1, 2, 2, 3, 3, 2, 2, 3},
	{0, 0, 1, 1, 3, 2, 1, 2, 2},
	{0, 0, 1, 1, 2, 2, 1, 1, 1},
	{0, 1, 1, 1, 2, 1, 1, 1, 1},
	{0, 1, 1, 1, 1, 1, 1, 1, 1},
	{0, 0, 0, 1, 1, 1, 0, 1, 1},
	{0, 0, 0, 0, 0, 0, 0, 1, 1},
	{0, 1, 1, 1, 4, 2, 1, 1, 1},
	{0, 0, 1, 1, 3, 2, 0, 0, 0},
	{0, 1, 1, 0, 2, 2, 1, 0, 1},
	{0, 1, 1, 1, 2, 1, 0, 1, 1},
	{0, 0, 0, 0, 1, 1, 1, 1, 1},
	{0, 1, 1, 1, 3, 2, 1, 1, 1},
	{0, 1, 1, 1, 2, 1, 1, 1, 1},
	{0, 1, 1, 1, 2, 1, 1, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0},
};

static int rgb_offset_evt1_second[][RGB_COMPENSATION] = {
/*	R255 G255 B255 R203 G203 B203 R151 G151 B151
	R87 G87 B87 R51 G51 B51 R35 G35 B35 
	R23 G23 B23 R11 G11 B11
*/
{-1, 0, 0, 0, 0, -2, -2, 0, -2, -5, 1, -5, -4, 3, -4, -6, 3, -5, -7, 5, -7, -9, 10, -10},
{-1, 0, 0, 0, 0, -2, -2, 0, -2, -5, 1, -5, -4, 3, -4, -6, 3, -5, -7, 5, -7, -10, 10, -11},
{-1, 0, 0, 0, 0, -2, -2, 0, -2, -4, 1, -5, -4, 3, -4, -6, 3, -5, -7, 5, -7, -11, 9, -12},
{-1, 0, 0, 0, 0, -1, -2, 0, -2, -3, 1, -4, -4, 3, -4, -5, 3, -4, -7, 4, -7, -12, 8, -14},
{-1, 0, 0, 0, 0, -1, -2, 0, -2, -3, 1, -4, -4, 3, -4, -5, 3, -4, -7, 4, -7, -12, 8, -15},
{-1, 0, 0, 0, 0, -1, -2, 0, -2, -2, 1, -3, -4, 3, -4, -5, 3, -4, -7, 4, -7, -13, 7, -16},
{-1, 0, 0, 0, 0, -1, -2, 0, -2, -2, 1, -3, -4, 3, -4, -5, 3, -4, -7, 4, -7, -13, 7, -16},
{-1, 0, 0, 0, 0, -1, -2, 0, -2, -2, 1, -3, -4, 3, -4, -5, 3, -4, -7, 4, -7, -13, 7, -17},
{0, 0, 0, 0, 0, 0, -2, 0, -2, -1, 1, -2, -3, 2, -3, -4, 2, -3, -6, 3, -6, -14, 6, -18},
{0, 0, 0, 0, 0, 0, -2, 0, -2, -1, 1, -2, -3, 2, -3, -4, 2, -3, -6, 3, -6, -14, 6, -18},
{0, 0, 1, -1, 0, -1, -2, 0, -2, -2, 1, -2, -3, 2, -3, -4, 2, -3, -6, 3, -6, -14, 7, -18},
{0, 0, 1, -1, 0, -1, -2, 0, -2, -2, 1, -2, -3, 2, -3, -4, 2, -3, -6, 3, -6, -14, 7, -18},
{0, 0, 1, -1, 0, -1, -2, 0, -2, -2, 1, -2, -3, 2, -3, -4, 2, -3, -6, 3, -6, -14, 7, -18},
{0, 0, 1, -1, 0, -1, -1, 0, -2, -2, 1, -2, -3, 2, -3, -3, 2, -2, -6, 3, -6, -13, 7, -17},
{0, 0, 1, -1, 0, -1, -1, 0, -2, -2, 1, -2, -3, 2, -3, -3, 2, -2, -6, 3, -6, -12, 7, -17},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 1, -2, -2, 1, -2, -2, 2, -1, -6, 2, -5, -11, 7, -16},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 1, -2, -2, 1, -2, -1, 2, -1, -6, 2, -5, -11, 7, -16},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 1, -2, -2, 1, -2, -2, 2, -2, -5, 2, -5, -11, 7, -15},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 1, -2, -2, 1, -2, -2, 2, -2, -5, 2, -5, -11, 7, -15},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 1, -2, -2, 1, -2, -2, 2, -2, -4, 2, -5, -11, 7, -15},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 0, -2, -2, 1, -1, -2, 1, -2, -3, 2, -4, -10, 6, -14},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 0, -2, -2, 1, -1, -2, 1, -2, -3, 2, -4, -10, 6, -14},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 0, -2, -2, 1, -2, -2, 1, -2, -4, 2, -5, -10, 6, -14},
{0, 0, 1, -1, 0, -1, 0, 0, -2, -2, 0, -2, -2, 1, -2, -2, 1, -1, -4, 2, -5, -10, 6, -13},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -1, -2, 0, -2, -1, 1, -1, -4, 1, -5, -10, 6, -12},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -1, -2, 0, -2, -2, 1, -2, -4, 1, -5, -10, 6, -12},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -1, -2, 0, -2, -2, 1, -2, -4, 1, -5, -9, 6, -11},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -1, -1, 0, -1, -2, 0, -2, -4, 0, -5, -8, 5, -9},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -1, -2, 0, -1, -2, 0, -2, -4, 0, -5, -8, 5, -9},
{0, 0, 0, -1, 0, 0, 0, 0, -1, -1, 0, -1, -2, 0, -1, -2, 0, -2, -3, 0, -4, -7, 4, -8},
{0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, -1, -2, 0, -1, -1, 0, -2, -2, 0, -3, -6, 3, -7},
{0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, -1, -2, 0, -1, -2, 0, -2, -2, 0, -3, -6, 3, -6},
{0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, -1, -2, 0, -1, -2, 0, -3, -5, 3, -4},
{0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, -1, -2, 0, -1, -2, 0, -3, -5, 2, -5},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0, -1, 0, -1, -2, 0, -2, -5, 1, -6},
{0, 0, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0, -1, 0, -1, -1, 0, -2, -3, 1, -6},
{0, 0, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0, -2, 0, -1, -1, 0, -1, -3, 0, -5},
{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -2, 0, -1, -2, 0, -3, -3, 0, -3, -8, 3, -6},
{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -2, 0, -1, -2, 0, -3, -3, 0, -3, -7, 3, -5},
{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -2, 0, -1, -1, 0, -2, -3, 0, -3, -7, 2, -5},
{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -2, 0, -1, -1, 0, -2, -3, 0, -3, -6, 2, -4},
{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0, -1, 0, -2, -3, 0, -3, -5, 3, -3},
{0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, -2, -3, 0, -3, -5, 3, -3},
{0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, -2, -3, 0, -3, -5, 3, -2},
{0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, -2, -2, 0, -2, -5, 3, -2},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

static void gamma_init_evt1_second(
				struct SMART_DIM *pSmart, char *str, int size)
{
	long long candela_level[S6E8FA_TABLE_MAX] = {-1, };
	int bl_index[S6E8FA_TABLE_MAX] = {-1, };

	long long temp_cal_data = 0;
	int bl_level;

	int level_255_temp_MSB = 0;
	int level_V255 = 0;

	int point_index;
	int cnt;
	int table_index;

	/*calculate candela level */
	if (pSmart->brightness_level > AOR_FIX_CD) {
		/* 300CD ~ 190CD */
		bl_level = pSmart->brightness_level;
	} else if ((pSmart->brightness_level <= AOR_FIX_CD) &&
				(pSmart->brightness_level >= AOR_ADJUST_CD)) {
		/* 180CD ~ 110CD */
		if (pSmart->brightness_level == 111)
			bl_level = 176;
		else if (pSmart->brightness_level == 119)
			bl_level = 187;
		else if (pSmart->brightness_level == 126)
			bl_level = 197;
		else if (pSmart->brightness_level == 134)
			bl_level = 209;
		else if (pSmart->brightness_level == 143)
			bl_level = 222;
		else if (pSmart->brightness_level == 152)
			bl_level = 234;
		else if (pSmart->brightness_level == 162)
			bl_level = 249;
		else
			bl_level = 263;
	} else {
		/* 100CD ~ 10CD */
		bl_level = AOR_ADJUST_CD;
	}

	if ((pSmart->brightness_level <= 300) &&
					(pSmart->brightness_level >= 249)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p9[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 234) &&
					(pSmart->brightness_level >= 183)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p85[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 172) &&
					(pSmart->brightness_level >= 111)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p8[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 105) &&
					(pSmart->brightness_level >= 87)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p85[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 82) &&
					(pSmart->brightness_level >= 68)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p8[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 64) &&
					(pSmart->brightness_level >= 44)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p75[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 41) &&
					(pSmart->brightness_level >= 24)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p7[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 22) &&
					(pSmart->brightness_level >= 13)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p65[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 12) &&
					(pSmart->brightness_level >= 10)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p6[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p2[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}
	}


#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n candela_1:%llu  candela_3:%llu  candela_11:%llu ",
		candela_level[0], candela_level[1], candela_level[2]);
	printk(KERN_INFO "candela_23:%llu  candela_35:%llu  candela_51:%llu ",
		candela_level[3], candela_level[4], candela_level[5]);
	printk(KERN_INFO "candela_87:%llu  candela_151:%llu  candela_203:%llu ",
		candela_level[6], candela_level[7], candela_level[8]);
	printk(KERN_INFO "candela_255:%llu\n", candela_level[9]);
#endif

	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
		if (searching_function(candela_level[cnt],
			&(bl_index[cnt]), GAMMA_CURVE_1P9)) {
			pr_info("%s searching functioin error cnt:%d\n",
			__func__, cnt);
		}
	}

	/*
	*	Candela compensation
	*/
	for (cnt = 1; cnt < S6E8FA_TABLE_MAX; cnt++) {
		table_index = find_cadela_table(pSmart->brightness_level);

		if (table_index == -1) {
			table_index = CCG6_MAX_TABLE;
			pr_info("%s fail candela table_index cnt : %d brightness %d",
				__func__, cnt, pSmart->brightness_level);
		}

		bl_index[S6E8FA_TABLE_MAX - cnt] +=
			gradation_offset_evt1_second[table_index][cnt - 1];

		/* THERE IS M-GRAY0 target */
		if (bl_index[S6E8FA_TABLE_MAX - cnt] == 0)
			bl_index[S6E8FA_TABLE_MAX - cnt] = 1;
	}

#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n bl_index_1:%d  bl_index_3:%d  bl_index_11:%d",
		bl_index[0], bl_index[1], bl_index[2]);
	printk(KERN_INFO "bl_index_23:%d bl_index_35:%d  bl_index_51:%d",
		bl_index[3], bl_index[4], bl_index[5]);
	printk(KERN_INFO "bl_index_87:%d  bl_index_151:%d bl_index_203:%d",
		bl_index[6], bl_index[7], bl_index[8]);
	printk(KERN_INFO "bl_index_255:%d\n", bl_index[9]);
#endif
	/*Generate Gamma table*/
	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++)
		(void)Make_hexa[cnt](bl_index , pSmart, str);

	/*
	*	RGB compensation
	*/
	for (cnt = 0; cnt < RGB_COMPENSATION; cnt++) {
		table_index = find_cadela_table(pSmart->brightness_level);

		if (table_index == -1) {
			table_index = CCG6_MAX_TABLE;
			pr_info("%s fail RGB table_index cnt : %d brightness %d",
				__func__, cnt, pSmart->brightness_level);
		}

		if (cnt < 3) {
			level_V255 = str[cnt * 2] << 8 | str[(cnt * 2) + 1];
			level_V255 +=
				rgb_offset_evt1_second[table_index][cnt];
			level_255_temp_MSB = level_V255 / 256;

			str[cnt * 2] = level_255_temp_MSB & 0xff;
			str[(cnt * 2) + 1] = level_V255 & 0xff;
		} else {
			str[cnt+3] += rgb_offset_evt1_second[table_index][cnt];
		}
	}
	/*subtration MTP_OFFSET value from generated gamma table*/
	mtp_offset_substraction(pSmart, str);
}


static int gradation_offset_evt1_third[][9] = {
/*	V255 V203 V151 V87 V51 V35 V23 V11 V3 */	
	{0, 2, 3, 9, 10, 13, 15, 19, 21,},
	{0, 2, 3, 9, 10, 12, 14, 18, 20,},
	{0, 2, 3, 8, 10, 11, 13, 17, 19,},
	{0, 2, 3, 8, 10, 11, 13, 16, 18,},
	{0, 2, 3, 7, 9, 10, 12, 15, 17,},
	{0, 2, 3, 7, 9, 10, 12, 15, 17,},
	{0, 2, 3, 7, 9, 9, 12, 14, 16,},
	{0, 2, 3, 7, 9, 9, 12, 14, 16,},
	{0, 1, 3, 6, 8, 8, 11, 13, 15,},
	{0, 1, 3, 6, 8, 8, 11, 13, 14,},
	{0, 1, 3, 6, 7, 8, 10, 12, 13,},
	{0, 1, 3, 5, 7, 7, 10, 12, 13,},
	{0, 1, 3, 5, 6, 7, 9, 11, 12,},
	{0, 1, 3, 5, 6, 7, 9, 11, 12,},
	{0, 1, 2, 4, 5, 6, 8, 10, 11,},
	{0, 1, 2, 4, 5, 6, 8, 10, 11,},
	{0, 1, 2, 4, 5, 6, 8, 10, 11,},
	{0, 1, 2, 4, 4, 6, 7, 9, 10,},
	{0, 1, 2, 3, 4, 5, 6, 8, 10,},
	{0, 1, 1, 3, 3, 5, 6, 8, 9,},
	{0, 1, 1, 2, 3, 4, 5, 7, 8,},
	{0, 1, 1, 2, 3, 4, 5, 6, 7,},
	{0, 1, 1, 2, 3, 3, 5, 5, 6,},
	{0, 1, 1, 1, 2, 3, 4, 5, 6,},
	{0, 1, 1, 1, 2, 2, 4, 5, 6,},
	{0, 1, 1, 1, 2, 2, 4, 4, 5,},
	{0, 0, 1, 1, 2, 2, 3, 3, 4,},
	{0, 0, 1, 1, 2, 2, 3, 3, 4,},
	{0, 0, 1, 1, 2, 2, 2, 3, 4,},
	{0, 0, 0, 1, 1, 1, 1, 2, 3,},
	{0, 0, 0, 1, 1, 1, 1, 2, 3,},
	{0, 0, 0, 1, 1, 1, 1, 2, 3,},
	{0, 0, 0, 1, 1, 1, 1, 2, 3,},
	{0, 0, 0, 1, 1, 1, 1, 1, 2,},
	{0, 0, 0, 1, 1, 1, 1, 1, 2,},
	{0, 0, 0, 0, 0, 0, 1, 1, 2,},
	{0, 0, 0, 0, 0, 0, 0, 1, 2,},
	{0, 1, 1, 1, 1, 1, 1, 1, 1,},
	{0, 1, 1, 1, 1, 1, 1, 1, 0,},
	{0, 1, 1, 1, 1, 1, 1, 0, 0,},
	{0, 1, 1, 1, 1, 0, 0, 0, 0,},
	{0, 1, 1, 1, 1, 0, 0, 0, 0,},
	{0, 1, 1, 0, 0, 0, 0, 0, 0,},
	{0, 0, 0, 0, 0, 0, 0, -1, 0,},
	{0, 0, 0, 0, 0, 0, 0, -1, 0,},
	{0, 1, 1, 1, 1, 1, 1, 0, 0,},
	{0, 0, 0, 0, 0, 0, 0, -1, 0,},
	{0, 0, 0, 0, 0, 0, -1, -1, 0,},
	{0, 0, 0, 0, 0, 0, 0, -1, 0,},
	{0, 0, 0, 0, 0, 0, 0, -2, 0,},
	{0, 0, 0, 0, 0, 0, 0, -2, 0,},
	{0, 0, 0, 0, 0, 0, 0, -2, 0,},
	{0, 0, 0, 0, 0, 0, 0, -1, 0,},
	{0, 0, 0, 0, 0, 0, 0, 0, 0,},
};

static int rgb_offset_evt1_third[][RGB_COMPENSATION] = {
/*	R255 G255 B255 R203 G203 B203 R151 G151 B151
	R87 G87 B87 R51 G51 B51 R35 G35 B35 
	R23 G23 B23 R11 G11 B11
*/
{0, 0, 1, -1, 1, -1, 0, 2, -2, -4, 2, -5, -6, 4, -6, -7, 4, -7, -8, 5, -7, -10, 7, -10,},
{0, 0, 1, -1, 1, -1, 0, 2, -2, -4, 2, -5, -6, 4, -6, -7, 4, -7, -8, 5, -7, -10, 7, -10,},
{0, 0, 1, -1, 1, -1, 0, 2, -2, -4, 2, -5, -6, 3, -6, -7, 3, -7, -8, 5, -7, -10, 6, -11,},
{0, 0, 1, -1, 1, -1, 0, 2, -2, -4, 2, -5, -6, 3, -6, -7, 3, -7, -8, 5, -7, -10, 6, -11,},
{0, 0, 1, -1, 1, -1, 0, 2, -2, -2, 2, -4, -5, 3, -6, -7, 3, -7, -8, 5, -7, -10, 6, -12,},
{0, 0, 1, -1, 1, -1, 0, 2, -2, -3, 2, -4, -5, 3, -6, -7, 3, -7, -8, 5, -7, -10, 6, -12,},
{0, 0, 1, -1, 1, -1, 0, 2, -2, -3, 2, -4, -5, 3, -6, -7, 3, -6, -7, 4, -7, -10, 6, -13,},
{0, 0, 1, -1, 1, -1, 0, 2, -2, -3, 2, -3, -5, 3, -6, -6, 3, -6, -7, 4, -6, -10, 6, -13,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 2, -3, -4, 2, -6, -6, 2, -6, -7, 4, -6, -9, 5, -14,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 2, -3, -4, 2, -6, -6, 2, -6, -7, 4, -6, -9, 5, -14,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 2, -3, -4, 2, -6, -6, 2, -6, -6, 4, -6, -9, 5, -14,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 2, -3, -4, 2, -6, -6, 2, -6, -6, 3, -6, -9, 5, -14,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 2, -3, -4, 1, -5, -6, 2, -5, -5, 3, -6, -10, 5, -14,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 2, -3, -4, 1, -5, -6, 2, -5, -5, 2, -6, -10, 5, -14,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 2, -3, -4, 1, -5, -6, 2, -5, -5, 2, -6, -10, 5, -14,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 2, -3, -4, 1, -5, -6, 2, -5, -5, 2, -6, -10, 4, -14,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 2, -3, -4, 1, -5, -6, 2, -5, -5, 2, -6, -10, 4, -14,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 2, -3, -4, 1, -5, -6, 2, -5, -5, 2, -6, -10, 4, -14,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 2, -3, -3, 1, -4, -5, 2, -4, -5, 2, -5, -10, 4, -14,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 1, -2, -3, 1, -4, -5, 1, -4, -5, 2, -5, -10, 4, -15,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 1, -2, -2, 1, -3, -4, 1, -3, -5, 2, -4, -10, 4, -15,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 1, -2, -2, 1, -3, -3, 1, -3, -5, 2, -4, -10, 4, -15,},
{0, 0, 1, 0, 1, -1, 0, 1, -1, -2, 1, -2, -2, 1, -3, -3, 1, -2, -4, 1, -4, -11, 3, -15,},
{0, 0, 1, 0, 0, -1, 0, 1, -1, -2, 1, -2, -2, 1, -2, -2, 1, -2, -4, 1, -3, -11, 2, -15,},
{0, 0, 1, 0, 0, -1, 0, 1, -1, -2, 1, -2, -2, 1, -2, -2, 1, -2, -3, 1, -3, -11, 2, -15,},
{0, 0, 1, 0, 0, -1, 0, 1, -1, -2, 1, -2, -2, 1, -2, -2, 1, -2, -3, 1, -3, -11, 2, -15,},
{0, 0, 1, 0, 1, -1, 0, 1, 0, -1, 1, -1, -1, 1, -2, -1, 1, -2, -3, 1, -2, -11, 2, -15,},
{0, 0, 1, 0, 1, -1, 0, 1, 0, -1, 1, -1, -1, 1, -2, -1, 1, -2, -3, 1, -2, -11, 2, -15,},
{0, 0, 1, 0, 1, -1, 0, 1, 0, -1, 1, -1, -1, 1, -1, -1, 1, -2, -2, 1, -2, -10, 2, -14,},
{0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, -1, 1, -1, -1, 1, -1, -9, 2, -13,},
{0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, -1, 1, -1, -1, 1, -1, -7, 2, -12,},
{0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, -1, 1, -1, -1, 1, -1, -6, 2, -12,},
{0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, -1, 1, -1, -1, 1, -1, -6, 2, -11,},
{0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, -1, 1, 0, -5, 2, -10,},
{0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, -1, 1, 0, -4, 2, -9,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0, -3, 1, -7,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0, -2, 1, -6,},
{0, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -1, 0, -1, -1, 1, -1, -1, 2, -1, -6, 2, -5,},
{0, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -2, 0, -2, -1, 1, -1, -1, 2, -1, -9, 1, -7,},
{0, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -2, 0, -2, -1, 1, -1, -1, 2, -1, -8, 1, -6,},
{0, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -1, 0, -1, -2, 1, -2, -1, 2, -1, -7, 2, -4,},
{0, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -2, 0, -2, 0, 1, -1, -1, 1, -1, -8, 2, -4,},
{0, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -1, 0, -1, -1, 1, -1, -1, 1, -1, -4, 2, -3,},
{0, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -2, 0, -1, -1, 1, -1, -1, 1, -1, -4, 2, -3,},
{0, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -1, 0, -1, -1, 1, -1, -1, 1, -1, -4, 2, -3,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
};

static void gamma_init_evt1_third(
				struct SMART_DIM *pSmart, char *str, int size)
{
	long long candela_level[S6E8FA_TABLE_MAX] = {-1, };
	int bl_index[S6E8FA_TABLE_MAX] = {-1, };

	long long temp_cal_data = 0;
	int bl_level;

	int level_255_temp_MSB = 0;
	int level_V255 = 0;

	int point_index;
	int cnt;
	int table_index;

	/*calculate candela level */
	if (pSmart->brightness_level > AOR_FIX_CD) {
		/* 300CD ~ 190CD */
		bl_level = pSmart->brightness_level;
	} else if ((pSmart->brightness_level <= AOR_FIX_CD) &&
				(pSmart->brightness_level >= AOR_ADJUST_CD)) {
		/* 180CD ~ 110CD */
		if (pSmart->brightness_level == 111)
			bl_level = 181;
		else if (pSmart->brightness_level == 119)
			bl_level = 192;
		else if (pSmart->brightness_level == 126)
			bl_level = 202;
		else if (pSmart->brightness_level == 134)
			bl_level = 213;
		else if (pSmart->brightness_level == 143)
			bl_level = 226;
		else if (pSmart->brightness_level == 152)
			bl_level = 239;
		else if (pSmart->brightness_level == 162)
			bl_level = 253;
		else
			bl_level = 267;
	} else {
		/* 100CD ~ 10CD */
		bl_level = AOR_ADJUST_CD;
	}

	if ((pSmart->brightness_level <= 300) &&
					(pSmart->brightness_level >= 183)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p15[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 172) &&
					(pSmart->brightness_level >= 111)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p1[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 105) &&
					(pSmart->brightness_level >= 77)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p15[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 72) &&
					(pSmart->brightness_level >= 68)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p1[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 64) &&
					(pSmart->brightness_level >= 56)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p05[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 53) &&
					(pSmart->brightness_level >= 14)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p0[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else if ((pSmart->brightness_level <= 13) &&
					(pSmart->brightness_level >= 10)) {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_1p95[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}

	} else {
		for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p2[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
		}
	}


#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n candela_1:%llu  candela_3:%llu  candela_11:%llu ",
		candela_level[0], candela_level[1], candela_level[2]);
	printk(KERN_INFO "candela_23:%llu  candela_35:%llu  candela_51:%llu ",
		candela_level[3], candela_level[4], candela_level[5]);
	printk(KERN_INFO "candela_87:%llu  candela_151:%llu  candela_203:%llu ",
		candela_level[6], candela_level[7], candela_level[8]);
	printk(KERN_INFO "candela_255:%llu brightness_level %d\n", candela_level[9], pSmart->brightness_level);
#endif

	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
		if (searching_function(candela_level[cnt],
			&(bl_index[cnt]), GAMMA_CURVE_2P2)) {
			pr_info("%s searching functioin error cnt:%d\n",
			__func__, cnt);
		}
	}

	/*
	*	Candela compensation
	*/
	for (cnt = 1; cnt < S6E8FA_TABLE_MAX; cnt++) {
		table_index = find_cadela_table(pSmart->brightness_level);

		if (table_index == -1) {
			table_index = CCG6_MAX_TABLE;
			pr_info("%s fail candela table_index cnt : %d brightness %d",
				__func__, cnt, pSmart->brightness_level);
		}

		bl_index[S6E8FA_TABLE_MAX - cnt] +=
			gradation_offset_evt1_third[table_index][cnt - 1];

		/* THERE IS M-GRAY0 target */
		if (bl_index[S6E8FA_TABLE_MAX - cnt] == 0)
			bl_index[S6E8FA_TABLE_MAX - cnt] = 1;
	}

#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n bl_index_1:%d  bl_index_3:%d  bl_index_11:%d",
		bl_index[0], bl_index[1], bl_index[2]);
	printk(KERN_INFO "bl_index_23:%d bl_index_35:%d  bl_index_51:%d",
		bl_index[3], bl_index[4], bl_index[5]);
	printk(KERN_INFO "bl_index_87:%d  bl_index_151:%d bl_index_203:%d",
		bl_index[6], bl_index[7], bl_index[8]);
	printk(KERN_INFO "bl_index_255:%d\n", bl_index[9]);
#endif
	/*Generate Gamma table*/
	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++)
		(void)Make_hexa[cnt](bl_index , pSmart, str);

	/*
	*	RGB compensation
	*/
	for (cnt = 0; cnt < RGB_COMPENSATION; cnt++) {
		table_index = find_cadela_table(pSmart->brightness_level);

		if (table_index == -1) {
			table_index = CCG6_MAX_TABLE;
			pr_info("%s fail RGB table_index cnt : %d brightness %d",
				__func__, cnt, pSmart->brightness_level);
		}

		if (cnt < 3) {
			level_V255 = str[cnt * 2] << 8 | str[(cnt * 2) + 1];
			level_V255 +=
				rgb_offset_evt1_third[table_index][cnt];
			level_255_temp_MSB = level_V255 / 256;

			str[cnt * 2] = level_255_temp_MSB & 0xff;
			str[(cnt * 2) + 1] = level_V255 & 0xff;
		} else {
			str[cnt+3] += rgb_offset_evt1_third[table_index][cnt];
		}
	}
	/*subtration MTP_OFFSET value from generated gamma table*/
	mtp_offset_substraction(pSmart, str);
}

#endif

static void pure_gamma_init(struct SMART_DIM *pSmart, char *str, int size)
{
	long long candela_level[S6E8FA_TABLE_MAX] = {-1, };
	int bl_index[S6E8FA_TABLE_MAX] = {-1, };

	long long temp_cal_data = 0;
	int bl_level, cnt;
	int point_index;

	bl_level = pSmart->brightness_level;

	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			point_index = S6E8FA_ARRAY[cnt+1];
			temp_cal_data =
			((long long)(candela_coeff_2p2[point_index])) *
			((long long)(bl_level));
			candela_level[cnt] = temp_cal_data;
	}

#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n candela_1:%llu  candela_3:%llu  candela_11:%llu ",
		candela_level[0], candela_level[1], candela_level[2]);
	printk(KERN_INFO "candela_23:%llu  candela_35:%llu  candela_51:%llu ",
		candela_level[3], candela_level[4], candela_level[5]);
	printk(KERN_INFO "candela_87:%llu  candela_151:%llu  candela_203:%llu ",
		candela_level[6], candela_level[7], candela_level[8]);
	printk(KERN_INFO "candela_255:%llu\n", candela_level[9]);
#endif

	/*calculate brightness level*/
	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++) {
			if (searching_function(candela_level[cnt],
				&(bl_index[cnt]), GAMMA_CURVE_2P2)) {
				pr_info("%s searching functioin error cnt:%d\n",
					__func__, cnt);
			}
	}

	/*
	*	210CD ~ 190CD compensation
	*	V3 level + 1
	*/
	if ((bl_level >= 190) && (bl_level <= 210))
		bl_index[1] += 1;

#ifdef SMART_DIMMING_DEBUG
	printk(KERN_INFO "\n bl_index_1:%d  bl_index_3:%d  bl_index_11:%d",
	bl_index[0], bl_index[1], bl_index[2]);
	printk(KERN_INFO "bl_index_23:%d bl_index_35:%d  bl_index_51:%d",
		bl_index[3], bl_index[4], bl_index[5]);
	printk(KERN_INFO "bl_index_87:%d  bl_index_151:%d bl_index_203:%d",
		bl_index[5], bl_index[7], bl_index[8]);
	printk(KERN_INFO "bl_index_255:%d\n", bl_index[9]);
#endif

	/*Generate Gamma table*/
	for (cnt = 0; cnt < S6E8FA_TABLE_MAX; cnt++)
		(void)Make_hexa[cnt](bl_index , pSmart, str);

	mtp_offset_substraction(pSmart, str);
}


static void set_max_lux_table(void)
{
	max_lux_table[0] = V255_300CD_R_MSB;
	max_lux_table[1] = V255_300CD_R_LSB;

	max_lux_table[2] = V255_300CD_G_MSB;
	max_lux_table[3] = V255_300CD_G_LSB;

	max_lux_table[4] = V255_300CD_B_MSB;
	max_lux_table[5] = V255_300CD_B_LSB;

	max_lux_table[6] = V203_300CD_R;
	max_lux_table[7] = V203_300CD_G;
	max_lux_table[8] = V203_300CD_B;

	max_lux_table[9] = V151_300CD_R;
	max_lux_table[10] = V151_300CD_G;
	max_lux_table[11] = V151_300CD_B;

	max_lux_table[12] = V87_300CD_R;
	max_lux_table[13] = V87_300CD_G;
	max_lux_table[14] = V87_300CD_B;

	max_lux_table[15] = V51_300CD_R;
	max_lux_table[16] = V51_300CD_G;
	max_lux_table[17] = V51_300CD_B;

	max_lux_table[18] = V35_300CD_R;
	max_lux_table[19] = V35_300CD_G;
	max_lux_table[20] = V35_300CD_B;

	max_lux_table[21] = V23_300CD_R;
	max_lux_table[22] = V23_300CD_G;
	max_lux_table[23] = V23_300CD_B;

	max_lux_table[24] = V11_300CD_R;
	max_lux_table[25] = V11_300CD_G;
	max_lux_table[26] = V11_300CD_B;

	max_lux_table[27] = V3_300CD_R;
	max_lux_table[28] = V3_300CD_G;
	max_lux_table[29] = V3_300CD_B;

	max_lux_table[30] = VT_300CD_R;
	max_lux_table[31] = VT_300CD_G;
	max_lux_table[32] = VT_300CD_B;

}


static void set_min_lux_table(struct SMART_DIM *psmart)
{
	psmart->brightness_level = MIN_CANDELA;
	pure_gamma_init(psmart, min_lux_table, GAMMA_SET_MAX);
}

void get_min_lux_table(char *str, int size)
{
	memcpy(str, min_lux_table, size);
}

void generate_gamma(struct SMART_DIM *psmart, char *str, int size)
{
	int lux_loop;
	struct illuminance_table *ptable = (struct illuminance_table *)
						(&(psmart->gen_table));

	/* searching already generated gamma table */
	for (lux_loop = 0; lux_loop < psmart->lux_table_max; lux_loop++) {
		if (ptable[lux_loop].lux == psmart->brightness_level) {
			memcpy(str, &(ptable[lux_loop].gamma_setting), size);
			break;
		}
	}

	/* searching fail... Setting 300CD value on gamma table */
	if (lux_loop == psmart->lux_table_max) {
		pr_info("%s searching fail lux : %d\n", __func__,
				psmart->brightness_level);
		memcpy(str, max_lux_table, size);
	}

#ifdef SMART_DIMMING_DEBUG
	if (lux_loop != psmart->lux_table_max)
		pr_info("%s searching ok index : %d lux : %d", __func__,
			lux_loop, ptable[lux_loop].lux);
#endif
}
static void gamma_cell_determine(int ldi_revision)
{
	pr_info("%s ldi_revision: 0x%x", __func__, ldi_revision);

	V255_300CD_R_MSB = V255_300CD_R_MSB_20;
	V255_300CD_R_LSB = V255_300CD_R_LSB_20;

	V255_300CD_G_MSB = V255_300CD_G_MSB_20;
	V255_300CD_G_LSB = V255_300CD_G_LSB_20;

	V255_300CD_B_MSB = V255_300CD_B_MSB_20;
	V255_300CD_B_LSB = V255_300CD_B_LSB_20;

	V203_300CD_R = V203_300CD_R_20;
	V203_300CD_G = V203_300CD_G_20;
	V203_300CD_B = V203_300CD_B_20;

	V151_300CD_R = V151_300CD_R_20;
	V151_300CD_G = V151_300CD_G_20;
	V151_300CD_B = V151_300CD_B_20;

	V87_300CD_R = V87_300CD_R_20;
	V87_300CD_G = V87_300CD_G_20;
	V87_300CD_B = V87_300CD_B_20;

	V51_300CD_R = V51_300CD_R_20;
	V51_300CD_G = V51_300CD_G_20;
	V51_300CD_B = V51_300CD_B_20;

	V35_300CD_R = V35_300CD_R_20;
	V35_300CD_G = V35_300CD_G_20;
	V35_300CD_B = V35_300CD_B_20;

	V23_300CD_R = V23_300CD_R_20;
	V23_300CD_G = V23_300CD_G_20;
	V23_300CD_B = V23_300CD_B_20;

	V11_300CD_R = V11_300CD_R_20;
	V11_300CD_G = V11_300CD_G_20;
	V11_300CD_B = V11_300CD_B_20;

	V3_300CD_R = V3_300CD_R_20;
	V3_300CD_G = V3_300CD_G_20;
	V3_300CD_B = V3_300CD_B_20;

	VT_300CD_R = VT_300CD_R_20;
	VT_300CD_G = VT_300CD_G_20;
	VT_300CD_B = VT_300CD_B_20;
}

static void mtp_sorting(struct SMART_DIM *psmart)
{
	int sorting[GAMMA_SET_MAX] = {
		0, 1, 6, 9, 12, 15, 18, 21, 24, 27, 30, /* R*/
		2, 3, 7, 10, 13, 16, 19, 22, 25, 28, 31, /* G */
		4, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, /* B */
	};
	int loop;
	char *pfrom, *pdest;

	pfrom = (char *)&(psmart->MTP_ORIGN);
	pdest = (char *)&(psmart->MTP);

	for (loop = 0; loop < GAMMA_SET_MAX; loop++)
		pdest[loop] = pfrom[sorting[loop]];

	if (get_ldi_chip() == LDI_MAGNA) {
		pdest[10] = 3;
		pdest[21] = 2;
		pdest[32] = 2;
	}
}

int smart_dimming_init(struct SMART_DIM *psmart)
{
	int lux_loop;
	int id1, id2, id3;
#ifdef SMART_DIMMING_DEBUG
	int cnt;
	char pBuffer[256];
	memset(pBuffer, 0x00, 256);
#endif
	id1 = (psmart->ldi_revision & 0x00FF0000) >> 16;
	id2 = (psmart->ldi_revision & 0x0000FF00) >> 8;
	id3 = psmart->ldi_revision & 0xFF;

	mtp_sorting(psmart);
	gamma_cell_determine(psmart->ldi_revision);
	set_max_lux_table();

	v255_adjustment(psmart);
	vt_adjustment(psmart);
	v203_adjustment(psmart);
	v151_adjustment(psmart);
	v87_adjustment(psmart);
	v51_adjustment(psmart);
	v35_adjustment(psmart);
	v23_adjustment(psmart);
	v11_adjustment(psmart);
	v3_adjustment(psmart);


	if (generate_gray_scale(psmart)) {
		pr_info(KERN_ERR "lcd smart dimming fail generate_gray_scale\n");
		return -EINVAL;
	}

	/*Generating lux_table*/
	for (lux_loop = 0; lux_loop < psmart->lux_table_max; lux_loop++) {
		/* To set brightness value */
		psmart->brightness_level = psmart->plux_table[lux_loop];
		/* To make lux table index*/
		psmart->gen_table[lux_loop].lux = psmart->plux_table[lux_loop];

#if defined(AID_OPERATION)
		if (get_ldi_chip() == LDI_LSI) {
			if (id3 == EVT1_THIRD_ID || id3 == EVT1_FOUTRH_ID ||
				id3 == EVT1_REV_H_ID3_1 || id3 == EVT1_REV_H_ID3_2 ||
				id3 == EVT1_REV_I_ID3_1 || id3 == EVT1_REV_I_ID3_2)
				gamma_init_evt1_third(psmart,
				(char *)(&(psmart->gen_table[lux_loop].gamma_setting)),
				GAMMA_SET_MAX);
			else if (id3 == EVT1_FIRST_ID || id3 == EVT1_SECOND_ID)
				gamma_init_evt1_second(psmart,
				(char *)(&(psmart->gen_table[lux_loop].gamma_setting)),
				GAMMA_SET_MAX);
			else if ((id3 == CCG6_ID) || (id3 == EVT1_ID))
				gamma_init_evt1(psmart,
				(char *)(&(psmart->gen_table[lux_loop].gamma_setting)),
				GAMMA_SET_MAX);
			else if (id3 == VT232_ID)
				gamma_init_vt232(psmart,
				(char *)(&(psmart->gen_table[lux_loop].gamma_setting)),
				GAMMA_SET_MAX);
			else
				gamma_init_vt888(psmart,
				(char *)(&(psmart->gen_table[lux_loop].gamma_setting)),
				GAMMA_SET_MAX);
		} else { 
			if (id3 == MAGNA_C_FIRST_ID || id3 == MAGNA_C_SECOND_ID)
				gamma_init_evt1_third(psmart,
				(char *)(&(psmart->gen_table[lux_loop].gamma_setting)),
				GAMMA_SET_MAX);
			else
				gamma_init_vt888(psmart,
				(char *)(&(psmart->gen_table[lux_loop].gamma_setting)),
				GAMMA_SET_MAX);

		}
#else
		pure_gamma_init(psmart,
			(char *)(&(psmart->gen_table[lux_loop].gamma_setting)),
			GAMMA_SET_MAX);
#endif
	}

	/* set 300CD max gamma table */
	memcpy(&(psmart->gen_table[lux_loop-1].gamma_setting),
			max_lux_table, GAMMA_SET_MAX);

	set_min_lux_table(psmart);

#ifdef SMART_DIMMING_DEBUG
	for (lux_loop = 0; lux_loop < psmart->lux_table_max; lux_loop++) {
		for (cnt = 0; cnt < GAMMA_SET_MAX; cnt++)
			snprintf(pBuffer + strnlen(pBuffer, 256), 256, " %d",
				psmart->gen_table[lux_loop].gamma_setting[cnt]);

		pr_info("lux : %d  %s", psmart->plux_table[lux_loop], pBuffer);
		memset(pBuffer, 0x00, 256);
	}

	for (lux_loop = 0; lux_loop < psmart->lux_table_max; lux_loop++) {
		for (cnt = 0; cnt < GAMMA_SET_MAX; cnt++)
			snprintf(pBuffer + strnlen(pBuffer, 256), 256,
				" %02X",
				psmart->gen_table[lux_loop].gamma_setting[cnt]);

		pr_info("lux : %d  %s", psmart->plux_table[lux_loop], pBuffer);
		memset(pBuffer, 0x00, 256);
	}
#endif
	return 0;
}


