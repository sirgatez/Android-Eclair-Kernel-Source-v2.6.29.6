/*
 * linux/drivers/power/s3c6410_battery.h
 *
 * Battery measurement code for S3C6410 platform.
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define DRIVER_NAME	"sec-battery"

/*
 * Spica Rev00 board Battery Table
 */
#define BATT_CAL		2447	/* 3.60V */

#define BATT_MAXIMUM		406	/* 4.176V */
#define BATT_FULL		353	/* 4.10V  */
#define BATT_SAFE_RECHARGE 353	/* 4.10V */
#define BATT_ALMOST_FULL	188 /* 3.8641V */	//322	/* 4.066V */
#define BATT_HIGH		112 /* 3.7554V */ 		//221	/* 3.919V */
#define BATT_MED		66 /* 3.6907V */ 		//146	/* 3.811V */
#define BATT_LOW		43 /* 3.6566V */		//112	/* 3.763V */
#define BATT_CRITICAL		8 /* 3.6037V */ 	//(74)	/* 3.707V */
#define BATT_MINIMUM		(-28) /* 3.554V */	//(38)	/* 3.655V */
#define BATT_OFF		(-128) /* 3.4029V */	//(-103)	/* 3.45V  */

/*
 * Aries Rev02 board Temperature Table
 */

const int temper_table[][2] =  {
	{ 151, 	-70 },
	{ 153, 	-60 },
	{ 154, 	-50 },
	{ 156, 	-40 },
	{ 157, 	-30 },
	{ 159, 	-20 },
	{ 160, 	-10 },
	{ 162, 	0 },
	{ 163, 	10 },
	{ 165, 	20 },
	{ 166, 	30 },
	{ 168, 	40 },
	{ 169, 	50 },
	{ 171, 	60 },
	{ 172, 	70 },
	{ 174, 	80 },
	{ 175, 	90 },
	{ 177, 	100 },
	{ 178, 	110 },
	{ 180, 	120 },
	{ 181, 	130 },
	{ 183, 	140 },
	{ 184, 	150 },
	{ 186, 	160 },
	{ 187, 	170 },
	{ 188, 	180 },	
	{ 190, 	190 },
	{ 191, 	200 },
	{ 193, 	210 },
	{ 194, 	220 },
	{ 196, 	230 },
	{ 197, 	240 },
	{ 199, 	250 },
	{ 200, 	260 },
	{ 202, 	270 },
	{ 203, 	280 },
	{ 205, 	290 },
	{ 206, 	300 },
	{ 207, 	310 },
	{ 209, 	320 },
	{ 210, 	330 },
	{ 212, 	340 },
	{ 214, 	350 },
	{ 215, 	360 },
	{ 216, 	370 },
	{ 218, 	380 },
	{ 219, 	390 },
	{ 221, 	400 },
	{ 222, 	410 },
	{ 224, 	420 },
	{ 225, 	430 },
	{ 227, 	440 },
	{ 229, 	450 },
	{ 230, 	460 },
	{ 232, 	470 },
	{ 233,	480 },
	{ 235,	490 },
	{ 236,	500 },
	{ 238,	510 },
	{ 239,	520 },
	{ 241,	530 },
	{ 242,	540 },
	{ 244,	550 },
	{ 245,	560 },
	{ 247,	570 },
	{ 248,	580 },
	{ 250,	590 },
	{ 251,	600 },
	{ 253,	610 },
	{ 254,	620 },
	{ 256,	630 },
	{ 257,	640 },
	{ 259,	650 },
	{ 260,	660 },
	{ 262,	670 },
};

#define TEMP_HIGH_BLOCK		234	// (Fix: 20100630) 225
#define TEMP_HIGH_RECOVER	224	// (Fix: 20100630) 219
#define TEMP_LOW_BLOCK		156	// (Fix: 20100630) 158
#define TEMP_LOW_RECOVER	161

#define TEMP_HIGH_BLOCK_LPM			223
#define TEMP_HIGH_RECOVER_LPM		217
#define TEMP_LOW_BLOCK_LPM			155	// 152
#define TEMP_LOW_RECOVER_LPM		161	// 158


/*
 * Aries Rev02 board ADC channel
 */
typedef enum s3c_adc_channel {
	// hanapark_Atlas
	S3C_ADC_VOLTAGE = 1,
	S3C_ADC_CHG_CURRENT = 2,
	S3C_ADC_TEMPERATURE = 6,
	ENDOFADC
} adc_channel_type;


/******************************************************************************
 * Battery driver features
 * ***************************************************************************/

#define __VZW_AUTH_CHECK__
#define __CHECK_BATTERY_V_F__
#define __ADJUST_RECHARGE_ADC__
#define __TEST_MODE_INTERFACE__
#define __FUEL_GAUGES_IC__ 
#define __CHECK_CHG_CURRENT__
#define __ADJUST_ADC_VALUE__
#define __TEMP_BLOCK_EXCEPTION__

//#define __TEMPERATURE_TEST__
//#define __SOC_TEST__
//#define __FULL_CHARGE_TEST__
//#define __MANUAL_TEMP_TEST__

/*****************************************************************************/

#ifdef __FULL_CHARGE_TEST__
#define TOTAL_CHARGING_TIME	(1*60*1000)	/* 1 min */
#else
#define TOTAL_CHARGING_TIME	(5*60*60*1000)	/* 5 hours */
#endif
#define TOTAL_RECHARGING_TIME	(2*60*60*1000)	/* 2 hours */

#ifdef __ADJUST_RECHARGE_ADC__
#define BATT_RECHARGE_CODE	0	// hanapark (fix compile error)
#define BATT_RECHARGE_COUNT	20
#endif

#ifdef __FUEL_GAUGES_IC__
#define FULL_CHARGE_COND_VOLTAGE	4000
#define RECHARGE_COND_VOLTAGE		4120	// hanapark_Atlas (Fix: 2010.05.18)
#define RECHARGE_COND_VOLTAGE_BACKUP		4000	// hanapark_DH17

#define LOW_BATT_COUNT	30
#define LOW_BATT_COND_VOLTAGE		3400
#define LOW_BATT_COND_LEVEL			0
#endif /* __FUEL_GAUGES_IC__ */

#define BATT_VF_MIN	0	//12	// hanapark_Victory
#define BATT_VF_MAX	50	//30	// hanapark_Victory

#ifdef __CHECK_CHG_CURRENT__
#define CURRENT_OF_FULL_CHG  70	// hanapark_Atlas (Fix: 2010.05.26)
#define CHG_CURRENT_COUNT		20
#endif

