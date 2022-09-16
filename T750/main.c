/* =========================================================================
 * Project:       PWM1~PWM4 output at the same time
 * File:          main.c
 * Description:
 *    PWM4(PA3/PA7)	: Period = 4.88KHz (Duty: 512/1024)
 *    PWM3(PA2) 	: Period = 4.88KHz (Duty: 768/1024)
 *    PWM2(PA4/PB2) : Period = 4.88KHz (Duty: 1023/1024)
 *    PWM1(PB3)		: Period = 4.88KHz (Duty: 1/1024)
 * Author:        Ranley
 * Version:       v1.0
 * Date:          2021/03/25
 =========================================================================*/
#include <ny8.h>
#include "ny8_constant.h"
// 宏定义
#define C_PWM_LB_DUTY_00H 0x00
#define C_PWM_LB_DUTY_40H 0x40
#define C_PWM_LB_DUTY_01H 0x01
#define C_PWM_LB_DUTY_FFH 0xFF
// *    PWM3(PA2) 	: Period = 3.91KHz (Duty: 768/1024)
// *    PWM2(PB2) 	: Period = 3.91KHz (Duty: 1023/1024)
// *	PWM1(PB3)		: Period = 3.91KHz (Duty: 1/1024)
#define ADC_NUM 2

#define AD_M Pb3
#define MotorPWM PB2
#define LED_Blue PB0  //	BLUE
#define LED_Green PA0 //	green
#define LED_Red PB1	  //	red
#define TOUCH_KEY PA4
#define INPUT PA7
#define CHG PA5
#define EN_5V PA6

#define ON 0
#define OFF 1

#define PowerOFF 0
#define PowerOn 1

#define MotorOFF 0
#define MotorON 1

#define MotorPwmOFF 0 // 1
#define MotorPwmON 1  // 0

#define PwmDutyMax 100

#define BatLevelLess20 1
#define BatLevelMore21Less40 2
#define BatLevelMore41less60 3
#define BatLevelMore60Less80 4
#define BatLevelMore80less100 5
#define BatLevel100 6

#define LedStatus_Zerostep 0
#define LedStatus_Onestep 1
#define LedStatus_Twostep 2
#define LedStatus_Threestep 3
#define LedStatus_Fourstep 4
#define LedStatus_Fivestep 5
#define LedStatus_Sixstep 6
#define LedStatus_Sevenstep 7

#define Direct_ComSeg_SetIn \
	{                       \
		IOSTA |= 0x07;      \
	} //全部设置为输入口

#define Led3_OutHi     \
	{                  \
		IOSTA &= 0xFB; \
		LED3 = 1;      \
	}
#define Led3_OutLo     \
	{                  \
		IOSTA &= 0xFB; \
		LED3 = 0;      \
	}
#define Led2_OutHi     \
	{                  \
		IOSTA &= 0xfd; \
		LED2 = 1;      \
	}
#define Led2_OutLo     \
	{                  \
		IOSTA &= 0xfd; \
		LED2 = 0;      \
	}
#define Led1_OutHi     \
	{                  \
		IOSTA &= 0xfe; \
		LED1 = 1;      \
	}
#define Led1_OutLo     \
	{                  \
		IOSTA &= 0xfe; \
		LED1 = 0;      \
	}

#define Led3_In        \
	{                  \
		IOSTA |= 0x04; \
	}
#define Led2_In        \
	{                  \
		IOSTA |= 0x02; \
	}
#define Led1_In        \
	{                  \
		IOSTA |= 0x01; \
	}

#define CHRG_SHIFT_In  \
	{                  \
		IOSTB |= 0x01; \
	}
#define CHRG_SHIFT_OutLo \
	{                    \
		IOSTB &= 0xfe;   \
		CHRG_SHIFT = 0;  \
	}

#define volt3_13V 2632
#define volt3_3V 2775
#define volt3_53V 2986
#define volt3_73V 3136
#define volt3_86V 3245

#define chgless20 1	 // <3.3V							//1灯闪
#define chgMore20 2	 // 3.3V < x < 3.53V	//1亮2灯闪
#define chgMore40 3	 // 3.53V < x < 3.73V	//1亮2亮3灯闪
#define chgMore60 4	 // 3.73V < x < 3.86V	//1亮2亮3亮4灯闪
#define chgMore80 5	 // 3.86V < x < 4.2V		//1亮2亮3亮4亮5亮
#define chgMore100 6 // 4.2V == x						//全闪

#define Dischgless10 1 // x < 3.13V		//1灯闪
#define DischgLess20 2 // 3.13V < x < 3.3V	1灯亮
#define DischgMore20 3 // 3.3V < x < 3.53V	1.2灯亮
#define DischgMore40 4 // 3.53V < x < 3.73V 1.2.3灯亮
#define DischgMore60 5 // 3.73V < x < 3.86V	1.2.3.4灯亮
#define DischgMore80 6 // 3.86V < x 					1.2.3.4.5灯亮

#define BoostFeedbackValue 1015 // 0.495V
#define BoostDeadzone 105		// 21 = 0.01V

#define RedLed 1
#define WieghtLed 0
//变量
volatile unsigned char u8CntAdSamp;
volatile unsigned char u8CntFeedbackAdSamp;
volatile unsigned char SystemFlag1;
volatile unsigned char SystemFlag2;
volatile unsigned char SystemFlag3;
volatile unsigned char SystemFlag4;
volatile unsigned char Timecnt_2ms;
volatile unsigned char Timecnt_10ms;
volatile unsigned char Timecnt_50ms;
volatile unsigned char Timecnt_100ms;
volatile unsigned char SleepWakeupcnt;
volatile unsigned char PoweronDelay2Stimecnt;
volatile unsigned char SystemStatus; //开关机状态
volatile unsigned char MotorStatus;
volatile unsigned char LedStatus; //LED闪烁模式
volatile unsigned char MotorRuntime;
volatile unsigned char Send_Timer_Cnt_10S;
volatile unsigned char Send_Timer_Cnt_50S;
volatile unsigned char Send_Timer_Cnt_10Min;
volatile unsigned char Send_Timer_Cnt_4Hl;
volatile unsigned char Send_Timer_Cnt_4Hh;
volatile unsigned char DumpSwitchDetectCnt;
volatile unsigned int PWM1DUTYCnt;
volatile unsigned int PWM3DUTYCnt;
volatile unsigned int MotorTotaltimeCntL;
volatile unsigned char MotorTotaltimeCntH;
volatile unsigned char GreenLedPwmTimeintvalCnt;
volatile unsigned char Check_WeakWaterTimeCnt;
volatile unsigned char Check_HaveWaterTimeCnt;

volatile unsigned char WorkMode;
volatile unsigned char DisplayBuffer;
volatile unsigned char DisplaySclCnt;
volatile unsigned char BatLevel;
volatile unsigned char BatLevelTmep;
volatile unsigned char DisplayBuffer;
volatile unsigned char LedOffTimeCnt;
volatile unsigned char LedFlashtimeCnt;
volatile unsigned char chargeDetectTimecnt;
volatile unsigned char chargeFullDetectTimecnt;
volatile unsigned char NochargeDetectTimecnt;
volatile unsigned int Keypresstime; //按键消抖计数
volatile unsigned int BKeypresstime;
volatile unsigned long MotorRunlongtimeCnt;
volatile unsigned int MotorRunintvaltimeCnt;
volatile unsigned int R_AIN0_DATA;
// volatile unsigned int Ad_Sum[ADC_NUM];
// volatile unsigned int u16AdcResult[ADC_NUM];
volatile unsigned long Ad_Sum;
volatile unsigned int u16AdcResult;
volatile unsigned int u16AdcResult1;
volatile unsigned char OverCurrentTimeCnt;
volatile unsigned char MotorOnDelayTimeCnt;
volatile unsigned char Led_Pwm_Cnt_Cycle;
volatile unsigned char Led_Pwm_Cnt_DutyBuffer;
volatile unsigned char FlashTimeCnt; //led闪烁次数计数

volatile unsigned char WeakWater;
volatile unsigned char Timecnt_5ms;

volatile unsigned int IntoTheSelfTestModeTimecnt;
volatile unsigned char SelfTestModeStep;
volatile unsigned char TestmodeLedflashTimecnt;

__sbit f_Power_ON_AD = SystemFlag1 : 0;
__sbit f_short_key_WuHua_on = SystemFlag1 : 1; //短按确认标志
__sbit f_wakeupfromKey = SystemFlag1 : 2;
__sbit KeyLongPressflag = SystemFlag1 : 3; //长按确认标志
__sbit f_EnableSleep = SystemFlag1 : 4;
__sbit Time_50S_Flag = SystemFlag1 : 5;
__sbit Time_10S_Flag = SystemFlag1 : 6;
__sbit StartWuHuaFLag = SystemFlag1 : 7;

__sbit f_Timer2ms = SystemFlag2 : 0;
__sbit f_Timer10ms = SystemFlag2 : 1;
__sbit f_Timer50ms = SystemFlag2 : 2;
__sbit f_Timer100ms = SystemFlag2 : 3;
__sbit f_BatChargFull = SystemFlag2 : 4; //电池充满电标志
__sbit f_BatCharging = SystemFlag2 : 5; //电池充电中标志
__sbit ModeLedOnFlag = SystemFlag2 : 6;
__sbit f_TestmodeLedflash = SystemFlag2 : 7;

__sbit f_wakeupfromCharge = SystemFlag3 : 0;
__sbit f_Time_5ms = SystemFlag3 : 1;
__sbit f_LowBat = SystemFlag3 : 2; //低电标志
__sbit f_PowerOnInit = SystemFlag3 : 3;
__sbit f_SelfTestMode = SystemFlag3 : 4; //进入自检标志
__sbit f_LoestBat = SystemFlag3 : 5; //掉电标志
__sbit f_CandoSelfTest = SystemFlag3 : 6;
__sbit f_sleepDelay = SystemFlag3 : 7;

#define UPDATE_REG(x) __asm__("MOVR _" #x ",F")

//函数定义
int ADCConvert(char B_CH);
void BufferClean(void);
void Init(void);
void IO_Init(void);
void Pwm_Init(void);
void delay(int count);
void ADCInit(void);
void Timer0Init(void);
void F_wait_eoc(void);
void Adc_Sample(void);
void Keyscan(void);
void OffAllExtDevice(void);
void Sleep(void);
void AllLedOffFunc(void);
void AllLedONFunc(void);
void chargeLedDisplay(void);
void LedDisplayFunc(void);
void FangdianLedDisplay(void);
void Flashled(unsigned char LedType);
void MotorFunc(void);
void MotorIntvialRunFunc(int Runtime, int Stoptime);
void ReadFeedbackVoltageAD(void);
void GreenOffInit(void);
void GreenOnInit(void);
void GreenOnOff(void);

void MotorOFFFunc(void);
void MotorONFunc(void);

void TestmodeLedflashFunc(void);
void TestmodeKeyScan(void);
void Testmode(void);

//--------------- IO init --------------------------------------------
//
// Initialize GPIO
//--------------------------------------------------------------------------
void IO_Init(void)
{
	PCON = C_WDT_En | C_LVR_En; // Enable WDT & LVR
	DISI(); //close all interrupt

	// PORT A I/O state 0:output 1:input def:1
	// PB0:OUTPUT; PB0:OUTPUT; PB1:OUTPUT; PB2:OUTPUT; PB3:INPUT; PB4:OUTPUT; PB5:OUTPUT;
	IOSTB = C_PB3_Input;
	
	//portB up res set 0:enable 1:disable def: 1
	// BPHCON 	= (char)~( C_PB0_PHB | C_PB1_PHB );
	// BPHCON 	= (char)~( C_PB2_PHB );	
	
	//port B data, def:x
	PORTB = 0x03; //PB0:1; PB1:1; PB2:0; PB3:0; PB4:0; PB5:0;

	//port A data, def:x
	PORTA = 0x01; // PA0:1; PA1:0; PA2:0; PA3:0; PA4:0; PA5:0; PA6:0; PA7:0;

	AWUCON = C_PA4_Wakeup | C_PA7_Wakeup; //使能PA4 PA7唤醒功能

	// PORT A I/O state 0:output 1:input def:1
	// PB0:OUTPUT; PA0:OUTPUT; PA1:INPUT; PA2:OUTPUT; PA3:OUTPUT; PA4:INPUT; PA5:INPUT; PA6:OUTPUT; PA7:INPUT
	IOSTA = C_PA4_Input | C_PA5_Input | C_PA7_Input | C_PA1_Input; 

	//portA up res set 0:enable 1:disable def: 1
	// APHCON 	= (char)~((C_PA4_PHB)|(C_PA5_PHB)|(C_PA7_PHB));
	APHCON = (char)~(C_PA4_PHB | C_PA5_PHB); // PA4,PA5开上拉
}

//--------------- PWM init --------------------------------------------
//--------------------------------------------------------------------------
void Pwm_Init(void)
{
	// Initialize Timer1/2 & PWM1/2
	TMRH = 0x00; // C_TMR2_Data_b9 | C_TMR2_Data_b8 | C_TMR1_Data_b9 | C_TMR1_Data_b8 | C_PWM2_Duty_b9 | C_PWM2_Duty_b8;
	//	TMR1		= 0xFF;						// TMR1[9:0]=3FFH
	TMR2 = 0x11; // 0x12;						// TMR2[9:0]=3FFH
	//	PWM1DUTY	= 0x01;						// PWM1DUTY[9:0]=001H
	PWM2DUTY = 0x07; // 0x04;						// PWM2DUTY[9:0]=3FFH
	//	T1CR2		= C_PS1_Dis | C_TMR1_ClkSrc_Inst;	// Prescaler 1:1, Timer1 clock source is instruction clock
	T2CR2 = C_PS2_Dis | C_TMR2_ClkSrc_Inst;							  // Prescaler 1:1, Timer2 clock source is instruction clock
																	  //	T1CR1		= C_PWM1_En | C_PWM1_Active_Hi | C_TMR1_Reload | C_TMR1_En;	// Enable PWM1, Active_High, Non-Stop mode, Reloaded from TMR1[9:0], Enable Timer1
	T2CR1 = C_PWM2_En | C_PWM2_Active_Hi | C_TMR2_Reload | C_TMR2_En; // Enable PWM2, Active_High, Non-Stop mode, Reloaded from TMR2[9:0], Enable Timer2
	T2CR1 &= ~(C_PWM2_En);
}

//--------------- Timer0 init --------------------------------------------
void Timer0Init(void)
{
	PCON1 = C_TMR0_Dis; // Disable Timer0
	TMR0 = 213;			// Load 0x00 to TMR0 (Initial Timer0 register)

	T0MD = C_PS0_TMR0 | C_PS0_Div4; // Prescaler0 is assigned to Timer0, Prescaler0 dividing rate = 1:8,clock source is instruction clock
	INTE = C_INT_TMR0;

	PCON1 = C_TMR0_En; // Enable Timer0
	ENI();			   // Enable all unmasked interrupts
}

//--------------- ADC init --------------------------------------------
//--------------------------------------------------------------------------
void ADCInit(void)
{
	//----- Initial ADC-----
	ADMD = C_ADC_En | C_ADC_CH_Dis; // Enable ADC power, Disable global ADC input channelt (SFR "ADMD")

	//----- ADC high reference voltage source select-----
	// ADVREFH = C_Vrefh_VDD;
	// ADVREFH = C_Vrefh_4V;					// ADC reference high voltage is supplied by internal 4V  (Note: ADC clock freq. must be equal or less 1MHz)
	// ADVREFH = C_Vrefh_3V;					// ADC reference high voltage is supplied by internal 3V  (Note: ADC clock freq. must be equal or less 500KHz)
	ADVREFH = C_Vrefh_2V; // ADC reference high voltage is supplied by internal 2V  (Note: ADC clock freq. must be equal or less 250KHz)

	//----- ADC clock frequency select----------------------------
	// ADR	 = C_Ckl_Div1;					// ADC clock=Fcpu/1, Clear ADIF, disable ADC interrupt
	ADR = C_Ckl_Div2; // ADC clock=Fcpu/2, Clear ADIF, disable ADC interrupt
					  // ADR	  = C_Ckl_Div8;						// ADC clock=Fcpu/8, Clear ADIF, disable ADC interrupt
	// ADR	 = C_Ckl_Div16;					// ADC clock=Fcpu/16, Clear ADIF, disable ADC interrupt

	//----- ADC Sampling pulse width select-------------
	ADCR = C_Sample_1clk | C_12BIT; // Sample pulse width=1 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 500KHz)
									// ADCR  = C_Sample_2clk | C_12BIT;		// Sample pulse width=2 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 1MHz)
	// ADCR  = C_Sample_4clk | C_12BIT;		// Sample pulse width=4 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 1.25MHz)
	// ADCR  = C_Sample_8clk | C_12BIT; 		// Sample pulse width=8 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 2MHz)

	//--------------------------------------------------
	// PACON = C_PB3_AIN8;// Set AIN0(PA0) as pure ADC input for reducing power consumption (SFR "PACON")
	// PACON = C_PA1_AIN1;
	ADCR |= C_PB3_AIN8;
	delay(50); // Delay 0.56ms(Instruction clock=4MHz/2T) for waiting ADC stable
}
//--------------- System init --------------------------------------------
//--------------------------------------------------------------------------
void System_Init(void)
{
	IO_Init();
	Pwm_Init();
	ADCInit();
	Timer0Init();
}

//=========================================================================
//  AD 转换
//=========================================================================
int ADCConvert(char B_CH)
{
	char i;
	ADMD = 0x90 | B_CH; // Select AIN0(PA0) pad as ADC input
	for (i = 0; i < 50; i++)
		;				//延时10几个us ，等待稳定
	ADMDbits.START = 1; // Start a ADC conversion session
	// F_wait_eoc();							// Wait for ADC conversion complete
	while (ADMDbits.EOC == 0)
		;

	R_AIN0_DATA = ADD;
	R_AIN0_DATA = R_AIN0_DATA << 4;
	R_AIN0_DATA |= (0x0F & ADR);
	return R_AIN0_DATA;
}
//=========================================================================
// void F_wait_eoc(void)
//{
//   while(ADMDbits.EOC==0)
//   ;
//}

//========================================================
// ad 数据读取
//========================================================
void Adc_Sample(void)
{
	if (u8CntAdSamp >= 200)
	{
		Ad_Sum -= u16AdcResult;
		f_Power_ON_AD = 1;
	}
	else
	{
		u8CntAdSamp++;
	}
	Ad_Sum += ADCConvert(8);
	// Ad_Sum += ADCConvert(1);
	R_AIN0_DATA = 0;
	u16AdcResult = Ad_Sum / u8CntAdSamp;
}

//-----------------------------------------------------------
//	电池容量检测  确定电池电量
//-----------------------------------------------------------
void BatLevelDetect(void)
{
	if (f_BatCharging)
	{
		f_LowBat = 0;
		f_LoestBat = 0;
	}
	else
	{
		if (f_LoestBat == 0)
		{
			if (u16AdcResult >= 3379)
			// if(u16AdcResult >= 2744)
			{
				if (f_LowBat == 0)
				{
					f_LowBat = 0;
					f_LoestBat = 0;
				}
			}
			// else if((u16AdcResult < 3379)&&(u16AdcResult >= 3074)) //低电低于3.3V
			else if ((u16AdcResult < 3379) && (u16AdcResult >= 2867)) //低电低于2.8V
			{
				f_LowBat = 1;
				f_LoestBat = 0;
			}
			else
			{
				f_LowBat = 0;
				f_LoestBat = 1;
				FlashTimeCnt = 0;
			}
		}
	}
}

//-----------------------------------------------------------
//	触摸按键检测
//-----------------------------------------------------------
void Keyscan(void)
{
	if (!TOUCH_KEY)
	{
		if (f_short_key_WuHua_on == 0)
		{
			Keypresstime++;
			if (Keypresstime >= 2) //消抖25ms
			{
				Keypresstime = 0;
				f_short_key_WuHua_on = 1; //确认按下
			}
		}
		else
		{
			if (KeyLongPressflag == 0)
			{
				Keypresstime = 0;
				f_wakeupfromKey = 0;	//按键唤醒标记
				f_wakeupfromCharge = 0; //充电唤醒标记

				if ((f_BatCharging == 0) && (SystemStatus == PowerOFF))
				{
					f_CandoSelfTest = 1;
				}
				if (WorkMode == 2)
				{
					SystemStatus = PowerOFF;
					WorkMode = 0;
					LedStatus = 0;
					EN_5V = 0;
					T2CR1 &= ~(C_PWM2_En);
					MotorPWM = MotorPwmOFF;
					f_sleepDelay = 1;

					LED_Red = OFF;
					LED_Blue = OFF;
				}
				// if(Keypresstime>=200)
				//{
				//	Keypresstime = 0;
				//	KeyLongPressflag = 1;
				// }
			}
		}
	}
	else
	{
		if (f_wakeupfromKey)
		{
			SystemStatus = PowerOFF;
			f_EnableSleep = 1; //关机睡眠
		}
		else
		{
			if ((f_short_key_WuHua_on) && (KeyLongPressflag == 0)) //短按处理
			{
				if (f_sleepDelay == 0)
				{
					if (SystemStatus == PowerOFF)
					{
						f_EnableSleep = 0;
						SystemStatus = PowerOn; //开机
						if (f_LoestBat == 0)
						{
							StartWuHuaFLag = 1;
							WorkMode = 1;
							EN_5V = 1;
						}
					}
					else
					{
						if (f_LoestBat == 0)
						{
							if (WorkMode == 1)
							{
								StartWuHuaFLag = 1;
								f_EnableSleep = 0; //开机
								WorkMode = 2;
							}
							else if (WorkMode == 2)
							{
								//	SystemStatus = PowerOFF;
								f_EnableSleep = 1; //关机睡眠
								//	WorkMode = 0;
								//	EN_5V = 0;
							}
						}
					}
				}
				else
				{
					//	SystemStatus = PowerOFF;
					f_sleepDelay = 0;
					f_EnableSleep = 1; //关机睡眠
					//	WorkMode = 0;
					//	EN_5V = 0;
				}
			}
		}
		f_short_key_WuHua_on = 0;
		KeyLongPressflag = 0;
		f_CandoSelfTest = 0;
	}
}

//-----------------------------------------------------------
//	充电检测判断
//-----------------------------------------------------------
void chargeDetect(void)
{
	if ((!CHG) && (INPUT)) //充电中
	{
		chargeDetectTimecnt++;
		chargeFullDetectTimecnt = 0;
		NochargeDetectTimecnt = 0;
		if (chargeDetectTimecnt > 10)
		{
			if (f_BatCharging == 0)
				LedOffTimeCnt = 0;

			f_BatCharging = 1;
			f_BatChargFull = 0;
			chargeDetectTimecnt = 0;
			f_LowBat = 0;
			f_LoestBat = 0;
		}
		if (f_CandoSelfTest)
		{
			IntoTheSelfTestModeTimecnt++;
			if (IntoTheSelfTestModeTimecnt > 500)
			{
				IntoTheSelfTestModeTimecnt = 0;
				if (f_SelfTestMode == 0)
				{
					f_SelfTestMode = 1;
					SelfTestModeStep = 0;
				}
			}
		}
		else
		{
			IntoTheSelfTestModeTimecnt = 0;
		}
	}
	else if ((CHG) && (INPUT)) //充满
	{
		chargeDetectTimecnt = 0;
		chargeFullDetectTimecnt++;
		NochargeDetectTimecnt = 0;
		if (chargeFullDetectTimecnt > 10)
		{
			f_BatCharging = 0;
			f_BatChargFull = 1;
			chargeFullDetectTimecnt = 0;
			f_LowBat = 0;
			f_LoestBat = 0;
		}

		if (f_CandoSelfTest)
		{
			IntoTheSelfTestModeTimecnt++;
			if (IntoTheSelfTestModeTimecnt > 500)
			{
				IntoTheSelfTestModeTimecnt = 0;
				if (f_SelfTestMode == 0)
				{
					f_SelfTestMode = 1;
					SelfTestModeStep = 0;
				}
			}
		}
		else
		{
			IntoTheSelfTestModeTimecnt = 0;
		}
	}
	else //未充电
	{
		chargeDetectTimecnt = 0;
		chargeFullDetectTimecnt = 0;
		NochargeDetectTimecnt++;
		if (NochargeDetectTimecnt > 50)
		{
			f_BatCharging = 0;
			f_BatChargFull = 0;
			chargeDetectTimecnt = 0;
			chargeFullDetectTimecnt = 0;
			NochargeDetectTimecnt = 0;
			if (f_wakeupfromCharge)
			{
				f_wakeupfromCharge = 0;
				if (SystemStatus == PowerOFF)
				{
					SystemStatus = PowerOFF;
					f_EnableSleep = 1; //关机睡眠
				}
			}
			if (f_SelfTestMode)
			{
				SystemStatus = PowerOFF;
				f_EnableSleep = 1; //关机睡眠
				f_SelfTestMode = 0;
				f_CandoSelfTest = 0;
				SelfTestModeStep = 0;
				f_short_key_WuHua_on = 0;
			}
		}
	}
}

//-----------------------------------------------------------
void MotorFunc(void)
{
	if (SystemStatus == PowerOn)
	{
		switch (WorkMode)
		{
		case 0:
			if (f_EnableSleep)
			{
				MotorStatus = MotorOFF;
				T2CR1 &= ~(C_PWM2_En);
				MotorPWM = MotorPwmOFF;
				SystemStatus = PowerOFF;
				f_EnableSleep = 1; //关机睡眠
			}
			break;
		case 1:
			if (StartWuHuaFLag)
			{
				StartWuHuaFLag = 0;
				MotorONFunc();
			}
			if (MotorRunlongtimeCnt >= 1200) // 2min  持续喷 时间到关机
			{
				MotorRunlongtimeCnt = 0;
				MotorOFFFunc();
				SystemStatus = PowerOFF;
				f_EnableSleep = 1; //关机睡眠
				WorkMode = 0;
			}
			break;
		case 2:
			if (StartWuHuaFLag)
			{
				StartWuHuaFLag = 0;
				MotorONFunc();
			}
			MotorIntvialRunFunc(1200, 1200); // 2min on  2min off
			break;
		default:
			break;
		}
	}
}

//-----------------------------------------------------------
void MotorIntvialRunFunc(int Runtime, int Stoptime)
{
	if (MotorStatus == MotorON)
	{
		if (MotorRunintvaltimeCnt >= Runtime)
		{
			MotorRunintvaltimeCnt = 0;
			MotorStatus = MotorOFF;
			// T3CR1 &= ~(C_PWM3_En);
			T2CR1 &= ~(C_PWM2_En);
			MotorPWM = MotorPwmOFF;
		}
	}
	else
	{
		if (MotorRunintvaltimeCnt >= Stoptime)
		{
			MotorRunintvaltimeCnt = 0;
			// T3CR1	 |= C_PWM3_En;		//开启PWM
			T2CR1 |= C_PWM2_En;
			MotorStatus = MotorON; //开启马达
								   // f_Tongbu= 1;
			// MotorPWM = MotorPwmON;
		}
	}
}

//-----------------------------------------------------------
void MotorONFunc(void)
{
	T2CR1 |= C_PWM2_En; //开启PWM
	MotorStatus = MotorON;
	MotorOnDelayTimeCnt = 0;
	MotorRunlongtimeCnt = 0;
	MotorRunintvaltimeCnt = 0;
	MotorTotaltimeCntL = 0;
	MotorTotaltimeCntH = 0;
}

//-----------------------------------------------------------
void MotorOFFFunc(void)
{
	MotorStatus = MotorOFF;
	T2CR1 &= ~(C_PWM2_En);
	MotorPWM = MotorPwmOFF;
	MotorOnDelayTimeCnt = 0;
	MotorRunlongtimeCnt = 0;
	MotorRunintvaltimeCnt = 0;
	MotorTotaltimeCntL = 0;
	MotorTotaltimeCntH = 0;
}

//-----------------------------------------------------------
void LedDisplayFunc(void)
{
	if (SystemStatus == PowerOn)
	{
		if ((f_LowBat) && (f_LoestBat == 0))
		{
			LED_Green = OFF;
			LED_Blue = OFF;
			LedStatus = LedStatus_Threestep; //低电闪灯
		}
		else if ((f_LowBat == 0) && (f_LoestBat))
		{
			LED_Green = OFF;
			LED_Blue = OFF;
			LedStatus = LedStatus_Fourstep; //缺电闪烁
		}
		else if (WorkMode == 1)
		{
			FlashTimeCnt = 0;
			LedStatus = LedStatus_Twostep; //充满
		}
		else if (WorkMode == 2)
		{
			FlashTimeCnt = 0;
			LedStatus = LedStatus_Twostep; //充满
		}
	}
	else
	{
		if (f_BatCharging)
		{
			LedStatus = LedStatus_Onestep; //充电
		}
		else if (f_BatChargFull)
		{
			LedStatus = LedStatus_Twostep; //充满
		}
	}
	FangdianLedDisplay();
}

//-----------------------------------------------------------
//		放电led显示
//-----------------------------------------------------------
void FangdianLedDisplay(void)
{
	switch (LedStatus)
	{
	case LedStatus_Zerostep:
		LED_Red = OFF;
		LED_Blue = OFF;
		break;
	case LedStatus_Onestep: //充电
		LED_Red = ON;
		LED_Blue = OFF;
		break;
	case LedStatus_Twostep: //充满
		LED_Red = OFF;
		LED_Blue = ON;
		break;
	case LedStatus_Threestep: //低电 灯闪
		Flashled(RedLed);
		break;
	case LedStatus_Fourstep: //缺电闪3下
		Flashled(RedLed);
		break;
	case LedStatus_Fivestep: //缺水闪3下
		Flashled(RedLed);
		break;
	default:
		break;
	}
}

/***************************************************************************
***
***
****************************************************************************/
void Flashled(unsigned char LedType)
{
	if (ModeLedOnFlag == 0)
	{
		if (LedType == RedLed)
		{
			LED_Red = OFF; // OFF LED
		}
		if (++LedOffTimeCnt >= 50) // 250ms
		{
			LedOffTimeCnt = 0;
			ModeLedOnFlag = 1;
			if (LedType == RedLed)
			{
				LED_Red = ON; // ON LED
			}
		}
	}
	else
	{
		if (LedType == RedLed)
		{
			LED_Red = ON; // ON LED
		}
		if (++LedOffTimeCnt >= 50)
		{
			LedOffTimeCnt = 0;
			ModeLedOnFlag = 0;
			if (LedType == RedLed)
			{
				LED_Red = OFF; // OFF LED
			}
			if (f_LoestBat)
			{
				FlashTimeCnt++;
				if (FlashTimeCnt >= 3) //闪3次进入关机睡眠
				{
					FlashTimeCnt = 0;
					SystemStatus = PowerOFF;
					f_EnableSleep = 1; //关机睡眠
				}
			}
		}
	}
}

//--------------- isr function --------------------------------------------
//! interrupt service routine   100uS 中断一次
//--------------------------------------------------------------------------
void isr(void) __interrupt(0)
{
	if (INTFbits.T0IF)
	{
		TMR0 = 213;
		INTF = (char)~(C_INT_TMR0); // Clear T0IF flag bit
		// PORTA ^= (1<<2);					// PA3 Toggle
		if (++Timecnt_5ms >= 50)
		{
			Timecnt_5ms = 0;
			f_Time_5ms = 1;
		}
		if (++Timecnt_2ms >= 20)
		{
			Timecnt_2ms = 0;
			f_Timer2ms = 1;
			if (++Timecnt_10ms >= 5) // 10ms
			{
				Timecnt_10ms = 0;
				f_Timer10ms = 1;
				SleepWakeupcnt++;
				if (++Timecnt_50ms >= 5) // 50ms
				{
					Timecnt_50ms = 0;
					f_Timer50ms = 1;
					if (++Timecnt_100ms >= 2) // 100ms
					{
						Timecnt_100ms = 0;
						f_Timer100ms = 1;
						MotorRunlongtimeCnt++;
						MotorRunintvaltimeCnt++;
						if (MotorOnDelayTimeCnt < 6)
							MotorOnDelayTimeCnt++;
					}
				}
			}
		}
	}
	if (INTFbits.PABIF)
	{
		INTFbits.PABIF = 0; // Clear PABIF(PortB input change interrupt flag bit)
	}
}

/***************************************************************************
***LED全亮
***
****************************************************************************/
void AllLedONFunc(void)
{
	LED_Red = 0; // led A 白光
	LED_Green = 0;
	LED_Blue = 0;
}

/***************************************************************************
***LED 全灭
***
****************************************************************************/
void AllLedOffFunc(void)
{
	LED_Red = 1; // led A 白光
	LED_Green = 1;
	LED_Blue = 1;
}

//-----------------------------------------------------------
//	关闭所有外设
//-----------------------------------------------------------
void OffAllExtDevice(void)
{
	SystemStatus = PowerOFF;
	WorkMode = 0;
	LedStatus = 0;
	StartWuHuaFLag = 0;
	MotorStatus = MotorOFF;
	// T3CR1 &= ~(C_PWM3_En);
	// GreenOffInit();
	T2CR1 &= ~(C_PWM2_En);
	MotorPWM = MotorPwmOFF;
	EN_5V = 0;
	//	EN_Check_Water = 0; //关机使能检水
	// CheckWaterIsOk = 0;
	FlashTimeCnt = 0;
	//	VON = 1;
	Send_Timer_Cnt_10S = 0;
	Send_Timer_Cnt_50S = 0;
	Send_Timer_Cnt_10Min = 0;
	Time_50S_Flag = 0;
	Time_10S_Flag = 0;
	Send_Timer_Cnt_4Hl = 0;
	Send_Timer_Cnt_4Hh = 0;
	//	CHRG_SHIFT_OutLo;//关机之后增加充电电流
	AllLedOffFunc();
}

//--------------- Sleep function --------------------------------------------
//--------------------------------------------------------------------------
void Sleep(void)
{
	if ((f_EnableSleep) && (f_PowerOnInit == 0)) //上电时, 待初始化所有数据后才进入睡眠
	{
		f_wakeupfromCharge = 0;
		f_wakeupfromKey = 0;
		// UPDATE_REG(PORTA);					// Read PORTB Data buffer

		OffAllExtDevice();
		if ((f_BatCharging) || (f_BatChargFull))
		{
			f_EnableSleep = 0;
			f_wakeupfromCharge = 1;
			return;
		}
		IOSTA &= 0xdf;
		APHCON = (char)~(C_PA4_PHB);
		CHG = 0;

		f_LowBat = 0;

		PCON = 0x18; // 0x10;			// Disable WDT
		ADMD = C_ADC_Dis;

		ModeLedOnFlag = 0;
		KeyLongPressflag = 0;
		// A. Normal mode into Halt mode. While PB1 input change then wakeup and set PB2 outputs low
		// UPDATE_REG(PORTB);					// Read PORTB Data buffer
		delay(200);
		delay(200);
		UPDATE_REG(PORTA); // Read PORTB Data buffer
		UPDATE_REG(PORTB); // Read PORTB Data buffer
		// Initial Interrupt Setting
		INTE |= C_INT_PABKey; // Enable PortB input change interrupt
		INTF = 0;			  // Clear all interrupt flags

		ENI();
		// choice one way to enter Halt mode
		SLEEP(); // 1. Execute instruction to enters Halt mode (from Normal mode)
				 // OSCCR = HALT_MODE | FHOSC_SEL;		// 2. Set OSCCR register to enters Halt mode (from Normal mode)
		do
		{
			ENI(); // enable golbal int
			if (!TOUCH_KEY)
			{
				f_EnableSleep = 0;
				f_wakeupfromKey = 1;
			}
			else if (INPUT)
			{
				f_EnableSleep = 0;
				f_wakeupfromCharge = 1;
				f_BatCharging = 1;
			}
			else
			{
				SleepWakeupcnt = 0;
				UPDATE_REG(PORTA); // Read PORTA Data buffer
				UPDATE_REG(PORTB); // Read PORTB Data buffer
				// Initial Interrupt Setting
				delay(200);
				delay(200);
				INTE |= C_INT_PABKey; // Enable PortB input change interrupt
				INTF = 0;			  // Clear all interrupt flags
				ENI();
				SLEEP();
			}
		} while (f_EnableSleep == 1);
		INTE &= ~(C_INT_PABKey);
		PCON = C_WDT_En | C_LVR_En; // Enable WDT & LVR
		CLRWDT();

		// IO_Init();
		IOSTA = C_PA4_Input | C_PA5_Input | C_PA7_Input | C_PA1_Input;
		APHCON = (char)~(C_PA4_PHB | C_PA5_PHB);
	}
}

//--------------- Main function --------------------------------------------
//--------------------------------------------------------------------------
void main(void)
{
	System_Init();
	f_PowerOnInit = 1;
	f_EnableSleep = 1;
	while (1)
	{
		CLRWDT(); // Clear WatchDog
		ENI();
		if (f_Timer2ms) //每2ms 采样AD
		{
			f_Timer2ms = 0;
			Adc_Sample();
		}

		if (f_Timer10ms)
		{
			f_Timer10ms = 0;
			if (f_SelfTestMode)
			{
				Testmode();
			}
			else
			{
				chargeDetect();	   //充电检测
				Keyscan();		   //按键扫描
				MotorFunc();	   // Motor intvial Contrl
				if (f_Power_ON_AD) // AD 检测完成再判断值
				{
					f_PowerOnInit = 0;
					BatLevelDetect(); //检测电池电量
					LedDisplayFunc(); // LED指示
				}
			}
		}
		Sleep();
	}
}

//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------
void delay(int count)
{
	int i;
	for (i = 1; i <= count; i++)
		;
}

//--------------------------------------------------------------------------
void Testmode(void)
{
	switch (SelfTestModeStep)
	{
	case 0:
		if (f_short_key_WuHua_on == 0)
			SelfTestModeStep = 1;

		EN_5V = 1;
		TestmodeLedflashFunc();
		break;
	case 1:
		TestmodeLedflashFunc();
		break;
	case 2:
		T2CR1 |= C_PWM2_En; //开启PWM
		MotorStatus = MotorON;

		LED_Red = 0;
		LED_Green = 0;
		LED_Blue = 0;
		break;
	case 3:
		MotorStatus = MotorOFF;
		T2CR1 &= ~(C_PWM2_En);
		MotorPWM = MotorPwmOFF;

		LED_Red = 1;
		LED_Green = 1;
		LED_Blue = 1;
		break;
	case 4:
		if (u16AdcResult >= 3990)
		{
			LED_Red = 1;
			LED_Green = 0;
			LED_Blue = 1;
		}
		else if ((u16AdcResult < 3990) && (u16AdcResult >= 3680)) //低电低于3.3V
		{
			LED_Red = 1;
			LED_Green = 1;
			LED_Blue = 0;
		}
		else
		{
			LED_Red = 0;
			LED_Green = 1;
			LED_Blue = 1;
		}
		break;
	default:
		break;
	}
	TestmodeKeyScan();
	chargeDetect(); //充电检测
}

//--------------------------------------------------------------------------
//		tset mode key scan
//--------------------------------------------------------------------------
void TestmodeKeyScan(void)
{
	if (!TOUCH_KEY)
	{
		if (f_short_key_WuHua_on == 0)
		{
			Keypresstime++;
			if (Keypresstime >= 2) //消抖25ms
			{
				Keypresstime = 0;
				f_short_key_WuHua_on = 1;
			}
		}
	}
	else
	{
		if ((f_short_key_WuHua_on) && (SelfTestModeStep > 0))
		{
			SelfTestModeStep++;
		}
		f_short_key_WuHua_on = 0;
	}
}

//--------------------------------------------------------------------------
void TestmodeLedflashFunc(void)
{
	if (f_TestmodeLedflash == 0)
	{
		AllLedONFunc();
		TestmodeLedflashTimecnt++;
		if (TestmodeLedflashTimecnt >= 50)
		{
			TestmodeLedflashTimecnt = 0;
			AllLedOffFunc();
			f_TestmodeLedflash = 1;
		}
	}
	else
	{
		AllLedOffFunc();
		TestmodeLedflashTimecnt++;
		if (TestmodeLedflashTimecnt >= 50)
		{
			TestmodeLedflashTimecnt = 0;
			AllLedONFunc();
			f_TestmodeLedflash = 0;
		}
	}
}
