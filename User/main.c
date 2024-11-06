/********************(C) COPRIGHT 2019 Crownto electronic **************************
 * 文件名  ：main.c
 * 描述    ：        
 * 硬件连接：
 * OLED  SCL:PB8  SDA:PB9
 * TDS传感器模块: VCC -> 3.3V; GND -> GND; AO -> PA2;
 *
 * 功能描述：测量TDS值液晶显示(ADC1、PA2、DMA方式读取)；
             串口接收测量所得的TDS值（波特率115200）；
 *           可连接上位机显示TDS值；

**********************************************************************************/
#include "stm32f10x.h"
//#include "bsp_usart1.h"
#include <string.h>
#include "delay.h"
#include "math.h"
#include <stdlib.h>
#include "bsp_adc.h"
#include "ds18b20.h"
#include "OLED.h"
float TDS_value=0.0;
float temp_data=23.00;    //设定的初始温度
float compensationCoefficient=1.0;//温度校准系数
float compensationVolatge;
float kValue=1.0;     //K值取1最合适  根据静止水中测出的数据，但是需要减去初始值（约为18，在无水调价条件下的初始电压值）
#define VREF 3.3     // analog reference voltage(Volt) of the ADC     
static uint32_t systickCount = 0; // 毫秒计数
static uint32_t analogSampleTimepoint = 0;
static uint32_t printTimepoint = 0;
static uint32_t analogBufferIndex = 0;
static uint32_t copyIndex = 0;
#define SCOUNT  30     // sum of sample point          取样本次数
  // store the analog value in the array, read from ADC
static int analogBuffer[SCOUNT];             // 读取模拟值并存储到缓冲区
static int analogBufferTemp[SCOUNT];         //每隔一段时间取一次缓冲区的数据用于计算
static float averageVoltage = 0.0;
u8 ISendByte(unsigned char sla,unsigned char c);
u8 IRcvByte(unsigned char sla);	
u8 SPIx_ReadWriteByte(u8 TxData);
int getMedianNum(int bArray[], int iFilterLen);
unsigned char  Tx[20];   //无线发送缓存

// SysTick_Handler() 函数
void SysTick_Handler(void) {
    systickCount++; // 每次滴答定时器中断时增加计数
}

// ADC1转换的电压值通过MDA方式传到SRAM
extern __IO uint16_t ADC_ConvertedValue;

// 局部变量，用于保存转换计算后的电压值 	 
float ADC_ConvertedValueLocal;  




/**************TDS值采集函数***************/
void TDS_Value_Conversion()
{
	//控制取样评率，40us读取一次
	   if (systickCount - analogSampleTimepoint > 1) {
        analogSampleTimepoint = systickCount;
        analogBuffer[analogBufferIndex] = ADC_ConvertedValue; // 读取模拟值并存储到缓冲区
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
            analogBufferIndex = 0;
    }
	   //40us*20= 800us打印一次
	if (systickCount - printTimepoint > 100) {
        printTimepoint = systickCount;
		for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
		analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
		//中位数滤波和TDS值计算 ...   12位的adc通道
		averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        //TDS_voltage =(float) ADC_ConvertedValue/4096*3.3; // 读取转换的AD值
		compensationCoefficient = 1.0 + 0.02 * (temp_data - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
		compensationVolatge=averageVoltage/compensationCoefficient;
		TDS_value =((66.71 * compensationVolatge * compensationVolatge * compensationVolatge - 127.93 * compensationVolatge * compensationVolatge + 428.7 * compensationVolatge)-18) ;
		//  流动水中的数据：flow water data：
		//TDS_value =((66.71 * compensationVolatge * compensationVolatge * compensationVolatge - 127.93 * compensationVolatge * compensationVolatge + 428.7 * compensationVolatge)-18)/2-20 ;
		if((TDS_value<=0)){TDS_value=0;}
		if((TDS_value>1400)){TDS_value=1400;}
	
		OLED_ShowString(2, 1, "TDS:"); // 重新打印，避免闪烁
		OLED_ShowNum(2, 5, TDS_value, 4);
		
		
    }
		
}


/**
  * @brief  获取数组的中位数
  * @param   bArray[] - 输入的整数数组
  * @param iFilterLen - 数组的长度
  * @retval bTemp -返回中位数
 */ 
int getMedianNum(int bArray[], int iFilterLen) {
    int i, j, bTemp;
    // 使用冒泡排序对数组进行排序
    for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
            if (bArray[i]>bArray[i + 1]) {
                bTemp = bArray[i];
                bArray[i]=bArray[i + 1];
                bArray[i + 1]=bTemp;
            }
        }
    }
    // 计算中位数
    if ((iFilterLen & 1)>0)
        bTemp = bArray[(iFilterLen - 1)/2];
    else
        bTemp = (bArray[iFilterLen / 2]+bArray[iFilterLen / 2 - 1])/2;
    return bTemp;
}
void TEMP_Value_Conversion()
{
		//temp_data=DS18B20_Get_Temp();
		OLED_ShowString(3, 1, "TEM:");
		OLED_ShowNum(3, 5, temp_data, 2);
		OLED_ShowString(3, 7, ".");
		OLED_ShowNum(3, 8, (uint16_t)(temp_data*100)%100, 2); //显示小数部分	
}

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
	// 配置SysTick定时器
    SysTick_Config(SystemCoreClock/1000000*20);
    /* 配置USART1 */
   // USART1_Config();
	// ADC 初始化
	  ADCx_Init();   
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	/*显示静态字符串*/
	OLED_ShowString(1, 1, "HaoShui:");
	OLED_ShowString(2, 1, "TDS:");
	OLED_ShowString(3, 1, "TEM:");
     
	
  while(1)
	{	
		TEMP_Value_Conversion();
		TDS_Value_Conversion();		
	}	
}
/*********************************************END OF FILE**********************/
