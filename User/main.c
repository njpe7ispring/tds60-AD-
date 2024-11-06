/********************(C) COPRIGHT 2019 Crownto electronic **************************
 * �ļ���  ��main.c
 * ����    ��        
 * Ӳ�����ӣ�
 * OLED  SCL:PB8  SDA:PB9
 * TDS������ģ��: VCC -> 3.3V; GND -> GND; AO -> PA2;
 *
 * ��������������TDSֵҺ����ʾ(ADC1��PA2��DMA��ʽ��ȡ)��
             ���ڽ��ղ������õ�TDSֵ��������115200����
 *           ��������λ����ʾTDSֵ��

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
float temp_data=23.00;    //�趨�ĳ�ʼ�¶�
float compensationCoefficient=1.0;//�¶�У׼ϵ��
float compensationVolatge;
float kValue=1.0;     //Kֵȡ1�����  ���ݾ�ֹˮ�в�������ݣ�������Ҫ��ȥ��ʼֵ��ԼΪ18������ˮ���������µĳ�ʼ��ѹֵ��
#define VREF 3.3     // analog reference voltage(Volt) of the ADC     
static uint32_t systickCount = 0; // �������
static uint32_t analogSampleTimepoint = 0;
static uint32_t printTimepoint = 0;
static uint32_t analogBufferIndex = 0;
static uint32_t copyIndex = 0;
#define SCOUNT  30     // sum of sample point          ȡ��������
  // store the analog value in the array, read from ADC
static int analogBuffer[SCOUNT];             // ��ȡģ��ֵ���洢��������
static int analogBufferTemp[SCOUNT];         //ÿ��һ��ʱ��ȡһ�λ��������������ڼ���
static float averageVoltage = 0.0;
u8 ISendByte(unsigned char sla,unsigned char c);
u8 IRcvByte(unsigned char sla);	
u8 SPIx_ReadWriteByte(u8 TxData);
int getMedianNum(int bArray[], int iFilterLen);
unsigned char  Tx[20];   //���߷��ͻ���

// SysTick_Handler() ����
void SysTick_Handler(void) {
    systickCount++; // ÿ�εδ�ʱ���ж�ʱ���Ӽ���
}

// ADC1ת���ĵ�ѹֵͨ��MDA��ʽ����SRAM
extern __IO uint16_t ADC_ConvertedValue;

// �ֲ����������ڱ���ת�������ĵ�ѹֵ 	 
float ADC_ConvertedValueLocal;  




/**************TDSֵ�ɼ�����***************/
void TDS_Value_Conversion()
{
	//����ȡ�����ʣ�40us��ȡһ��
	   if (systickCount - analogSampleTimepoint > 1) {
        analogSampleTimepoint = systickCount;
        analogBuffer[analogBufferIndex] = ADC_ConvertedValue; // ��ȡģ��ֵ���洢��������
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
            analogBufferIndex = 0;
    }
	   //40us*20= 800us��ӡһ��
	if (systickCount - printTimepoint > 100) {
        printTimepoint = systickCount;
		for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
		analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
		//��λ���˲���TDSֵ���� ...   12λ��adcͨ��
		averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        //TDS_voltage =(float) ADC_ConvertedValue/4096*3.3; // ��ȡת����ADֵ
		compensationCoefficient = 1.0 + 0.02 * (temp_data - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
		compensationVolatge=averageVoltage/compensationCoefficient;
		TDS_value =((66.71 * compensationVolatge * compensationVolatge * compensationVolatge - 127.93 * compensationVolatge * compensationVolatge + 428.7 * compensationVolatge)-18) ;
		//  ����ˮ�е����ݣ�flow water data��
		//TDS_value =((66.71 * compensationVolatge * compensationVolatge * compensationVolatge - 127.93 * compensationVolatge * compensationVolatge + 428.7 * compensationVolatge)-18)/2-20 ;
		if((TDS_value<=0)){TDS_value=0;}
		if((TDS_value>1400)){TDS_value=1400;}
	
		OLED_ShowString(2, 1, "TDS:"); // ���´�ӡ��������˸
		OLED_ShowNum(2, 5, TDS_value, 4);
		
		
    }
		
}


/**
  * @brief  ��ȡ�������λ��
  * @param   bArray[] - �������������
  * @param iFilterLen - ����ĳ���
  * @retval bTemp -������λ��
 */ 
int getMedianNum(int bArray[], int iFilterLen) {
    int i, j, bTemp;
    // ʹ��ð������������������
    for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
            if (bArray[i]>bArray[i + 1]) {
                bTemp = bArray[i];
                bArray[i]=bArray[i + 1];
                bArray[i + 1]=bTemp;
            }
        }
    }
    // ������λ��
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
		OLED_ShowNum(3, 8, (uint16_t)(temp_data*100)%100, 2); //��ʾС������	
}

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{	
	// ����SysTick��ʱ��
    SysTick_Config(SystemCoreClock/1000000*20);
    /* ����USART1 */
   // USART1_Config();
	// ADC ��ʼ��
	  ADCx_Init();   
	/*ģ���ʼ��*/
	OLED_Init();		//OLED��ʼ��
	/*��ʾ��̬�ַ���*/
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
