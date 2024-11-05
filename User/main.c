/********************(C) COPRIGHT 2019 Crownto electronic **************************
 * �ļ���  ��main.c
 * ����    ��        
 * Ӳ�����ӣ�
 * LCD1602:RS -> PA11; RW -> PA12; E -> PA15;
 *         D0~D7 -> PB3~PB10
 * TDS������ģ��: VCC -> 5V; GND -> GND; AO -> PA2;
 *
 * ��������������TDSֵҺ����ʾ(ADC1��PA2��DMA��ʽ��ȡ)��
             ���ڽ��ղ������õ�TDSֵ��������115200����
 *           ��������λ����ʾTDSֵ��
 * �Ա�    ��https://iloveMCU.taobao.com
**********************************************************************************/
#include "stm32f10x.h"
#include "bsp_usart1.h"
#include <string.h>
#include "delay.h"
#include "driver_1602.h"
#include "math.h"
#include <stdlib.h>
#include "bsp_adc.h"
//#include "ds18b20.h"
#include "OLED.h"
GPIO_InitTypeDef  GPIO_InitStructure; 
unsigned char AD_CHANNEL=0;
unsigned long PH_num=0,PU_V=0;
float PH_Value=0;
u8 ph_temp=0,tu_temp=0;
u16 ph_result=0,tu_result=0;
u16 adc_1,adc_2;
u16 adc_v_1,adc_v_2;
float TDS=0.0;
float TDS_voltage;
float TDS_value=0.0,voltage_value;
float temp_data=23.00;    //������25
float compensationCoefficient=1.0;//�¶�У׼ϵ��
float compensationVolatge;
float kValue=1.0;
#define VREF 3.3     // analog reference voltage(Volt) of the ADC
float K = 4.2;       //K=��׼ֵ/����ֵ   k=63/15=4.2 ��׼
//ȡk=61/20=3.05
static uint32_t systickCount = 0; // �������
static uint32_t analogSampleTimepoint = 0;
static uint32_t printTimepoint = 0;
static uint32_t analogBufferIndex = 0;
static uint32_t copyIndex = 0;
#define SCOUNT  5     // sum of sample point
  // store the analog value in the array, read from ADC
static int analogBuffer[SCOUNT];
static int analogBufferTemp[SCOUNT];
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

/***************************************************************************
 * ��  �� : MAIN����
 * ��  �� : ��
 * ����ֵ : ��
 **************************************************************************/
 
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable the GPIO  Clock */					 		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO|RCC_APB2Periph_SPI1,ENABLE);
	
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);		//����������ΪJTAG�ڵ�GPIO��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		//����PB����IO��JTAG����
	
	
//1602 D0~D7 ����	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|
	GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; /*I/O ���� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; /*I/O ����ٶ�*/
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
//1602 EN RS RW ����	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; /*I/O ���� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; /*I/O ����ٶ�*/
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
   /* Configure  DRDY */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}


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
		//compensationCoefficient=1.0+0.02*((temp_data/10)-25.0); 
		//compensationVolatge=TDS_voltage/compensationCoefficient;
		compensationVolatge=averageVoltage/compensationCoefficient;
		TDS_value =((66.71 * compensationVolatge * compensationVolatge * compensationVolatge - 127.93 * compensationVolatge * compensationVolatge + 428.7 * compensationVolatge)-18)/2-20 ;
		
		//convert voltage value to tds value

//		TDS_value=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 
//		255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5*kValue;
		
		if((TDS_value<=0)){TDS_value=0;}
		if((TDS_value>1400)){TDS_value=1400;}
		
//		/*��ʾTDS*/
//		Tx[0]=(int)(TDS_value)/1000+'0';
//		Tx[1]=(int)(TDS_value)%1000/100+'0';	
//		Tx[2]=(int)(TDS_value)%100/10+'0';
//		Tx[3]=(int)(TDS_value)%10+'0';
//		
//		LCD_printchar(0x5,0,Tx[0]);
//		LCD_printchar(0x6,0,Tx[1]);
//		LCD_printchar(0x7,0,Tx[2]);
//		LCD_printchar(0x8,0,Tx[3]);
		OLED_ShowString(2, 1, "TDS:"); // ���´�ӡ��������˸
		OLED_ShowNum(2, 5, TDS_value, 4);
		
		
    }
		
}

//int getMedianNum(int bArray[], int iFilterLen)
//{

//	int *bTab = (int *)malloc(iFilterLen * sizeof(int));
//	  int i, j, bTemp;
//    if (bTab == NULL)
//    {
//        // ����ڴ����ʧ�ܣ����Ը���ʵ��������д�������򵥷��� -1
//        return -1;
//    }
//  for ( i = 0; i < iFilterLen; i++)
//    bTab[i] = bArray[i];

//  for (j = 0; j < iFilterLen - 1; j++)
//  {
//    for (i = 0; i < iFilterLen - j - 1; i++)
//    {
//      if (bTab[i] > bTab[i + 1])
//      {
//        bTemp = bTab[i];
//        bTab[i] = bTab[i + 1];
//        bTab[i + 1] = bTemp;
//      }
//    }
//  }
//  if ((iFilterLen & 1) > 0)
//    bTemp = bTab[(iFilterLen - 1) / 2];
//  else
//    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
//  return bTemp;
// }

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
//	
//	  Tx[4]=(int)(temp_data)%1000/100+'0';	
//	  Tx[5]=(int)(temp_data)%100/10+'0';
//	  Tx[6]='.';
//	  Tx[7]=(int)(temp_data)%10+'0';
//	
//	  LCD_printchar(0x5,1,Tx[4]);
//		LCD_printchar(0x6,1,Tx[5]);
//		LCD_printchar(0x7,1,Tx[6]);
//		LCD_printchar(0x8,1,Tx[7]);
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
	GPIO_Configuration();
	 //DS18B20_Init();
    /* ����USART1 */
   // USART1_Config();
		// ADC ��ʼ��
	  ADCx_Init();  
    /* ��ʼ��LCD1602 */
	//	LCD_init(); 
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
		
		//printf("%s",Tx);
		//delay_ms(1000);
		
	}	
}
/*********************************************END OF FILE**********************/
