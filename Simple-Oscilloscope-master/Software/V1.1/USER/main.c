/**********************************************************
����ʾ������PA6��������
ADC(TIME����)->DMA
PA4ͨ��DACͨ��1������Ҳ���Ƶ����TIME3��Ƶֵ����
PA5ͨ��DACͨ��2������ǲ���������Ƶ����TIME4����
KEY_WAKE���Ƹ�������ͣ
KEY0���Ӳ���Ƶ��
KEY1���Ͳ���Ƶ��
��ʱ��4����DACͨ��2������ǲ�������
��ʱ��3�����ж�DACͨ��1������Ҳ�
***********************************************************/
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "adc.h"
#include "dma.h"
#include "timer.h"
#include "table_fft.h"
#include "stm32_dsp.h"
#include "math.h"
#include "key.h"
#include "BEEP.h"
#include "dac.h"
#include "exti.h"
#define NPT 1024 //��������
#define PI2 6.28318530717959

void window(void);//������
void clear_point(u16 mode);//������ʾ����ǰ��
void InitBufInArray(void);//���Ҳ��������
void sinout(void);//���Ҳ����
void GetPowerMag(void);//FFT�任�����Ƶ��

int long fftin [NPT];//FFT����
int long fftout[NPT];//FFT���
u32 FFT_Mag[NPT/2]={0};//��Ƶ����
u16 magout[NPT];//ģ�����Ҳ����������
u16 freq_index[20] = {9, 17, 26, 34, 43, 51, 60, 68, 77, 85, 94, 102, 111, 119, 128,137, 145, 154, 162, 171};//5K-100KƵ�ʵ�index
u32 freq_table[20] = {5000, 10000, 15000, 20000, 25000, 30000, 35000,40000, 45000, 50000, 55000, 60000, 65000, 70000, 75000,
80000, 85000, 90000, 95000,100000};
u16 table[15] ={16,32,48,64,80,96,112,128,144,160,176,192,208,224,240};//��������
u16 currentadc;//ʵʱ��������
u16 adcx[NPT];//adc��ֵ����
u32 adcmax;//�������ֵ����Сֵ
u32 adcmin;
u8 adc_flag=0;//����������־
u8 key_flag=0;//����ɨ���־
u8 show_flag=1;//������ͣ��־
u16 T= 10 - 1;//��ʱ��2����ֵ������С��PWM��Pluseֵ
u16 pre= 6 - 1;//��ʱ��2Ԥ��Ƶֵ
u32 fre;//����Ƶ�� kHz
u32 F;//����Ƶ��
u32 aF_max, aF_secondmax, aF_triple;//���Ƶ���θ�Ƶ��Ӧ�ķ�ֵ, ����г����Ӧ�ķ�ֵ
u16 max_index, second_index, freq_temp, freq_max, freq_secondmax;
u16 V=660;//�����굥λ�̶� mv/div
u32 temp = 0;//��ֵ����Ƶ�ʳɷ�
u16 t = 0;
u16 key;//����ֵ
u16 signal_detection(u32 F_max, u32 F_secondmax);
u8 wave_type_a, wave_type_b;//���ò��ε���״
void compare_val(u32 F1, u32 F2, u32* max_val, u32* second_min_val);

int main()
{
	u16 i;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	MYDMA1_Config(DMA1_Channel1,(u32)&ADC1->DR,(u32)&currentadc,1);
	uart_init(115200);
	delay_init();
	LED_Init();
	EXTIX_Init();
	LCD_Init();
	Adc_Init();
	BEEP_Init();
	InitBufInArray();
	TIM2_PWM_Init(T, pre);	//���Ƶ��72000000/20/12 = 300KHz
	LCD_Clear(BLACK);
	window();
	while(1)
	{
		//�ȴ��������
		while(adc_flag==0)
		{
			LED1=!LED1;
			delay_ms(100);
		}
		adc_flag=0;
		
		//��ȡ�����Сֵ
		adcmax=adcx[1];
		adcmin=adcx[1];
		for(i=0;i<NPT;i++)
		{
			fftin[i] = 0;
			fftin[i] = adcx[i] << 16;
			
			if(adcx[i] >= adcmax)
			{			
				adcmax = adcx[i];
			}			
			if(adcx[i] <= adcmin)
			{			
				adcmin = adcx[i];
			}						
		}	
		POINT_COLOR=BLUE;
		GetPowerMag();	
		adcmax=adcmax*0.8;   //0.8 �� 3300/4096	
		adcmin=adcmin*0.8;		
		// if(show_flag==1)
		// {
		// 	clear_point(1);    //���²���Ƶ�ʣ�������ʾ����ǰ�У��������߻���
		// }	
		
		LED0=!LED0;
		DMA_Cmd(DMA1_Channel1,ENABLE);//ʹ��DMA1-CH1
		//delay_ms(100);
	}
}

/**********************************************************
��飺���������
***********************************************************/
void window(void)
{
	u16 x,i;
	static u16 h; 
	
	POINT_COLOR=GREEN;	  
	POINT_COLOR=GRAY;

}

/******************************************************************
��������:clear_point()
��������:ѭ�����²���
����˵��:mode ����ģʽѡ�� 1��������ģʽ��0�������ģʽ
��    ע:���ε���ʾ�ɲ��ô�㷽ʽ�ͻ����߷�ʽ
*******************************************************************/
void clear_point(u16 mode)
{
	u16 x,i,past_vol,pre_vol;
	static u16 h; 
	
	POINT_COLOR=BLUE;
	fre = 72000000 / (T+1) / (pre + 1) / 2;//���²���Ƶ��, ʱ��Ƶ��72M/��װ��ֵ20/��Ƶϵ��12 = 300Khz
	
		if( fre > 10000) { 
		temp = fre / 1000;   //�������ʴ���10K��ʹ��khz��ʾ
	}
	else {

	}

}

/******************************************************************
��������:GetPowerMag()
��������:�������г����ֵ
����˵��:
������ע:�Ƚ�lBufOutArray�ֽ��ʵ��(X)���鲿(Y)��Ȼ������ֵ(sqrt(X*X+Y*Y)

*******************************************************************/
void GetPowerMag(void)
{
    float X,Y,Mag,magmax;//ʵ�����鲿����Ƶ�ʷ�ֵ������ֵ
    u16 i;
	u16 index_gaps, F_triple_index;
	u32 F_temp, F_temp_2, F_secondmax, F_triple, real_index; //��ʱ�洢����ֵ��Ӧ��index 
	u32 max_val, second_min_val;
    u8 ramp_flag;
	magmax = 0; //��ʼ��magmax
	max_index = 0;
    second_index = 0;
    freq_temp = 0;
    freq_max = 0;
	freq_secondmax = 0;
	//������cr4_fft_1024_stm32
	cr4_fft_1024_stm32(fftout, fftin, NPT);	
	
    for(i=1; i<NPT/2; i++)
    {
		X = (fftout[i] << 16) >> 16;	//��16λ��ʵ��
		Y = (fftout[i] >> 16);			//��16λ���鲿
		
		Mag = sqrt(X * X + Y * Y); 		//����ģֵ
		FFT_Mag[i] = Mag;					//���뻺�棬����������飬type:u32
		//��ȡ���Ƶ�ʷ��������ֵ
		if(Mag > magmax)
		{
			magmax = Mag;
			F_temp = i;//����ֵ��Ӧ��index
		}
    }
	//F=(u32)(F_temp  * (fre  / NPT ));	//FFT����ʵ��Ƶ��f=����i*(����Ƶ��fre/�ܲ�������N)		
	//fre = 300,000, NPT = 1024, F_temp = 410�� Ƶ�ʷֱ���146.484
	for( i = 0; i <20; i++) {			
		freq_temp = FFT_Mag[freq_index[i]];
		if(freq_temp > freq_max) {
			freq_max = freq_temp;
			max_index = i;
		}					
	}
	F = freq_table[max_index];
	aF_max = FFT_Mag[freq_index[max_index]];//��ȡ�����ķ�ֵ
		for( i = 0; i <20; i++) {			
			freq_temp = FFT_Mag[freq_index[i]];
			if(freq_temp > freq_secondmax && freq_temp != freq_max) {
				freq_secondmax = freq_temp;
				second_index = i;
			}					
	}
	F_secondmax = freq_table[second_index];//�������飬�ڶ�����Ӧ��Ƶ��
	aF_secondmax = FFT_Mag[freq_index[second_index]];//�ڶ�������Ƶ�ʵķ�ֵ
    compare_val(F, F_secondmax, &max_val, &second_min_val);
	//���з�ֵ�ĵ�����ʵ�ʵ����Ƶ����max_val, 
	//F�ķ�ֵ��ӦaF_max, F_secondmax��Ӧ����aF_secondmax
	if(F < max_val){
		//aF_max = temp;
		temp = aF_max;
		aF_max = aF_secondmax;
		aF_secondmax = temp;

        //����ʵ��Ƶ�ʵ�ָ��
		//max_index = temp;
		temp = max_index;
		max_index = second_index;
		second_index = max_index;
	}
	F = max_val;//Maxval�洢��Ӧ�����Ƶ��
	F_secondmax = second_min_val;//����ʵ�ʵ�Ƶ��
	
	ramp_flag = signal_detection(F, F_secondmax);//����B��A��Ƶ��,Fa < Fb, ����Ƿ������ǲ�

	LCD_ShowNum(260,180, F, 6, 16);//��ʾ��õ�Ƶ��
	LCD_ShowString(5,8,200,16,16,"A: ");
	LCD_ShowNum(15,8, F_secondmax,6, 16);			
}

/* �����ж����ǲ�
* �����A��B����Ϊ��Ƶ�� �͸�Ƶ
* ���ǲ������˻���������3��5��7��г����������3��г������Ϊ����������1/9.
* ȷ����F�����Ƶ����F_secondmax �󣬼����������г���� ����10K, �������30K�ķ�ֵ, �ǲ�������1/9, n
* �������һ��10k�����ǲ�, ��30K���Ҳ�����Ƶ)�������ڵ�30K�ķ���Ӧ�ñ�10K�ĸߣ��ߴ�Լ1/3
*/

u16 signal_detection(u32 F_max, u32 F_secondmax) {
	
	u32 F_temp, F_temp_2, F_triple, real_index; //��ʱ�洢����ֵ��Ӧ��index 
	u32 index_gaps, F_triple_index;
	u16 i, index_max;
	u32 temp_max;
	temp_max = 0;
	
	if(F_max < F_secondmax) {
		return 0;
	}
	/*�ж�Ƶ�����ֵ��ʲô����*/
	//����F_max�����ǲ��Ļ�Ƶ	
	F_triple = F_max * 3; //�鿴F_max������г��
	//����F_triple��F(�źŵ����Ƶ)��Ƶ�ʵĲ�ֵ���������ǵļ������, fre/1024�ǲ����ֱ��ʣ�F_triple - F
	index_gaps = (F_triple - F_max) / (fre / 1024);
	real_index = freq_index[max_index];//�ҳ����Ƶʵ�ʵ�index
	F_triple_index = real_index + index_gaps - 1;//����г����Ӧ��index
	//��һ�����ҵ�����г����Ӧindex�ķ�Χ��ȥ��������ƽ��ֵ
	if(F_triple_index == 0x1ff) {
			LCD_ShowString(260,128,200,16,16,"    ");
			LCD_ShowString(260,128,200,16,16,"Sin");
			wave_type_b = 1;//ǿ����ΪB·�ź�������
			
	}
	// �ڷ�Χ�ڲ��ҷ������ֵ
	for(i = F_triple_index - 3; i <= F_triple_index + 3; i++) {
		if(FFT_Mag[i] > temp_max) {
			temp_max = FFT_Mag[i];//����Ӧ�����ķ�ֵ��temp_max
			index_max = i; //�洢����Ƶ�����ķ�ֵ��index
		}
	}
	aF_triple = temp_max;//����г���ķ�ֵ
	//aF_triple = FFT_Mag[F_triple_index];//�ҳ�����г���ķ�ֵ
	//�����ǲ��Ļ�Ƶ�����Ƶʱ���������Ƶ���ص������
	//����г��Ƶ�ʲ����ڵڶ�������Ƶ��ʱ

		if((1 * aF_triple <= aF_max) && (15 * aF_triple) >= aF_max ) {
			LCD_ShowString(260,128,200,16,16,"Ramp");
			//LCD_ShowString(100,13,200,16,16,"Ramp");
			wave_type_b = 2; //��ȷ��B·�ź�������
	
		}
		else {
			LCD_ShowString(260,128,200,16,16,"    ");
			LCD_ShowString(260,128,200,16,16,"Sin");
			wave_type_b = 1;//ȷ��B·�ź�������
		
		}
	

/* �жϴθ�Ƶ�Ĳ���*/
	temp_max = 0;
	//����F_secondmax�����ǲ��Ļ�Ƶ	
	F_triple = F_secondmax * 3; //�鿴F_max������г��
	//����F_triple��F(�źŵ����Ƶ)��Ƶ�ʵĲ�ֵ���������ǵļ������, fre/1024�ǲ����ֱ��ʣ�F_triple - F
	index_gaps = (F_triple - F_secondmax) / (fre / 1024);
	real_index = freq_index[second_index];//�ҳ��θ�Ƶʵ�ʵ�index
	F_triple_index = real_index + index_gaps - 1;//����г����Ӧ��index
	//��һ�����ҵ�����г����Ӧindex�ķ�Χ��ȥ�����������ֵ
	for(i = F_triple_index - 3; i <= F_triple_index + 3; i++) {
		if(FFT_Mag[i] > temp_max) {
			temp_max = FFT_Mag[i];//����Ӧ�����ķ�ֵ��temp_max
			index_max = i; //�洢����Ƶ�����ķ�ֵ��index
		}
	}

	aF_triple = temp_max;//����г���ķ��ȸ�ֵ��aF_triple

	//����г���ķ�ֵ�ǻ�Ƶ��1/9, �Ҵ�ʱ����г�����ǵڶ��������źŵ�Ƶ��,��Ҫ��������Ϊ��Ƶ���źŻ������г���ķ�����һ����ǿ
	if(F_triple != F_max) {
		if((1 * aF_triple <= aF_secondmax) && (15 * aF_triple) >= aF_secondmax ) {
			//LCD_ShowString(260,128,200,16,16,"Ramp");
			LCD_ShowString(70,8,200,16,16,"Ramp");
			wave_type_a = 2;	//ȷ��A·�ź�������
		}
		//����
		else {
	
			LCD_ShowString(70,8,200,16,16,"     ");
			LCD_ShowString(70,8,200,16,16,"Sin");
			wave_type_a = 1;//ȷ��A·������
	
		}
	}
	
	//����г����ڶ�������Ƶ����ȵ������ aF_second = 9/10 aF_max, 10aF_second = 9aF_max
	if(F_triple == F_max) {
		if((7 * (int) aF_secondmax - 9 * (int) aF_max <= 0) && (10 * (int) aF_secondmax - 10 * (int )aF_max  >= 0)) {
			LCD_ShowString(70,8,200,16,16,"Ramp");
			//LCD_ShowString(100,13,200,16,16,"Ramp");	
			wave_type_a = 2;
		}
	}
   	
}

/*�������ֵ�ʹ�Сֵ */
void compare_val(u32 F1, u32 F2, u32* max_val, u32* second_min_val) {
    if (F1 >= F2) {
        *max_val = F1;
        *second_min_val = F2;
    } else {
        *max_val = F2;
        *second_min_val = F1;
    }
}

/******************************************************************
��飺DMA�ж�������������һ�Σ�����1024�Σ���
	  ������洢��adcx[]���������У��ȴ��������ݴ���
*******************************************************************/	
void DMA1_Channel1_IRQHandler(void) 
{
	if(DMA_GetITStatus(DMA1_IT_TC1)!=RESET)
	{
		adcx[t]=currentadc;
		t++;
		if(t==NPT)
		{
			t=0;
			adc_flag=1;//��ʾ�ɼ���
			DMA_Cmd(DMA1_Channel1, DISABLE);        //ʧ��DMA
		}
	}
	DMA_ClearITPendingBit(DMA1_IT_TC1);
}



/******************************************************************
��飺�����ⲿ�ж����ڰ����Ķ�ȡ
WK_UP�������Ʋ�����ʾ�ĸ��º���ͣ
KEY1�������Ͳ���Ƶ��
KEY0�������Ӳ���Ƶ��
*******************************************************************/
void EXTI0_IRQHandler(void)
{
	delay_ms(10);//����
	if(WK_UP==1)	 	 //WK_UP����
	{	
		BEEP=1;
		delay_ms(50);
		BEEP=0;
		show_flag=!show_flag;
		
		POINT_COLOR=MAGENTA;
		if(show_flag)
			LCD_ShowString(260,128,200,16,16,"ing...");
		else
			LCD_ShowString(260,128,200,16,16,"stop");
	}
	EXTI_ClearITPendingBit(EXTI_Line0); //���LINE0�ϵ��жϱ�־λ  
}
 
void EXTI3_IRQHandler(void)
{
	delay_ms(10);//����
	if(KEY1==0)	 //����KEY1
	{	
		BEEP=1;
		delay_ms(50);
		BEEP=0;
		pre=pre+5;
		if(pre>72)
		{
			pre=1;
		}
		TIM_PrescalerConfig(TIM2,pre-1,TIM_PSCReloadMode_Immediate);
	}		 
	EXTI_ClearITPendingBit(EXTI_Line3);  //���LINE3�ϵ��жϱ�־λ  
}

void EXTI4_IRQHandler(void)
{
	delay_ms(10);//����
	if(KEY0==0)	 //����KEY0
	{
		BEEP=1;
		delay_ms(50);
		BEEP=0;
		pre = pre-5;
		if(pre <= 1)
		{
			pre=1;
		}
		TIM_PrescalerConfig(TIM2,pre-1,TIM_PSCReloadMode_Immediate);
	}		 
	EXTI_ClearITPendingBit(EXTI_Line4);  //���LINE4�ϵ��жϱ�־λ  
}
