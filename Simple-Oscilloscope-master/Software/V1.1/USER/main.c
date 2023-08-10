/**********************************************************
简易示波器，PA6采样输入
ADC(TIME触发)->DMA
PA4通过DAC通道1输出正弦波，频率由TIME3分频值控制
PA5通过DAC通道2输出三角波或噪声，频率由TIME4控制
KEY_WAKE控制更新与暂停
KEY0增加采样频率
KEY1降低采样频率
定时器4触发DAC通道2输出三角波和噪声
定时器3产生中断DAC通道1输出正弦波
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
#define NPT 1024 //采样次数
#define PI2 6.28318530717959

void window(void);//主界面
void clear_point(u16 mode);//更新显示屏当前列
void InitBufInArray(void);//正弦波输出缓存
void sinout(void);//正弦波输出
void GetPowerMag(void);//FFT变换，输出频率

int long fftin [NPT];//FFT输入
int long fftout[NPT];//FFT输出
u32 FFT_Mag[NPT/2]={0};//幅频特性
u16 magout[NPT];//模拟正弦波输出缓存区
u16 freq_index[20] = {9, 17, 26, 34, 43, 51, 60, 68, 77, 85, 94, 102, 111, 119, 128,137, 145, 154, 162, 171};//5K-100K频率的index
u32 freq_table[20] = {5000, 10000, 15000, 20000, 25000, 30000, 35000,40000, 45000, 50000, 55000, 60000, 65000, 70000, 75000,
80000, 85000, 90000, 95000,100000};
u16 table[15] ={16,32,48,64,80,96,112,128,144,160,176,192,208,224,240};//标点横坐标
u16 currentadc;//实时采样数据
u16 adcx[NPT];//adc数值缓存
u32 adcmax;//采样最大值和最小值
u32 adcmin;
u8 adc_flag=0;//采样结束标志
u8 key_flag=0;//按键扫描标志
u8 show_flag=1;//更新暂停标志
u16 T= 10 - 1;//定时器2重载值，不能小于PWM的Pluse值
u16 pre= 6 - 1;//定时器2预分频值
u32 fre;//采样频率 kHz
u32 F;//波形频率
u32 aF_max, aF_secondmax, aF_triple;//最高频，次高频对应的幅值, 三次谐波对应的幅值
u16 max_index, second_index, freq_temp, freq_max, freq_secondmax;
u16 V=660;//纵坐标单位刻度 mv/div
u32 temp = 0;//幅值最大的频率成分
u16 t = 0;
u16 key;//按键值
u16 signal_detection(u32 F_max, u32 F_secondmax);
u8 wave_type_a, wave_type_b;//设置波形的形状
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
	TIM2_PWM_Init(T, pre);	//最大频率72000000/20/12 = 300KHz
	LCD_Clear(BLACK);
	window();
	while(1)
	{
		//等待采样完成
		while(adc_flag==0)
		{
			LED1=!LED1;
			delay_ms(100);
		}
		adc_flag=0;
		
		//获取最大最小值
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
		adcmax=adcmax*0.8;   //0.8 ≈ 3300/4096	
		adcmin=adcmin*0.8;		
		// if(show_flag==1)
		// {
		// 	clear_point(1);    //更新采样频率，更新显示屏当前列，采用连线绘制
		// }	
		
		LED0=!LED0;
		DMA_Cmd(DMA1_Channel1,ENABLE);//使能DMA1-CH1
		//delay_ms(100);
	}
}

/**********************************************************
简介：主界面绘制
***********************************************************/
void window(void)
{
	u16 x,i;
	static u16 h; 
	
	POINT_COLOR=GREEN;	  
	POINT_COLOR=GRAY;

}

/******************************************************************
函数名称:clear_point()
函数功能:循环更新波形
参数说明:mode 波形模式选择 1——连线模式，0——打点模式
备    注:波形的显示可采用打点方式和绘制线方式
*******************************************************************/
void clear_point(u16 mode)
{
	u16 x,i,past_vol,pre_vol;
	static u16 h; 
	
	POINT_COLOR=BLUE;
	fre = 72000000 / (T+1) / (pre + 1) / 2;//更新采样频率, 时钟频率72M/重装载值20/分频系数12 = 300Khz
	
		if( fre > 10000) { 
		temp = fre / 1000;   //若采样率大于10K，使用khz显示
	}
	else {

	}

}

/******************************************************************
函数名称:GetPowerMag()
函数功能:计算各次谐波幅值
参数说明:
备　　注:先将lBufOutArray分解成实部(X)和虚部(Y)，然后计算幅值(sqrt(X*X+Y*Y)

*******************************************************************/
void GetPowerMag(void)
{
    float X,Y,Mag,magmax;//实部，虚部，各频率幅值，最大幅值
    u16 i;
	u16 index_gaps, F_triple_index;
	u32 F_temp, F_temp_2, F_secondmax, F_triple, real_index; //临时存储最大幅值对应的index 
	u32 max_val, second_min_val;
    u8 ramp_flag;
	magmax = 0; //初始化magmax
	max_index = 0;
    second_index = 0;
    freq_temp = 0;
    freq_max = 0;
	freq_secondmax = 0;
	//调用自cr4_fft_1024_stm32
	cr4_fft_1024_stm32(fftout, fftin, NPT);	
	
    for(i=1; i<NPT/2; i++)
    {
		X = (fftout[i] << 16) >> 16;	//低16位存实部
		Y = (fftout[i] >> 16);			//高16位存虚部
		
		Mag = sqrt(X * X + Y * Y); 		//计算模值
		FFT_Mag[i] = Mag;					//存入缓存，用于输出查验，type:u32
		//获取最大频率分量及其幅值
		if(Mag > magmax)
		{
			magmax = Mag;
			F_temp = i;//最大幅值对应的index
		}
    }
	//F=(u32)(F_temp  * (fre  / NPT ));	//FFT所得实际频率f=点数i*(采样频率fre/总采样点数N)		
	//fre = 300,000, NPT = 1024, F_temp = 410， 频率分辨率146.484
	for( i = 0; i <20; i++) {			
		freq_temp = FFT_Mag[freq_index[i]];
		if(freq_temp > freq_max) {
			freq_max = freq_temp;
			max_index = i;
		}					
	}
	F = freq_table[max_index];
	aF_max = FFT_Mag[freq_index[max_index]];//获取基波的幅值
		for( i = 0; i <20; i++) {			
			freq_temp = FFT_Mag[freq_index[i]];
			if(freq_temp > freq_secondmax && freq_temp != freq_max) {
				freq_secondmax = freq_temp;
				second_index = i;
			}					
	}
	F_secondmax = freq_table[second_index];//查找数组，第二个对应的频率
	aF_secondmax = FFT_Mag[freq_index[second_index]];//第二个正弦频率的幅值
    compare_val(F, F_secondmax, &max_val, &second_min_val);
	//进行幅值的调换，实际的最大频率是max_val, 
	//F的幅值对应aF_max, F_secondmax对应的是aF_secondmax
	if(F < max_val){
		//aF_max = temp;
		temp = aF_max;
		aF_max = aF_secondmax;
		aF_secondmax = temp;

        //调换实际频率的指数
		//max_index = temp;
		temp = max_index;
		max_index = second_index;
		second_index = max_index;
	}
	F = max_val;//Maxval存储对应的最高频率
	F_secondmax = second_min_val;//调换实际的频率
	
	ramp_flag = signal_detection(F, F_secondmax);//输入B和A的频率,Fa < Fb, 检测是否有三角波

	LCD_ShowNum(260,180, F, 6, 16);//显示测得的频率
	LCD_ShowString(5,8,200,16,16,"A: ");
	LCD_ShowNum(15,8, F_secondmax,6, 16);			
}

/* 用于判断三角波
* 输入的A和B定义为低频， 和高频
* 三角波：除了基波，还有3，5，7次谐波分量，但3次谐波分量为基波分量的1/9.
* 确认了F（最大频）和F_secondmax 后，检查他的三次谐波， 比如10K, 检查它的30K的幅值, 是不是它的1/9, n
* 如果输入一个10k的三角波, 和30K正弦波（基频)，那现在的30K的幅度应该比10K的高，高大约1/3
*/

u16 signal_detection(u32 F_max, u32 F_secondmax) {
	
	u32 F_temp, F_temp_2, F_triple, real_index; //临时存储最大幅值对应的index 
	u32 index_gaps, F_triple_index;
	u16 i, index_max;
	u32 temp_max;
	temp_max = 0;
	
	if(F_max < F_secondmax) {
		return 0;
	}
	/*判断频率最高值是什么波形*/
	//假设F_max是三角波的基频	
	F_triple = F_max * 3; //查看F_max的三次谐波
	//计算F_triple与F(信号的最高频)的频率的差值，计算他们的间隔点数, fre/1024是采样分辨率，F_triple - F
	index_gaps = (F_triple - F_max) / (fre / 1024);
	real_index = freq_index[max_index];//找出最高频实际的index
	F_triple_index = real_index + index_gaps - 1;//三次谐波对应的index
	//下一步，找到三次谐波对应index的范围，去这个区间的平均值
	if(F_triple_index == 0x1ff) {
			LCD_ShowString(260,128,200,16,16,"    ");
			LCD_ShowString(260,128,200,16,16,"Sin");
			wave_type_b = 1;//强制认为B路信号是正弦
			
	}
	// 在范围内查找幅度最大值
	for(i = F_triple_index - 3; i <= F_triple_index + 3; i++) {
		if(FFT_Mag[i] > temp_max) {
			temp_max = FFT_Mag[i];//将对应的最大的幅值给temp_max
			index_max = i; //存储三倍频处最大的幅值的index
		}
	}
	aF_triple = temp_max;//三次谐波的幅值
	//aF_triple = FFT_Mag[F_triple_index];//找出三次谐波的幅值
	//当三角波的基频是最高频时，不会出现频率重叠的情况
	//三次谐波频率不等于第二个正弦频率时

		if((1 * aF_triple <= aF_max) && (15 * aF_triple) >= aF_max ) {
			LCD_ShowString(260,128,200,16,16,"Ramp");
			//LCD_ShowString(100,13,200,16,16,"Ramp");
			wave_type_b = 2; //定确定B路信号是三角
	
		}
		else {
			LCD_ShowString(260,128,200,16,16,"    ");
			LCD_ShowString(260,128,200,16,16,"Sin");
			wave_type_b = 1;//确定B路信号是正弦
		
		}
	

/* 判断次高频的波形*/
	temp_max = 0;
	//假设F_secondmax是三角波的基频	
	F_triple = F_secondmax * 3; //查看F_max的三次谐波
	//计算F_triple与F(信号的最高频)的频率的差值，计算他们的间隔点数, fre/1024是采样分辨率，F_triple - F
	index_gaps = (F_triple - F_secondmax) / (fre / 1024);
	real_index = freq_index[second_index];//找出次高频实际的index
	F_triple_index = real_index + index_gaps - 1;//三次谐波对应的index
	//下一步，找到三次谐波对应index的范围，去这个区间的最大值
	for(i = F_triple_index - 3; i <= F_triple_index + 3; i++) {
		if(FFT_Mag[i] > temp_max) {
			temp_max = FFT_Mag[i];//将对应的最大的幅值给temp_max
			index_max = i; //存储三倍频处最大的幅值的index
		}
	}

	aF_triple = temp_max;//三次谐波的幅度赋值给aF_triple

	//三次谐波的幅值是基频的1/9, 且此时三次谐波不是第二个正弦信号的频率,需要调整，因为高频的信号会对三次谐波的幅度有一定加强
	if(F_triple != F_max) {
		if((1 * aF_triple <= aF_secondmax) && (15 * aF_triple) >= aF_secondmax ) {
			//LCD_ShowString(260,128,200,16,16,"Ramp");
			LCD_ShowString(70,8,200,16,16,"Ramp");
			wave_type_a = 2;	//确定A路信号是三角
		}
		//三次
		else {
	
			LCD_ShowString(70,8,200,16,16,"     ");
			LCD_ShowString(70,8,200,16,16,"Sin");
			wave_type_a = 1;//确定A路是正弦
	
		}
	}
	
	//三次谐波与第二个正弦频率相等的情况， aF_second = 9/10 aF_max, 10aF_second = 9aF_max
	if(F_triple == F_max) {
		if((7 * (int) aF_secondmax - 9 * (int) aF_max <= 0) && (10 * (int) aF_secondmax - 10 * (int )aF_max  >= 0)) {
			LCD_ShowString(70,8,200,16,16,"Ramp");
			//LCD_ShowString(100,13,200,16,16,"Ramp");	
			wave_type_a = 2;
		}
	}
   	
}

/*返回最大值和次小值 */
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
简介：DMA中断用于完整采样一次（采样1024次），
	  并将其存储于adcx[]缓存数组中，等待后续数据处理
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
			adc_flag=1;//表示采集满
			DMA_Cmd(DMA1_Channel1, DISABLE);        //失能DMA
		}
	}
	DMA_ClearITPendingBit(DMA1_IT_TC1);
}



/******************************************************************
简介：三个外部中断用于按键的读取
WK_UP按键控制波形显示的更新和暂停
KEY1按键降低采样频率
KEY0按键增加采样频率
*******************************************************************/
void EXTI0_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(WK_UP==1)	 	 //WK_UP按键
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
	EXTI_ClearITPendingBit(EXTI_Line0); //清除LINE0上的中断标志位  
}
 
void EXTI3_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(KEY1==0)	 //按键KEY1
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
	EXTI_ClearITPendingBit(EXTI_Line3);  //清除LINE3上的中断标志位  
}

void EXTI4_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(KEY0==0)	 //按键KEY0
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
	EXTI_ClearITPendingBit(EXTI_Line4);  //清除LINE4上的中断标志位  
}
