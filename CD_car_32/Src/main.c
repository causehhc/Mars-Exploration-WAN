/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFLEN 10
typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t z;
}Control; //上位机控制指�???????
typedef struct {
    float x;
    float y;
    float z;
}Move; //处理后�?�度

typedef struct {
    int A;
    int B;
    int C;
    int D;
}Target; //目标�???????

typedef struct {
    int A;
    int B;
    int C;
    int D;
}Encoder; //编码器�??
typedef struct {
    int A;
    int B;
    int C;
    int D;
}Motor; //电机�???????

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RxBuffer; //串口接收
Control order; //上位机指�???????
Move mov; //XYZ目标速度
Target tgt; //电机目标�???????
Encoder enc; //编码器�??
Motor motr; //电机PWM�???????
float Velocity_KP=10,Velocity_KI=10; //速度控制PID参数

uint16_t usartCnt = 0;
uint16_t beforeCnt = 0;
uint8_t checkTime = 0;

//int befor_count=0; //1s以前的串口完整传输次�?
//int now_count=0;    //1s以后的串口完整传输次�?
//int time=0;
//int flag=0;
//
//int ir_A4=0;    //红外三个标志位
//int ir_B0=0;
//int ir_B1=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void flash_ENC(){
    enc.A = (short)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    enc.B = (short)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    enc.C = (short)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    enc.D = (short)__HAL_TIM_GET_COUNTER(&htim5);
    __HAL_TIM_SET_COUNTER(&htim5, 0);

}
void order_To_Move(){
    mov.x = (float )(order.x-100);
    mov.y = (float )(order.y-100);
    mov.z = (float )(order.z-100);
    mov.x = -mov.x;
//    mov.y = -mov.y;
    mov.z = mov.z;
}
void check_ir(){
    if(!HAL_GPIO_ReadPin(BZ1_GPIO_Port, BZ1_Pin)){  //左
        if(mov.x > 0){
            mov.x = 0;
        }
    }
    if(!HAL_GPIO_ReadPin(BZ2_GPIO_Port, BZ2_Pin)){  //左
        if(mov.x < 0){
            mov.x = 0;
        }
    }
    if(!HAL_GPIO_ReadPin(BZ3_GPIO_Port, BZ3_Pin)){  //后
        if(mov.y < 0){
            mov.y = 0;
        }
    }
}
void check_timeout(){
    if(checkTime++ == 25){
        checkTime = 0;
        if(beforeCnt == usartCnt){
            order.x = 0x64;
            order.y = 0x64;
            order.z = 0x64;
        } else{
            beforeCnt = usartCnt;
        }
    }
}
void Kinematic_Analysis_4() {
    float wide = (float) 0.18652, lenth = (float) 0.1525;
    tgt.A = (int) (mov.y - mov.x + mov.z * (wide + lenth));
    tgt.B = (int) (mov.y + mov.x - mov.z * (wide + lenth));
    tgt.C = (int) (mov.y - mov.x - mov.z * (wide + lenth));
    tgt.D = (int) (mov.y + mov.x + mov.z * (wide + lenth));

    //dir chaenge
    tgt.A=-tgt.A;
    tgt.D=-tgt.D;
}
int Incremental_PI_A(int Encoder,int Target){
    static int Bias,Pwm,Last_bias;
    Bias=Target-Encoder;
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>7200)Pwm=7200;
    if(Pwm<-7200)Pwm=-7200;
    Last_bias=Bias;
    return Pwm;
}
int Incremental_PI_B(int Encoder,int Target){
    static int Bias,Pwm,Last_bias;
    Bias=Target-Encoder;
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>7200)Pwm=7200;
    if(Pwm<-7200)Pwm=-7200;
    Last_bias=Bias;
    return Pwm;
}
int Incremental_PI_C(int Encoder,int Target){
    static int Bias,Pwm,Last_bias;
    Bias=Target-Encoder;
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>7200)Pwm=7200;
    if(Pwm<-7200)Pwm=-7200;
    Last_bias=Bias;
    return Pwm;
}
int Incremental_PI_D(int Encoder,int Target){
    static int Bias,Pwm,Last_bias;
    Bias=Target-Encoder;
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>7200)Pwm=7200;
    if(Pwm<-7200)Pwm=-7200;
    Last_bias=Bias;
    return Pwm;
}
void range_Pwm(int amplitude){
    if(motr.A<-amplitude) motr.A=-amplitude;
    if(motr.A>amplitude)  motr.A=amplitude;
    if(motr.B<-amplitude) motr.B=-amplitude;
    if(motr.B>amplitude)  motr.B=amplitude;
    if(motr.C<-amplitude) motr.C=-amplitude;
    if(motr.C>amplitude)  motr.C=amplitude;
    if(motr.D<-amplitude) motr.D=-amplitude;
    if(motr.D>amplitude)  motr.D=amplitude;
}
void Set_Pwm(){
    if(motr.A>0){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 7200);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 7200-motr.A);
    }else{
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 7200+motr.A);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 7200);
    }

    if(motr.B>0){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 7200);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 7200-motr.B);
    }else{
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 7200+motr.B);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 7200);
    }

    if(motr.C>0){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 7200);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 7200-motr.C);
    }
    else{
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 7200+motr.C);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 7200);
    }

    if(motr.D>0){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 7200);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 7200-motr.D);
    }else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 7200+motr.D);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 7200);
    }
}
void init_allPara(){
    RxBuffer=0; //串口接收
    order.x=100; //上位机指�???????
    order.y=100;
    order.z=100;

    mov.x=0; //XYZ目标速度
    mov.y=0;
    mov.z=0;

    tgt.A=0;
    tgt.B=0;
    tgt.C=0;
    tgt.D=0;

    enc.A=0;
    enc.B=0;
    enc.C=0;
    enc.D=0;

    motr.A=0;
    motr.B=0;
    motr.C=0;
    motr.D=0;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM8_Init();
    MX_USART2_UART_Init();
    MX_TIM6_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1|TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1|TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1|TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);

    HAL_TIM_Base_Start_IT(&htim6);								//base timer
    HAL_UART_Receive_IT(&huart2, &RxBuffer, 1);
    init_allPara();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */


    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ //串口2接收中断
    if(huart == &huart2){
        static uint8_t flag, i, receive[BUFLEN];
        if(RxBuffer==0xf0) flag = 1; // 开始采集标记位
        if(RxBuffer==0xfa) flag = 2; //结束采集标记

        if(flag==1){ //采集数据
            receive[i++] = RxBuffer;
            if(i==10){
                memset(receive, 0, sizeof(uint8_t) * BUFLEN);
                flag = 0;
            }
        }
        if(flag==2){ //分析数据
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //翻转LED
            HAL_UART_Transmit(&huart2, receive, sizeof(uint8_t) * 10, 100); //发送至串口
            //数据更新
            order.x = receive[1];
            order.y = receive[2];
            order.z = receive[3];
            if(usartCnt++ == 60000) usartCnt=0;
            // 重置数据
            flag = 0;
            i = 0;
            memset(receive, 0, sizeof(uint8_t) * BUFLEN);
        }
        HAL_UART_Receive_IT(&huart2, &RxBuffer, 1); //重新使能串口接收中断
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //10ms控制�???????�???????
    if (htim->Instance == htim6.Instance){
        flash_ENC(); //刷新编码器
        check_timeout(); //检查信号传输状态（周期1s）
        order_To_Move(); //串口-计算数据转换
        check_ir(); //检查红外避障
        Kinematic_Analysis_4(); //运动学解算
        motr.A=Incremental_PI_A(enc.A, tgt.A); //A轮PI控制
        motr.B=Incremental_PI_B(enc.B, tgt.B); //B轮PI控制
        motr.C=Incremental_PI_C(enc.C, tgt.C); //C轮PI控制
        motr.D=Incremental_PI_D(enc.D, tgt.D); //D轮PI控制
        range_Pwm(7000); //PWM限制幅度
        Set_Pwm(); //PWM赋值
        if(!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) { //DEBUG
//            printf("%d\t%d\t%d\n", tgt.A, motr.A, enc.A);
//            order.x = 100;
//            order.y = 0x6e;
//            order.z = 100;
//            printf("%d\t%d\t%d\n",order.x,order.y,order.z);
//            printf("%f\t%f\t%f\n", mov.x, mov.y, mov.z);
//            printf("%d\t%d\t%d\t%d\n",tgt.A,tgt.B,tgt.C,tgt.D);
//            printf("%d\t%d\t%d\t%d\n",motr.A,motr.B,motr.C,motr.D);
        }

    }
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
