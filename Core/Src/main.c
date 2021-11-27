/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//������������

//���⴫��������������
uchar irT1=0, irT2=0, irT3=0, irT4=0, irT5=0; //��������ir���

//��ĳ��ir����
void ir_on(uchar num){
    num -= 1;
    if(num&0x01){
        HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, GPIO_PIN_RESET);
    }
    if(num&0x02){
        HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
    }
    if(num&0x04){
        HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
    }
}

//��ȡ�����־λ
uchar get_ir(uchar num){
    if(num==1)
        return irT1;
    else if(num==2)
        return irT2;
    else if(num==3)
        return irT3;
    else if(num==4)
        return irT4;
    else if(num==5)
        return irT5;
    else 
			return 0;
}

//��ȡ��Է���ĺ�����ֵ
uchar read_ir(uchar xd){
    if(xd == 0)
        return get_ir(3);  // ǰ����
    if(xd == 1)
        return get_ir(5);  // �Һ���
    if(xd == 3)
        return get_ir(1);  // �����
    return 0;
}

//���������������
static uint step = 0;
uchar left[]={0x11,0x33,0x22,0x66,0x44,0xcc,0x88,0x99};//��ת
uchar right[]={0x11,0x99,0x88,0xcc,0x44,0x66,0x22,0x33};//��ת
uchar forward[]={0x11,0x93,0x82,0xc6,0x44,0x6c,0x28,0x39};//ֱ��

void run(uchar temp){
    if(temp&0x01)
        HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, GPIO_PIN_RESET);
    if(temp&0x02)
        HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, GPIO_PIN_RESET);
    if(temp&0x04)
        HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, GPIO_PIN_RESET);
    if(temp&0x08)
        HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, GPIO_PIN_RESET);

    if(temp&0x10)
        HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_RESET);
    if(temp&0x20)
        HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_RESET);
    if(temp&0x40)
        HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_RESET);
    if(temp&0x80)
        HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);
}

// ��ʱ����
void delay(uint z){
    if(z>10)
        run(0);
    HAL_Delay(z);
}

//·������
uchar xz(uchar i){
    step = 0;
    while((get_ir(2)||get_ir(4)) && !get_ir(3)){
        if(get_ir(2))
            run(left[i++] | 0xf0);
        else if(get_ir(4))
            run(right[i++] | 0x0f);
        if(i==8)
            i=0;
        step++;
        delay(3);
    }
    return i;
}

//ǰ��
void qianjin(uchar xd){
    uchar num;
    uchar i, j;
    if(xd == 0)
        num = 102;
    if(xd == 1||xd == 3)
        num = 40;
    if(xd == 2)
        num = 81;
    for(j=0;j<num;j++){
        for(i=0;i<8;i++){
            if(xd == 1||xd == 2)
                run(left[i]);
            if(xd == 3)
                run(right[i]);
            if(xd == 0){
                run(forward[i]);
                i = xz(i);
            }
            if(xd==0)
                num -= step/16;
            delay(1);
        }
    }
}

uchar jd_xd(uchar jd_now, uchar jd_now_t){  // ���Է���->��Է���
    //jd_now����ǰ���Է���
    //jd_now_t��ת������Է���
    uchar xd = 0;
    xd = (jd_now_t - jd_now) % 4;
    if(xd >= 126)
        xd += 4;
    //������Է���
    return xd;
}

//�㷨����
void init_map(uchar map[SIZE][SIZE]){  // ��ʼ���Թ�����
    uchar i, j;
    for(i=0;i<SIZE;i++){
        for(j=0;j<SIZE;j++){
            map[i][j] = 0xff;
        }
    }
}

uchar pd_new(uchar maze[SIZE][SIZE], zb now, uchar jd_now){  // �жϴ˷����Ƿ�Ϊ�µ�
    if(jd_now==0){
        if(now.x == 0){
            return 0;
        }else{
            return (maze[now.x-1][now.y]>>4)==0x0f;
        }
    }
    if(jd_now==1){
        if(now.y == 7){
            return 0;
        }else{
            return (maze[now.x][now.y+1]>>4)==0x0f;
        }
    }
    if(jd_now==2){
        if(now.x == 7){
            return 0;
        }else{
            return (maze[now.x+1][now.y]>>4)==0x0f;
        }
    }
    if(jd_now==3){
        if(now.y == 0){
            return 0;
        }else{
            return (maze[now.x][now.y-1]>>4)==0x0f;
        }
    }
    return 0;
}

void jilu(uchar maze[SIZE][SIZE], zb now, uchar jd_now){ // ��¼��Ϣ
    if(maze[now.x][now.y] == 0xff){  //д������
        uchar val_wall = 0xf0;
        uchar k = 0;
        uchar i;
        for(i=0; i<4; i++){  //�ж�4�����Է���
            k = read_ir(jd_xd(jd_now, i));
            val_wall |= (k<<i);
        }
        maze[now.x][now.y] &= val_wall;  //������Ϣ��¼�ڵ���λ
        maze[now.x][now.y] &= ((jd_now<<4)|0x0f);  //�����¼�ڸ���λ
    }
}

uchar pd_way(uchar maze[SIZE][SIZE], zb now, uchar jd_now){ //�ж��Ƿ�Ϊ����·��
    return !((maze[now.x][now.y]>>jd_now)&0x01);
}

uchar find_fx(uchar maze[SIZE][SIZE], zb now, uchar flag){ //ѡ����
    uchar i;
    uchar high = maze[now.x][now.y] >> 4;
    uchar j;
    if(!flag){//��̱��λ
        for(i=0; i<4; i++){
            if(pd_way(maze, now, i)){
                if(pd_new(maze, now, i)){
                    return i;
                }
            }
        }
    }
    // �����̻��ĸ����򲻿��ߣ�ֱ�Ӷ�ȡ����λ���г�����������
    if(high<=1)
        j = high+2;
    if(high>=2)
        j = high-2;
    return j;
}

void next(zb *now, uchar *jd_now, uchar jd_now_t){
    uchar xd = jd_xd(*jd_now, jd_now_t);
    // ˢ�µ�ǰ����͵�ǰ���Է���
    if(jd_now_t == 0)
        (now->x)--;
    if(jd_now_t == 1)
        (now->y)++;
    if(jd_now_t == 2)
        (now->x)++;
    if(jd_now_t == 3)
        (now->y)--;
    *jd_now = jd_now_t;
    // ִ��
    qianjin(xd);
    if(xd != 0)
        qianjin(0);
}

uchar bfs(uchar maze[SIZE][SIZE],zb beg, zb end){ // ��������㷨
		uchar i,j;
    uchar height[SIZE][SIZE] = {0xff};  //��ʼ���ȸ߱�
		uchar queue_len = 1;//��ʼ�����г��ȱ����
    zb queue[15];//��ʼ������
    zb head;//��ʼ������
    zb linshi;//��ʼ��һ����ʱ����
    zb now;//��ʼ����ǰλ��

    init_map(height);
    height[beg.x][beg.y] = 0;
    queue[0].x = beg.x;
    queue[0].y = beg.y;
    while(queue_len>0){// �����в�Ϊ��
        head = queue[0];// ����Ԫ�س���
        queue_len--;
        for(j=0; j<queue_len; j++){//����������ģ����У�������Ҫ��Ԫ���ֶ�ǰ��
            queue[j] = queue[j+1];
        }
        for(i=0; i<4; i++){//�ж��ĸ�����
            linshi = head;
            if(i == 0)
                linshi.x = head.x-1;
            if(i == 1)
                linshi.y = head.y+1;
            if(i == 2)
                linshi.x = head.x+1;
            if(i == 3)
                linshi.y = head.y-1;
            if(linshi.x>127||linshi.y>127)
                continue;
            if(pd_way(maze, head, i) && height[linshi.x][linshi.y]==255){  // �����������ͨ�ҵ�һ�η���
                height[linshi.x][linshi.y] = height[head.x][head.y] + 1;  // ������ȸ߱�+1
                queue[queue_len++] = linshi;  // �����������
            }
        }
    }
    // �ȸ߱�����ϣ���ʼ�����������·��
    now.x = end.x;
    now.y = end.y;
    while(!(now.x==0&& now.y==0)){ // �����û�����
        for(i=0; i<4; i++){ // ɨ���ĸ�����
            linshi = now;
            if(i == 0)
                linshi.x = now.x-1;
            if(i == 1)
                linshi.y = now.y+1;
            if(i == 2)
                linshi.x = now.x+1;
            if(i == 3)
                linshi.y = now.y-1;
            if(linshi.x>127||linshi.y>127)
                continue;
            if(pd_way(maze, now, i) && (height[linshi.x][linshi.y]==height[now.x][now.y]-1)){//�����������ͨ�Ҹ߶ȵݼ�
                maze[linshi.x][linshi.y] |= 0xf0;
                maze[linshi.x][linshi.y] &= ((i<<4)|0x0f);
                now = linshi;
                break;
            }
        }
    }
    return 1;
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_Base_Start_IT(&htim6);

    uchar maze[SIZE][SIZE] = {0xff}; //�Թ�����
    uchar jd_now = 1;	//���Է���
    uchar jd_now_t;
    uchar flag = 0;		//��̱��

    zb beg;	//���
    zb now;
    zb end;	//�յ�
		
    beg.x = 0;
    beg.y = 0;
    now = beg;
    end.x = 7;
    end.y = 7;

    init_map(maze);
    delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(now.x==end.x&&now.y==end.y&&flag == 1){
            delay(500);
            break;
        }

        if(now.x==beg.x&&now.y==beg.y&&jd_now==3){
            delay(500);
          flag = bfs(maze, beg, end);
        }

        jilu(maze, now, jd_now);
        jd_now_t = find_fx(maze, now, flag);
        next(&now, &jd_now, jd_now_t);		        //ִ��
        delay(500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == htim6.Instance){ //10ms
        static uint8_t Flag = 0;
        static uint8_t n = 1;
        if(!Flag){
            ir_on(n);
        }else{
            if(n == 5){
                if(HAL_GPIO_ReadPin(IR1_GPIO_Port, IR1_Pin))
                    irT1 = 0;
                else
                    irT1 = 1;
            }else if(n == 4){
                if(HAL_GPIO_ReadPin(IR2_GPIO_Port, IR2_Pin))
                    irT2 = 0;
                else
                    irT2 = 1;
            }else if(n == 3){
                if(HAL_GPIO_ReadPin(IR3_GPIO_Port, IR3_Pin))
                    irT3 = 0;
                else
                    irT3 = 1;
            }else if(n == 2){
                if(HAL_GPIO_ReadPin(IR4_GPIO_Port, IR4_Pin))
                    irT4 = 0;
                else
                    irT4 = 1;
            }else if(n == 1){
                if(HAL_GPIO_ReadPin(IR5_GPIO_Port, IR5_Pin))
                    irT5 = 0;
                else
                    irT5 = 1;
            }
        }
        if(Flag)
            n++;
        if(n > 5)
            n=0;
        Flag = !Flag;
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
