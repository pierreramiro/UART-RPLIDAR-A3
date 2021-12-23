/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*Comandos*/
#define STOP_RPL            0x25
#define RESET_RPL           0x40
#define SCAN_RPL            0x20
#define EXPRESS_SCAN_RPL    0x82
#define FORCE_SCAN_RPL      0x21
#define GET_INFO_RPL        0x50
#define GET_HEALTH_RPL      0x52
#define GET_SAMPLERATE_RPL  0x59
#define GET_LIDAR_CONF_RPL  0x84
/*StarFlags*/
#define START_FLAG_1         0xA5
#define START_FLAG_2         0x5A
/*Tamaños del buffer*/
#define  RxBuf_SIZE 		10
#define  MainBuf_SIZE 		20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
__IO ITStatus UartReady = RESET;
__IO ITStatus UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */
uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */







void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle){
  /* Set transmission flag: transfer complete */
  UartReady = SET;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart->Instance==USART1){
		memcpy (MainBuf,RxBuf,Size);
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart1,RxBuf,Size)
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
  /* Set transmission flag: transfer complete */
  UartReady = SET;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle){
  Error_Handler();
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    UserButtonStatus = 1;
  }
}
uint8_t BinToAsc(uint8_t BinValue)
{
    BinValue &= 0x0F;
    if(BinValue > 9) BinValue += 7;
    return(BinValue + '0');
}

// Reverses a string 'str' of length 'len'
void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}
// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 3);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

void printf_pkt(uint8_t* cmd, uint8_t size){
	uint8_t ascii_chars[size*3];
	for (int i=0;i<size;i++){
		ascii_chars[3*i]=BinToAsc(cmd[i]>>4);
		ascii_chars[3*i+1]=BinToAsc(cmd[i]);
		ascii_chars[3*i+2]='-';
	}
	HAL_UART_Transmit(&hlpuart1, ascii_chars, size*3, 300);
	uint8_t new_line[2]="\n\r";
	HAL_UART_Transmit(&hlpuart1, new_line, 2, 100);
}

void printf_data(uint8_t* pkt){

	uint8_t quality;
	uint16_t angle,distance;
	float result;
	char ascii_chars[30];
	/****Decodificamos el Quality****/
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)"Quality: ", 9, 30);
	//Realizamos el desplazamiento y el bitmasking
	quality=(pkt[1]>>2)&0x3F;
	//Convertimos el uint8_t en ascii
	intToStr((int)quality,ascii_chars,2);
	//Transmitimos por el serial
	HAL_UART_Transmit(&hlpuart1,ascii_chars, 2, 30);
	/****Decodificamos el Angle****/
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)"\tAngle: ", 8, 30);
	//Desplazamos los bits
	angle=(pkt[2]>>1)&0x7F;
	angle|=(pkt[3]<<7);
	//Procedemos a convertir el halfword en ascii
	result=(float)angle/64.0;
	ftoa(result,ascii_chars,3);
	HAL_UART_Transmit(&hlpuart1,ascii_chars, 7, 30);
	/****Decodificamos la distancia****/
	HAL_UART_Transmit(&hlpuart1,(uint8_t*)"\tDistance: ", 11, 30);
	//Desplazamos los bits
	distance=pkt[4];
	distance|=(pkt[4]<<8);
	//Procedemos a convertir el halfword en ascii
	result=(float)distance/4.0;
	ftoa(result,ascii_chars,5);
	HAL_UART_Transmit(&hlpuart1,ascii_chars, 9, 30);
	//Imprimos una nueva línea
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\n\r", 2, 100);
}

void SEND_STOP_REQUEST(){
	//Definimos el comando
	uint8_t cmd[2]={START_FLAG_1,STOP_RPL};
	//Enviamos el comando por uart
	if (HAL_UART_Transmit(&huart1,cmd,2,3) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}
	//Delay >2ms para poder enviar otro request
	HAL_Delay(2);
}
void SEND_RESET_REQUEST(){
	//Definimos el comando
	uint8_t cmd[2]={START_FLAG_1,RESET_RPL};
	//Enviamos el comando por uart
	if (HAL_UART_Transmit(&huart1,cmd,2,3) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}
	//Delay >2ms para poder enviar otro request
	HAL_Delay(2);
}
void SEND_SCAN_REQUEST(){
	//Encendemos el motor
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(900);
	//Definimos el comando
	uint8_t cmd[2]={START_FLAG_1,SCAN_RPL};
	//Definimos el "response descriptor" a recibir
	uint8_t resp_des[7]={START_FLAG_1,START_FLAG_2,0x05,0x00,0x00,0x40,0x81};
	//Definimos el tamaño del buffer de rx
	uint8_t rx_size=7;
	//Definimos el buffer de recepción
	uint8_t rx_buffer[rx_size];
	//Enviamos el comando por uart
	if (HAL_UART_Transmit_IT(&huart1,cmd,2) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}
	//Esperamos que se envie todo
	while (UartReady != SET);
	UartReady = RESET;
	//Comenzamos la recepción del descriptor
	if (HAL_UART_Receive(&huart1,rx_buffer,rx_size,3) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}
	//Verificamos si hubo error
	int count=0;
	for (int i=0;i<7;i++){
		if (rx_buffer[i]!=resp_des[i]) break;
		count++;
	}
	//verificamos si la data llego correctamente
	if (count!=7){
		//Hubo ERROR de transferencia.
		//Imprimimos data enviandolo por el UART del COM7 de la compu
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Error\n\r", 7, 300);
		printf_pkt(resp_des,7);
		printf_pkt(rx_buffer,7);
		while(1);
	}

	/*//Comenzamos la recepción de la data
	if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1,RxBuf,RxBuf_SIZE) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}*/
	//Continuamos recibiendo datos de 5 bytes y lo guardamos en la memoria
	while(1){
		HAL_UART_Receive(&huart1,rx_buffer,5,1);//(uart handle,tx pointer,size, timeout)
		printf_pkt(rx_buffer,5);
	}


	/*
	//Comenzamos la recepción
	if (HAL_UART_Receive_IT(&huart1,rx_buffer,rx_size+99*5) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}
	//Esperamos a completar la recepción
	//while (UartReady != SET);
	//UartReady = RESET;
	//Imprimimos data enviandolo por el UART del COM7 de la compu
	HAL_UART_Transmit(&hlpuart1, (uint8_t*) "Scan Data:\n\r", 12, 300);
	printf_pkt(rx_buffer,rx_size);
	for(unsigned int i=0;i<99*5;i++){
		printf_pkt(&rx_buffer[rx_size+5*i],rx_size-7);
	}
	HAL_UART_Transmit(&hlpuart1, (uint8_t*) "Fin Scan:\n\r", 11, 300);
	*/
}
void SEND_EXPRESS_SCAN_REQUEST(){
	//Definimos el comando
	uint8_t cmd[2]={START_FLAG_1,EXPRESS_SCAN_RPL};
	//Enviamos el comando por uart
	if (HAL_UART_Transmit(&huart1,cmd,2,3) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}
	//Delay >2ms para poder enviar otro request
	HAL_Delay(2);
}
void SEND_FORCE_SCAN_REQUEST(){
	//Definimos el comando
	uint8_t cmd[2]={START_FLAG_1,FORCE_SCAN_RPL};
	//Enviamos el comando por uart
	if (HAL_UART_Transmit(&huart1,cmd,2,3) != HAL_OK){//(uart handle,tx pointer,size, timeout)
	    Error_Handler();
	}
	//Delay >2ms para poder enviar otro request
	HAL_Delay(2);
}
void SEND_GET_INFO_REQUEST(){
	//Definimos el comando
	uint8_t cmd[2]={START_FLAG_1,GET_INFO_RPL};
	//Enviamos el comando por uart
	if (HAL_UART_Transmit(&huart1,cmd,2,3) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}
	//Delay >2ms para poder enviar otro request
	HAL_Delay(2);
}
void SEND_GET_HEALTH_REQUEST(){
	//Definimos el comando
	uint8_t cmd[2]={START_FLAG_1,GET_HEALTH_RPL};
	//Definimos el "response descriptor" a recibir
	uint8_t resp_des[7]={START_FLAG_1,START_FLAG_2,0x03,0x00,0x00,0x00,0x06};
	//Definimos el buffer de recepción
	uint8_t rx_buffer[10];
	//Enviamos el comando por uart
	if (HAL_UART_Transmit_IT(&huart1,cmd,2) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}
	while (UartReady != SET);
	//Reseteamos la bandera
	UartReady = RESET;
	//Verificamos correcta transacción
	HAL_UART_Receive(&huart1,rx_buffer,10,3);//(uart handle,tx pointer,size, timeout)
	int count=0;
	for (int i=0;i<7;i++){
		if (rx_buffer[i]!=resp_des[i]) break;
		count++;
	}
	//Imprimimos data enviandolo por el UART del COM7 de la compu
	if (count==7){
		uint8_t text[]="No error\n\r";
		HAL_UART_Transmit(&hlpuart1, text, 10, 300);
		//HAL_UART_Receive(&huart1,rx_buffer,3,3);//(uart handle,tx pointer,size, timeout)
		printf_pkt(rx_buffer,10);
	}else{
		uint8_t text[]="Error\n\r";
		HAL_UART_Transmit(&hlpuart1, text, 7, 300);
		printf_pkt(resp_des,7);
		//printf_pkt(rx_buffer,7);
		//HAL_UART_Receive(&huart1,rx_buffer,3,3);//(uart handle,tx pointer,size, timeout)
		printf_pkt(rx_buffer,10);
	}
}
void SEND_GET_SAMPLERATE_REQUEST(){
	//Definimos el comando
	uint8_t cmd[2]={START_FLAG_1,GET_SAMPLERATE_RPL};
	//Definimos el "response descriptor" a recibir
	uint8_t resp_des[7]={START_FLAG_1,START_FLAG_2,0x04,0x00,0x00,0x00,0x15};
	//Definimos el tamaño del buffer de recepción
	uint8_t rx_size=11;
	//Definimos el buffer de recepción
	uint8_t rx_buffer[rx_size];
	//Enviamos el comando por uart
	if (HAL_UART_Transmit(&huart1,cmd,2,3) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}
	//Esperamos que se envie todo
	while (UartReady != SET);
	//Reseteamos la bandera
	UartReady = RESET;
	//Comenzamos la recepción
	if (HAL_UART_Receive_IT(&huart1,rx_buffer,rx_size) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}
	//Esperamos a completar la recepción
	while (UartReady != SET);
	UartReady = RESET;
	//Comparamos con la data que deberia recibirse
	int count=0;
	for (int i=0;i<7;i++){
		if (rx_buffer[i]!=resp_des[i]) break;
		count++;
	}
	//Imprimimos data enviandolo por el UART del COM7 de la compu
	if (count==7){
		uint8_t text[]="No error\n\r";
		HAL_UART_Transmit(&hlpuart1, text, 10, 300);
		printf_pkt(rx_buffer,rx_size);
	}else{
		uint8_t text[]="Orror\n\r";
		HAL_UART_Transmit(&hlpuart1, text, 7, 300);
		printf_pkt(resp_des,7);
		printf_pkt(rx_buffer,rx_size);
	}
}
void SEND_GET_LIDAR_CONF_REQUEST(){
	//Definimos el comando
	uint8_t cmd[2]={START_FLAG_1,GET_LIDAR_CONF_RPL};
	//Enviamos el comando por uart
	if (HAL_UART_Transmit_IT(&huart1,cmd,2) != HAL_OK){//(uart handle,tx pointer,size, timeout)
		Error_Handler();
	}
	//Esperamos que se envie todo
	while (UartReady != SET);
	//Reseteamos la bandera
	UartReady = RESET;
	//Delay >2ms para poder enviar otro request
	HAL_Delay(5);
}

void test_func(){
	uint8_t cmd[2]={START_FLAG_1,GET_LIDAR_CONF_RPL};
	printf_pkt(cmd,2);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_LPUART1_UART_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t eraser_reg[70];
  /* USER CODE END 2 */
  SEND_STOP_REQUEST();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_UART_Receive(&huart1, eraser_reg,70 , 4);
	  //Esperamos a que se presione el Boton
	  HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Comando linea:\n\r", 16, 300);
	  while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
	  //Esperamos que se suelte
	  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
	  /*Enviamos el comando GET_HEALTH*/
	  SEND_SCAN_REQUEST();


	  //SEND_GET_HEALTH_REQUEST();
	  /*Verificamos si estamos en Protection STOP*/

	  /*Reseteamos en caso sea necesario*/

	  //go to get_health_request
	  /*Habilitamos el motos*/
	  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	  //HAL_Delay(500);
	  /*Enviamos el comando SCAN*/
	  //SEND_SCAN_REQUEST();

	  /*
	  uint8_t text[]="Comienza trama\n\r";
	  HAL_UART_Transmit(&hlpuart1, text, 16, 300);
	  //SEND_GET_SAMPLERATE_REQUEST();
	  //SEND_RESET_REQUEST();
	  SEND_GET_HEALTH_REQUEST();
	  SEND_SCAN_REQUEST();
	  //while (UserButtonStatus == 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  //HAL_Delay(3000);
	  SEND_SCAN_REQUEST();
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  	  HAL_Delay(6000);
  	  SEND_GET_HEALTH_REQUEST();//ocurre error
  	  //SEND_STOP_REQUEST();
  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  	  HAL_Delay(6000);
  	  SEND_GET_HEALTH_REQUEST();
  	  */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 256000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
  /* Toggle LED2 for error */
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_Delay(200);
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

