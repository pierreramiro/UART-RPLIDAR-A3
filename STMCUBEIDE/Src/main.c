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
#include "math.h"
#include "uart_buf_g4.h"

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
#define  MainBuf_SIZE 		(2<<13)//8192
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
__IO ITStatus UartReady = RESET;
__IO ITStatus UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */
uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];
const uint8_t STOP_REQUEST[2]={START_FLAG_1,STOP_RPL};
const uint8_t RESET_REQUEST[2]={START_FLAG_1,RESET_RPL};
const uint8_t SCAN_REQUEST[2]={START_FLAG_1,SCAN_RPL};
const uint8_t SCAN_DESCRIPTOR[7]={START_FLAG_1,START_FLAG_2,0x05,0x00,0x00,0x40,0x81};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle){
  /* Set transmission flag: transfer complete */
  UartReady = SET;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart->Instance==USART1){
		//memcpy (MainBuf,RxBuf,Size);
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
uint8_t  BinToAsc(uint8_t  BinValue)
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
void ftoa(float n, char* res, int beforepoint,int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, beforepoint);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * (float)(pow(10, afterpoint));

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

void printf_pkt(uint8_t * cmd, uint8_t size){
	char  ascii_chars[size*3];
	for (int i=0;i<size;i++){
		ascii_chars[3*i]=(char)BinToAsc(cmd[i]>>4);
		ascii_chars[3*i+1]=(char)BinToAsc(cmd[i]);
		ascii_chars[3*i+2]='-';
	}
	ascii_chars[3*size-1]=0;
	LPUART1buf_puts(ascii_chars);
	LPUART1buf_puts((char *) "\n\r");
}

void printf_data(uint8_t* pkt){
	char S,notS,C,quality;
	uint16_t angle,distance;
	float result;
	char ascii_chars[30];
	/****Decodificamos el checkbit****/
	C=pkt[1]&0x01;
	/****Decodificamos la bandera Flag****/
	S=pkt[0]&0x01;
	notS=(pkt[0]>>1)&0x01;
	if ((S^notS)&&(C)){
		LPUART1buf_puts((char*)"S: ");
		//Convertimos el uint8_t en ascii
		intToStr(S,ascii_chars,1);
		//Transmitimos por el serial
		LPUART1buf_puts(ascii_chars);
		/****Decodificamos el Quality****/
		LPUART1buf_puts((char*)"\tQuality: ");
		//Realizamos el desplazamiento y el bitmasking
		quality=(pkt[1]>>2)&0x3F;
		//Convertimos el uint8_t en ascii
		intToStr((int)quality,ascii_chars,2);
		//Transmitimos por el serial
		LPUART1buf_puts(ascii_chars);
		/****Decodificamos el Angle****/
		LPUART1buf_puts((char*)"\tAngle: ");
		//Desplazamos los bits
		angle=(pkt[2]>>1)&0x7F;
		angle|=(pkt[3]<<7);
		//Procedemos a convertir el halfword en ascii
		result=(float)angle/64.0;
		ftoa(result,ascii_chars,3,3);
		LPUART1buf_puts(ascii_chars);
		/****Decodificamos la distancia****/
		LPUART1buf_puts((char*)"\tDistance: ");
		//Desplazamos los bits
		distance=pkt[4];
		distance|=(pkt[4]<<8);
		//Procedemos a convertir la distancia
		result=(float)distance/4.0;
		ftoa(result,ascii_chars,5,3);
		LPUART1buf_puts(ascii_chars);
	}else{
		LPUART1buf_puts((char*)"Data error");
	}
	//Imprimos una nueva línea
	LPUART1buf_puts((char*)"\n\r");
}
void setMotorDutyCycle(float duty){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_Delay(3);
	if (duty!=0){
		//El ARR tiene como valor máximo 6799
		TIM1->CCR2 = duty*68;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_Delay(3);

	}
}
void getRPM(float duty){
	//Definimos las variables
	uint8_t byte,S,notS,C;
	uint32_t tiempo=0;
	int count=0;
	float velocity;
	//Detenemos el motor y establecemos el dutycycle
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_Delay(3);
	TIM1->CCR2 = (uint32_t)(duty*68);//El ARR tiene como valor máximo 6799
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	//Enviamos el comando por uart
	UART1buf_flushRx();
	UART1buf_putn(SCAN_REQUEST,2);
	//Comenzamos a recibir los valores y lo escribimos en el buffer principal.
	for(unsigned int i=0;i<7;i++){
		while(UART1buf_peek()<0);
		UART1buf_getc();
	}
	for(unsigned int i=0;i<1500;i++){
		while(UART1buf_peek()<0);
		byte=UART1buf_getc();
		/****Decodificamos la bandera Flag****/
		S=byte&0x01;
		if (S){
			notS=(byte>>1)&0x01;
			if(notS==0){
				tiempo=tiempo-HAL_GetTick();
				count++;
			}
		}
		for(unsigned int j=0;j<4;j++){
			while(UART1buf_peek()<0);
			UART1buf_getc();
		}
	}
	//Detenemos la trama del SCAN
	UART1buf_putn(STOP_REQUEST, 2);
	UART1buf_flushRx();
	//Calculamos la velocidad
	velocity=60000.0/(float)tiempo;
	char ascii_chars[10];
	//Procedemos a convertir el float en ascii
	ftoa(velocity,ascii_chars,4,3);
	//Imprimimos en pantalla el resultado
	if(count==0){
		LPUART1buf_puts((char*)"Error");
	}else{
		LPUART1buf_puts((char*)"Velocidad: ");
		LPUART1buf_puts(ascii_chars);
	}
	//Imprimos una nueva línea
	LPUART1buf_puts((char*)"\n\r");
}
void SEND_STOP_REQUEST(){
	//Enviamos el comando por uart
	setMotorDutyCycle(0);
	UART1buf_putn(STOP_REQUEST,2);
	//Delay >2ms para poder enviar otro request
	HAL_Delay(2);
}
void SEND_RESET_REQUEST(){
	//Enviamos el comando por uart
	UART1buf_putn(RESET_REQUEST,2);
	//Delay >2ms para poder enviar otro request
	HAL_Delay(2);
	//Esperamos a recibir la data basura
	while(UART1buf_peek()<0);
	//Esperamos a leer/flush la data basura
	while(UART1buf_getc()>0);
	UART1buf_flushRx();
}

void SEND_SCAN_REQUEST(){
	//Encendemos el motor
	setMotorDutyCycle(50);
	//Enviamos el comando por uart
	UART1buf_flushRx();
	UART1buf_putn(SCAN_REQUEST,2);
	//Comenzamos a recibir los valores y lo escribimos en el buffer principal.
	for(unsigned int i=0;i<1600*5+7;i++){
		while(UART1buf_peek()<0);
		MainBuf[i]=UART1buf_getc();
	}
	//Detenemos la trama del SCAN
	UART1buf_putn(STOP_REQUEST, 2);
	UART1buf_flushRx();
	//while(1);//analizamos si el comando STOP funciona.
	//Comparamos con lo que se debe recibir y si es correcto
	for (int i=0;i<7;i++){
		if (SCAN_DESCRIPTOR[i]!=MainBuf[i]){
			LPUART1buf_puts((char*)"Error\nA5-5A-05-00-00-40-81\n\r");
			printf_pkt(MainBuf,7);
			while(1);
		}
	}
	//Decodificamos los valores escaneados
	for(int i=0;i<1440;i++){
		printf_pkt(&MainBuf[5*i+7],5);
		printf_data(&MainBuf[5*i+7]);
	}
}


void SEND_SCAN_REQUEST_ov(){
	//Encendemos el motor
	setMotorDutyCycle(50);
	//Esperamos que se estabilice
	HAL_Delay(900);
	//definimos el arreglo que contendrá los datos
	uint8_t data[7];
	//Enviamos el comando por uart
	UART1buf_putn(SCAN_REQUEST,2);
	//Comenzamos a recibir los valores. Y comparamos con lo que se debe recibir
	uint8_t count=0;
	for (int i=0;i<7;i++){
		//Debemos esperar a que el buffer no este vacío
		while(UART1buf_peek()<0);
		data[i]=UART1buf_getc();
		if (SCAN_DESCRIPTOR[i]!=data[i]) break;
		count++;
	}
	//verificamos si la data llego correctamente
	if (count!=7){
		//Hubo ERROR de transferencia.
		//Imprimimos data enviandolo por el LPUART
		LPUART1buf_puts((char*)"Error\n\r");
		printf_pkt(SCAN_DESCRIPTOR,count);
		printf_pkt(data,count);
		while(1);
	}
	//Continuamos recibiendo obteniendo datos de 5 del buffer y lo imprimimos en pantalla
	while(1){
		for (int i=0;i<5;i++){
			//Debemos esperar a que el buffer no este vacío
			while(UART1buf_peek()<0);
			data[i]=UART1buf_getc();
		}
		printf_pkt(data,5);
		printf_data(data);
	}
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
  //MX_LPUART1_UART_Init();
  //MX_DMA_Init();
  //MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  LPUART1buf_init(115200,SERIAL_8N1,0);
  UART1buf_init(256000,SERIAL_8N1,0);
  //SEND_RESET_REQUEST();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_UART_Receive(&huart1, eraser_reg,70 , 4);
	  //Esperamos a que se presione el Boton
	  LPUART1buf_puts((char*)"Comando linea:\n\r");
	  //HAL_UART_Transmit(&hlpuart1, (uint8_t*)"Comando linea:\n\r", 16, 300);
	  while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
	  //Esperamos que se suelte
	  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
	  /*Enviamos el comando GET_HEALTH*/
	  SEND_SCAN_REQUEST();
	  //getRPM(50);

  	  //SEND_GET_HEALTH_REQUEST();
	  /*Verificamos si estamos en Protection STOP*/

	  /*Reseteamos en caso sea necesario*/

	  //go to get_health_request
	  /*Habilitamos el motos*/
	  //setMotorDutyCycle(50);
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
	  setMotorDutyCycle(50);
	  //HAL_Delay(3000);
	  SEND_SCAN_REQUEST();
	  setMotorDutyCycle(50);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  	  HAL_Delay(6000);
  	  SEND_GET_HEALTH_REQUEST();//ocurre error
  	  //SEND_STOP_REQUEST();
  	  setMotorDutyCycle(50);
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 6799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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

