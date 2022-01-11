/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "uart_buf_l4.h"
#include "string.h"
#include "fatfs_sd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
#define  MainBuf_SIZE 		(2<<14)//13->8192 14->16384 16->65536
#define precision 3  //precision for decimal digits

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
__IO ITStatus UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */
char StringBuf[MainBuf_SIZE];
const uint8_t STOP_REQUEST[2]={START_FLAG_1,STOP_RPL};
const uint8_t RESET_REQUEST[2]={START_FLAG_1,RESET_RPL};
const uint8_t SCAN_REQUEST[2]={START_FLAG_1,SCAN_RPL};
const uint8_t SCAN_DESCRIPTOR[7]={START_FLAG_1,START_FLAG_2,0x05,0x00,0x00,0x40,0x81};
const uint8_t GET_HEALTH_REQUEST[2]={START_FLAG_1,GET_HEALTH_RPL};
const uint8_t GET_HEALTH_DESCRIPTOR[7]={START_FLAG_1,START_FLAG_2,0x03,0x00,0x00,0x00,0x06};
const uint8_t GET_INFO_REQUEST[2]={START_FLAG_1,GET_INFO_RPL};
const uint8_t GET_INFO_DESCRIPTOR[7]={START_FLAG_1,START_FLAG_2,0x4,0x00,0x00,0x00,0x04};
const uint8_t EXPRESS_SCAN_REQUEST[7]={START_FLAG_1,EXPRESS_SCAN_RPL};

const uint8_t FORCE_SCAN_REQUEST[7]={START_FLAG_1,FORCE_SCAN_RPL};

/*Variables involucradas para la SD*/
FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

#define BUFFER_SIZE 128
char buffer[BUFFER_SIZE];  // to store strings..
int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void clear_buffer (void)
{
	for (int i=0; i<BUFFER_SIZE; i++) buffer[i] = '\0';
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //Button PB0
  if (GPIO_Pin == GPIO_PIN_4)
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
void float_to_char(float f, char *p) {
	int a,b,c,k,l=0,m,i=0;
	// check for negetive float
	if(f<0.0)
	{
		p[i++]='-';
		f*=-1;
	}
	a=f;	// extracting whole number
	f-=a;	// extracting decimal part
	k = precision;
	// number of digits in whole number
	while(k>-1)
	{
		l = pow(10,k);
		m = a/l;
		if(m>0)
		{
			break;
		}
	k--;
	}
	// number of digits in whole number are k+1
	/*
	extracting most significant digit i.e. right most digit , and concatenating to string
	obtained as quotient by dividing number by 10^k where k = (number of digit -1)
	*/
	for(l=k+1;l>0;l--)
	{
		b = pow(10,l-1);
		c = a/b;
		p[i++]=c+48;
		a%=b;
	}
	p[i++] = '.';
	/* extracting decimal digits till precision */
	for(l=0;l<precision;l++)
	{
		f*=10.0;
		b = f;
		p[i++]=b+48;
		f-=b;
	}
	p[i]='\0';
}
void setMotorDutyCycle(float duty){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(3);
	if (duty!=0){
		//El ARR tiene como valor máximo 3400@80Mhz
		TIM1->CCR1 = (uint32_t)duty*34;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_Delay(3);

	}
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
	//Esperamos a recibir la data basura
	while(UART1buf_peek()<0);
	//Delay >20ms para poder recibir toda la data basura
	HAL_Delay(50);
	//Limpiamos la data basura
	UART1buf_flushRx();
}
void SAVE_SCAN_DATA(){
	//Define variables
	uint16_t temp;
	float angle,distance,x,y;
	char ascii_chars[30];
	//Primero para crear el archivo en donde almacenaremos la data, debemos eliminar el existente
	f_unlink("/data.csv");
	//Ahora lo creamos
	while(f_open(&fil, "data.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE)!= FR_OK);
	//Escribimos la primera línea
	strcpy (buffer, "Data a almacenar [x,y]:\n");
	while(f_write(&fil, buffer, bufsize(buffer), &bw)!= FR_OK);
	//Ahora encendemos el motor
	setMotorDutyCycle(60);
	//Limpiamos el buffer de Recepción
	UART1buf_flushRx();
	//Enviamos el comando por uart
	UART1buf_putn(SCAN_REQUEST,2);
	//Comenzamos a recibir los primeros 7 valores y verificamos si hay error
	for (int i=0;i<7;i++){
		while(UART1buf_peek()<0);
		if (SCAN_DESCRIPTOR[i]!=UART1buf_getc()){
			LPUART1buf_puts((char*)"Error\nA5-5A-05-00-00-40-81\n\r");
			//Enviar nuevamente el comando

			//Analizar si está en protección
			while(1);
		}
	}
	//Procedemos a recibir la data del RPLIDAR hasta obtener el punto de inicio de SCAN
	while(1){
		while(UART1buf_peek()<0);
		/****Decodificamos la bandera Flag S****/
		if ((UART1buf_peek()&0x03)==(0x01)){
			//Leemos el quality
			UART1buf_getc();
			/****Decodificamos el checkbit****/
			while(UART1buf_peek()<0);
			if (UART1buf_peek()&0x01){
				/****Decodificamos el Angle****/
				//Desplazamos los bits
				temp=(UART1buf_getc()>>1)&0x7F;
				while(UART1buf_peek()<0);
				temp|=(UART1buf_getc()<<7);
				angle=(float)temp/64.0;
				angle=angle*M_PI/180.0;
				/****Decodificamos la distancia****/
				//Desplazamos los bits
				while(UART1buf_peek()<0);
				temp=UART1buf_getc();
				while(UART1buf_peek()<0);
				temp|=(UART1buf_getc()<<8);
				distance=(float)temp/4.0;
				//Procedemos a convertir en coordenadas cartesianas
				x=distance*cosf(angle+M_PI_2);
				y=distance*sinf(angle+M_PI_2);
				//Guardamos los valores en StringBuf
				float_to_char(x,StringBuf);
				strcat(StringBuf,",");
				float_to_char(y,ascii_chars);
				strcat(StringBuf,ascii_chars);
				strcat(StringBuf,"\n");
				break;
			}else{
				//Es dato errado
				for (int i=0;i<3;i++){
					UART1buf_getc();
					while(UART1buf_peek()<0);
				}
				UART1buf_getc();
			}
		}else{
			//No sirve este dato
			for (int i=0;i<4;i++){
				UART1buf_getc();
				while(UART1buf_peek()<0);
			}
			UART1buf_getc();
		}
	}
	//Luego de tener el punto de inicio de SCAN, entramos en un bucle
	//que solo se detendrá cuando se presione el botón Azul
	while(1){
		//Escribimos en el buffer principal el nuevo punto
		while(UART1buf_peek()<0);
		if ((UART1buf_peek()&0x03)==0x02){
			UART1buf_getc();
			/****Decodificamos el checkbit****/
			while(UART1buf_peek()<0);
			if (UART1buf_peek()&0x01){
				/****Decodificamos el Angle****/
				//Desplazamos los bits
				temp=(UART1buf_getc()>>1)&0x7F;
				while(UART1buf_peek()<0);
				temp|=(UART1buf_getc()<<7);
				angle=(float)temp/64.0;
				angle=angle*M_PI/180.0;
				/****Decodificamos la distancia****/
				//Desplazamos los bits
				while(UART1buf_peek()<0);
				temp=UART1buf_getc();
				while(UART1buf_peek()<0);
				temp|=(UART1buf_getc()<<8);
				distance=(float)temp/4.0;
				//Procedemos a convertir en coordenadas cartesianas
				x=distance*cosf(angle+M_PI_2);
				y=distance*sinf(angle+M_PI_2);
				float_to_char(x,ascii_chars);
				strcat(StringBuf,ascii_chars);
				strcat(StringBuf,",");
				float_to_char(y,ascii_chars);
				strcat(StringBuf,ascii_chars);
				strcat(StringBuf,"\n");
			}else	{
				UART1buf_getc();
				while(UART1buf_peek()<0);
				UART1buf_getc();
				while(UART1buf_peek()<0);
				UART1buf_getc();
				while(UART1buf_peek()<0);
				UART1buf_getc();
			}
		//Verificamos que no sea el punto de un nuevo SCAN
		}else if((UART1buf_peek()&0x03)==0x01){
			//Leemos el quality
			UART1buf_getc();
			/****Decodificamos el checkbit****/
			while(UART1buf_peek()<0);
			if (UART1buf_peek()&0x01){
				//Es un nuevo SCAN, tenemos que enviar la data que se tiene hasta el momento
				//en el buffer a la SD
				//Procedemos a guardar la data en la SD
				while(f_lseek(&fil, f_size(&fil))!= FR_OK);
				f_puts(StringBuf, &fil);
				//Volvemos a repetir la operación, pero debemos sincronizar nuevamente la data
				//para ello, esperamos a que el byte recibido sea una nuevo SCAN
				UART1buf_flushRx();
				while(1){
					while(UART1buf_peek()<0);
					if ((UART1buf_peek()&0x03)==0x01){
						UART1buf_getc();
						if (UART1buf_peek()&0x01){
							//Se verifica el checkbit, estamos frente a un nuevo SCAN
							break;
						}
					}else
						UART1buf_getc();
				}
				temp=(UART1buf_getc()>>1)&0x7F;
				while(UART1buf_peek()<0);
				temp|=(UART1buf_getc()<<7);
				angle=(float)temp/64.0;
				angle=angle*M_PI/180.0;
				/****Decodificamos la distancia****/
				//Desplazamos los bits
				while(UART1buf_peek()<0);
				temp=UART1buf_getc();
				while(UART1buf_peek()<0);
				temp|=(UART1buf_getc()<<8);
				distance=(float)temp/4.0;
				//Procedemos a convertir en coordenadas cartesianas
				x=distance*cosf(angle+M_PI_2);
				y=distance*sinf(angle+M_PI_2);
				//Guardamos los valores en StringBuf desde el inicio
				float_to_char(x,StringBuf);
				strcat(StringBuf,",");
				float_to_char(y,ascii_chars);
				strcat(StringBuf,ascii_chars);
				strcat(StringBuf,"\n");
			}else{
				//Es dato errado
				for (int i=0;i<3;i++){
					UART1buf_getc();
					while(UART1buf_peek()<0);
				}
				UART1buf_getc();
				//Punto no válido
				strcat(StringBuf,"Inf,Inf\n");
			}
		}else{
			//Dato errado
			for (int i=0;i<4;i++){
				UART1buf_getc();
				while(UART1buf_peek()<0);
			}
			UART1buf_getc();
			//Punto no válido
			strcat(StringBuf,"Inf,Inf\n");
		}
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)){
			//Cerramos el archivo
			f_close(&fil);
			//Salimos del while
			UserButtonStatus=0;
			break;
		}
	}
	/* Unmount SDCARD */
	while(f_mount(NULL, "/", 1)!= FR_OK);
	LPUART1buf_puts ("SD CARD UNMOUNTED successfully, puedes retirar la tarjeta\n\r");
	//Mandamos el comando de STOP
	UART1buf_putn(STOP_REQUEST, 2);
	//Detenemos el motor
	setMotorDutyCycle(0);
	HAL_Delay(2);
	//Borramos data del buffer
	UART1buf_flushRx();
	//Enviamos "fin de SAVE_SCAN_DATA"
	LPUART1buf_puts ("Fin de la operación\n\r");
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  //Inicializamos los UARTs
  LPUART1buf_init(115200,SERIAL_8N1,0);
  UART1buf_init(256000,SERIAL_8N1,0);
  //Mount SD card
  LPUART1buf_puts("Intentando mounted SD CARD...\n\r");
  HAL_Delay (500);
  while (f_mount(&fs,"/", 1) != FR_OK);
  LPUART1buf_puts("SD CARD mounted successfully...\n\r");
  /*************** Card capacity details ********************/
  /* Check free space */
  while(f_getfree("", &fre_clust, &pfs)!= FR_OK);
  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
  sprintf (buffer,"SD CARD Total Size: \t%lu\n\r",total);
  LPUART1buf_puts(buffer);
  clear_buffer();
  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
  sprintf (buffer, "SD CARD Free Space: \t%lu\n\r",free_space);
  LPUART1buf_puts(buffer);
  clear_buffer();
  //Enviamos el comando de RESET
  SEND_RESET_REQUEST();//DESCOMENTAR LUEGO DE TEMRINAR CON EL SD
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Esperamos a que se presione el Boton
	  LPUART1buf_puts((char*)"Iniciamos:\n\r");
	  while(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4));
	  //Esperamos que se suelte
	  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4));
	  UserButtonStatus=0;
	  LPUART1buf_puts((char*)"Mandamos SCAN request:\n\r");
	  SAVE_SCAN_DATA();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Period = 3400-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
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
