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
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STOP_RPL            0x25
#define RESET_RPL           0x40
#define SCAN_RPL            0x20
#define FORCE_SCAN_RPL      0x21
#define GET_INFO_RPL        0x50
#define GET_HEALTH_RPL      0x52
#define GET_SAMPLERATE_RPL  0x59
#define GET_LIDAR_CONF_RPL  0x84
/*StarFlags*/
#define START_FLAG_1         0xA5
#define START_FLAG_2         0x5A
/*Tamaños del buffer*/
#define  MainBuf_SIZE 		(1<<11)//13->8192 14->16384 15->32768 16->65536
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
//char MainBuf[MainBuf_SIZE];
char MainBuf[MainBuf_SIZE];
//float ValBuf[4000];
const uint8_t STOP_REQUEST[2]={START_FLAG_1,STOP_RPL};
const uint8_t RESET_REQUEST[2]={START_FLAG_1,RESET_RPL};
const uint8_t SCAN_REQUEST[2]={START_FLAG_1,SCAN_RPL};
const uint8_t SCAN_DESCRIPTOR[7]={START_FLAG_1,START_FLAG_2,0x05,0x00,0x00,0x40,0x81};
const uint8_t EXPRESS_SCAN_REQUEST[9]={START_FLAG_1,0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22};
const uint8_t EXPRESS_SCAN_DESCRIPTOR[7]={START_FLAG_1,START_FLAG_2,0x54,0x00,0x00,0x40,0x82};
const uint8_t GET_HEALTH_REQUEST[2]={START_FLAG_1,GET_HEALTH_RPL};
const uint8_t GET_HEALTH_DESCRIPTOR[7]={START_FLAG_1,START_FLAG_2,0x03,0x00,0x00,0x00,0x06};
const uint8_t GET_INFO_REQUEST[2]={START_FLAG_1,GET_INFO_RPL};
const uint8_t GET_INFO_DESCRIPTOR[7]={START_FLAG_1,START_FLAG_2,0x14,0x00,0x00,0x00,0x04};
const uint8_t GET_SAMPLERATE_REQUEST[2]={START_FLAG_1,GET_SAMPLERATE_RPL};
const uint8_t GET_SAMPLERATE_DESCRIPTOR[7]={START_FLAG_1,START_FLAG_2,0x4,0x00,0x00,0x00,0x15};
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void add_theta_rho_to_StringBuf (char *p,float theta,float rho,unsigned int *pointer_n_chars){
	int int_part,k,ten_pow,temp_int,index=pointer_n_chars[0];
		/*Iniciamos con el x*/
		//verificamos si es negativo
		if(theta<0.0){
			p[index++]='-';
			//lo volvemos positivo
			theta*=-1;
		}
		//Extraemos la parte entera
		int_part=theta;
		//Nos quedamos con los decimales
		theta-=int_part;
		//Definimos el número de dígitos máximos en la parte entera
		k=2;
		while(k>-1){
			ten_pow = pow(10,k);
			temp_int = int_part/ten_pow;
			if(temp_int>0){
				break;
			}
			k--;
		}
		//La cantidad de dígitos en la parte entera es k+1, donde k es el valor obtenido
		//en el bucle anterior
		//Procedemos a guarda el valor de la parte entera de x
		for(int z=k+1;z>0;z--){
			ten_pow = pow(10,z-1);
			//Obtenemos el digito entero más alejado de la coma
			temp_int = int_part/ten_pow ;
			//Lo guardamos en el StringBuf
			p[index++]=temp_int+48;
			//Actualizamos el dígito, obteniendo el residuo de la operación de división
			int_part%=ten_pow;
		}
		//Añadimos el punto decimal
		p[index++] = '.';
		/* Guardamos en el StringBuf los dígitos decimales*/
		for(int z=0;z<precision;z++){
			theta*=10.0;
			int_part = theta;
			p[index++]=int_part+48;
			theta-=int_part ;
		}
		//Añadimos la coma para dar pase al otro número
		p[index++]=',';
		/*Continuamos con la conversión de y a string, es prácticamente repetir lo anterior*/
		//verificamos si es negativo
		if(rho<0.0){
			p[index++]='-';
			//lo volvemos positivo
			rho*=-1;
		}
		//Extraemos la parte entera
		int_part=rho;
		//Nos quedamos con los decimales
		rho-=int_part;
		//Definimos nuevamente el número de dígitos máximos en la parte entera
		k=4;
		while(k>-1){
			ten_pow = pow(10,k);
			temp_int = int_part/ten_pow;
			if(temp_int>0){
				break;
			}
			k--;
		}
		//La cantidad de dígitos en la parte entera es k+1, donde k es el valor obtenido
		//en el bucle anterior
		//Procedemos a guarda el valor de la parte entera de x
		for(int z=k+1;z>0;z--){
			ten_pow = pow(10,z-1);
			//Obtenemos el digito entero más alejado de la coma
			temp_int = int_part/ten_pow ;
			//Lo guardamos en el StringBuf
			p[index++]=temp_int+48;
			//Actualizamos el dígito, obteniendo el residuo de la operación de división
			int_part%=ten_pow;
		}
		//Añadimos el punto decimal
		p[index++] = '.';
		/* Guardamos en el StringBuf los dígitos decimales*/
		for(int z=0;z<precision;z++){
			rho*=10.0;
			int_part = rho;
			p[index++]=int_part+48;
			rho-=int_part ;
		}
		//Añadimos el salto de línea
		p[index++]='\n';
		//Actualizamos el valor de la cantidad de caracteres
		pointer_n_chars[0]=index;
}
void add_xy_to_StringBuf (char *p,float x,float y,unsigned int *pointer_n_chars){
	int int_part,k,ten_pow,temp_int,index=pointer_n_chars[0];
	/*Iniciamos con el x*/
	//verificamos si es negativo
	if(x<0.0){
		p[index++]='-';
		//lo volvemos positivo
		x*=-1;
	}
	//Extraemos la parte entera
	int_part=x;
	//Nos quedamos con los decimales
	x-=int_part;
	//Definimos el número de dígitos máximos en la parte entera
	//(en este caso k=4 para definir 5 digitos válidos. Ya que mas distance es 25000mm)
	k=4;
	while(k>-1){
		ten_pow = pow(10,k);
		temp_int = int_part/ten_pow;
		if(temp_int>0){
			break;
		}
		k--;
	}
	//La cantidad de dígitos en la parte entera es k+1, donde k es el valor obtenido
	//en el bucle anterior
	//Procedemos a guarda el valor de la parte entera de x
	for(int z=k+1;z>0;z--){
		ten_pow = pow(10,z-1);
		//Obtenemos el digito entero más alejado de la coma
		temp_int = int_part/ten_pow ;
		//Lo guardamos en el StringBuf
		p[index++]=temp_int+48;
		//Actualizamos el dígito, obteniendo el residuo de la operación de división
		int_part%=ten_pow;
	}
	//Añadimos el punto decimal
	p[index++] = '.';
	/* Guardamos en el StringBuf los dígitos decimales*/
	for(int z=0;z<precision;z++){
		x*=10.0;
		int_part = x;
		p[index++]=int_part+48;
		x-=int_part ;
	}
	//Añadimos la coma para dar pase al otro número
	p[index++]=',';
	/*Continuamos con la conversión de y a string, es prácticamente repetir lo anterior*/
	//verificamos si es negativo
	if(y<0.0){
		p[index++]='-';
		//lo volvemos positivo
		y*=-1;
	}
	//Extraemos la parte entera
	int_part=y;
	//Nos quedamos con los decimales
	y-=int_part;
	//Definimos nuevamente el número de dígitos máximos en la parte entera
	k=4;
	while(k>-1){
		ten_pow = pow(10,k);
		temp_int = int_part/ten_pow;
		if(temp_int>0){
			break;
		}
		k--;
	}
	//La cantidad de dígitos en la parte entera es k+1, donde k es el valor obtenido
	//en el bucle anterior
	//Procedemos a guarda el valor de la parte entera de x
	for(int z=k+1;z>0;z--){
		ten_pow = pow(10,z-1);
		//Obtenemos el digito entero más alejado de la coma
		temp_int = int_part/ten_pow ;
		//Lo guardamos en el StringBuf
		p[index++]=temp_int+48;
		//Actualizamos el dígito, obteniendo el residuo de la operación de división
		int_part%=ten_pow;
	}
	//Añadimos el punto decimal
	p[index++] = '.';
	/* Guardamos en el StringBuf los dígitos decimales*/
	for(int z=0;z<precision;z++){
		y*=10.0;
		int_part = y;
		p[index++]=int_part+48;
		y-=int_part ;
	}
	//Añadimos el salto de línea
	p[index++]='\n';
	//Actualizamos el valor de la cantidad de caracteres
	pointer_n_chars[0]=index;
}





void setMotorDutyCycle(float duty){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(3);
	if (duty!=0){
		//El ARR tiene como valor máximo 3200@80Mhz
		TIM1->CCR1 = (uint32_t)duty*32;
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
void SEND_GET_SAMPLERATE(){
	unsigned int temp;
	char charBuf[40];
	//Enviamos el comando por uart
	LPUART1buf_puts("Enviando comando GET_SAMPLERATE\n\r");
	UART1buf_putn(GET_SAMPLERATE_REQUEST,2);
	//Comenzamos a recibir los primeros 7 valores y verificamos si hay error
	for (int i=0;i<7;i++){
		while(UART1buf_peek()<0);
		if (GET_SAMPLERATE_DESCRIPTOR[i]!=UART1buf_getc()){
			LPUART1buf_puts((char*)"Error\n");
			//Enviar nuevamente el comando

			//Analizar si está en protección
			while(1);
		}
	}
	//Recibimos el T standard
	while(UART1buf_peek()<0);
	temp=UART1buf_getc();
	while(UART1buf_peek()<0);
	temp=(UART1buf_getc()<<8)|temp;
	//enviamos el Tstandar
	sprintf(charBuf,"Tiempo standard: %d us\n\r",temp);
	//Recibimos el T express
	LPUART1buf_puts(charBuf);
	while(UART1buf_peek()<0);
	temp=UART1buf_getc();
	while(UART1buf_peek()<0);
	temp=(UART1buf_getc()<<8)|temp;
	//enviamos el Texpress
	sprintf(charBuf,"Tiempo express: %d us\n\r",temp);
	LPUART1buf_puts(charBuf);
	HAL_Delay(50);
}
void SEND_GET_HEALTH(){
	unsigned int temp;
	char charBuf[40];
	//Enviamos el comando por uart
	LPUART1buf_puts("Enviando comando GET_HEALTH\n\r");
	UART1buf_putn(GET_HEALTH_REQUEST,2);
	//Comenzamos a recibir los primeros 7 valores y verificamos si hay error
	for (int i=0;i<7;i++){
		while(UART1buf_peek()<0);
		if (GET_HEALTH_DESCRIPTOR[i]!=UART1buf_getc()){
			LPUART1buf_puts((char*)"Error\n");
			//Enviar nuevamente el comando

			//Analizar si está en protección
			while(1);
		}
	}
	//Recibimos el status
	while(UART1buf_peek()<0);
	temp=UART1buf_getc();
	if (temp==0){
		LPUART1buf_puts("Todo en orden!\n\r");
		while(UART1buf_peek()<0);
		UART1buf_getc();
		while(UART1buf_peek()<0);
		UART1buf_getc();
	}
	//Warning
	else if (temp==1){
		LPUART1buf_puts("Advertencia: ");
		while(UART1buf_peek()<0);
		temp=UART1buf_getc();
		while(UART1buf_peek()<0);
		temp=(UART1buf_getc()<<8)|temp;
		sprintf(charBuf,"code %d\n\r",temp);
		LPUART1buf_puts(charBuf);

	}
	//
	else{
		LPUART1buf_puts("Error: ");
		while(UART1buf_peek()<0);
		temp=UART1buf_getc();
		while(UART1buf_peek()<0);
		temp=(UART1buf_getc()<<8)|temp;
		sprintf(charBuf,"code %d\n\r",temp);
		LPUART1buf_puts(charBuf);
	}
	HAL_Delay(50);
}
void SEND_GET_LIDAR_CONF(){
	/*Realizaremos varios request*/
	uint8_t charBuf[40];
	/***************************/
	//Enviamos el comando por uart
	LPUART1buf_puts("RPLIDAR_CONF_SCAN_MODE_TYPICAL\n\r");
	charBuf[0]=START_FLAG_1;
	charBuf[1]=0x84;
	charBuf[2]=0x04;//tamaño
	charBuf[3]=0x00;//type
	charBuf[4]=0x00;//type
	charBuf[5]=0x00;//type
	charBuf[6]=0x7C;//type
	charBuf[7]=0x59;//checksum
	UART1buf_putn(charBuf,8);
	//Definimos el descriptor y el tipo
	charBuf[0]=START_FLAG_1;
	charBuf[1]=START_FLAG_2;
	charBuf[2]=0x04;//tamaño del request
	charBuf[3]=0x00;
	charBuf[4]=0x00;
	charBuf[5]=0x00;
	charBuf[6]=0x20;
	charBuf[7]=0x00;//type
	charBuf[8]=0x00;//type
	charBuf[9]=0x00;//type
	charBuf[10]=0x7C;//type

	//Comenzamos a recibir los primeros 7+4 valores y verificamos si hay error
	for (int i=0;i<11;i++){
		while(UART1buf_peek()<0);
		if (charBuf[i]!=UART1buf_getc()){
			LPUART1buf_puts((char*)"Error 0\n");
			//Enviar nuevamente el comando

			//Analizar si está en protección
			while(1);
		}
	}
	LPUART1buf_puts("No value received\n\r");
	//while(UART1buf_peek()<0);
	//temp=UART1buf_getc();
	//while(UART1buf_peek()<0);
	//temp=(UART1buf_getc()<<8)|temp;
	//sprintf(charBuf,"ID: %d \n\r",temp);
	//LPUART1buf_puts(charBuf);
	HAL_Delay(50);
	/***************************/
	//Enviamos el comando por uart
	LPUART1buf_puts("RPLIDAR_CONF_SCAN_MODE_COUNT\n\r");
	charBuf[0]=START_FLAG_1;
	charBuf[1]=0x84;
	charBuf[2]=0x04;//tamaño
	charBuf[3]=0x00;//type
	charBuf[4]=0x00;//type
	charBuf[5]=0x00;//type
	charBuf[6]=0x70;//type
	charBuf[7]=0x55;//checksum
	UART1buf_putn(charBuf,8);
	//Definimos el descriptor y el tipo
	charBuf[0]=START_FLAG_1;
	charBuf[1]=START_FLAG_2;
	charBuf[2]=0x04;//tamaño response
	charBuf[3]=0x00;
	charBuf[4]=0x00;
	charBuf[5]=0x00;
	charBuf[6]=0x20;
	charBuf[7]=0x00;//type
	charBuf[8]=0x00;//type
	charBuf[9]=0x00;//type
	charBuf[10]=0x70;//type

	//Comenzamos a recibir los primeros 7+4 valores y verificamos si hay error
	for (int i=0;i<11;i++){
		while(UART1buf_peek()<0);
		if (charBuf[i]!=UART1buf_getc()){
			LPUART1buf_puts((char*)"Error 1\n");
			//Enviar nuevamente el comando

			//Analizar si está en protección
			while(1);
		}
	}
	LPUART1buf_puts("No value received\n\r");
	//LPUART1buf_puts("N. modos: 0");
	//while(UART1buf_peek()<0);
	//temp=UART1buf_getc();
	//while(UART1buf_peek()<0);
	//temp=(UART1buf_getc()<<8)|temp;
	//sprintf(charBuf,"%d\n\r",temp-1);
	//LPUART1buf_puts(charBuf);
	HAL_Delay(50);
	/***************************/
	//Enviamos el comando por uart
	LPUART1buf_puts("RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE\n\r");
	charBuf[0]=START_FLAG_1;
	charBuf[1]=0x84;
	charBuf[2]=0x06;//tamaño
	charBuf[3]=0x00;//type
	charBuf[4]=0x00;//type
	charBuf[5]=0x00;//type
	charBuf[6]=0x71;//type
	charBuf[7]=0x00;//payload
	charBuf[8]=0x00;//payload
	charBuf[9]=0x56;//checksum
	UART1buf_putn(charBuf,10);
	//Definimos el descriptor y el tipo
	charBuf[0]=START_FLAG_1;
	charBuf[1]=START_FLAG_2;
	charBuf[2]=0x04;//tamaño del request
	charBuf[3]=0x00;
	charBuf[4]=0x00;
	charBuf[5]=0x00;
	charBuf[6]=0x20;
	charBuf[7]=0x00;//type
	charBuf[8]=0x00;//type
	charBuf[9]=0x00;//type
	charBuf[10]=0x71;//type

	//Comenzamos a recibir los primeros 7+4 valores y verificamos si hay error
	for (int i=0;i<11;i++){
		while(UART1buf_peek()<0);
		if (charBuf[i]!=UART1buf_getc()){
			LPUART1buf_puts((char*)"Error 2\n");
			//Enviar nuevamente el comando

			//Analizar si está en protección
			while(1);
		}
	}
	LPUART1buf_puts("No value received\n\r...\n\r");
	//while(UART1buf_peek()<0);
	//temp=UART1buf_getc();
	//while(UART1buf_peek()<0);
	//temp=(UART1buf_getc()<<8)|temp;
	//while(UART1buf_peek()<0);
	//temp=(UART1buf_getc()<<8)|temp;
	//while(UART1buf_peek()<0);
	//temp=(UART1buf_getc()<<8)|temp;
	//sprintf(charBuf,"%d \n\r",temp);
	//LPUART1buf_puts(charBuf);
	HAL_Delay(50);
}
uint8_t  BinToAsc(uint8_t  BinValue)
{
	//tomamos los 4 bits menos significativos
    BinValue &= 0x0F;
    //hacemos la conversion
    if(BinValue > 9) BinValue += 7;
    return(BinValue + '0');
}
void SEND_GET_INFO(){
	unsigned int temp,temp_aux_1;
	char charBuf[40];
	//Enviamos el comando por uart
	LPUART1buf_puts("Enviando comando GET_INFO\n\r");
	UART1buf_putn(GET_INFO_REQUEST,2);
	//Comenzamos a recibir los primeros 7 valores y verificamos si hay error
	for (int i=0;i<7;i++){
		while(UART1buf_peek()<0);
		if (GET_INFO_DESCRIPTOR[i]!=UART1buf_getc()){
			LPUART1buf_puts((char*)"Error\n");
			//Enviar nuevamente el comando

			//Analizar si está en protección
			while(1);
		}
	}
	//Recibimos el modelo
	while(UART1buf_peek()<0);
	temp=UART1buf_getc();
	sprintf(charBuf,"modelo: %d\n\r",temp);
	LPUART1buf_puts(charBuf);
	while(UART1buf_peek()<0);
	temp_aux_1=UART1buf_getc();
	while(UART1buf_peek()<0);
	temp=UART1buf_getc();
	sprintf(charBuf,"firmware: %d.%d\n\r",temp,temp_aux_1);
	LPUART1buf_puts(charBuf);
	while(UART1buf_peek()<0);
	temp=UART1buf_getc();
	sprintf(charBuf,"hardware: %d\n\r",temp);
	LPUART1buf_puts(charBuf);
	LPUART1buf_puts("Serial number: ");
	for (int z=0;z<16;z++){
		while(UART1buf_peek()<0);
		temp=UART1buf_getc();
		charBuf[z*2+0]=BinToAsc((temp>>4));
		charBuf[z*2+1]=BinToAsc(temp);
	}
	charBuf[32]='\0';
	LPUART1buf_puts(charBuf);
	LPUART1buf_puts("\n\r");
	HAL_Delay(50);
}
//
//void SEND_EXPRESS_SCAN(){
//	uint8_t sync,ChkSum_lidar,ChkSum;
//	uint8_t dtheta_1,dtheta_2;
//	uint16_t start_angle_q6,dist_1,dist_2;
//	//bool S;
//	float wi,wip1,angle,distance;//x,y;
//	//Ahora encendemos el motor
//	setMotorDutyCycle(60);
//	UART1buf_flushRx();
//	//Enviamos el comando por uart
//	UART1buf_putn(EXPRESS_SCAN_REQUEST,9);
//	//Comenzamos a recibir los primeros 7 valores y verificamos si hay error
//	for (int i=0;i<7;i++){
//		while(UART1buf_peek()<0);
//		if (EXPRESS_SCAN_DESCRIPTOR[i]!=UART1buf_getc()){
//			LPUART1buf_puts((char*)"Error EXPRESS SCAN\n");
//			//Enviar nuevamente el comando
//
//			//Analizar si está en protección
//			while(1);
//		}
//	}
//	//Procedemos a decodificar un paquete de 32 muestras del LIDAR
//	ChkSum=0;
//	while(UART1buf_peek()<0);
//	ChkSum^=UART1buf_peek();
//	sync=(UART1buf_peek()&0xF0);
//	ChkSum_lidar=UART1buf_getc()&0x0F;
//	while(UART1buf_peek()<0);
//	ChkSum^=UART1buf_peek();
//	sync|=(UART1buf_peek()>>4);
//	ChkSum_lidar|=(UART1buf_getc()<<4);
//	while(UART1buf_peek()<0);
//	ChkSum^=UART1buf_peek();
//	start_angle_q6=UART1buf_getc();
//	while(UART1buf_peek()<0);
//	ChkSum^=UART1buf_peek();
//	S=UART1buf_peek()>>7;
//	start_angle_q6|=(UART1buf_getc()<<8)&0x7F;
//	wi=(float)start_angle_q6/64.0;
//	//Debemos analizar la siguiente banderaif (S)
//	if (sync!=0xA5){
//		//Procedemos a recibir las 16 cabinas copn 32 muestras
//		for (int k=0;k<16;k++){
//			while(UART1buf_peek()<0);
//			ChkSum^=UART1buf_peek();
//			dtheta_1=((UART1buf_peek())<<4);
//			dist_1=(UART1buf_getc()>>2);
//			while(UART1buf_peek()<0);
//			ChkSum^=UART1buf_peek();
//			dist_1|=(UART1buf_getc()>>6);
//			while(UART1buf_peek()<0);
//			ChkSum^=UART1buf_peek();
//			dtheta_2=((UART1buf_peek())<<4);
//			dist_2=(UART1buf_getc()>>2);
//			while(UART1buf_peek()<0);
//			ChkSum^=UART1buf_peek();
//			dist_2|=(UART1buf_getc()>>6);
//			while(UART1buf_peek()<0);
//			ChkSum^=UART1buf_peek();
//			dtheta_1|=UART1buf_peek()&0x0F;
//			dtheta_2|=(UART1buf_getc()>>4);
//			ValBuf[k*4+0]=dtheta_1;
//			ValBuf[k*4+1]=dist_1;
//			ValBuf[k*4+2]=dtheta_2;
//			ValBuf[k*4+3]=dist_2;
//		}
//		//Chequeamos si el Checksum es correcto.
//
//	}else{
//		//error comunicacion
//		while(1);
//	}
//	//Para poder procesar necesitamos el ángulo del siguiente estado wi+1
//	while(1){
//		ChkSum=0;
//		while(UART1buf_peek()<0);
//		ChkSum^=UART1buf_peek();
//		sync=(UART1buf_peek()&0xF0);
//		ChkSum_lidar=UART1buf_getc()&0x0F;
//		while(UART1buf_peek()<0);
//		ChkSum^=UART1buf_peek();
//		sync|=(UART1buf_peek()>>4);
//		ChkSum_lidar|=(UART1buf_getc()<<4);
//		while(UART1buf_peek()<0);
//		ChkSum^=UART1buf_peek();
//		start_angle_q6=UART1buf_getc();
//		while(UART1buf_peek()<0);
//		ChkSum^=UART1buf_peek();
//		S=UART1buf_peek()>>7;
//		start_angle_q6|=(UART1buf_getc()<<8)&0x7F;
//		wip1=(float)start_angle_q6/64.0;
//		//Debemos analizar sync,ChkSum y S
//
//		//...Continuar aqui. Falta decodificar la data con el AngleDiff
//
//		//if (butonpressed){
//		//	break
//		//}
//	}
//
//
//
//
//}
//
/***************************************************************/
/** @brief Proponemos usar un buffer que almacene los valores
 * float de x e y. Para luego, de tener un SCAN. Enviar los datos
 * a la SD. El envío a la tarjeta SD será cada 100 ms aprox, es
 * decir. Luego de completar un SCAN (que se verifica con el S
 * flag), enviamos archivos.
 *
 *
 */
//oid SAVE_DATA_bufdouble(){
//	//Define variables
//	float angle,distance,x,y;
//	uint8_t C,S,first_point_set;
//	unsigned int temp,index=0,wrong_points=0;
//	uint8_t chars_buf[40];
//	//Primero para crear el archivo en donde almacenaremos la data, debemos eliminar el existente
//	f_unlink("/data.csv");
//	//Ahora lo creamos
//	while(f_open(&fil, "data.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE)!= FR_OK);
//	//Escribimos la primera línea
//	while(f_write(&fil, "Data a almacenar [x,y]:\n",sizeof("Data a almacenar [x,y]:\n") , &bw)!= FR_OK);
//	//Ahora encendemos el motor
//	setMotorDutyCycle(60);
//	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//	HAL_Delay(70);
//	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//	//Limpiamos el buffer de Recepción
//	UART1buf_flushRx();
//	//Enviamos el comando por uart
//	UART1buf_putn(SCAN_REQUEST,2);
//	//Comenzamos a recibir los primeros 7 valores y verificamos si hay error
//	for (int i=0;i<7;i++){
//		while(UART1buf_peek()<0);
//		if (SCAN_DESCRIPTOR[i]!=UART1buf_getc()){
//			LPUART1buf_puts((char*)"Error\nA5-5A-05-00-00-40-81\n\r");
//			//Enviar nuevamente el comando
//
//			//Analizar si está en protección
//			while(1);
//		}
//	}
//	//Procedemos a recibir la data del RPLIDAR hasta obtener el punto de inicio de SCAN y guardamos en SD
//	while(1){
//		while(UART1buf_peek()<0);
//		/****Decodificamos la bandera Flag****/
//		S=UART1buf_getc()&0x03;
//		if (S==0x01){
//			/****Decodificamos el checkbit****/
//			while(UART1buf_peek()<0);
//			chars_buf[1]=
//			C=UART1buf_peek()&0x01;
//			if (C){
//				/****Decodificamos el Angle****/
//				//Desplazamos los bits
//				temp=(UART1buf_getc()>>1);
//				temp|=(UART1buf_getc()<<7);
//				angle=(float)temp/64.0;
//				angle=angle*M_PI/180.0;
//				/****Decodificamos la distancia****/
//				//Desplazamos los bits
//				temp=UART1buf_getc();
//				temp|=(UART1buf_getc()<<8);
//				distance=(float)temp/4.0;
//				///Procedemos a convertir en coordenadas cartesianas
//				ValBuf[index*2+0]=distance*cosf(angle+M_PI_2);
//				ValBuf[index*2+1]=distance*sinf(angle+M_PI_2);
//				index++;
//				first_point_set=false;
//				break;
//			}
//			else{
//				//No sirve dato
//				while(UART1buf_peek()<0);
//				UART1buf_getc();
//				while(UART1buf_peek()<0);
//				UART1buf_getc();
//				while(UART1buf_peek()<0);
//				UART1buf_getc();
//				while(UART1buf_peek()<0);
//				UART1buf_getc();
//			}
//		}else{
//			//No sirve dato
//			for (int i=0;i<4;i++){
//				while(UART1buf_peek()<0);
//				UART1buf_getc();
//			}
//		}
//	}
//	while(1){
//		while(UART1buf_peek()<0);
//		/****Decodificamos la bandera Flag****/
//		S=UART1buf_getc()&0x03;
//		if ((S==(0x01))||(S==(0x02))){
//			if (S==(0x01)){
//				first_point_set=true;
//			}
//			/****Decodificamos el checkbit****/
//			while(UART1buf_peek()<0);
//
//			if (UART1buf_peek()&0x01){
//				/****Decodificamos el Angle****/
//				//Desplazamos los bits
//				temp=(UART1buf_getc()>>1);
//				temp|=(UART1buf_getc()<<7);
//				angle=(float)temp/64.0;
//				angle=angle*M_PI/180.0;
//				/****Decodificamos la distancia****/
//				//Desplazamos los bits
//				temp=UART1buf_getc();
//				temp|=(UART1buf_getc()<<8);
//				distance=(float)temp/4.0;
//				///Procedemos a convertir en coordenadas cartesianas
//				ValBuf[index*2+0]=distance*cosf(angle+M_PI_2);
//				ValBuf[index*2+1]=distance*sinf(angle+M_PI_2);
//				index++;
//			}
//			else{
//				//Dato errado
//				UART1buf_getc();
//				while(UART1buf_peek()<0);
//				UART1buf_getc();
//				while(UART1buf_peek()<0);
//				UART1buf_getc();
//				while(UART1buf_peek()<0);
//				UART1buf_getc();
//				wrong_points++;
//			}
//		}else{
//			//Dato errado
//			for (int i=0;i<5;i++){
//				while(UART1buf_peek()<0);
//				UART1buf_getc();
//			}
//			wrong_points++;
//		}
//		//Si terminamos un SCAN procesamos y enviamos a la SD
//		if (first_point_set){
//			first_point_set=false;
//			x=ValBuf[index*2-2];
//			y=ValBuf[index*2-1];
//			//escribimos en la SD
//			for(int z=0;z<index;z++){
//				sprintf(chars_buf,"%.3f,%.3f\n",ValBuf[z*2+0],ValBuf[z*2+0]);
//				while(f_lseek(&fil, f_size(&fil))!= FR_OK);
//				f_puts(chars_buf, &fil);
//			}
//			for(int z=0;z<wrong_points;z++){
//				while(f_lseek(&fil, f_size(&fil))!= FR_OK);
//				f_puts("NaN,NaN\n", &fil);
//			}
//			wrong_points=0;
//			ValBuf[0]=x;
//			ValBuf[1]=y;
//			index=1;
//			//read aditional data to sync
//			for(int z=0;z<7*5;z++){
//				while(UART1buf_peek()<0);
//				UART1buf_getc();
//			}
//		}
//		//Verificamos estado del butón
//		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)){
//			HAL_Delay(50);
//			//Cerramos el archivo
//			f_close(&fil);
//			//Salimos del while
//			break;
//		}
//	}
//	///* Unmount SDCARD */
//	while(f_mount(NULL, "/", 1)!= FR_OK);
//	LPUART1buf_puts ("SD CARD UNMOUNTED successfully, puedes retirar la tarjeta\n\r");
//	//Mandamos el comando de STOP
//	UART1buf_putn(STOP_REQUEST, 2);
//	//Detenemos el motor
//	setMotorDutyCycle(0);
//	HAL_Delay(2);
//	//Borramos data del buffer
//	UART1buf_flushRx();
//	//Enviamos "fin de SAVE_SCAN_DATA"
//	LPUART1buf_puts ("Fin de la operación\n\r");
//
//
//
///************************************************************/

void SAVE_SCAN_DATA(){
	//Define variables
	uint16_t temp,init;
	float angle,distance,x,y;
	unsigned int n_points=0,n_wrong_points=0;
	char C,S,chars_buf[40];
	//Primero para crear el archivo en donde almacenaremos la data, debemos eliminar el existente
	f_unlink("/data.csv");
	//Ahora lo creamos
	while(f_open(&fil, "data.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE)!= FR_OK);
	//Escribimos la primera línea
	while(f_write(&fil, "Data a almacenar [x,y]:\n",sizeof("Data a almacenar [x,y]:\n") , &bw)!= FR_OK);
	//Ahora encendemos el motor
	setMotorDutyCycle(60);
	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	//HAL_Delay(70);
	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	//Limpiamos el buffer de Recepción
	UART1buf_flushRx();
	//Enviamos el comando por uart
	UART1buf_putn(SCAN_REQUEST,2);
	//Comenzamos a recibir los primeros 7 valores y verificamos si hay error
	for (int i=0;i<7;i++){
		while(UART1buf_peek()<0);//Analizar
		//
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
		/****Decodificamos la bandera Flag****/
		if ((UART1buf_peek()&0x03)==(0x01)){
			MainBuf[0]=UART1buf_getc();
			/****Decodificamos el checkbit****/
			while(UART1buf_peek()<0);
			C=UART1buf_peek()&0x01;
			MainBuf[1]=UART1buf_getc();
			while(UART1buf_peek()<0);
			MainBuf[2]=UART1buf_getc();
			while(UART1buf_peek()<0);
			MainBuf[3]=UART1buf_getc();
			while(UART1buf_peek()<0);
			MainBuf[4]=UART1buf_getc();
			if(C){
				n_points++;
				init=0;
				break;
			}
		}else{
			for (int i=0;i<4;i++){
				UART1buf_getc();
				while(UART1buf_peek()<0);
			}
			UART1buf_getc();
		}
	}
	//Al inicio debemos leer y procesar los datos para aprovechar el delay
	//Ahora, entramos en un bucle que solo se detendrá cuando se presione el botón Azul
	while(1){
		while(UART1buf_peek()<0);
		/****Decodificamos la bandera Flag****/
		S=(UART1buf_peek()&0x03);
		if ((S==(0x01))||(S==(0x02))){
			MainBuf[n_points*5+0]=UART1buf_getc();
			/****Decodificamos el checkbit****/
			while(UART1buf_peek()<0);
			C=UART1buf_peek()&0x01;
			MainBuf[n_points*5+1]=UART1buf_getc();
			while(UART1buf_peek()<0);
			MainBuf[n_points*5+2]=UART1buf_getc();
			while(UART1buf_peek()<0);
			MainBuf[n_points*5+3]=UART1buf_getc();
			while(UART1buf_peek()<0);
			MainBuf[n_points*5+4]=UART1buf_getc();
			if(C){
				n_points++;
			}else{
				//Dato errado
				n_wrong_points++;
			}
			if(S){
				init=1;
			}
		}else{
			//Dato errado
			for (int i=0;i<4;i++){
				UART1buf_getc();
				while(UART1buf_peek()<0);
			}
			UART1buf_getc();
			n_wrong_points++;
		}
		if(init!=0){
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
			//Si terminamos un SCAN procesamos y enviamos a la SD
			init=0;
			//procesamos data
			for(int z=0;z<n_points-1;z++){
				temp=(MainBuf[z*5+1]>>1)&0x7F;
				temp|=(MainBuf[z*5+2]<<7);
				angle=(float)temp/64.0;
				//angle=angle*M_PI/180.0;
				/****Decodificamos la distancia****/
				//Desplazamos los bits
				temp=MainBuf[z*5+3];
				temp|=(MainBuf[z*5+4]<<8);
				distance=(float)temp/4.0;
				//Procedemos a convertir en coordenadas cartesianas
				//x=distance*cosf(angle+M_PI_2);
				//y=distance*sinf(angle+M_PI_2);
				//Realizamos la conversión float a string
				sprintf(chars_buf,"%.3f,%.3f\n",angle,distance);
				//sprintf(chars_buf,"%.3f,%.3f\n",x,y);
				//escribimos en la SD
				//while(f_lseek(&fil, f_size(&fil))!= FR_OK);
				f_puts(chars_buf, &fil);
			}
			n_points--;
			MainBuf[0]=MainBuf[n_points*5+0];
			MainBuf[1]=MainBuf[n_points*5+1];
			MainBuf[2]=MainBuf[n_points*5+2];
			MainBuf[3]=MainBuf[n_points*5+3];
			MainBuf[4]=MainBuf[n_points*5+4];
			for(int z=0;z<n_wrong_points;z++){
				while(f_lseek(&fil, f_size(&fil))!= FR_OK);
				f_puts("NaN,NaN\n", &fil);
			}
			n_points=1;
			n_wrong_points=0;
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
			//read aditional data to sync
			/*for(int z=0;z<7*5;z++){
				while(UART1buf_peek()<0);
				UART1buf_getc();
			}*/
		}
		//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		//Verificamos que se presionó el botón
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)){
			HAL_Delay(50);
			//Cerramos el archivo
			f_close(&fil);
			//Salimos del while
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

//void SAVE_SCAN_DATA_old(){
//	//Define variables
//	uint16_t temp;
//	float angle,distance;
//	unsigned int n_scans=0,n_points=0,n_wrong_points=0;
//	char chars_buf[40];
//	//Primero para crear el archivo en donde almacenaremos la data, debemos eliminar el existente
//	f_unlink("/data.csv");
//	//Ahora lo creamos
//	while(f_open(&fil, "data.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE)!= FR_OK);
//	//Escribimos la primera línea
//	while(f_write(&fil, "Data a almacenar [x,y]:\n",sizeof("Data a almacenar [x,y]:\n") , &bw)!= FR_OK);
//	//Ahora encendemos el motor
//	setMotorDutyCycle(60);
//	//Limpiamos el buffer de Recepción
//	UART1buf_flushRx();
//	//Enviamos el comando por uart
//	UART1buf_putn(SCAN_REQUEST,2);
//	//Comenzamos a recibir los primeros 7 valores y verificamos si hay error
//	for (int i=0;i<7;i++){
//		while(UART1buf_peek()<0);
//		if (SCAN_DESCRIPTOR[i]!=UART1buf_getc()){
//			LPUART1buf_puts((char*)"Error\nA5-5A-05-00-00-40-81\n\r");
//			//Enviar nuevamente el comando
//
//			//Analizar si está en protección
//			while(1);
//		}
//	}
//	//Procedemos a recibir la data del RPLIDAR hasta obtener el punto de inicio de SCAN
//	while(1){
//		while(UART1buf_peek()<0);
//		/****Decodificamos la bandera Flag S****/
//		if ((UART1buf_peek()&0x03)==(0x01)){
//			//Leemos el quality
//			UART1buf_getc();
//			/****Decodificamos el checkbit****/
//			while(UART1buf_peek()<0);
//			if (UART1buf_peek()&0x01){
//				/****Decodificamos el Angle****/
//				//Desplazamos los bits
//				temp=(UART1buf_getc()>>1)&0x7F;
//				while(UART1buf_peek()<0);
//				temp|=(UART1buf_getc()<<7);
//				angle=(float)temp/64.0;
//				angle=angle*M_PI/180.0;
//				/****Decodificamos la distancia****/
//				//Desplazamos los bits
//				while(UART1buf_peek()<0);
//				temp=UART1buf_getc();
//				while(UART1buf_peek()<0);
//				temp|=(UART1buf_getc()<<8);
//				distance=(float)temp/4.0;
//				//Procedemos a convertir en coordenadas cartesianas
//				ValBuf[n_points*2+0]=distance*cosf(angle+M_PI_2);
//				ValBuf[n_points*2+1]=distance*sinf(angle+M_PI_2);
//				n_points++;
//				break;
//			}else{
//				//Es dato errado
//				for (int i=0;i<3;i++){
//					UART1buf_getc();
//					while(UART1buf_peek()<0);
//				}
//				UART1buf_getc();
//			}
//		}else{
//			//No sirve este dato
//			for (int i=0;i<4;i++){
//				UART1buf_getc();
//				while(UART1buf_peek()<0);
//			}
//			UART1buf_getc();
//		}
//	}
//	//Luego de tener el punto de inicio de SCAN, entramos en un bucle
//	//que solo se detendrá cuando se presione el botón Azul
//	while(1){
//		//Verificamos si estamos en el 4to SCAN
//		if(n_scans>=4){
//			//Mandamos el comando de STOP para detener la transmisión
//			UART1buf_putn(STOP_REQUEST, 2);
//			//Procesamos los datos y lo enviamos a la SD
//			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//			HAL_Delay(300);
//			for (unsigned int i = 0; i < n_points-1; ++i) {
//				HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//				sprintf(chars_buf,"%.3f,%.3f\n",ValBuf[i*2+0],ValBuf[i*2+1]);
//				HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//				while(f_lseek(&fil, f_size(&fil))!= FR_OK);
//				f_puts(chars_buf, &fil);
//			}
//			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//			for (unsigned int i = 0; i < n_wrong_points; ++i) {
//				while(f_lseek(&fil, f_size(&fil))!= FR_OK);
//				f_puts("Inf,Inf\n", &fil);
//			}
//			n_points=0;
//			n_wrong_points=0;
//			n_scans=0;
//			//Limpiamos el buffer Rx del Uart
//			UART1buf_flushRx();
//			/************************************/
//			/*Volvemos a sincronizar con la data*/
//			/************************************/
//			UART1buf_putn(SCAN_REQUEST,2);
//			//Comenzamos a recibir los primeros 7 valores y verificamos si hay error
//			for (int i=0;i<7;i++){
//				while(UART1buf_peek()<0);
//				if (SCAN_DESCRIPTOR[i]!=UART1buf_getc()){
//					LPUART1buf_puts((char*)"Error\nA5-5A-05-00-00-40-81\n\r");
//					//Enviar nuevamente el comando
//
//					//Analizar si está en protección
//					while(1);
//				}
//			}
//			//Procedemos a recibir la data del RPLIDAR hasta obtener el punto de inicio de SCAN
//			while(1){
//				while(UART1buf_peek()<0);
//				/****Decodificamos la bandera Flag S****/
//				if ((UART1buf_peek()&0x03)==(0x01)){
//					//Leemos el quality
//					UART1buf_getc();
//					/****Decodificamos el checkbit****/
//					while(UART1buf_peek()<0);
//					if (UART1buf_peek()&0x01){
//						/****Decodificamos el Angle****/
//						//Desplazamos los bits
//						temp=(UART1buf_getc()>>1)&0x7F;
//						while(UART1buf_peek()<0);
//						temp|=(UART1buf_getc()<<7);
//						angle=(float)temp/64.0;
//						angle=angle*M_PI/180.0;
//						/****Decodificamos la distancia****/
//						//Desplazamos los bits
//						while(UART1buf_peek()<0);
//						temp=UART1buf_getc();
//						while(UART1buf_peek()<0);
//						temp|=(UART1buf_getc()<<8);
//						distance=(float)temp/4.0;
//						//Procedemos a convertir en coordenadas cartesianas
//						ValBuf[n_points*2+0]=distance*cosf(angle+M_PI_2);
//						ValBuf[n_points*2+1]=distance*sinf(angle+M_PI_2);
//						n_points++;
//						break;
//					}else{
//						//Es dato errado
//						for (int i=0;i<3;i++){
//							UART1buf_getc();
//							while(UART1buf_peek()<0);
//						}
//						UART1buf_getc();
//					}
//				}else{
//					//No sirve este dato
//					for (int i=0;i<4;i++){
//						UART1buf_getc();
//						while(UART1buf_peek()<0);
//					}
//					UART1buf_getc();
//				}
//			}
//			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//		}
//		//Verificamos que se presionó el botón
//		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)){
//			HAL_Delay(50);
//			//Cerramos el archivo
//			f_close(&fil);
//			//Salimos del while
//			break;
//		}
//		//Guardamos el punto en el ValBuf
//		while(UART1buf_peek()<0);
//		if (((UART1buf_peek()&0x03)==0x02)||((UART1buf_peek()&0x03)==0x01)){
//			if ((UART1buf_peek()&0x03)==0x01){
//				//Estamos ante un nuevo SCAN
//				n_scans++;
//			}
//			//Leemos el Quality
//			UART1buf_getc();
//			/****Decodificamos el checkbit****/
//			while(UART1buf_peek()<0);
//			if (UART1buf_peek()&0x01){
//				/****Decodificamos el Angle****/
//				//Desplazamos los bits
//				temp=(UART1buf_getc()>>1)&0x7F;
//				while(UART1buf_peek()<0);
//				temp|=(UART1buf_getc()<<7);
//				angle=(float)temp/64.0;
//				angle=angle*M_PI/180.0;
//				/****Decodificamos la distancia****/
//				//Desplazamos los bits
//				while(UART1buf_peek()<0);
//				temp=UART1buf_getc();
//				while(UART1buf_peek()<0);
//				temp|=(UART1buf_getc()<<8);
//				distance=(float)temp/4.0;
//				//Procedemos a convertir en coordenadas cartesianas
//				ValBuf[n_points*2+0]=distance*cosf(angle+M_PI_2);
//				ValBuf[n_points*2+1]=distance*sinf(angle+M_PI_2);
//				n_points++;
//			}else{
//				//Dato errado
//				n_wrong_points++;
//			}
//		}else{
//			//Dato errado
//			n_wrong_points++;
//		}
//	}
//	/* Unmount SDCARD */
//	while(f_mount(NULL, "/", 1)!= FR_OK);
//	LPUART1buf_puts ("SD CARD UNMOUNTED successfully, puedes retirar la tarjeta\n\r");
//	//Mandamos el comando de STOP
//	UART1buf_putn(STOP_REQUEST, 2);
//	//Detenemos el motor
//	setMotorDutyCycle(0);
//	HAL_Delay(2);
//	//Borramos data del buffer
//	UART1buf_flushRx();
//	//Enviamos "fin de SAVE_SCAN_DATA"
//	LPUART1buf_puts ("Fin de la operación\n\r");
//}

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
  //MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  char buffer[64];
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
  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
  sprintf (buffer, "SD CARD Free Space: \t%lu\n\r",free_space);
  LPUART1buf_puts(buffer);
  //Enviamos el comando de RESET
  SEND_RESET_REQUEST();//DESCOMENTAR LUEGO DE TEMRINAR CON EL SD
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //SEND_GET_SAMPLERATE();
	  //SEND_GET_HEALTH();
	  //SEND_GET_LIDAR_CONF();
	  //SEND_GET_INFO();

	  //SEND_EXPRESS_SCAN();

	  //Esperamos a que se presione el Boton
	  LPUART1buf_puts((char*)"Iniciamos:\n\r");
	  while(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4));
	  //Esperamos que se suelte
	  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4));
	  LPUART1buf_puts((char*)"Mandamos SCAN request:\n\r");
	  //SAVE_DATA_bufdouble();
	  SAVE_SCAN_DATA();
	  //setMotorDutyCycle(60);
	  while(1);
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
  htim1.Init.Period = 3200-1;
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
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
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

