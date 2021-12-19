#include <stdio.h>
#include <stdlib.h>
#include <conio.h>//Para getch()
#include <time.h>//Para delay

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
#define STAR_FLAG_1         0xA5
#define STAR_FLAG_2         0x5A
/*void doChecksum(){

}*/

void delay(unsigned int milli_seconds){   
    // Storing start time
    clock_t start_time = clock();
    // looping till required time is not achieved
    while (clock() < start_time + milli_seconds) ;
}

void sendUART(unsigned char* data,long datalength){
    for (unsigned int z=0;z<datalength;z++){
        printf("Enviando byte %d",z);
        for (unsigned int i=0;i<10;i++){
            delay(200);
            printf(".");
        }
        printf("Enviado!\n");
    }
}

void stopcommand(void){
    unsigned char data[2]={STAR_FLAG_1,STOP_RPL};
    printf("Comando STOP:\n");
    sendUART (data,2);
}

void resetcommand(void){
    unsigned char data[2]={STAR_FLAG_1,RESET_RPL};
}


void sendcommand2RPL (unsigned char command,bool recieveData){
    //printf("Elija el comando:\n0: STOP\n1: RESET\n2: SCAN\n3: EXPRESS SCAN\n4: FORCE SCAN\n5: GET INFO\n6: GET HEALTH\n7: GET SAMPLE RATE\n8: GET LIDAR CONF\n");
    //scanf("%d",&command);
    switch (command){
        case STOP_RPL:
            stopcommand();
            break;
        default :
            printf("Default zone\n");
    }
}

int main(){
    //unsigned char command=9;    
    sendcommand2RPL(STOP_RPL,false);
    printf("Presione tecla para salir.");
    getch();
    return 0;
}