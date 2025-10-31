/*! @mainpage Template
 *
 * @section genDesc General Description
 *
 * This section describes how the program works.
 *
 * <a href="https://drive.google.com/...">Operation Example</a>
 *
 * @section hardConn Hardware Connection
 *
 * |    Peripheral  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	PIN_X	 	| 	GPIO_X		|
 *
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 12/09/2023 | Document creation		                         |
 *
 * @author Albano Peñalva (albano.penalva@uner.edu.ar)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
//Ahora con acelerometro analogico (ADXL335 + UART)
#include "ADXL335.h"
#include "uart.h"  // Tu librería UART

void enviar_uart_ADXL335() {
    float accel_x, accel_y, accel_z;
    
    // Leer valores en g's
    accel_x = ReadXValue();
    accel_y = ReadYValue();
    accel_z = ReadZValue();
    
    // Enviar por UART
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "ADXL,X:%.3f,Y:%.3f,Z:%.3f\n", 
             accel_x, accel_y, accel_z);
    UART_SendString(buffer);
}

void inicializar_sistema_ADXL335() {
    // Inicializar UART
    UART_Init(115200);  // 9600, 115200, etc.
    
    // Inicializar sensor
    if (ADXL335Init()) {
        UART_SendString("ADXL335 inicializado OK\n");
    } else {
        UART_SendString("ERROR: ADXL335 no responde\n");
    }
}

//Ahora acelerometro digital (MPU6050 + UART)
// #include "mpu6050.h"
// #include "uart.h"

// void enviar_uart_MPU6050() {
//     int16_t ax, ay, az;
    
//     // Leer valores raw
//     MPU6050_getAcceleration(&ax, &ay, &az);
    
//     // Convertir a g's
//     float accel_x = ax / 16384.0;
//     float accel_y = ay / 16384.0;
//     float accel_z = az / 16384.0;
    
//     // Enviar por UART
//     char buffer[64];
//     snprintf(buffer, sizeof(buffer), "MPU,X:%.3f,Y:%.3f,Z:%.3f\n", 
//             accel_x, accel_y, accel_z);
//     UART_SendString(buffer);
// }

// void inicializar_sistema_MPU6050() {
//     // Inicializar UART
//     UART_Init(115200);
    
//     // Inicializar sensor
//     MPU6050_initialize();
    
//     if (MPU6050_testConnection()) {
//         UART_SendString("MPU6050 inicializado OK\n");
//         MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//         UART_SendString("Configurado: ±2g\n");
//     } else {
//         UART_SendString("ERROR: MPU6050 no responde\n");
//     }
// }


void app_main(void){
	printf("Hello world!\n");
}
/*==================[end of file]============================================*/