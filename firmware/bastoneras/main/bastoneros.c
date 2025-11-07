/*! @mainpage ejemplo Bluetooth LED-RGB
 *
 * @section genDesc General Description
 *
 * Este proyecto ejemplifica el uso del módulo de comunicación Bluetooth Low Energy (BLE) 
 * junto con el manejo de tiras de LEDs RGB. 
 * Permite manejar la tonalidad e intensidad del LED RGB incluído en la placa ESP-EDU, 
 * mediante una aplicación móvil.
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 02/04/2024 | Document creation		                         |
 *
 * @author Albano Peñalva (albano.penalva@uner.edu.ar)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"
#include "neopixel_stripe.h"
#include "ble_mcu.h"
/*==================[macros and definitions]=================================*/
#define CONFIG_BLINK_PERIOD 500
#define LED_BT	LED_1
/*==================[internal data definition]===============================*/

/*==================[internal functions declaration]=========================*/
/**
 * @brief Función a ejecutarse ante un interrupción de recepción 
 * a través de la conexión BLE.
 * 
 * @param data      Puntero a array de datos recibidos
 * @param length    Longitud del array de datos recibidos
 */
void read_data(uint8_t * data, uint8_t length){
	uint8_t i = 1;
    static uint8_t red = 0, green = 0, blue = 0;
	char msg[30];

	if(data[0] == 'R'){
        /* El slidebar Rojo envía los datos con el formato "R" + value + "A" */
		red = 0;
		while(data[i] != 'A'){
            /* Convertir el valor ASCII a un valor entero */
			red = red * 10;
			red = red + (data[i] - '0');
			i++;
		}
	}else if(data[0] == 'G'){   
        /* El slidebar Verde envía los datos con el formato "G" + value + "A" */
		green = 0;
		while(data[i] != 'A'){
            /* Convertir el valor ASCII a un valor entero */
			green = green * 10;
			green = green + (data[i] - '0');
			i++;
		}
	}else if(data[0] == 'B'){
        /* El slidebar Azul envía los datos con el formato "B" + value + "A" */
		blue = 0;
		while(data[i] != 'A'){
            /* Convertir el valor ASCII a un valor entero */
			blue = blue * 10;
			blue = blue + (data[i] - '0');
			i++;
		}
	}
    NeoPixelAllColor(NeoPixelRgb2Color(red, green, blue));
    /* Se envía una realimentación de los valores actuales de brillo del LED */
    sprintf(msg, "R: %d, G: %d, B: %d\n", red, green, blue);
    BleSendString(msg);
}

/*==================[external functions definition]==========================*/
void app_main(void){
    static neopixel_color_t color;
    ble_config_t ble_configuration = {
        "ESP_EDU_1",
        read_data
    };

    LedsInit();
    BleInit(&ble_configuration);
    /* Se inicializa el LED RGB de la placa */
    NeoPixelInit(BUILT_IN_RGB_LED_PIN, BUILT_IN_RGB_LED_LENGTH, &color);
    NeoPixelAllOff();
    while(1){
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        switch(BleStatus()){
            case BLE_OFF:
                LedOff(LED_BT);
            break;
            case BLE_DISCONNECTED:
                LedToggle(LED_BT);
            break;
            case BLE_CONNECTED:
                LedOn(LED_BT);
            break;
        }
    }
}

/*==================[end of file]============================================*/

/*==================[macros and definitions]=================================*/

/*==================[internal data definition]===============================*/

/*==================[internal functions declaration]=========================*/

/*==================[external functions definition]==========================*/
v/*! @mainpage Template
 *
 * @section genDesc General Description
 *
 * This section describes how the program works.
 *
 * <a href="https://drive.google.com/...">Operation Example</a>
 *
 * @section hardConn Hardware Connection
 *
 * |    Peripheral  |   ESP32       |
 * |:--------------:|:--------------|
 * |    PIN_X       |   GPIO_X      |
 *
 *
 * @section changelog Changelog
 *
 * |   Date     | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 12/09/2023 | Document creation                              |
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



/*==================[end of file]============================================*/