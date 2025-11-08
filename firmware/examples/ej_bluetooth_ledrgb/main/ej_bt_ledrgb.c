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
#include "ADXL335.h"
#include "led.h"
#include "neopixel_stripe.h"
#include "ble_mcu.h"
#include "uart_mcu.h"  
#include "led.h"
#include <math.h>
#include "gpio_mcu.h"
/*==================[macros and definitions]=================================*/
/*==================[inclusions]=============================================*/
// ...existing code...

/*==================[macros and definitions]=================================*/
#define CONFIG_BLINK_PERIOD 500
#define LED_BT	LED_1

// Configuración detector de caídas
#define FALL_THRESH_HIGH    2.5f   
#define FALL_THRESH_LOW    -1.5f    
#define FALL_TIME_WINDOW   1000    
#define MIN_PEAKS          4       

/*==================[internal data declaration]==============================*/
// typedef struct {
//     float peaks[MIN_PEAKS];        
//     uint32_t peak_times[MIN_PEAKS]; 
//     uint8_t peak_count;            
//     uint32_t last_peak_time;       
//     float last_accel;              
// } fall_detector_t;

TaskHandle_t fall_detector_task = NULL;


static volatile bool led_alert_active = false;
/* duración de la alerta en ms */
#define LED_ALERT_DURATION_MS 3000
/*==================[internal functions definition]==========================*/
void enviar_uart_ADXL335() {
    float accel_x, accel_y, accel_z;
   
    
    accel_x = ReadXValue();
    accel_y = ReadYValue();
    accel_z = ReadZValue();
    
    float accel_mag = sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);

    if (accel_mag > 25 || accel_mag < 5) {
        xTaskNotifyGive(fall_detector_task);
    }
    
    char buffer[64];
    sprintf(buffer, "%.3f,%.3f,%.3f,%.3f\r\n", 
             accel_x, accel_y, accel_z, accel_mag);
    UartSendString(UART_PC, buffer);
}

void inicializar_sistema_ADXL335() {
    serial_config_t cfg;
    cfg.baud_rate = 230400;
    cfg.port = UART_PC;
    cfg.func_p = UART_NO_INT;
    cfg.param_p = NULL;
    UartInit(&cfg);
    
    if (ADXL335Init()) {
        UartSendString(UART_PC, "ADXL335 inicializado OK\n");
    } else {
        UartSendString(UART_PC, "ERROR: ADXL335 no responde\n");
    }
}


static void visualizer_task(void *pvParameters)
{
    const TickType_t sample_period = pdMS_TO_TICKS(100);  // 20Hz para visualización
    
    vTaskDelay(pdMS_TO_TICKS(20));

    for (;;)
    {
        enviar_uart_ADXL335();  // envia datos para graficar
        vTaskDelay(sample_period);
    }
}
static void led_alert_task(void *pvParameters)
{
    while (1)
    {
        /* encender los 3 leds: mask b0:LED_3, b1:LED_2, b2:LED_1 -> 0x07 */
        LedsMask( (1 << 0) | (1 << 1) | (1 << 2) );
        GPIOOn(GPIO_19);
        vTaskDelay(pdMS_TO_TICKS(LED_ALERT_DURATION_MS));

        LedsOffAll();
        GPIOOff(GPIO_19);
        led_alert_active = false;
        vTaskDelete(NULL);
        
        
    }
    
    (void)pvParameters;


}

static void caida_detector_task(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        UartSendString(UART_PC, "¡CAÍDA DETECTADA!\r\n");
        BleSendString("¡CAÍDA DETECTADA!\r\n");
        // TODO: Acciones al detectar caída (LED, buzzer, etc)
      
         if (!led_alert_active) {
                led_alert_active = true;
                xTaskCreate(led_alert_task, "led_alert", 2048, NULL, 5, NULL);
            }
    }
    
        
    }


void read_data(uint8_t * data, uint8_t length) {
}

void app_main(void)
{
    inicializar_sistema_ADXL335();
    vTaskDelay(pdMS_TO_TICKS(50));
    GPIOInit(GPIO_19,GPIO_OUTPUT);

    // Crear ambas tareas con diferentes prioridades
    xTaskCreate(visualizer_task, "visualizer", 4096, NULL, 1, NULL);  // Menor prioridad
    xTaskCreate(caida_detector_task, "fall_detect", 4096, NULL, 2, &fall_detector_task);  // Mayor prioridad

    // Configuración BLE y LED RGB (del código original)
    ble_config_t ble_configuration = {
        .device_name= "bastoneras",
        .func_p= read_data,
    };

    LedsInit();
    BleInit(&ble_configuration);

}