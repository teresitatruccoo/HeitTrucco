/*! @mainpage Template
 *
 * @section genDesc Proyecto Integrador - Bomba de anestesia
 *
 * Este proyecto consiste en, a partir de una señal analogica correspondiente a la actividad muscular de un paciente sedado en
 * cirugia, implementar una bomba de anestesia que inyectara una unidad de anestesia adicional cada vez que se detecte que la
 * misma esta superficial. La anestesia superficial puede evidenciarse con una actividad muscular aumentada.
 *
 * <a href="https://drive.google.com/...">Operation Example</a>
 *
 * @section hardConn Hardware Connection
 *
 * | Peripheral EMG |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	+5V	 	| 	+5V		|
 * | 	EMG	 	| 	GPIO_1		|
 * | 	GND	 	| 	GND			|
 *
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 09/10/2023 | Document creation		                         |
 *
 * @author Teresita Trucco (teresita.trucco@ingenieria.uner.edu.ar)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "timer_mcu.h"
#include "uart_mcu.h"
#include "analog_io_mcu.h"
#include "pwm_mcu.h"
#include "servo_sg90.h"
#include "iir_filter.h"
#include "math.h"
#include "ble_mcu.h"
/*==================[macros and definitions]=================================*/

/*==================[internal data definition]===============================*/
// #define CONFIG_BLINK_PERIOD_medicion 1000 //Fm siendo FmaxEMG 500Hz

#define FS 1000							   // Frecuencia de muestreo. Banda de interes 10Hz-500Hz
#define FC_LOW 0.2						   // Frecuencia de corte
#define ORDER_2 2						   // Orden del filtro
#define SIGNAL_SIZE 1000				   // Tamaño de la señal adquirida
#define UMBRAL_VALUE 300				   // Umbral de señal que mueve el servo
#define SERVO_SG90_PIN GPIO_2			   // Pin del servo
#define SERVO_SG90_POSITION 90			   // Angulo de mov del servo
#define CONFIG_BLINK_PERIOD_medicion 1000  // Periodo timer A para Fm 1000Hz
#define CONFIG_BLINK_PERIOD_DETECCION 5000 // 50ms es suficiente y da margen para el filtrado y procesado.
#define CHUNK 4
uint16_t signalEMG[CHUNK];
static float signalEMGfloat[CHUNK];
static float signalEMGfiltrada[CHUNK];
static float signalEMGprocesada[CHUNK];
uint16_t signalEMGChunk[CHUNK]; // Arreglo para almacenar los valores del chunk

TaskHandle_t medicionEMG_task_handle = NULL;
TaskHandle_t procesamientoEMG_task_handle = NULL;
TaskHandle_t deteccionEMG_task_handle = NULL;

/*==================[internal functions declaration]=========================*/

void calcularEnvolvente(float *senialFiltrada, float *envolvente, uint16_t tamanio)
{
	float alpha = 0.9;						 // Factor de suavizado
	envolvente[0] = fabs(senialFiltrada[0]); // Inicializar la envolvente

	for (uint16_t i = 1; i < tamanio; ++i)
	{
		envolvente[i] = alpha * fabs(senialFiltrada[i]) + (1 - alpha) * envolvente[i - 1]; // Suavizado exponencial
	}
}

///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

void FuncTimerA(void *param)
{
	xTaskNotifyGive(medicionEMG_task_handle); // Envía una notificación
}
/**
 * @brief Función invocada en la interrupción del timer B
 *
 * La función es invocada en la interrupción del timer B y envía una notificación
 * a la tarea asociada a la detección de la señal EMG.
 *
 * @param param No se utiliza
 */
void FuncTimerB(void *param)
{
	xTaskNotifyGive(deteccionEMG_task_handle); // Envía una notificación
}

void signalEMG_Task(void *pvParameter)
{
	uint8_t i = 0;
	//uint16_t signalEMGprocesada[CHUNK]; // Vector para almacenar los valores procesados

	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Recibe una notificación

		
		char msg[128];
		char msg_chunk[24];
		AnalogInputReadSingle(CH1, &signalEMGChunk[i]); // Lee el dato del canal 1 y lo almacena en el arreglo
		i++;
		

		if (i == CHUNK)
		{
			i=0;
			// Procesa el chunk
			for (uint8_t k = 0; k < CHUNK; k++)
			{
				signalEMGfloat[k] = signalEMGChunk[k];
			}
			HiPassFilter(signalEMGfloat, signalEMGfiltrada, CHUNK);
			calcularEnvolvente(signalEMGfiltrada, signalEMGprocesada, CHUNK);
			strcpy(msg, "");

			// Envía el chunk procesado por puerto serie
			for (uint8_t k = 0; k < CHUNK; k++)
			{
				sprintf(msg_chunk, "%.2f\r\n", signalEMGprocesada[k]);
				strcat(msg,msg_chunk);
			}
			printf(msg);
			//i = 0; // Reinicia el índice del chunk
		}
	}
}
void deteccionumbralEMG_Task(void *pvParameter) // TimerB
{
	while(1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Espera la notificación de que se ha procesado un nuevo chunk

		for (uint8_t i = 0; i < CHUNK; i++)
		{
			if (signalEMGprocesada[i] > UMBRAL_VALUE)
			{
				// Mover el servo si el valor procesado excede el umbral
				//ServoMove(SERVO_0, SERVO_SG90_POSITION);
				UartSendString(UART_PC, "Se inyecto una unidad de anestesia.\r\n"); // Mensaje por puerto serie
				//Aca habria que poner un delay?
				//ServoMove(SERVO_0, 0); // Regresar el servo a su posición inicial
			}
		}
	}
}

void inyectaranestesia_Task(void *pvParameter) // TimerB
{
	while (1)
	{
		//ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Recibe una notificacion
	}
}
/*==================[external functions definition]==========================*/
void app_main(void)
{
	timer_config_t timer_medicion_senial = {
		.timer = TIMER_A,
		.period = CONFIG_BLINK_PERIOD_medicion * CHUNK,
		.func_p = FuncTimerA,
		.param_p = NULL,
	};
	timer_config_t timer_deteccion_senial = {
		.timer = TIMER_B,
		.period = CONFIG_BLINK_PERIOD_DETECCION * CHUNK,
		.func_p = FuncTimerB,
		.param_p = NULL,
	};
	analog_input_config_t senial_CH1 = {
		.input = CH1,
		.mode = ADC_SINGLE,
		.func_p = NULL,
		.param_p = NULL,
		.sample_frec = 0,
	};
	serial_config_t puerto = {
		.port = UART_PC,
		.baud_rate = 115200,
		.func_p = NULL,
		.param_p = NULL,
	};

	TimerInit(&timer_medicion_senial);
	TimerInit(&timer_deteccion_senial);

	AnalogInputInit(&senial_CH1);
	AnalogOutputInit();
	UartInit(&puerto);
	HiPassInit(FS, FC_LOW, ORDER_2);
	ServoInit(SERVO_0,SERVO_SG90_PIN);
	ServoMove(SERVO_0, 30);

	xTaskCreate(&signalEMG_Task, "senial", 2048, NULL, 5, &medicionEMG_task_handle);
	xTaskCreate(&deteccionumbralEMG_Task, "senial", 2048, NULL, 5, &deteccionEMG_task_handle);
	TimerStart(timer_medicion_senial.timer);
	TimerStart(timer_deteccion_senial.timer);
}
/*==================[end of file]============================================*/