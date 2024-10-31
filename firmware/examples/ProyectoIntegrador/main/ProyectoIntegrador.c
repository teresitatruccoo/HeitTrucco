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
 * |:----------:|:------|
 * | 	+5V	 	| 	+5V		|
 * | 	EMG	 	| 	CH1		|
 * | 	GND	 	| 	GND		|
 * 
 * | Peripheral Servo |   ESP32   	|
 * |:----------:|:----------|
 * | 	+5V	 	| 	+5V		|
 * | 	EMG	 	| 	CH2		|
 * | 	GND	 	| 	GND		|
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
/** @def FS
 * @brief Frecuencia de muestreo
 */
#define FS 1000	// Frecuencia de muestreo. Banda de interes 10Hz-500Hz
/** @def FC_LOW
 * @brief Frecuencia de corte filtro PA
 */						   
#define FC_LOW 0.2						   // Frecuencia de corte
/** @def ORDER_2
 * @brief Orden del filtro PA
 */	
#define ORDER_2 2	
/** @def 	SIGNAL_SIZE
 * @brief Tamaño de la señal adquirida
 */						   
#define SIGNAL_SIZE 1000
/** @def 	UMBRAL_VALUE
 * @brief Valor de umbral a superar para inyectar anestesia
 */					   
#define UMBRAL_VALUE 250				   // Umbral de señal que mueve el servo
/** @def 	SERVO_SG90_POSITION
 * @brief GPIO de conexion del servo
 */	
#define SERVO_SG90_PIN GPIO_2			   // Pin del servo
/** @def 	SERVO_SG90_POSITION
 * @brief Posocicion inicial del servo
 */	
#define SERVO_SG90_POSITION 90			   
/** @def 	CONFIG_BLINK_PERIOD
 * @brief Periodo timer A para Fm 1000Hz
 */	
#define CONFIG_BLINK_PERIOD_medicion 1000  // Periodo timer A para Fm 1000Hz
/** @def CONFIG_BLINK_PERIOD
 * @brief Periodo timer B 
 */	
#define CONFIG_BLINK_PERIOD_DETECCION 5000 // 50ms es suficiente y da margen para el filtrado y procesado.
/** @def 	CHUNK
 * @brief Tamanio de los chunks
 */	
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

/**
 * @brief Calcula la envolvente de una senial filtrada.
 *
 * @param senialFiltrada Senial filtrada de la cual se quiere calcular la envolvente.
 * @param envolvente Arreglo donde se guardara la envolvente de la senial.
 * @param tamanio Tamanio de los arreglos senialFiltrada y envolvente.
 */
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

/**
 * @brief Función invocada en la interrupción del timer A
 *
 * La función es invocada en la interrupción del timer A y envía una notificación
 * a la tarea asociada a la medición de la señal EMG.
 *
 * @param param puntero a void, no se utiliza
 */
void FuncTimerA(void *param)
{
	xTaskNotifyGive(medicionEMG_task_handle); // Envía una notificación
}
/**
 * @brief Función invocada en la interrupción del timer B
 *
 * La función es invocada en la interrupción del timer B y envía una notificación
 * a la tarea asociada a la detección de umbral de la señal EMG.
 *
 * @param param No se utiliza
 */
void FuncTimerB(void *param)
{
	xTaskNotifyGive(deteccionEMG_task_handle); // Envía una notificación
}

/**
 * @brief Tarea que lee la señal EMG del canal 1 del ADC,
 *        la procesa y envía el chunk procesado por puerto serie.
 *
 * La tarea lee la señal EMG del canal 1 del ADC y la almacena en un arreglo.
 * Luego, cuando se ha completado un chunk, lo procesa con un filtro pasa alto
 * y calculo de la envolvente y envía el chunk procesado por puerto serie.
 *
 * @param pvParameter puntero a void, no se utiliza
 */
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
/**
 * @brief Tarea que detecta si la señal EMG procesada excede un umbral y mueve el servo 10°.
 *
 * La tarea espera una notificación que indica que un nuevo chunk de la señal EMG ha sido procesado.
 * Recorre los valores del chunk procesado y verifica si alguno excede el umbral definido. Si es así,
 * incrementa la posición del servo y envía un mensaje por el puerto serie indicando que se inyectó
 * una unidad de anestesia.
 *
 * @param pvParameter puntero a void, no se utiliza
 */
void deteccionumbralEMG_Task(void *pvParameter) // TimerB
{
	uint16_t posicionServo = 0;
	while(1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Espera la notificación de que se ha procesado un nuevo chunk

		for (uint8_t i = 0; i < CHUNK; i++)
		{
			if (signalEMGprocesada[i] > UMBRAL_VALUE)
			{
				// Mover el servo si el valor procesado excede el umbral
				posicionServo += 10; // Incrementa la posición del servo en 10 grados
                ServoMove(SERVO_0, posicionServo); // Mueve el servo a la nueva posición
				UartSendString(UART_PC, "Se inyecto una unidad de anestesia.\r\n"); // Mensaje por puerto serie
			}
		}
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
	//ServoMove(SERVO_0, 30);

	xTaskCreate(&signalEMG_Task, "senial", 2048, NULL, 5, &medicionEMG_task_handle);
	xTaskCreate(&deteccionumbralEMG_Task, "senial", 2048, NULL, 5, &deteccionEMG_task_handle);
	TimerStart(timer_medicion_senial.timer);
	TimerStart(timer_deteccion_senial.timer);
}
/*==================[end of file]============================================*/