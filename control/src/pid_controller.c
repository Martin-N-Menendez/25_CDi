/*=====[pid_controller]========================================================
 * Copyright 2018 Diego Fernandez <dfernandez202@gmail.com>
 * Copyright 2018 Eric Nicolas Pernia <ericpernia@gmail.com>
 * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.2.0
 * Creation Date: 2018/09/24
 */

/*=====[Inclusions of private function dependencies]=========================*/

#include "pid_controller.h"

/*=====[Definition macros of private constants]==============================*/

// ADC0 Channels for r and y
#define ADC0_CH_R       CH1
#define ADC0_CH_Y       CH3

// Saturation of actuator (DAC)
#define U_MIN           0
#define U_MAX           1023


#define N_MUESTRAS       200				// Muestras a guardar

#define ABIERTO	0
#define PID 1
#define POLE 2

#define MODO	PID


#define PRINT_RESULT

/*=====[Private function-like macros]========================================*/

#define getVoltsSampleFrom(adc0Channel) 3.3*(float)adcRead((adc0Channel))/1023.0

/*=====[Definitions of external public global variables]=====================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

static float r_array[N_MUESTRAS];
static float y_array[N_MUESTRAS];
static float u_array[N_MUESTRAS];
extern float R[N_MUESTRAS];

static uint32_t i = 0;

/*=====[Prototypes (declarations) of private functions]======================*/

static void Actualizar_Controlador( float U_controller );

/*=====[Implementations of public functions]=================================*/

void pidInit( pidConfig_t* config, pidState_t* state )
{
   adcInit( ADC_ENABLE );
   dacInit( DAC_ENABLE );
   
   state->pastD = 0;
   state->pastY = 0;
   state->futureI = 0;
}

void pidControl( pidConfig_t* config, pidState_t* state )
{
   /*----------- Leer ADC y pasar a Volts ------------------------------------*/

   float y = getVoltsSampleFrom( ADC0_CH_Y );
   //float r = getVoltsSampleFrom( ADC0_CH_R );
   float r = R[i];							 // Generada internamente como R[i]

   /*----- Calcular salida U del controlador --------------------------------*/

	#if(MODO == PID)

	   // P[k] = Kp * (b*r[k] - y[k])
	   float P = config->Kp * (config->b * r - y);

	   // D[k] = (Kd/(Kd + N*h)) * D[k-1] - (N*h*Kd/(Kd+N*h)) * (y[k]-y[k-1])
	   float D = ( config->Kd * state->pastD
				 - config->N * config->h * config->Kd * (y - state->pastY) )
				 / (config->Kd + config->N * config->h);

	   // I[k] se calculo en la enterior iteracion (en la primera se asume 0)
	   float I = state->futureI;


	   float U = P + I + D;		// U[k] = P[k] + I[k] + D[k]
	#endif

	#if(MODO == ABIERTO)
	   float U = r;
	#endif

#if (MODO == POLE)

      K  = 929148;
      K0 = 1.929148;
      Capacitor = CAPACITOR;

      float x = y * Capacitor;	 // x[k] = y[k]*c // Variable de estado x

      /*----- Calcular salida del controlador ----------------------------------*/

      // u[k] = K0*r[k] - K*x[k]
      float U = K0 * r  - K * x;
#endif

   /*----- Actualizar controlador -------------------------------------*/
#if( MODO != ID)
   Actualizar_Controlador(U);
#endif

#if (MODO == PID)
   // D[k-1] = D[k]
   state->pastD = D;

   // Y[k-1] = Y[k]
   state->pastY = y;

   // Anti wind-up
   if( U_MIN <= U && U <= U_MAX ){
      // I[k+1] = I[k] + Ki*h*e[k], con e[k] = r[k] - y[k]
      state->futureI = I + config->Ki * config->h * (r - y);
   }
#endif

#if (MODO != ID)
#ifdef PRINT_RESULT
   // Save and show samples
   if( i< N_MUESTRAS ) {
      // Guardar muestras de R[k], Y[k] y U[k]
      r_array[i] = r;
      y_array[i] = y;
      u_array[i] = U;
      i++;

   } else {
      // Finalizar lectura de muestras
      gpioWrite( LEDB, OFF );
      gpioWrite( LED1, ON );

      // Imprimir R[k]
      uartWriteString( UART_USB, "R = [" );
      for( i=0; i<N_MUESTRAS; i++) {
         uartWriteString( UART_USB, float_to_string( r_array[i] ) );
         uartWriteString( UART_USB, " " );
      }
      uartWriteString( UART_USB, "];\r\n\r\n" );
      
      // Imprimir Y[k]
      uartWriteString( UART_USB, "Y = [" );
      for( i=0; i<N_MUESTRAS; i++) {
         uartWriteString( UART_USB, float_to_string( y_array[i] ) );
         uartWriteString( UART_USB, " " );
      }
      uartWriteString( UART_USB, "];\r\n\r\n" );

      // Imprimir U[k]
      uartWriteString( UART_USB, "U = [" );
      for( i=0; i<N_MUESTRAS; i++) {
         uartWriteString( UART_USB, float_to_string( u_array[i] ) );
         uartWriteString( UART_USB, " " );
      }
      uartWriteString( UART_USB, "];\r\n\r\n" );

      // Imprimir Usat[k]
      uartWriteString( UART_USB, "Usat = [" );
      for( i=0; i<N_MUESTRAS; i++) {
         if( u_array[i] > 3.3 )
             u_array[i] = 3.3;
         if( u_array[i] < 0.0 )
             u_array[i] = 0.0;
         uartWriteString( UART_USB, float_to_string( u_array[i] ) );
         uartWriteString( UART_USB, " " );
      }
      uartWriteString( UART_USB, "];\r\n\r\n" );

      uartWriteString( UART_USB, "Calculo finalizado!\r\n" );
      gpioWrite( LED2, ON );
      while(1);
   }
#endif
#endif
}

/*=====[Implementations of interrupt functions]==============================*/

/*=====[Implementations of private functions]================================*/

static void Actualizar_Controlador( float U_controller )
{

   float U_dacSample = 310.0 * U_controller;
   
   uint16_t U_dacSample_sat = (uint16_t)U_dacSample;
   
   if( U_dacSample_sat > U_MAX){
      U_dacSample_sat = U_MAX;
   }
   
   if( U_dacSample_sat < U_MIN){
      U_dacSample_sat = U_MIN;
   }
   
   dacWrite(DAC, U_dacSample_sat);
}
