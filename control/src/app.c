/*=====[Inclusions of function dependencies]=================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "pid_controller.h"
#include "sapi.h"
#include <string.h>
#include "arm_math.h"
#include <stdlib.h>

/*=====[Definition macros of private constants]==============================*/

#define N_MUESTRAS   200
#define n_MUESTRAS 	10

#define N_ITERACIONES   1000

#define UART_PC        UART_USB
#define UART_BLUETOOTH UART_232

#define PLANTA 0
#define ID 1

#define MODO PLANTA

#define TEST FALSE

#define ADC0_CH_Y       CH3					// Canal ADC0  para Y

#define Q_ORDER         1					// Orden del proceso
#define VAL             100					// Value for Pinitial(k)
#define M_DIM          ((2*Q_ORDER)+1)		// Orden de la matriz para identificacion


#define DAC_MAX   666   // 1023 = 3.3V, 666 = 2.15V
#define DAC_MIN    356  // 1023 = 3.3V, 356 = 1.15V

/*=====[Private function-like macros]========================================*/

// adcRead() returns 10 bits integer sample (uint16_t)
// sampleInVolts = (3.3 / 1023.0) * adcSample
#define getVoltsSampleFrom(adc0Channel) 3.3*(float)adcRead((adc0Channel))/1023.0

/*=====[Definitions of private global variables]=============================*/

static float Y_step[N_MUESTRAS];
static float Y_PID[N_MUESTRAS];
static float U[N_MUESTRAS];

tick_t timeCount   = 0;
uint16_t adcValue = 0;

uint32_t i_R = 0;
uint32_t i_r = 1;

float R[N_MUESTRAS];
float muestra = 0;
uint16_t lastDacValue = DAC_MIN;

static float NumZ[N_MUESTRAS] = {0, 0.0201, -0.0191};
static float DenZ[N_MUESTRAS] = {1, -1.9598, 0.9608};

static float u_buff[Q_ORDER];		// Previous q samples of output = [ u(k-q), ..., u(k-1) ]
static float y_buff[Q_ORDER];		// Previous q samples of output = [ y(k-q), ..., y(k-1) ]

float y_k = 0.0; // y(k)
float u_k = 0.0; // u(k)

/*----- phi[k] and phi'[k] --------------------------------------------------*/

// a Matrix is form with a matrix instance and its memory
static arm_matrix_instance_f32 phi_k;  // phi(k)           // 1 x M_DIM
static float32_t phi_k_f32[M_DIM] = { 0.0,  0.0,  0.0, };

static arm_matrix_instance_f32 phiT_k; // phi'(k)          // M_DIM x 1
static float32_t phiT_k_f32[M_DIM] = { 0.0, 0.0, 0.0, };

/*----- K[k] ----------------------------------------------------------------*/
static arm_matrix_instance_f32 M1;   // M1 = P(k-1) phi'(k)      // M_DIM x 1
static float32_t M1_f32[M_DIM] = { 0.0, 0.0, 0.0, };

static arm_matrix_instance_f32 M2;   // M2 = phi(k) M1     // 1 x 1
static float32_t M2_f32[M_DIM] = {0.0,};

static float32_t M3 = 0;             // M3 = 1 + M2        // 1 x 1

static arm_matrix_instance_f32 M4;   // M4 = phi(k) P(k-1)       // 1 x M_DIM
static float32_t M4_f32[M_DIM] = {0.0,       0.0,       0.0,};

static arm_matrix_instance_f32 K_k;  // K_k = M1/M3        // M_DIM x 1
static float32_t K_k_f32[M_DIM] = { 0.0,  0.0,  0.0, };

/*----- Theta[k] ------------------------------------------------------------*/

static arm_matrix_instance_f32 M5;   // M5 = phi(k) Theta(k-1)   // 1 x 1
static float32_t M5_f32[1] = { 0.0, };

static float32_t M6 = 0;             // M6 = y(k) - M5     // 1 x 1

static arm_matrix_instance_f32 M7;   // M7 = K(k) M6       // M_DIM x 1
static float32_t M7_f32[M_DIM] = { 0.0,  0.0,  0.0, };

// Public: Theta(k) = Theta(k-1) + M7
arm_matrix_instance_f32 Theta_k;                    // M_DIM x 1
// Theta_initial(k)
float32_t Theta_k_f32[M_DIM] = { 0.0, 0.0, 0.0, };
static arm_matrix_instance_f32 Theta;
static float32_t Theta_f32[M_DIM] = { 0.0, 0.0, 0.0, };

/*----- P(k) ----------------------------------------------------------------*/

static arm_matrix_instance_f32 M8;  // M8 = K(k) M4        // M_DIM x M_DIM
static float32_t M8_f32[M_DIM*M_DIM] = { 0.0,  0.0,  0.0, 0.0,  0.0,  0.0,  0.0,  0.0, 0.0, };
arm_matrix_instance_f32 P_k; // Public: P(k) = P(k-1) - M8  // M_DIM x M_DIM
// P_initiak(k) = VAL I_3
float32_t P_k_f32[M_DIM*M_DIM] = { VAL*1.0, 0.0, 0.0, 0.0, VAL*1.0, 0.0, 0.0, 0.0, VAL*1.0, };
static arm_matrix_instance_f32 P;
static float32_t P_f32[M_DIM*M_DIM] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, };

/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

// Prototipo de funcion de la tarea
void watch_dog( void* taskParmPtr );
void generar_referencia( void* taskParmPtr );
void control_planta( void* taskParmPtr );
void generar_R(void);
void taskIdentification( void* taskParmPtr );
static float generateNoiseSampleOnDac( void );

/*=====[Main function, program entry point]==================================*/

void generar_R(void)
{
	while( i_R < N_MUESTRAS)
	   {
		   if( lastDacValue == DAC_MAX){
			 if  (i_r == n_MUESTRAS)
			 {
				lastDacValue = DAC_MIN;
				i_r = 0;
			 }
			 R[i_R] = (float)DAC_MIN * 3.3/1023.0;
		   } else{
			   if  (i_r == n_MUESTRAS)
			   {
				lastDacValue = DAC_MAX;
				i_r = 0;
			   }
			 R[i_R] = (float)DAC_MAX * 3.3/1023.0;
		   }

		   uartWriteString( UART_PC, float_to_string(R[i_R]));
		   uartWriteString( UART_PC, " ");
		   i_R++;
		   i_r++;
	   }
	uartWriteString( UART_PC, "\r\n");
}

int main (void)
{

   boardConfig();											// Inicializar y configurar la plataforma


   uartConfig( UART_PC, 115200 );							// Inicializar UART_USB para conectar a la PC
   uartWriteString( UART_PC, "###################### Iniciando ###################\r\n" );


   adcConfig( ADC_ENABLE ); /* ADC */
   dacConfig( DAC_ENABLE ); /* DAC */

   generar_R();

   // Crear tarea en freeRTOS
      xTaskCreate(
         watch_dog,                     // Funcion de la tarea a ejecutar
         (const char *)"watch_dog",     // Nombre de la tarea como String amigable para el usuario
         configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
         0,                          // Parametros de tarea
         tskIDLE_PRIORITY+1,         // Prioridad de la tarea
         0                           // Puntero a la tarea creada en el sistema
      );
	#if(MODO == PLANTA)
	  xTaskCreate(
	   generar_referencia,                     // Funcion de la tarea a ejecutar
	   (const char *)"generar_referencia",     // Nombre de la tarea como String amigable para el usuario
	   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
	   0,                          // Parametros de tarea
	   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
	   0                           // Puntero a la tarea creada en el sistema
	  );
	#endif
	#if(MODO == ID)
	  xTaskCreate(
		taskIdentification,                     // Funcion de la tarea a ejecutar
	   (const char *)"taskIdentification",     // Nombre de la tarea como String amigable para el usuario
	   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
	   0,                          // Parametros de tarea
	   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
	   0                           // Puntero a la tarea creada en el sistema
	  );
	#endif
	#if(MODO == PLANTA)
	  xTaskCreate(
	   control_planta,                     // Funcion de la tarea a ejecutar
	   (const char *)"control_planta",     // Nombre de la tarea como String amigable para el usuario
	   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
	   0,                          // Parametros de tarea
	   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
	   0                           // Puntero a la tarea creada en el sistema
	  );
	#endif
     // Iniciar scheduler
     vTaskStartScheduler();


   // ---------- REPETIR POR SIEMPRE --------------------------
      while( TRUE ) {
      };
      // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
      // directamenteno sobre un microcontroladore y no es llamado por ningun
      // Sistema Operativo, como en el caso de un programa para PC.
      return 0;
}


/*==================[definiciones de funciones externas]=====================*/

// Implementacion de funcion de la tarea
void watch_dog( void* taskParmPtr )
{

   // Tarea periodica cada 500 ms
   portTickType xPeriodicity =  500 / portTICK_RATE_MS;
   portTickType xLastWakeTime = xTaskGetTickCount();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {
      gpioToggle( LEDR );
      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );	// Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
   }
}

void generar_referencia( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------
	if (TEST == TRUE)
		printf( "Tarea generadora de señal de referencia.\r\n" );

   // Tarea periodica cada 50 ms
   portTickType xPeriodicity =  50 / portTICK_RATE_MS;
   portTickType xLastWakeTime = xTaskGetTickCount();

   if (TEST == TRUE)
	   uartWriteString( UART_PC, "\n\rr[k] =      [" );

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {

	   if ( i_R < N_MUESTRAS)
	   {
		   //dacWrite( DAC, lastDacValue );

		   if( lastDacValue == DAC_MAX){
			 if  (i_r == n_MUESTRAS)
			 {
				lastDacValue = DAC_MIN;
				i_r = 0;
			 }
			 R[i_R] = (float)DAC_MIN * 3.3/1023.0;
			 gpioWrite( LED1, OFF );
			 if (TEST == TRUE)
				 uartWriteString( UART_PC, float_to_string( R[i_R] ) );
			 i_R++;
			 i_r++;
		   } else{
			   if  (i_r == n_MUESTRAS)
			   {
			   	lastDacValue = DAC_MAX;
			   	i_r = 0;
			   }
			 R[i_R] = (float)DAC_MAX * 3.3/1023.0;
			 gpioWrite( LED1, ON );
			 if (TEST == TRUE)
				 uartWriteString( UART_PC, float_to_string( R[i_R] ) );
			 i_R++;
			 i_r++;
		   }

		   if (i_R < N_MUESTRAS)
			   if (TEST == TRUE)
				   uartWriteString( UART_PC, ";");

		   if( i_R == N_MUESTRAS )
		   {
			  if (TEST == TRUE)
				   uartWriteString( UART_PC, "];\r\n");
			  i_R++;
		   }
	   }

      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );	// Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
   }
}


void control_planta( void* taskParmPtr )
{

	   pidConfig_t pidConfig;
	   pidState_t pidState;

	   uint32_t h_ms = 3;

	   pidConfig.h  = h_ms/1000.0;
	   pidConfig.Kp = 2.8;
	   pidConfig.Ki = 1.1 / pidConfig.h;
	   pidConfig.Kd = 0.09 * pidConfig.h;
	   pidConfig.b  = 1;
	   pidConfig.N  = 200;

	   printf( "PID:\r\n" );
	   printf( "----------------------\r\n\r\n" );
	   printf( "Configuracion:\r\n" );

	   uartWriteString( UART_PC, "\r\n  h = " );
	   uartWriteString( UART_PC, float_to_string(pidConfig.h*1000.0) );
	   uartWriteString( UART_PC, " ms\r\n  K_p = " );
	   uartWriteString( UART_PC, float_to_string(pidConfig.Kp) );
	   uartWriteString( UART_PC, "\r\n  K_i = " );
	   uartWriteString( UART_PC, float_to_string(pidConfig.Ki) );
	   uartWriteString( UART_PC, "\r\n  K_d = " );
	   uartWriteString( UART_PC, float_to_string(pidConfig.Kd) );
	   uartWriteString( UART_PC, "\r\n  b = " );
	   uartWriteString( UART_PC, float_to_string(pidConfig.b) );
	   uartWriteString( UART_PC, "\r\n  N = " );
	   uartWriteString( UART_PC, float_to_string(pidConfig.N) );
	   uartWriteString( UART_PC, "\r\n\r\n"  );

	   pidInit( &pidConfig, &pidState );

	   portTickType xPeriodicity =  h_ms / portTICK_RATE_MS;
	   portTickType xLastWakeTime = xTaskGetTickCount();


	   cyclesCounterConfig(EDU_CIAA_NXP_CLOCK_SPEED);			   // Configura el contador de ciclos con el clock de la EDU-CIAA NXP
	   volatile uint32_t cyclesElapsed = 0;

	   while(TRUE) {

	      pidControl( &pidConfig, &pidState );


	      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );			// Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
	   }
}

void rlsIdentificationInit( void )
{
   adcInit( ADC_ENABLE );
   dacInit( DAC_ENABLE );

   uint16_t adcValue = 0;

   adcValue = adcRead(CH2);						// Inicializar semilla leyendo ADC no conectado a nada
   srand( adcValue );

   /*----- Initialize phi[k] and phi'[k] ------------------------------------*/
   arm_mat_init_f32(   &phi_k,      1, M_DIM, phi_k_f32   );
   arm_mat_init_f32(  &phiT_k, M_DIM,      1, phiT_k_f32  );

   /*----- Initialize K[k] --------------------------------------------------*/
   arm_mat_init_f32(      &M1, M_DIM,      1, M1_f32      );
   arm_mat_init_f32(      &M2,      1,      1, M2_f32      );
   arm_mat_init_f32(      &M4,      1, M_DIM, M4_f32      );
   arm_mat_init_f32(     &K_k, M_DIM,      1, K_k_f32     );

   /*----- Initialize Theta[k] ----------------------------------------------*/
   arm_mat_init_f32(      &M5,      1,      1, M5_f32      );
   arm_mat_init_f32(      &M7, M_DIM,      1, M7_f32      );
   arm_mat_init_f32( &Theta_k, M_DIM,      1, Theta_k_f32 );
   arm_mat_init_f32(   &Theta, M_DIM,      1, Theta_f32   );

   /*----- Initialize P(k) --------------------------------------------------*/
   arm_mat_init_f32(      &M8, M_DIM, M_DIM, M8_f32      );
   arm_mat_init_f32(     &P_k, M_DIM, M_DIM, P_k_f32     );
   arm_mat_init_f32(       &P, M_DIM, M_DIM, P_f32       );
}

bool_t rlsIdentification( void )
{
   static uint32_t n = 0;
   static bool_t lock = FALSE;

   uint32_t i = 0;
   arm_status status = ARM_MATH_SUCCESS;

   u_k = generateNoiseSampleOnDac();      // Inyecto ruido a la planta u[k]
   y_k = getVoltsSampleFrom( ADC0_CH_Y ); // Leo la salida de la planta y[k]

   if( n >= Q_ORDER ) {
      lock = TRUE;
   }

   if( lock ) {
      /*----- Calculo de phi[k] y phi'[k] -----------------------------*/
      // Con q=2 ==> phi(k) = [y(k-1) y(k-2) u(k) u(k-1) u(k-2)]
      // Con q=1 ==> phi(k) = [y(k-1) u(k) u(k-1)]

      // Calculo de phi(k)
      phi_k_f32[0] = y_buff[0];
      phi_k_f32[1] = u_k;
      phi_k_f32[2] = u_buff[0];

      // Calculo de phi'(k)
      status = arm_mat_trans_f32( &phi_k, &phiT_k );

      /*--------------------- Calculo de  K[k] -------------------------*/
      //              P(k-1) phi'(k)
      // K(k) = ____________________________
      //         1 + phi(k) P(k-1) phi'(k)
      //

      // M1 = P(k-1) phi'(k)                                 // M_DIM x 1
      status = arm_mat_mult_f32( &P_k, &phiT_k, &M1 );

      // M2 = phi(k) M1                                      //  1 x 1
      status = arm_mat_mult_f32( &phi_k, &M1, &M2 );

      // M3 = 1 + M2                                         // 1 x 1
      M3 = 1 + M2_f32[0];

      // M4 = phi(k) P(k-1)                                  // 1 x M_DIM
      status = arm_mat_mult_f32( &phi_k, &P_k, &M4 );

      //         M1
      // K(k) = ____                                         // M_DIM x 1
      //         M3
      for( i=0; i<M_DIM; i++ ){
         K_k_f32[i] = M1_f32[i]/M3;
      }

      /*-------------------- Calculo de  Theta[k] -----------------------*/
      // Theta(k) = Theta(k-1) + K(k)[ y(k) - phi(k) Theta(k-1) ]

      // M5 = phi(k) Theta(k-1)                              // 1 x 1
      status = arm_mat_mult_f32( &phi_k, &Theta_k, &M5 );

      // M6 = y(k) - M5                                      // 1 x 1
      M6 = y_k - M5_f32[0];

      // M7 = K(k) M6                                        // M_DIM x 1
      for( i=0; i<M_DIM; i++ ){
         M7_f32[i] = K_k_f32[i] * M6;
      }

      // Theta(K) = Theta(k-1) + M7                          // M_DIM x 1
      status = arm_mat_add_f32( &Theta_k, &M7, &Theta );

      // Copia de  Theta a Theta_k
      for( i=0; i<M_DIM; i++ ){
         Theta_k_f32[i] = Theta_f32[i];
      }

      /*-----  Actualizar P[k] para la proxima iteracion ( P[k] será p[k+1] ) ----*/
      //                  P(k-1) phi'(k) phi(k) P(k-1)
      // P(k) = P(k-1) - ______________________________
      //                   1 + phi(k) P(k-1) phi'(k)

      // M8 = K(k) M4                                        // M_DIM x M_DIM
      status = arm_mat_mult_f32( &K_k, &M4, &M8 );

      // M8 = -M8                                            // M_DIM x M_DIM
      for( i=0; i<(M_DIM*M_DIM); i++ ){
         M8_f32[i] = -M8_f32[i];
      }

      // P(k) = P(k-1) + M8                                  // M_DIM x M_DIM
      status = arm_mat_add_f32( &P_k, &M8, &P );

      // Copiar P a P_k
      for( i=0; i<(M_DIM*M_DIM); i++ ){
         P_k_f32[i] = P_f32[i];
      }
   }

   /*----- Actualizar u_buff y y_buff(k) para la siguiente iteracion -------------*/
   u_buff[0] = u_k;
   y_buff[0] = y_k;

   n++;

   return TRUE;
}


static float generateNoiseSampleOnDac( void )
{
   uint16_t dacValue = 0;
   float u_sampleInVolts;

   dacValue = DAC_MIN +  rand() % (DAC_MAX+1 - DAC_MIN);

   dacWrite( DAC, dacValue );

   delayInaccurateUs(5);

   u_sampleInVolts = (float)dacValue * 3.3 / 1023.0;

   return u_sampleInVolts;
}


void taskIdentification( void* taskParmPtr )
{
   uint32_t h_ms = 3;
   float h = (((float)h_ms)/1000.0);

   volatile float a0 = 0.0;
   volatile float Restimated = 0.0;
   bool_t algorithmOk = TRUE;

   uint32_t i = 0;

   gpioWrite(LEDR, OFF);
   gpioWrite(LEDG, OFF);

   rlsIdentificationInit();

   // Tarea periodica cada h ms
   portTickType xPeriodicity =  h_ms / portTICK_RATE_MS;
   portTickType xLastWakeTime = xTaskGetTickCount();

   // Run model identification
   for( i = 0; i < N_ITERACIONES; i++ ) {

      algorithmOk = rlsIdentification();

      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );		// Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
   }

   // Calcular estimacion de R
   a0 = Theta_k_f32[0];
   Restimated = (-h/log(a0)) / CAPACITOR;

   gpioWrite(LEDG, ON);

   	uartWriteString( UART_USB, "\r\n  h = " );
   	uartWriteString( UART_USB, float_to_string(h*1000.0) );
   	uartWriteString( UART_USB, "ms" );
   	uartWriteString( UART_USB, "\r\n  Theta = [" );
   	uartWriteString( UART_USB, float_to_string(Theta_k_f32[0]) );
   	uartWriteString( UART_USB, "," );
   	uartWriteString( UART_USB, float_to_string(Theta_k_f32[1]) );
   	uartWriteString( UART_USB, "," );
   	uartWriteString( UART_USB, float_to_string(Theta_k_f32[2]) );
   	uartWriteString( UART_USB, " ];" );
   	uartWriteString( UART_USB, "\r\n  a_0 = " );
   	uartWriteString( UART_USB, float_to_string(a0) );
   	uartWriteString( UART_USB, "\r\n  R estimado = " );
   	uartWriteString( UART_USB, float_to_string(Restimated) );
   	uartWriteString( UART_USB, "\r\n" );


   uartWriteString( UART_USB, "\r\nCalculo finalizado!!\r\n" );
   while(TRUE){
      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );	// Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
   }
}
