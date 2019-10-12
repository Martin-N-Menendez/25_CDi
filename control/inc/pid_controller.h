/*=====[pid_controller]========================================================
 * Copyright 2018 Diego Fernandez <dfernandez202@gmail.com>
 * Copyright 2018 Eric Nicolas Pernia <ericpernia@gmail.com>
 * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.2.0
 * Creation Date: 2018/09/24
 */

/*=====[Avoid multiple inclusion - begin]====================================*/

#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

/*=====[Inclusions of public function dependencies]==========================*/

#include "sapi.h"

/*=====[C++ - begin]=========================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=====[Definition macros of public constants]===============================*/

#define CAPACITOR   0.000001f //  1 uF
#define RESISTOR    10000.0f  // 10 Kohm

#define DAC_REFERENCE_VALUE_HIGH   666  // 1023 = 3.3V, 666 = 2.15V
#define DAC_REFERENCE_VALUE_LOW    356  // 1023 = 3.3V, 356 = 1.15V

/*=====[Public function-like macros]=========================================*/


/*=====[Definitions of public data types]====================================*/

typedef struct {
   float Kp;
   float Ki;
   float Kd;
   float h;
   float N;
   float b;
} pidConfig_t;

typedef struct {
   float pastD;
   float pastY;
   float futureI;
} pidState_t;

typedef struct {
   float K;
   float K0;
   float Capacitor;
} ppConfig_t;


/*=====[Prototypes (declarations) of public functions]=======================*/

void pidInit( pidConfig_t* config, pidState_t* state );

void pidControl( pidConfig_t* config, pidState_t* state );

void polePlacementControlInit( void );

void polePlacementControl( ppConfig_t* config);


/*=====[Prototypes (declarations) of public interrupt functions]=============*/

/*=====[C++ - end]===========================================================*/

#ifdef __cplusplus
}
#endif

/*=====[Avoid multiple inclusion - end]======================================*/

#endif /* _PID_CONTROLLER_H_ */
