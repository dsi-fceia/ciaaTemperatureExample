/* Copyright 2015, Juan Pablo Vecchio
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Short description of this file
 **
 ** Long description of this file
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Projects CIAA Firmware Projects
 ** @{ */
/** \addtogroup ciaaTemperatureExample ciaa Temperature Example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * JPV         Juan Pablo Vecchio
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20150904 v0.0.1   JPV   initial version
 */

/*==================[inclusions]=============================================*/
#include "tempController.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

static uint32_t controllerCounter = 0;
static uint32_t tempAVG = 0;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void controller_init(void)
{
   SetRelAlarm(ActivatePeriodicTask, 350, SAMPLES_TIME);
}

static void turnON_Heater(void)
{
   /* write RGB R */
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs |= RGBR;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

static void turnON_Cooler(void){
	/* write RGB B */
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs |= RGBB;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

static void turnOFF_Heater(void)
{
   /* write RGB R */
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs &= ~RGBR;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

static void turnOFF_Cooler(void)
{
	/* write RGB B */
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs &= ~RGBB;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

static void sendTemp_Uart(uint32_t tempNow)
{
	char data_to_send[7]; /* example 28 °C */

	/* Conversion int to ASCII */
   data_to_send[0] = 48+(tempNow)/10;
   data_to_send[1] = 48+(tempNow%10);
   data_to_send[2] = ' ';
   data_to_send[3] = '°';
   data_to_send[4] = 'C';
   data_to_send[5] = 10; // Salto de linea
   data_to_send[6] = 13; // Retorno de carro

   ciaaPOSIX_write(fd_uart1, data_to_send, ciaaPOSIX_strlen(data_to_send));
}


/*==================[external functions definition]==========================*/


TASK(ControllerTask)
{
   tempAVG += sensorLM35_getTempCelcius();
   controllerCounter++;

   if(controllerCounter == SAMPLES_NUM)
   {
      sendTemp_Uart(tempAVG);

      if(tempNow > TEMP_MAX)
      {
    	  turnON_Heater();
    	  turnOFF_Cooler();
      }
      else if(tempNow < TEMP_MIN)
      {
    	  turnON_Cooler();
    	  turnOFF_Heater();
      }
      else
      {
     	  turnOFF_Heater();
    	  turnOFF_Cooler();
      }
      controllerCounter = 0;
      tempAVG = 0;
   }

   /* terminate task */
   TerminateTask();
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
