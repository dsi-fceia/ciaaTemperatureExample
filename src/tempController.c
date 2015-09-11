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
#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "tempController.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

static uint32_t controllerCounter = 0;
static uint32_t tempAVG = 0;

/** \brief File descriptor for digital output ports
 *
 * Device path /dev/dio/out/0
 */
static int32_t fd_out;

/** \brief File descriptor for ADC
 *
 * Device path /dev/serial/aio/in/0
 */
static int32_t fd_adc;

/** \brief File descriptor of the USB uart
 *
 * Device path /dev/serial/uart/1
 */
static int32_t fd_uart1;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void turnON_Heater(int32_t fd_out)
{
   /* write RGB R */
   uint8_t outputs;
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs |= RGBR;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

static void turnON_Cooler(int32_t fd_out){
	/* write RGB B */
   uint8_t outputs;
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs |= RGBB;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

static void turnOFF_Heater(int32_t fd_out)
{
   /* write RGB R */
   uint8_t outputs;
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs &= ~RGBR;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

static void turnOFF_Cooler(int32_t fd_out)
{
	/* write RGB B */
   uint8_t outputs;
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs &= ~RGBB;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

static void sendTemp_Uart(uint32_t tempNow)
{
   char data_to_send[9]; /* example 28.5 °C */

   /* Conversion int to ASCII */

   if ((tempNow/100) != 0)
   {
      data_to_send[0] = 48+tempNow/1000;
      tempNow %= 1000;
   }
   else
   {
      data_to_send[0] = ' ';
   }

   data_to_send[1] = 48+tempNow/100;
   tempNow %= 100;
   data_to_send[2] = 48+tempNow/10;
   tempNow %= 10;
   data_to_send[3] = '.';
   data_to_send[4] = 48+tempNow;
   data_to_send[5] = ' ';
   data_to_send[6] = '°';
   data_to_send[7] = 'C';
   data_to_send[8] = 13; // carriage return

   ciaaPOSIX_write(fd_uart1, data_to_send, ciaaPOSIX_strlen(data_to_send));
}


/*==================[external functions definition]==========================*/

extern void controller_init(void)
{
   /* open CIAA digital outputs */
   fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);

   /* open CIAA ADC */
   fd_adc = ciaaPOSIX_open("/dev/serial/aio/in/0", ciaaPOSIX_O_RDONLY);

   /* open UART connected to USB bridge (FT2232) */
   fd_uart1 = ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR);

   sensorLM35_init(fd_adc, ciaaCHANNEL_0);

   SetRelAlarm(ActivatePeriodicTask, 350, SAMPLES_TIME);
}


TASK(ControllerTask)
{
   tempAVG += sensorLM35_getTempCelcius(fd_adc);
   controllerCounter++;

   if(controllerCounter == SAMPLES_NUM)
   {
      tempAVG *= 10; /* To use one decimal */
      tempAVG /= controllerCounter; /* Average *10 */
      sendTemp_Uart(tempAVG);

      if(tempAVG > (TEMP_MAX*10))
      {
    	  turnON_Heater(fd_out);
    	  turnOFF_Cooler(fd_out);
      }
      else if(tempAVG < (TEMP_MIN*10))
      {
    	  turnON_Cooler(fd_out);
    	  turnOFF_Heater(fd_out);
      }
      else
      {
     	  turnOFF_Heater(fd_out);
    	  turnOFF_Cooler(fd_out);
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
