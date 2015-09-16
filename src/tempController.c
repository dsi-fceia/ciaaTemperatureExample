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

/** \brief tempController source file
 **
 ** This is a mini example of a temperature Controller.
 ** Temperature is sent to the UART.
 ** Using Leds as Cooler and Heater
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
#include "tempController.h"   /* <= own header */

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

/** \brief Turn ON the Heater -> LED RGB RED
 **
 ** \param[in] fd_out file descriptor for digital output ports
 **/
static void turnON_Heater(int32_t fd_out)
{
   /* write RGB R */
   uint8_t outputs;
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs |= RGBR;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

/** \brief Turn ON the Cooler -> LED RGB BLUE
 **
 ** \param[in] fd_out file descriptor for digital output ports
 **/
static void turnON_Cooler(int32_t fd_out){
	/* write RGB B */
   uint8_t outputs;
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs |= RGBB;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

/** \brief Turn OFF the Heater -> LED RGB RED
 **
 ** \param[in] fd_out file descriptor for digital output ports
 **/
static void turnOFF_Heater(int32_t fd_out)
{
   /* write RGB R */
   uint8_t outputs;
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs &= ~RGBR;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

/** \brief Turn OFF the Cooler -> LED RGB BLUE
 **
 ** \param[in] fd_out file descriptor for digital output ports
 **/
static void turnOFF_Cooler(int32_t fd_out)
{
	/* write RGB B */
   uint8_t outputs;
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs &= ~RGBB;
   ciaaPOSIX_write(fd_out, &outputs, 1);
}

/** \brief Convert temperature to ascii
 **
 **  This function Converts an int temperature increased tenfold to ascii
 **  Adds a space, degree symbol, celsius letter and carriage return
 **  Example: 285 -> "28.5 °C"
 **
 ** \param[in] temp_to_ascii temperature to convert multiplied by 10
 ** \param[in] asciiTemp buffer size 10 for string conversion
 ** \return pointer to converted temperature
 **/
static char* convert_temp_to_ascii(uint32_t temp_to_ascii, char* asciiTemp)
{
   if ((temp_to_ascii/1000) != 0)
   {
      asciiTemp[0] = 48+temp_to_ascii/1000;
      temp_to_ascii %= 1000;
   }
   else
   {
      asciiTemp[0] = 32;
   }

   asciiTemp[1] = 48+temp_to_ascii/100;
   temp_to_ascii %= 100;
   asciiTemp[2] = 48+temp_to_ascii/10;
   temp_to_ascii %= 10;
   asciiTemp[3] = '.';
   asciiTemp[4] = 48+temp_to_ascii;
   asciiTemp[5] = ' ';
   asciiTemp[6] = 167;
   asciiTemp[7] = 'C';
   asciiTemp[8] = 13; // carriage return
   asciiTemp[9] = '\0';

   return asciiTemp;
}

/** \brief UART initialization
 **
 ** \param[in] baudrate baud rate for uart usb
 ** \param[in] fifolevel fifo trigger level for uart usb
 **/
static void uart_init(int32_t baudrate, int32_t fifolevel)
{
   /* change baud rate for uart usb */
   ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)baudrate);

   /* change FIFO TRIGGER LEVEL for uart usb */
   ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)fifolevel);

   /* Send to UART controller configuration */
   char tempMaxascii[10];
   convert_temp_to_ascii((TEMP_MAX)*10, tempMaxascii);
   char tempMinascii[10];
   convert_temp_to_ascii((TEMP_MIN)*10, tempMinascii);

   char message[] = "------ Temperature Controller ------\r\n\n";
   ciaaPOSIX_write(fd_uart1, message, ciaaPOSIX_strlen(message));

   char message2[] = "Temperature Maxima:\r\n";
   ciaaPOSIX_write(fd_uart1, message2, ciaaPOSIX_strlen(message2));

   ciaaPOSIX_write(fd_uart1, tempMaxascii, ciaaPOSIX_strlen(tempMaxascii));

   char message3[] = "\nTemperature Minima:\r\n";
   ciaaPOSIX_write(fd_uart1, message3, ciaaPOSIX_strlen(message3));

   ciaaPOSIX_write(fd_uart1, tempMinascii, ciaaPOSIX_strlen(tempMinascii));

   char message4[] = "\n\nTemperature Actual:\r\n";
   ciaaPOSIX_write(fd_uart1, message4, ciaaPOSIX_strlen(message4));
}

/** \brief Send Temperature to UART
 **
 ** \param[in] tempNow temperature to send multiplied by 10
 **/
static void sendTemp_Uart(uint32_t tempNow)
{
   char data_to_send[10];
   uint8_t outputs;

   convert_temp_to_ascii(tempNow, data_to_send);

   ciaaPOSIX_write(fd_uart1, data_to_send, ciaaPOSIX_strlen(data_to_send));

   /* Toggle LED 1 */
   ciaaPOSIX_read(fd_out, &outputs, 1);
   outputs ^= LED1;
   ciaaPOSIX_write(fd_out, &outputs, 1);
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

   uart_init(ciaaBAUDRATE_115200, ciaaFIFO_TRIGGER_LEVEL3);

   sensorLM35_init(fd_adc, ciaaCHANNEL_3);

   SetRelAlarm(ActivatePeriodicTask, 350, SAMPLES_TIME);
}

/** \brief Controller task
 *
 * This task is started automatically every time that the alarm
 * ActivatePeriodicTask expires.
 *
 * Read temperature every SAMPLES_TIME
 * Calculate temperature average for SAMPLES_NUM samples
 * Send temperature to UART
 * Turn ON/OFF the actuators
 */
TASK(ControllerTask)
{
   tempAVG += sensorLM35_getTempCelsius();
   controllerCounter++;

   if(controllerCounter == SAMPLES_NUM)
   {
      tempAVG *= 10; /* To use one decimal */
      tempAVG /= controllerCounter; /* Average *10 */
      sendTemp_Uart(tempAVG);

      if(tempAVG > (TEMP_MAX*10))
      {
    	  turnOFF_Heater(fd_out);
    	  turnON_Cooler(fd_out);
      }
      else if(tempAVG < (TEMP_MIN*10))
      {
    	  turnOFF_Cooler(fd_out);
    	  turnON_Heater(fd_out);
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
