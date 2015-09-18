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

#ifndef _SENSORLM35_H_
#define _SENSORLM35_H_
/** \brief sensorLM35 header file
 **
 ** Configuration for sensor LM35
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

/*==================[macros]=================================================*/

/** \brief Conversion constant for LM35
 **
 **  From LM35 Datasheet  1°C --> 0.010V
 **  ADC Counts vs Voltage = 1023 --> 3.3 V
 **
 **/
#define LM35_CONVERSION_CONSTANT 330/1023

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/** \brief Sensor LM35 initialization
 **
 ** \param[in] fdAin file descriptor for ADC
 ** \param[in] channel channel of ADC
 **/
extern void sensorLM35_init(int32_t fdAin, int32_t channel);

/** \brief Get temperature in celsius
 **
 ** \return temperature in Celsius
 **/
extern int32_t sensorLM35_getTempCelsius(void);

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef _SENSORLM35_H_ */

