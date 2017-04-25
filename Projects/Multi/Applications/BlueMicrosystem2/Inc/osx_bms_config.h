/**
  ******************************************************************************
  * @file    osx_bms_config.h
  * @author  Central LAB
  * @version V2.2.0
  * @date    23-December-2016
  * @brief   osx Bluemicrosystem configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OSX_BMS_CONFIG_H
#define __OSX_BMS_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/* For reading the oxMotion licenses from osx_license.h files */
/* #define OSX_BMS_LICENSE_H_FILE */

/* For enabling AcousticSL integration */
#ifndef STM32_SENSORTILE
#define OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
#endif /* STM32_SENSORTILE */


/* Define the BlueMicrosystem MAC address, otherwise it will create a Unique MAC */
//#define MAC_BLUEMS 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

#ifndef MAC_BLUEMS
/* For creating one MAC related to STM32 UID, Otherwise the BLE will use it's random MAC */
#define MAC_STM32UID_BLUEMS
#endif /* MAC_BLUEMS */

/* Define The transmission interval in Multiple of 10ms for quaternions*/
#define QUAT_UPDATE_MUL_10MS 3

/* Define How Many quaterions you want to trasmit (from 1 to 3) */
#define SEND_N_QUATERNIONS 3

/* IMPORTANT 
The Sensors fusion runs at 100Hz so like MAXIMUM it possible to send:
1 quaternion every 10ms
2 quaternions every 20ms
3 quaternions every 30ms

if QUAT_UPDATE_MUL_10MS!=3, then SEND_N_QUATERNIONS must be ==1
*/

/*************** Debug Defines ******************/
/* For enabling the printf on UART */
#ifdef STM32_SENSORTILE
  /* Enabling this define for SensorTile..
   * it will introduce a delay of 10Seconds before starting the application
   * for having time to open the Terminal
   * for looking the BlueMicrosystem Initialization phase */
 // #define OSX_BMS_ENABLE_PRINTF
#else /* STM32_SENSORTILE */
  /* For Nucleo it's enable by default */
  #define OSX_BMS_ENABLE_PRINTF
#endif /* STM32_SENSORTILE */

/* For enabling connection and notification subscriptions debug */
#define OSX_BMS_DEBUG_CONNECTION

/* For enabling trasmission for notified services (except for quaternions) */
#define OSX_BMS_DEBUG_NOTIFY_TRAMISSION

/* Define The transmission interval in Multiple of 10ms for Microphones dB Values */
#define MICS_DB_UPDATE_MUL_10MS 5

/* Define The transmission interval in Multiple of 10ms for Microphones dB Values */
#define ENV_UPDATE_MUL_100MS 5

/* For enabling the License Parsing Debug */
#define OSX_BMS_LIC_PARSING


/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define OSX_BMS_VERSION_MAJOR '2'
#define OSX_BMS_VERSION_MINOR '2'
#define OSX_BMS_VERSION_PATCH '0'

/* Define the BlueMicrosystem Name MUST be 7 char long */
#define NAME_BLUEMS 'B','M','2','V',OSX_BMS_VERSION_MAJOR,OSX_BMS_VERSION_MINOR,OSX_BMS_VERSION_PATCH

/* Package Name */
#define OSX_BMS_PACKAGENAME "BLUEMICROSYSTEM2"

#ifdef STM32_NUCLEO
#define AUDIO_VOLUME_VALUE       64
#define AUDIO_CHANNELS           2
#define AUDIO_SAMPLING_FREQUENCY 16000
#define PCM_AUDIO_IN_SAMPLES     AUDIO_SAMPLING_FREQUENCY/1000

/* Code for Acoustic Source Localization integration - Start Section */     
#define M1_EXT_B 0
#define M4_EXT_B 1
#define SIDE     147 /* Microphone distance */
/* Code for Acoustic Source Localization integration - End Section */   
     
#endif /* STM32_NUCLEO */

#ifdef STM32_SENSORTILE
#define AUDIO_VOLUME_VALUE       4
#define AUDIO_CHANNELS           1
#define AUDIO_SAMPLING_FREQUENCY 16000
#define PCM_AUDIO_IN_SAMPLES     AUDIO_SAMPLING_FREQUENCY/1000
#endif /* STM32_SENSORTILE */
     
/* Code for BlueVoice integration - Start Section */
#define BV_AUDIO_SAMPLING_FREQUENCY     8000
#define BV_AUDIO_VOLUME_VALUE           64
#define PCM_IN_SAMPLES_PER_MS           (((uint16_t)BV_AUDIO_SAMPLING_FREQUENCY)/1000)
#define AUDIO_IN_MS                     (1)       /*!< Number of ms of Audio given as input to the BlueVoice library.*/ 
#define BV_PCM_AUDIO_IN_SAMPLES         (PCM_IN_SAMPLES_PER_MS * AUDIO_IN_MS)
/* Code for BlueVoice integration - End Section */

#ifdef OSX_BMS_ENABLE_PRINTF
  #ifdef STM32_NUCLEO
    #define OSX_BMS_PRINTF(...) printf(__VA_ARGS__)
  #elif STM32_SENSORTILE
    #include "usbd_cdc_interface.h"
    #define OSX_BMS_PRINTF(...) {\
      char TmpBufferToWrite[256];\
      int32_t TmpBytesToWrite;\
      TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
      CDC_Fill_Buffer(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
    }
  #endif /* STM32_NUCLEO */
#else /* OSX_BMS_ENABLE_PRINTF */
  #define OSX_BMS_PRINTF(...)
#endif /* OSX_BMS_ENABLE_PRINTF */

/* STM32 Unique ID */
#ifdef USE_STM32F4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7590)
#endif /* USE_STM32L4XX_NUCLEO */

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)
/* Control Section */

#if ((SEND_N_QUATERNIONS<1) || (SEND_N_QUATERNIONS>3))
  #error "SEND_N_QUATERNIONS could be only 1,2 or 3"
#endif

#if ((QUAT_UPDATE_MUL_10MS!=3) && (SEND_N_QUATERNIONS!=1))
  #error "If QUAT_UPDATE_MUL_10MS!=3 then SEND_N_QUATERNIONS must be = 1"
#endif

#endif /* __OSX_BMS_CONFIG_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
