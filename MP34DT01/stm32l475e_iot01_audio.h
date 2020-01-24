/**
  ******************************************************************************
  * @file    stm32l475e_iot01_audio.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32l475e_iot01_audio.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32L475E_IOT01_AUDIO_H
#define STM32L475E_IOT01_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l475e_iot01_conf.h"
#include "stm32l475e_iot01_errno.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L475E_IOT01
  * @{
  */

/** @addtogroup STM32L475E_IOT01_AUDIO
  * @{
  */

/** @defgroup STM32L475E_IOT01_AUDIO_IN_Exported_Types STM32L475E_IOT01 AUDIO_IN Exported Types
  * @{
  */
typedef struct
{
  uint32_t Device;        /* Output or input device */
  uint32_t SampleRate;    /* From 8kHz to 192 kHz */
  uint32_t BitsPerSample; /* From 8 bits per sample to 32 bits per sample */
  uint32_t ChannelsNbr;   /* 1 for mono and 2 for stereo */
  uint32_t Volume;        /* In percentage from 0 to 100 */
} BSP_AUDIO_Init_t;

#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
typedef struct
{
  pDFSDM_Filter_CallbackTypeDef   pMspFilterInitCb;
  pDFSDM_Filter_CallbackTypeDef   pMspFilterDeInitCb;
  pDFSDM_Channel_CallbackTypeDef  pMspChannelInitCb;
  pDFSDM_Channel_CallbackTypeDef  pMspChannelDeInitCb;
} BSP_AUDIO_IN_Cb_t;
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */

typedef struct
{
  /* Filter parameters */
  DFSDM_Filter_TypeDef   *FilterInstance;
  uint32_t               RegularTrigger;
  uint32_t               SincOrder;
  uint32_t               Oversampling;
  /* Channel parameters */
  DFSDM_Channel_TypeDef *ChannelInstance;
  uint32_t              DigitalMicPins;
  uint32_t              DigitalMicType;
  uint32_t              Channel4Filter;
  uint32_t              ClockDivider;
  uint32_t              RightBitShift;
} MX_DFSDM_InitTypeDef;

/* Audio in context */
typedef struct
{
  uint32_t  Device;              /* Audio IN device to be used     */
  uint32_t  SampleRate;          /* Audio IN Sample rate           */
  uint32_t  BitsPerSample;       /* Audio IN Sample resolution     */
  uint32_t  ChannelsNbr;         /* Audio IN number of channel     */
  uint8_t   *pBuff;              /* Audio IN record buffer         */
  uint32_t  Size;                /* Audio IN record buffer size    */
  uint32_t  Volume;              /* Audio IN volume                */
  uint32_t  State;               /* Audio IN State                 */
} AUDIO_IN_Ctx_t;
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_AUDIO_IN_Exported_Constants STM32L475E_IOT01 AUDIO_IN Exported Constants
  * @{
  */
/* Audio in instances */
#define AUDIO_IN_INSTANCES_NBR 1U

/* Audio input devices */
#define AUDIO_IN_DIGITAL_MIC1      0x10U /* Left digital microphone  */
#define AUDIO_IN_DIGITAL_MIC2      0x20U /* Right digital microphone */
#define AUDIO_IN_DIGITAL_MIC       (AUDIO_IN_DIGITAL_MIC1 | AUDIO_IN_DIGITAL_MIC2)

/* Audio in states */
#define AUDIO_IN_STATE_RESET     0U
#define AUDIO_IN_STATE_RECORDING 1U
#define AUDIO_IN_STATE_STOP      2U
#define AUDIO_IN_STATE_PAUSE     3U

/* Audio sample rate */
#define AUDIO_FREQUENCY_192K 192000U
#define AUDIO_FREQUENCY_176K 176400U
#define AUDIO_FREQUENCY_96K   96000U
#define AUDIO_FREQUENCY_88K   88200U
#define AUDIO_FREQUENCY_48K   48000U
#define AUDIO_FREQUENCY_44K   44100U
#define AUDIO_FREQUENCY_32K   32000U
#define AUDIO_FREQUENCY_22K   22050U
#define AUDIO_FREQUENCY_16K   16000U
#define AUDIO_FREQUENCY_11K   11025U
#define AUDIO_FREQUENCY_8K     8000U

/* Audio bits per sample */
#define AUDIO_RESOLUTION_8b   8U
#define AUDIO_RESOLUTION_16b 16U
#define AUDIO_RESOLUTION_24b 24U
#define AUDIO_RESOLUTION_32b 32U

/* Audio in GPIOs */
#define AUDIO_DFSDM1_CKOUT_GPIO_PORT          GPIOE
#define AUDIO_DFSDM1_CKOUT_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define AUDIO_DFSDM1_CKOUT_GPIO_PIN           GPIO_PIN_9
#define AUDIO_DFSDM1_CKOUT_GPIO_AF            GPIO_AF6_DFSDM1

#define AUDIO_DFSDM1_DATIN2_GPIO_PORT         GPIOE
#define AUDIO_DFSDM1_DATIN2_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define AUDIO_DFSDM1_DATIN2_GPIO_PIN          GPIO_PIN_7
#define AUDIO_DFSDM1_DATIN2_GPIO_AF           GPIO_AF6_DFSDM1

#define AUDIO_DFSDM1_CLK_ENABLE()             __HAL_RCC_DFSDM1_CLK_ENABLE()
#define AUDIO_DFSDM1_CLK_DISABLE()            __HAL_RCC_DFSDM1_CLK_DISABLE()
/**
  * @}
  */

/** @addtogroup STM32L475E_IOT01_AUDIO_IN_Exported_Variables
  * @{
  */
/* Audio in context */
extern AUDIO_IN_Ctx_t  Audio_In_Ctx[AUDIO_IN_INSTANCES_NBR];

/* Audio in DFSDM handles */
extern DFSDM_Channel_HandleTypeDef haudio_in_dfsdm_channel[2];
extern DFSDM_Filter_HandleTypeDef  haudio_in_dfsdm_filter[2];

/* Audio in DFSDM internal buffers and global varibales */
extern int32_t  Audio_DigMic1RecBuff[BSP_AUDIO_IN_DEFAULT_BUFFER_SIZE];
extern int32_t  Audio_DigMic2RecBuff[BSP_AUDIO_IN_DEFAULT_BUFFER_SIZE];
extern uint32_t Audio_DmaDigMic1RecHalfBuffCplt;
extern uint32_t Audio_DmaDigMic1RecBuffCplt;
extern uint32_t Audio_DmaDigMic2RecHalfBuffCplt;
extern uint32_t Audio_DmaDigMic2RecBuffCplt;
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_AUDIO_IN_Exported_Functions STM32L475E_IOT01 AUDIO_IN Exported Functions
  * @{
  */
int32_t           BSP_AUDIO_IN_Init(uint32_t Instance, BSP_AUDIO_Init_t *AudioInit);
int32_t           BSP_AUDIO_IN_DeInit(uint32_t Instance);
int32_t           BSP_AUDIO_IN_Record(uint32_t Instance, uint8_t *pData, uint32_t NbrOfBytes);
int32_t           BSP_AUDIO_IN_Pause(uint32_t Instance);
int32_t           BSP_AUDIO_IN_Resume(uint32_t Instance);
int32_t           BSP_AUDIO_IN_Stop(uint32_t Instance);
int32_t           BSP_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume);
int32_t           BSP_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume);
int32_t           BSP_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate);
int32_t           BSP_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);
int32_t           BSP_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device);
int32_t           BSP_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device);
int32_t           BSP_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);
int32_t           BSP_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);
int32_t           BSP_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr);
int32_t           BSP_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr);
int32_t           BSP_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State);

#if ((USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) || (USE_HAL_SAI_REGISTER_CALLBACKS == 1))
int32_t           BSP_AUDIO_IN_RegisterDefaultMspCallbacks(uint32_t Instance);
int32_t           BSP_AUDIO_IN_RegisterMspCallbacks(uint32_t Instance, BSP_AUDIO_IN_Cb_t *CallBacks);
#endif /* ((USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) || (USE_HAL_SAI_REGISTER_CALLBACKS == 1)) */

void              BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance);
void              BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance);
void              BSP_AUDIO_IN_Error_CallBack(uint32_t Instance);

void              BSP_AUDIO_IN_IRQHandler(uint32_t Instance, uint32_t Device);

HAL_StatusTypeDef MX_DFSDM1_ClockConfig(DFSDM_Channel_HandleTypeDef *hDfsdmChannel, uint32_t SampleRate);
HAL_StatusTypeDef MX_DFSDM1_Init(DFSDM_Filter_HandleTypeDef *hDfsdmFilter, DFSDM_Channel_HandleTypeDef *hDfsdmChannel, MX_DFSDM_InitTypeDef *MXInit);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32L475E_IOT01_AUDIO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
