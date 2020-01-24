/**
  ******************************************************************************
  * @file    stm32l475e_iot01_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio driver for the B-L475E-IOT01
  *          board.
  @verbatim
  How To use this driver:
  -----------------------
   + Call the function BSP_AUDIO_IN_Init() for AUDIO IN initialization:
        Instance : Select the input instance. Can be only 0 (DFSDM).
        AudioInit: Audio In structure to select the following parameters.
                   - Device: Select the input device (digital mic1, mic2, mic1 & mic2).
                   - SampleRate: Select the input sample rate (8Khz .. 96Khz).
                   - BitsPerSample: Select the input resolution (16 or 32bits per sample).
                   - ChannelsNbr: Select the input channels number(1 for mono, 2 for stereo).
                   - Volume: Select the input volume(0% .. 100%).

      This function configures all the hardware required for the audio application (DFSDM
      GPIOs, DMA and interrupt if needed). This function returns BSP_ERROR_NONE if configuration is OK.

      User can update the DFSDM or the clock configurations by overriding the weak MX functions MX_DFSDM1_Init()
      and MX_DFDSM1_ClockConfig()
      User can override the default MSP configuration and register his own MSP callbacks (defined at application level)
      by calling BSP_AUDIO_IN_RegisterMspCallbacks() function.
      User can restore the default MSP configuration by calling BSP_AUDIO_IN_RegisterDefaultMspCallbacks().
      To use these two functions, user have to enable USE_HAL_DFSDM_REGISTER_CALLBACKS within stm32l4xx_hal_conf.h file.

   + Call the function BSP_EVAL_AUDIO_IN_Record() to record audio stream. The recorded data are stored to user buffer in raw
        (L, R, L, R ...).
        Instance : Select the input instance. Can be only 0 (DFSDM).
        pBuf: pointer to user buffer.
        NbrOfBytes: Total size of the buffer to be sent in Bytes.

   + Call the function BSP_AUDIO_IN_Pause() to pause recording.
   + Call the function BSP_AUDIO_IN_Resume() to resume recording.
   + Call the function BSP_AUDIO_IN_Stop() to stop recording.
   + Call the function BSP_AUDIO_IN_SetDevice() to update the AUDIO IN device.
   + Call the function BSP_AUDIO_IN_GetDevice() to get the AUDIO IN device.
   + Call the function BSP_AUDIO_IN_SetSampleRate() to update the AUDIO IN sample rate.
   + Call the function BSP_AUDIO_IN_GetSampleRate() to get the AUDIO IN sample rate.
   + Call the function BSP_AUDIO_IN_SetBitPerSample() to update the AUDIO IN resolution.
   + Call the function BSP_AUDIO_IN_GetBitPerSample() to get the AUDIO IN resolution.
   + Call the function BSP_AUDIO_IN_SetChannelsNbr() to update the AUDIO IN number of channels.
   + Call the function BSP_AUDIO_IN_GetChannelsNbr() to get the AUDIO IN number of channels.
   + Call the function BSP_AUDIO_IN_SetVolume() to update the AUDIO IN volume.
   + Call the function BSP_AUDIO_IN_GetVolume() to get the AUDIO IN volume.
   + Call the function BSP_AUDIO_IN_GetState() to get the AUDIO IN state.

   + For each mode, you may need to implement the relative callback functions into your code.
      The Callback functions are named AUDIO_IN_XXX_CallBack() and only their prototypes are declared in
      the stm32l475e_iot01_audio.h file (refer to the example for more details on the callbacks implementations).

   + The driver API and the callback functions are at the end of the stm32l475e_iot01_audio.h file.

  @endverbatim
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32l475e_iot01_audio.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L475E_IOT01
  * @{
  */

/** @defgroup STM32L475E_IOT01_AUDIO STM32L475E_IOT01 AUDIO
  * @{
  */

/** @defgroup STM32L475E_IOT01_AUDIO_IN_Private_Macros STM32L475E_IOT01 AUDIO_IN Private Macros
  * @{
  */
#define DFSDM_OVER_SAMPLING(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (256U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (256U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (32U) : (16U)

#define DFSDM_CLOCK_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (32U) : (32U)

#define DFSDM_FILTER_ORDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (DFSDM_FILTER_SINC3_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (DFSDM_FILTER_SINC3_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (DFSDM_FILTER_SINC3_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (DFSDM_FILTER_SINC3_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (DFSDM_FILTER_SINC3_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (DFSDM_FILTER_SINC4_ORDER) : (DFSDM_FILTER_SINC5_ORDER)

#define DFSDM_MIC_BIT_SHIFT(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (5U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (6U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (3U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (3U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (6U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (0U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (2U) : (4U)

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_AUDIO_IN_Exported_Variables STM32L475E_IOT01 AUDIO_IN Exported Variables
  * @{
  */
/* Audio in context */
AUDIO_IN_Ctx_t  Audio_In_Ctx[AUDIO_IN_INSTANCES_NBR] = {{AUDIO_IN_DIGITAL_MIC,
                                                         AUDIO_FREQUENCY_8K,
                                                         AUDIO_RESOLUTION_16b,
                                                         2U,
                                                         NULL,
                                                         0U,
                                                         50U,
                                                         AUDIO_IN_STATE_RESET}};

/* Audio in DFSDM handles */
DFSDM_Channel_HandleTypeDef haudio_in_dfsdm_channel[2] = {{0}, {0}};
DFSDM_Filter_HandleTypeDef  haudio_in_dfsdm_filter[2] = {{0}, {0}};

/* Audio in DFSDM internal buffers and global varibales */
int32_t  Audio_DigMic1RecBuff[BSP_AUDIO_IN_DEFAULT_BUFFER_SIZE];
int32_t  Audio_DigMic2RecBuff[BSP_AUDIO_IN_DEFAULT_BUFFER_SIZE];
uint32_t Audio_DmaDigMic1RecHalfBuffCplt;
uint32_t Audio_DmaDigMic1RecBuffCplt;
uint32_t Audio_DmaDigMic2RecHalfBuffCplt;
uint32_t Audio_DmaDigMic2RecBuffCplt;
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_AUDIO_IN_Private_Variables STM32L475E_IOT01 AUDIO_IN Private Variables
  * @{
  */
/* Audio in DMA handles used by DFSDM */
static DMA_HandleTypeDef hDmaDfsdm[2];

#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
static uint32_t AudioIn_IsMspCbValid[AUDIO_IN_INSTANCES_NBR] = {0};
#endif
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_AUDIO_IN_Private_Function_Prototypes STM32L475E_IOT01 AUDIO_IN Private Function Prototypes
  * @{
  */
static int32_t DFSDM_DeInit(DFSDM_Filter_HandleTypeDef *hDfsdmFilter, DFSDM_Channel_HandleTypeDef *hDfsdmChannel);
static void    DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
static void    DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
static void    DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
static void    DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
static void    DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
static void    DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
static void    DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
/**
  * @}
  */

/** @addtogroup STM32L475E_IOT01_AUDIO_IN_Exported_Functions
  * @{
  */
/**
  * @brief  Initialize the audio in peripherals.
  * @param  Instance Audio in instance.
  * @param  AudioInit Audio in init structure.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_Init(uint32_t Instance, BSP_AUDIO_Init_t *AudioInit)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  else if (AudioInit->BitsPerSample != AUDIO_RESOLUTION_16b)
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  else if (((AudioInit->Device == AUDIO_IN_DIGITAL_MIC) && (AudioInit->ChannelsNbr != 2U)) ||
           ((AudioInit->Device == AUDIO_IN_DIGITAL_MIC1) && (AudioInit->ChannelsNbr != 1U)) ||
           ((AudioInit->Device == AUDIO_IN_DIGITAL_MIC2) && (AudioInit->ChannelsNbr != 1U)))
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  else
  {
    /* Fill audio in context structure */
    Audio_In_Ctx[Instance].Device         = AudioInit->Device;
    Audio_In_Ctx[Instance].SampleRate     = AudioInit->SampleRate;
    Audio_In_Ctx[Instance].BitsPerSample  = AudioInit->BitsPerSample;
    Audio_In_Ctx[Instance].ChannelsNbr    = AudioInit->ChannelsNbr;
    Audio_In_Ctx[Instance].Volume         = AudioInit->Volume;

    /* Set DFSDM instances */
    haudio_in_dfsdm_channel[0].Instance = DFSDM1_Channel2;
    haudio_in_dfsdm_channel[1].Instance = DFSDM1_Channel1;
    haudio_in_dfsdm_filter[0].Instance  = DFSDM1_Filter0;
    haudio_in_dfsdm_filter[1].Instance  = DFSDM1_Filter1;

    /* Configure DFSDM clock according to the requested audio frequency */
    if (MX_DFSDM1_ClockConfig(&haudio_in_dfsdm_channel[0], AudioInit->SampleRate) != HAL_OK)
    {
      status = BSP_ERROR_CLOCK_FAILURE;
    }
    else
    {
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
      if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
      {
        DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[0]);
        DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[0]);
      }
      if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
      {
        DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[1]);
        DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[1]);
      }
#else
      /* Register the DFSDM MSP Callbacks */
      if(AudioIn_IsMspCbValid[Instance] == 0U)
      {
        if(BSP_AUDIO_IN_RegisterDefaultMspCallbacks(Instance) != BSP_ERROR_NONE)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      if (status == BSP_ERROR_NONE)
      {
        /* Prepare DFSDM peripheral initialization */
        MX_DFSDM_InitTypeDef mxDfsdmInit;
        if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
        {
          mxDfsdmInit.ChannelInstance = DFSDM1_Channel2;
          mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
          mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_FALLING;
          mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_2;
          mxDfsdmInit.FilterInstance  = DFSDM1_Filter0;
          mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
          mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(Audio_In_Ctx[Instance].SampleRate);
          if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0], &mxDfsdmInit) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
          if (status == BSP_ERROR_NONE)
          {
            /* Register DFSDM filter TC, HT and Error callbacks */
            if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
            else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
            else
            {
              if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
              {
                status = BSP_ERROR_PERIPH_FAILURE;
              }
            }
          }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
        }
        if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
        {
          mxDfsdmInit.ChannelInstance = DFSDM1_Channel1;
          mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
          mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_RISING;
          mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_1;
          mxDfsdmInit.FilterInstance  = DFSDM1_Filter1;
          if (Audio_In_Ctx[Instance].Device == AUDIO_IN_DIGITAL_MIC2)
          {
            mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
          }
          else
          {
            mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SYNC_TRIGGER;
          }
          mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(Audio_In_Ctx[Instance].SampleRate);
          mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(Audio_In_Ctx[Instance].SampleRate);
          if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1], &mxDfsdmInit) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
          if (status == BSP_ERROR_NONE)
          {
            /* Register DFSDM filter TC, HT and Error callbacks */
            if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
            else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
            else
            {
              if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
              {
                status = BSP_ERROR_PERIPH_FAILURE;
              }
            }
          }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
        }
        if (status == BSP_ERROR_NONE)
        {
          /* Initialise transfer control flag */
          Audio_DmaDigMic1RecHalfBuffCplt = 0;
          Audio_DmaDigMic1RecBuffCplt     = 0;
          Audio_DmaDigMic2RecHalfBuffCplt = 0;
          Audio_DmaDigMic2RecBuffCplt     = 0;

          /* Update audio in context state */
          Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_STOP;
        }
      }
    }
  }
  return status;
}

/**
  * @brief  De-initialize the audio in peripherals.
  * @param  Instance Audio in instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_DeInit(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_RESET)
  {
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      /* DFSDM peripheral de-initialization */
      if (DFSDM_DeInit(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1]) != BSP_ERROR_NONE)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
        DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[1]);
        DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[1]);
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      }
    }

    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      /* DFSDM peripheral de-initialization */
      if (DFSDM_DeInit(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0]) != BSP_ERROR_NONE)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
        DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[0]);
        DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[0]);
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      }
    }

    if (status == BSP_ERROR_NONE)
    {
      /* Update audio in context */
      Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_RESET;
    }
  }
  else
  {
    /* Nothing to do */
  }
  return status;
}

/**
  * @brief  Start recording audio stream to a data buffer for a determined size.
  * @param  Instance Audio in instance.
  * @param  pData Pointer on data buffer.
  * @param  NbrOfBytes Size of buffer in bytes. Maximum size is 65535 bytes.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_Record(uint32_t Instance, uint8_t* pData, uint32_t NbrOfBytes)
{
  int32_t  status = BSP_ERROR_NONE;

  if ((Instance >= AUDIO_IN_INSTANCES_NBR) || (pData == NULL) || (NbrOfBytes > 65535U))
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check the internal buffer size */
  else if ((NbrOfBytes / 2U) > BSP_AUDIO_IN_DEFAULT_BUFFER_SIZE)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    Audio_In_Ctx[Instance].pBuff = pData;
    Audio_In_Ctx[Instance].Size  = NbrOfBytes;

    /* Initialise transfer control flag */
    Audio_DmaDigMic1RecHalfBuffCplt = 0;
    Audio_DmaDigMic1RecBuffCplt     = 0;
    Audio_DmaDigMic2RecHalfBuffCplt = 0;
    Audio_DmaDigMic2RecBuffCplt     = 0;

    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      printf("Set up MIC2\n");
      /* Call the Media layer start function for MIC2 channel */
      if(HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_filter[1],
                                           Audio_DigMic2RecBuff,
                                          (Audio_In_Ctx[Instance].Size / (2U * Audio_In_Ctx[Instance].ChannelsNbr))) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }

    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      // printf("Set up MIC1\n");
      /* Call the Media layer start function for MIC1 channel */
      if(HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_filter[0],
                                           Audio_DigMic1RecBuff,
                                          (Audio_In_Ctx[Instance].Size / (2U * Audio_In_Ctx[Instance].ChannelsNbr))) != HAL_OK)
      {
        // printf("FAIL HAL_DFSDM_FilterRegularStart_DMA\n");
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else {
        // printf("OK HAL_DFSDM_FilterRegularStart_DMA\n");
      }
    }
    if (status == BSP_ERROR_NONE)
    {
      /* Update audio in state */
      Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_RECORDING;
    }
  }
  return status;
}

/**
  * @brief  Pause record of audio stream.
  * @param  Instance Audio in instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_Pause(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_RECORDING)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Call the Media layer stop function */
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      if(HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_filter[1]) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      if(HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_filter[0]) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (status == BSP_ERROR_NONE)
    {
      /* Update audio in state */
      Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_PAUSE;
    }
  }
  return status;
}

/**
  * @brief  Resume record of audio stream.
  * @param  Instance Audio in instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_Resume(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_PAUSE)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Initialise transfer control flag */
    Audio_DmaDigMic1RecHalfBuffCplt = 0;
    Audio_DmaDigMic1RecBuffCplt     = 0;
    Audio_DmaDigMic2RecHalfBuffCplt = 0;
    Audio_DmaDigMic2RecBuffCplt     = 0;

    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      /* Call the Media layer start function for MIC2 channel */
      if(HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_filter[1],
                                           Audio_DigMic2RecBuff,
                                          (Audio_In_Ctx[Instance].Size / (2U * Audio_In_Ctx[Instance].ChannelsNbr))) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      /* Call the Media layer start function for MIC1 channel */
      if(HAL_DFSDM_FilterRegularStart_DMA(&haudio_in_dfsdm_filter[0],
                                           Audio_DigMic1RecBuff,
                                          (Audio_In_Ctx[Instance].Size / (2U * Audio_In_Ctx[Instance].ChannelsNbr))) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (status == BSP_ERROR_NONE)
    {
      /* Update audio in state */
      Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_RECORDING;
    }
  }
  return status;
}

/**
  * @brief  Stop record of audio stream.
  * @param  Instance Audio in instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_Stop(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    /* Nothing to do */
  }
  else if ((Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_RECORDING) &&
           (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_PAUSE))
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Call the Media layer stop function */
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      if(HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_filter[1]) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      if(HAL_DFSDM_FilterRegularStop_DMA(&haudio_in_dfsdm_filter[0]) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
    }
    if (status == BSP_ERROR_NONE)
    {
      /* Update audio in state */
      Audio_In_Ctx[Instance].State = AUDIO_IN_STATE_STOP;
    }
  }
  return status;
}

/**
  * @brief  Set audio in volume.
  * @param  Instance Audio in instance.
  * @param  Volume Volume level in percentage from 0% to 100%.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume)
{
  int32_t status;

  if ((Instance >= AUDIO_IN_INSTANCES_NBR) || (Volume > 100U))
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Feature not supported */
  else
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  return status;
}

/**
  * @brief  Get audio in volume.
  * @param  Instance Audio in instance.
  * @param  Volume Pointer to volume level in percentage from 0% to 100%.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  int32_t status;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Feature not supported */
  else
  {
    *Volume = 0U;
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  return status;
}

/**
  * @brief  Set audio in sample rate.
  * @param  Instance Audio in instance.
  * @param  SampleRate Sample rate of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Check if sample rate is modified */
  else if (Audio_In_Ctx[Instance].SampleRate == SampleRate)
  {
    /* Nothing to do */
  }
  else
  {
    /* Configure DFSDM clock according to the requested audio frequency */
    if (MX_DFSDM1_ClockConfig(&haudio_in_dfsdm_channel[0], SampleRate) != HAL_OK)
    {
      status = BSP_ERROR_CLOCK_FAILURE;
    }
    else
    {
      MX_DFSDM_InitTypeDef mxDfsdmInit;

      /* DeInitialize DFSDM */
      if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
      {
        if (DFSDM_DeInit(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1]) != BSP_ERROR_NONE)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
        else
        {
          DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[1]);
          DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[1]);
        }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      }
      if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
      {
        if (DFSDM_DeInit(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0]) != BSP_ERROR_NONE)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
        else
        {
          DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[0]);
          DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[0]);
        }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      }

      /* ReInitialize DFSDM with new sample rate */
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
      if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
      {
        DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[0]);
        DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[0]);
      }
      if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
      {
        DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[1]);
        DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[1]);
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
      if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
      {
        mxDfsdmInit.ChannelInstance = DFSDM1_Channel2;
        mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(SampleRate);
        mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
        mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_FALLING;
        mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(SampleRate);
        mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_2;
        mxDfsdmInit.FilterInstance  = DFSDM1_Filter0;
        mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
        mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(SampleRate);
        mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(SampleRate);
        if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0], &mxDfsdmInit) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
        if (status == BSP_ERROR_NONE)
        {
          /* Register DFSDM filter TC, HT and Error callbacks */
          if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
          else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
          else
          {
            if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
          }
        }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
      }
      if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
      {
        mxDfsdmInit.ChannelInstance = DFSDM1_Channel1;
        mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(SampleRate);
        mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
        mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_RISING;
        mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(SampleRate);
        mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_1;
        mxDfsdmInit.FilterInstance  = DFSDM1_Filter1;
        if (Audio_In_Ctx[Instance].Device == AUDIO_IN_DIGITAL_MIC2)
        {
          mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
        }
        else
        {
          mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SYNC_TRIGGER;
        }
        mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(SampleRate);
        mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(SampleRate);
        if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1], &mxDfsdmInit) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
        if (status == BSP_ERROR_NONE)
        {
          /* Register DFSDM filter TC, HT and Error callbacks */
          if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
          else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
          else
          {
            if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
            {
              status = BSP_ERROR_PERIPH_FAILURE;
            }
          }
        }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
      }
    }
    /* Store new sample rate on audio in context */
    if (status == BSP_ERROR_NONE)
    {
      Audio_In_Ctx[Instance].SampleRate = SampleRate;
    }
  }
  return status;
}

/**
  * @brief  Get audio in sample rate.
  * @param  Instance Audio in instance.
  * @param  SampleRate Pointer to sample rate of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State == AUDIO_IN_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current audio in sample rate */
  else
  {
    *SampleRate = Audio_In_Ctx[Instance].SampleRate;
  }
  return status;
}

/**
  * @brief  Set audio in device.
  * @param  Instance Audio in instance.
  * @param  Device Device of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device)
{
  int32_t status = BSP_ERROR_NONE;

  UNUSED(Device);

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else if (Audio_In_Ctx[Instance].Device != Device)
  {
    MX_DFSDM_InitTypeDef mxDfsdmInit;

    /* DeInitialize DFSDM for previous microphones */
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2)
    {
      if (DFSDM_DeInit(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1]) != BSP_ERROR_NONE)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
      else
      {
        DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[1]);
        DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[1]);
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
    }
    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      if (DFSDM_DeInit(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0]) != BSP_ERROR_NONE)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
      else
      {
        DFSDM_FilterMspDeInit(&haudio_in_dfsdm_filter[0]);
        DFSDM_ChannelMspDeInit(&haudio_in_dfsdm_channel[0]);
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
    }

    /* ReInitialize DFSDM for new microphones */
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0)
    if (((Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[0]);
      DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[0]);
    }
    if (((Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
    {
      DFSDM_ChannelMspInit(&haudio_in_dfsdm_channel[1]);
      DFSDM_FilterMspInit(&haudio_in_dfsdm_filter[1]);
    }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 0) */
    if (((Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1) && (status == BSP_ERROR_NONE))
    {
      mxDfsdmInit.ChannelInstance = DFSDM1_Channel2;
      mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
      mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_FALLING;
      mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_2;
      mxDfsdmInit.FilterInstance  = DFSDM1_Filter0;
      mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
      mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(Audio_In_Ctx[Instance].SampleRate);
      if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[0], &haudio_in_dfsdm_channel[0], &mxDfsdmInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
      if (status == BSP_ERROR_NONE)
      {
        /* Register DFSDM filter TC, HT and Error callbacks */
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
        else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
        else
        {
          if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
        }
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
    }
    if (((Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
    {
      mxDfsdmInit.ChannelInstance = DFSDM1_Channel1;
      mxDfsdmInit.ClockDivider    = DFSDM_CLOCK_DIVIDER(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.DigitalMicPins  = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
      mxDfsdmInit.DigitalMicType  = DFSDM_CHANNEL_SPI_RISING;
      mxDfsdmInit.RightBitShift   = DFSDM_MIC_BIT_SHIFT(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.Channel4Filter  = DFSDM_CHANNEL_1;
      mxDfsdmInit.FilterInstance  = DFSDM1_Filter1;
      if (Device == AUDIO_IN_DIGITAL_MIC2)
      {
        mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SW_TRIGGER;
      }
      else
      {
        mxDfsdmInit.RegularTrigger  = DFSDM_FILTER_SYNC_TRIGGER;
      }
      mxDfsdmInit.SincOrder       = DFSDM_FILTER_ORDER(Audio_In_Ctx[Instance].SampleRate);
      mxDfsdmInit.Oversampling    = DFSDM_OVER_SAMPLING(Audio_In_Ctx[Instance].SampleRate);
      if (MX_DFSDM1_Init(&haudio_in_dfsdm_filter[1], &haudio_in_dfsdm_channel[1], &mxDfsdmInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
      if (status == BSP_ERROR_NONE)
      {
        /* Register DFSDM filter TC, HT and Error callbacks */
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_FilterRegConvCpltCallback) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
        else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_FilterRegConvHalfCpltCallback) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
        else
        {
          if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_ERROR_CB_ID, DFSDM_FilterErrorCallback) != HAL_OK)
          {
            status = BSP_ERROR_PERIPH_FAILURE;
          }
        }
      }
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
    }
    if (status == BSP_ERROR_NONE)
    {
      /* Store new device on context and update channel numbers */
      Audio_In_Ctx[Instance].Device = Device;
      Audio_In_Ctx[Instance].ChannelsNbr = (Device == AUDIO_IN_DIGITAL_MIC) ? 2U : 1U;
    }
  }
  return status;
}

/**
  * @brief  Get audio in device.
  * @param  Instance Audio in instance.
  * @param  Device Pointer to device of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State == AUDIO_IN_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current audio in device */
  else
  {
    *Device = Audio_In_Ctx[Instance].Device;
  }
  return status;
}

/**
  * @brief  Set bits per sample for the audio in stream.
  * @param  Instance Audio in instance.
  * @param  BitsPerSample Bits per sample of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else if (BitsPerSample != AUDIO_RESOLUTION_16b)
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Nothing to do because there is only one bits per sample supported (AUDIO_RESOLUTION_16b) */
  }
  return status;
}

/**
  * @brief  Get bits per sample for the audio in stream.
  * @param  Instance Audio in instance.
  * @param  BitsPerSample Pointer to bits per sample of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State == AUDIO_IN_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current bits per sample of audio in stream */
  else
  {
    *BitsPerSample = Audio_In_Ctx[Instance].BitsPerSample;
  }
  return status;
}

/**
  * @brief  Set channels number for the audio in stream.
  * @param  Instance Audio in instance.
  * @param  ChannelNbr Channels number of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else if (((Audio_In_Ctx[Instance].Device == AUDIO_IN_DIGITAL_MIC) && (ChannelNbr != 2U)) ||
           ((Audio_In_Ctx[Instance].Device == AUDIO_IN_DIGITAL_MIC1) && (ChannelNbr != 1U)) ||
           ((Audio_In_Ctx[Instance].Device == AUDIO_IN_DIGITAL_MIC2) && (ChannelNbr != 1U)))
  {
    status = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State != AUDIO_IN_STATE_STOP)
  {
    status = BSP_ERROR_BUSY;
  }
  else
  {
    /* Nothing to do because channels are already configurated and can't be modified */
  }
  return status;
}

/**
  * @brief  Get channels number for the audio in stream.
  * @param  Instance Audio in instance.
  * @param  ChannelNbr Pointer to channels number of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Check audio in state */
  else if (Audio_In_Ctx[Instance].State == AUDIO_IN_STATE_RESET)
  {
    status = BSP_ERROR_BUSY;
  }
  /* Get the current channels number of audio in stream */
  else
  {
    *ChannelNbr = Audio_In_Ctx[Instance].ChannelsNbr;
  }
  return status;
}

/**
  * @brief  Get current state for the audio in stream.
  * @param  Instance Audio in instance.
  * @param  State Pointer to state of the audio in stream.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  /* Get the current state of audio in stream */
  else
  {
    *State = Audio_In_Ctx[Instance].State;
  }
  return status;
}

#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register default BSP AUDIO IN msp callbacks.
  * @param  Instance AUDIO IN Instance.
  * @retval BSP status.
  */
int32_t BSP_AUDIO_IN_RegisterDefaultMspCallbacks(uint32_t Instance)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit callbacks */
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
    {
      if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[0], HAL_DFSDM_CHANNEL_MSPINIT_CB_ID, DFSDM_ChannelMspInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_MSPINIT_CB_ID, DFSDM_FilterMspInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[0], HAL_DFSDM_CHANNEL_MSPDEINIT_CB_ID, DFSDM_ChannelMspDeInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_MSPDEINIT_CB_ID, DFSDM_FilterMspDeInit) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
      }
    }

    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
    {
      if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[1], HAL_DFSDM_CHANNEL_MSPINIT_CB_ID, DFSDM_ChannelMspInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_MSPINIT_CB_ID, DFSDM_FilterMspInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[1], HAL_DFSDM_CHANNEL_MSPDEINIT_CB_ID, DFSDM_ChannelMspDeInit) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_MSPDEINIT_CB_ID, DFSDM_FilterMspDeInit) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
      }
    }

    if (status == BSP_ERROR_NONE)
    {
      AudioIn_IsMspCbValid[Instance] = 1U;
    }
  }
  /* Return BSP status */
  return status;
}

/**
  * @brief  Register BSP AUDIO IN msp callbacks.
  * @param  Instance AUDIO IN Instance.
  * @param  CallBacks Pointer to MspInit/MspDeInit callback functions.
  * @retval BSP status
  */
int32_t BSP_AUDIO_IN_RegisterMspCallbacks(uint32_t Instance, BSP_AUDIO_IN_Cb_t *CallBacks)
{
  int32_t status = BSP_ERROR_NONE;

  if (Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    status = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Register MspInit/MspDeInit callbacks */
    if ((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC1) == AUDIO_IN_DIGITAL_MIC1)
    {
      if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[0], HAL_DFSDM_CHANNEL_MSPINIT_CB_ID, CallBacks->pMspChannelInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_MSPINIT_CB_ID, CallBacks->pMspFilterInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[0], HAL_DFSDM_CHANNEL_MSPDEINIT_CB_ID, CallBacks->pMspChannelDeInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[0], HAL_DFSDM_FILTER_MSPDEINIT_CB_ID, CallBacks->pMspFilterDeInitCb) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
      }
    }

    if (((Audio_In_Ctx[Instance].Device & AUDIO_IN_DIGITAL_MIC2) == AUDIO_IN_DIGITAL_MIC2) && (status == BSP_ERROR_NONE))
    {
      if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[1], HAL_DFSDM_CHANNEL_MSPINIT_CB_ID, CallBacks->pMspChannelInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_MSPINIT_CB_ID, CallBacks->pMspFilterInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else if (HAL_DFSDM_Channel_RegisterCallback(&haudio_in_dfsdm_channel[1], HAL_DFSDM_CHANNEL_MSPDEINIT_CB_ID, CallBacks->pMspChannelDeInitCb) != HAL_OK)
      {
        status = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        if (HAL_DFSDM_Filter_RegisterCallback(&haudio_in_dfsdm_filter[1], HAL_DFSDM_FILTER_MSPDEINIT_CB_ID, CallBacks->pMspFilterDeInitCb) != HAL_OK)
        {
          status = BSP_ERROR_PERIPH_FAILURE;
        }
      }
    }

    if (status == BSP_ERROR_NONE)
    {
      AudioIn_IsMspCbValid[Instance] = 1U;
    }
  }
  /* Return BSP status */
  return status;
}
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */

/**
  * @brief  Manage the BSP audio in transfer complete event.
  * @param  Instance Audio in instance.
  * @retval None.
  */
__weak void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
}

/**
  * @brief  Manage the BSP audio in half transfer complete event.
  * @param  Instance Audio in instance.
  * @retval None.
  */
__weak void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
}

/**
  * @brief  Manages the BSP audio in error event.
  * @param  Instance Audio in instance.
  * @retval None.
  */
__weak void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
}

/**
  * @brief  BSP AUDIO IN interrupt handler.
  * @param  Instance Audio in instance.
  * @param  Device Device of the audio in stream.
  * @retval None.
  */
void BSP_AUDIO_IN_IRQHandler(uint32_t Instance, uint32_t Device)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);

  if (Device == AUDIO_IN_DIGITAL_MIC1)
  {
    HAL_DMA_IRQHandler(haudio_in_dfsdm_filter[0].hdmaReg);
  }
  else
  {
    HAL_DMA_IRQHandler(haudio_in_dfsdm_filter[1].hdmaReg);
  }
}

/**
  * @brief  DFSDM1 clock Config.
  * @param  hDfsdmChannel DFSDM channel handle.
  * @param  SampleRate Audio sample rate used to record the audio stream.
  * @note   The SAI PLL configuration done within this function assumes that
  *         the SAI PLL input is MSI clock and that MSI is already enabled at 4 MHz.
  * @retval HAL status.
  */
__weak HAL_StatusTypeDef MX_DFSDM1_ClockConfig(DFSDM_Channel_HandleTypeDef *hDfsdmChannel, uint32_t SampleRate)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hDfsdmChannel);

  HAL_StatusTypeDef        status;
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;

  /* Configure the SAI PLL according to the requested audio frequency */
  /* Retrieve actual RCC configuration */
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);

  /* Set the PLL configuration according to the audio frequency */
  if((SampleRate == AUDIO_FREQUENCY_11K) || (SampleRate == AUDIO_FREQUENCY_22K) || (SampleRate == AUDIO_FREQUENCY_44K) ||
     (SampleRate == AUDIO_FREQUENCY_88K) || (SampleRate == AUDIO_FREQUENCY_176K))
  {
    /* SAI1 clock config
    PLLSAI1_VCO = (4 Mhz / PLLSAI1M) * PLLSAI1N = 4 * 48 = VCO_192M
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 192/17 = 11.294 Mhz */
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1Source   = RCC_PLLSOURCE_MSI;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1M        = 1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 48;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 17;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_32K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K or AUDIO_FREQUENCY_192K */
  {
    /* SAI1 clock config
    PLLSAI1_VCO = (4 Mhz / PLLSAI1M) * PLLSAI1N = 4 * 86 = VCO_344M
    SAI_CK_x = PLLSAI1_VCO/PLLSAI1P = 344/7 = 49.142 Mhz */
    RCC_ExCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1Source   = RCC_PLLSOURCE_MSI;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1M        = 1;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1N        = 86;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1P        = 7;
    RCC_ExCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    RCC_ExCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;
  }
  status = HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);

  return status;
}

/**
  * @brief  Initialize DFSDM1.
  * @param  hDfsdmFilter  DFSDM filter handle.
  * @param  hDfsdmChannel DFSDM channel handle.
  * @param  MXInit DFSDM configuration structure.
  * @retval HAL_status.
  */
__weak HAL_StatusTypeDef MX_DFSDM1_Init(DFSDM_Filter_HandleTypeDef *hDfsdmFilter, DFSDM_Channel_HandleTypeDef *hDfsdmChannel, MX_DFSDM_InitTypeDef *MXInit)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* MIC channels initialization */
  hDfsdmChannel->Instance                      = MXInit->ChannelInstance;
  hDfsdmChannel->Init.OutputClock.Activation   = ENABLE;
  hDfsdmChannel->Init.OutputClock.Selection    = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  hDfsdmChannel->Init.OutputClock.Divider      = MXInit->ClockDivider;
  hDfsdmChannel->Init.Input.Multiplexer        = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hDfsdmChannel->Init.Input.DataPacking        = DFSDM_CHANNEL_STANDARD_MODE;
  hDfsdmChannel->Init.Input.Pins               = MXInit->DigitalMicPins;
  hDfsdmChannel->Init.SerialInterface.Type     = MXInit->DigitalMicType;
  hDfsdmChannel->Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hDfsdmChannel->Init.Awd.FilterOrder          = DFSDM_CHANNEL_SINC1_ORDER;
  hDfsdmChannel->Init.Awd.Oversampling         = 10;
  hDfsdmChannel->Init.Offset                   = 0;
  hDfsdmChannel->Init.RightBitShift            = MXInit->RightBitShift;

  if(HAL_OK != HAL_DFSDM_ChannelInit(hDfsdmChannel))
  {
    status = HAL_ERROR;
  }
  else
  {
    /* MIC filters  initialization */
    hDfsdmFilter->Instance                          = MXInit->FilterInstance;
    hDfsdmFilter->Init.RegularParam.Trigger         = MXInit->RegularTrigger;
    hDfsdmFilter->Init.RegularParam.FastMode        = ENABLE;
    hDfsdmFilter->Init.RegularParam.DmaMode         = ENABLE;
    hDfsdmFilter->Init.InjectedParam.Trigger        = DFSDM_FILTER_SW_TRIGGER;
    hDfsdmFilter->Init.InjectedParam.ScanMode       = DISABLE;
    hDfsdmFilter->Init.InjectedParam.DmaMode        = DISABLE;
    hDfsdmFilter->Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM8_TRGO;
    hDfsdmFilter->Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_BOTH_EDGES;
    hDfsdmFilter->Init.FilterParam.SincOrder        = MXInit->SincOrder;
    hDfsdmFilter->Init.FilterParam.Oversampling     = MXInit->Oversampling;
    hDfsdmFilter->Init.FilterParam.IntOversampling  = 1;

    if(HAL_DFSDM_FilterInit(hDfsdmFilter) != HAL_OK)
    {
      status = HAL_ERROR;
    }
    else
    {
      /* Configure regular channel */
      if(HAL_DFSDM_FilterConfigRegChannel(hDfsdmFilter, MXInit->Channel4Filter, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
      {
        status = HAL_ERROR;
      }
    }
  }

  return status;
}
/**
  * @}
  */

/** @defgroup STM32L475E_IOT01_AUDIO_Private_Functions STM32L475E_IOT01 AUDIO_IN Private Functions
  * @{
  */
/**
  * @brief  DeInitialize DFSDM.
  * @param  hDfsdmFilter  DFSDM filter handle.
  * @param  hDfsdmChannel DFSDM channel handle.
  * @retval BSP status.
  */
static int32_t DFSDM_DeInit(DFSDM_Filter_HandleTypeDef *hDfsdmFilter, DFSDM_Channel_HandleTypeDef *hDfsdmChannel)
{
  int32_t status = BSP_ERROR_NONE;

  /* MIC filters Deinitialization */
  if(HAL_DFSDM_FilterDeInit(hDfsdmFilter) != HAL_OK)
  {
    status = BSP_ERROR_PERIPH_FAILURE;
  }
  else
  {
    /* MIC channels Deinitialization */
    if(HAL_OK != HAL_DFSDM_ChannelDeInit(hDfsdmChannel))
    {
      status = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return status;
}

/**
  * @brief  Initialize DFSDM channel MSP.
  * @param  hdfsdm_channel DFSDM channel handle.
  * @retval None.
  */
static void DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel)
{
  if (((hdfsdm_channel->Instance == DFSDM1_Channel2) && ((Audio_In_Ctx[0].Device & AUDIO_IN_DIGITAL_MIC1) != 0U)) || \
      ((hdfsdm_channel->Instance == DFSDM1_Channel1) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable DFSDM clock */
    AUDIO_DFSDM1_CLK_ENABLE();

    /* DFSDM pins configuration: DFSDM1_CKOUT, DFSDM1_DATIN2 pins */
    AUDIO_DFSDM1_CKOUT_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = AUDIO_DFSDM1_CKOUT_GPIO_AF;
    GPIO_InitStruct.Pin       = AUDIO_DFSDM1_CKOUT_GPIO_PIN;
    HAL_GPIO_Init(AUDIO_DFSDM1_CKOUT_GPIO_PORT, &GPIO_InitStruct);

    AUDIO_DFSDM1_DATIN2_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Alternate = AUDIO_DFSDM1_DATIN2_GPIO_AF;
    GPIO_InitStruct.Pin       = AUDIO_DFSDM1_DATIN2_GPIO_PIN;
    HAL_GPIO_Init(AUDIO_DFSDM1_DATIN2_GPIO_PORT, &GPIO_InitStruct);
  }
}

/**
  * @brief  DeInitialize DFSDM channel MSP.
  * @param  hdfsdm_channel DFSDM channel handle.
  * @retval None.
  */
static void DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel)
{
  if (((hdfsdm_channel->Instance == DFSDM1_Channel2) && ((Audio_In_Ctx[0].Device & AUDIO_IN_DIGITAL_MIC1) != 0U)) || \
      ((hdfsdm_channel->Instance == DFSDM1_Channel1) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    /* De-initialize DFSDM1_CKOUT, DFSDM1_DATIN2 pins */
    HAL_GPIO_DeInit(AUDIO_DFSDM1_CKOUT_GPIO_PORT, AUDIO_DFSDM1_CKOUT_GPIO_PIN);
    HAL_GPIO_DeInit(AUDIO_DFSDM1_DATIN2_GPIO_PORT, AUDIO_DFSDM1_DATIN2_GPIO_PIN);

    /* Disable DFSDM1 */
    AUDIO_DFSDM1_CLK_DISABLE();
  }
}

/**
  * @brief  Initialize DFSDM filter MSP.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
static void DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  if(hdfsdm_filter->Instance == DFSDM1_Filter0)
  {
    /* Enable the DMA clock */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Configure the hDmaDfsdm[0] handle parameters */
    hDmaDfsdm[0].Init.Request             = DMA_REQUEST_0;
    hDmaDfsdm[0].Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hDmaDfsdm[0].Init.PeriphInc           = DMA_PINC_DISABLE;
    hDmaDfsdm[0].Init.MemInc              = DMA_MINC_ENABLE;
    hDmaDfsdm[0].Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hDmaDfsdm[0].Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hDmaDfsdm[0].Init.Mode                = DMA_CIRCULAR;
    hDmaDfsdm[0].Init.Priority            = DMA_PRIORITY_HIGH;
    hDmaDfsdm[0].Instance                 = DMA1_Channel4;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hdfsdm_filter, hdmaReg, hDmaDfsdm[0]);

    /* Deinitialize the DMA channel for new transfer */
    if (HAL_DMA_DeInit(&hDmaDfsdm[0]) != HAL_OK)
    {
      /* Nothing to do */
    }

    /* Configure the DMA Channel */
    if (HAL_DMA_Init(&hDmaDfsdm[0]) != HAL_OK)
    {
      /* Nothing to do */
    }

    /* DMA IRQ Channel configuration */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, BSP_AUDIO_IN_IT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  }
  else /* DFSDM1_Filter1 */
  {
    if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)
    {
      /* Enable the DMA clock needed if only MIC2 is used */
      __HAL_RCC_DMA1_CLK_ENABLE();
    }

    /* Configure the hDmaDfsdm[1] handle parameters */
    hDmaDfsdm[1].Init.Request             = DMA_REQUEST_0;
    hDmaDfsdm[1].Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hDmaDfsdm[1].Init.PeriphInc           = DMA_PINC_DISABLE;
    hDmaDfsdm[1].Init.MemInc              = DMA_MINC_ENABLE;
    hDmaDfsdm[1].Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hDmaDfsdm[1].Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hDmaDfsdm[1].Init.Mode                = DMA_CIRCULAR;
    hDmaDfsdm[1].Init.Priority            = DMA_PRIORITY_HIGH;
    hDmaDfsdm[1].Instance                 = DMA1_Channel5;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hdfsdm_filter, hdmaReg, hDmaDfsdm[1]);

    /* Deinitialize the DMA channel for new transfer */
    if (HAL_DMA_DeInit(&hDmaDfsdm[1]) != HAL_OK)
    {
      /* Nothing to do */
    }

    /* Configure the DMA Channel */
    if (HAL_DMA_Init(&hDmaDfsdm[1]) != HAL_OK)
    {
      /* Nothing to do */
    }

    /* DMA IRQ Channel configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, BSP_AUDIO_IN_IT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  }
}

/**
  * @brief  DeInitialize DFSDM filter MSP.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
static void DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  if(hdfsdm_filter->Instance == DFSDM1_Filter0)
  {
    /* Disable DMA  Channel IRQ */
    HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);

    /* De-initialize the DMA Channel */
    if (HAL_DMA_DeInit(&hDmaDfsdm[0]) != HAL_OK)
    {
      /* Nothing to do */
    }
  }
  else /* DFSDM1_Filter1 */
  {
    /* Disable DMA  Channel IRQ */
    HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);

    /* De-initialize the DMA Channel */
    if (HAL_DMA_DeInit(&hDmaDfsdm[1]) != HAL_OK)
    {
      /* Nothing to do */
    }
  }
}

#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1)
/**
  * @brief  DFSDM filter regular conversion complete callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
static void DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t     index;
  uint32_t     recbufsize = (Audio_In_Ctx[0].Size / (2U * Audio_In_Ctx[0].ChannelsNbr));
  __IO int32_t tmp;

  if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)
  {
    for (index = (recbufsize / 2U); index < recbufsize; index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[4U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      tmp = Audio_DigMic2RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[(4U * index) + 2U] = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 3U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)
  {
    for (index = (recbufsize / 2U); index < recbufsize; index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else
  {
    if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)
    {
      for (index = (recbufsize / 2U); index < recbufsize; index++)
      {
        tmp = Audio_DigMic2RecBuff[index] / 256;
        tmp = SaturaLH(tmp, -32768, 32767);
        Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
        Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      }
    }
  }

  /* Invoke 'TransferCompete' callback function */
  if(hdfsdm_filter == &haudio_in_dfsdm_filter[0])
  {
    Audio_DmaDigMic1RecBuffCplt = 1;
  }
  else
  {
    Audio_DmaDigMic2RecBuffCplt = 1;
  }

  if (((Audio_DmaDigMic1RecBuffCplt != 0U) && (Audio_DmaDigMic2RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)) ||
      ((Audio_DmaDigMic1RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)) ||
      ((Audio_DmaDigMic2RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    BSP_AUDIO_IN_TransferComplete_CallBack(0);
    Audio_DmaDigMic1RecBuffCplt = 0;
    Audio_DmaDigMic2RecBuffCplt = 0;
  }
}

/**
  * @brief  DFSDM filter regular conversion half complete callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
static void DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t     index;
  uint32_t     recbufsize = (Audio_In_Ctx[0].Size / (2U * Audio_In_Ctx[0].ChannelsNbr));
  __IO int32_t tmp;

  if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)
  {
    for (index = 0; index < (recbufsize / 2U); index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[4U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      tmp = Audio_DigMic2RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[(4U * index) + 2U] = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 3U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)
  {
    for (index = 0; index < (recbufsize / 2U); index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else
  {
    if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)
    {
      for (index = 0; index < (recbufsize / 2U); index++)
      {
        tmp = Audio_DigMic2RecBuff[index] / 256;
        tmp = SaturaLH(tmp, -32768, 32767);
        Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
        Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      }
    }
  }

  /* Invoke the 'HalfTransfer' callback function */
  if(hdfsdm_filter == &haudio_in_dfsdm_filter[0])
  {
    Audio_DmaDigMic1RecHalfBuffCplt = 1;
  }
  else
  {
    Audio_DmaDigMic2RecHalfBuffCplt = 1;
  }

  if (((Audio_DmaDigMic1RecHalfBuffCplt != 0U) && (Audio_DmaDigMic2RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)) ||
      ((Audio_DmaDigMic1RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)) ||
      ((Audio_DmaDigMic2RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    BSP_AUDIO_IN_HalfTransfer_CallBack(0);
    Audio_DmaDigMic1RecHalfBuffCplt = 0;
    Audio_DmaDigMic2RecHalfBuffCplt = 0;
  }
}

/**
  * @brief  DFSDM filter error callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
static void DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  UNUSED(hdfsdm_filter);

  BSP_AUDIO_IN_Error_CallBack(0);
}
#else /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
/**
  * @brief  DFSDM filter regular conversion complete callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t     index;
  uint32_t     recbufsize = (Audio_In_Ctx[0].Size / (2U * Audio_In_Ctx[0].ChannelsNbr));
  __IO int32_t tmp;

  if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)
  {
    for (index = (recbufsize / 2U); index < recbufsize; index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[4U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      tmp = Audio_DigMic2RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[(4U * index) + 2U] = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 3U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)
  {
    for (index = (recbufsize / 2U); index < recbufsize; index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else
  {
    if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)
    {
      for (index = (recbufsize / 2U); index < recbufsize; index++)
      {
        tmp = Audio_DigMic2RecBuff[index] / 256;
        tmp = SaturaLH(tmp, -32768, 32767);
        Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
        Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      }
    }
  }

  /* Invoke 'TransferCompete' callback function */
  if(hdfsdm_filter == &haudio_in_dfsdm_filter[0])
  {
    Audio_DmaDigMic1RecBuffCplt = 1;
  }
  else
  {
    Audio_DmaDigMic2RecBuffCplt = 1;
  }

  if (((Audio_DmaDigMic1RecBuffCplt != 0U) && (Audio_DmaDigMic2RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)) ||
      ((Audio_DmaDigMic1RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)) ||
      ((Audio_DmaDigMic2RecBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    BSP_AUDIO_IN_TransferComplete_CallBack(0);
    Audio_DmaDigMic1RecBuffCplt = 0;
    Audio_DmaDigMic2RecBuffCplt = 0;
  }
}

/**
  * @brief  DFSDM filter regular conversion half complete callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t     index;
  uint32_t     recbufsize = (Audio_In_Ctx[0].Size / (2U * Audio_In_Ctx[0].ChannelsNbr));
  __IO int32_t tmp;

  if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)
  {
    for (index = 0; index < (recbufsize / 2U); index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[4U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      tmp = Audio_DigMic2RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[(4U * index) + 2U] = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(4U * index) + 3U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)
  {
    for (index = 0; index < (recbufsize / 2U); index++)
    {
      tmp = Audio_DigMic1RecBuff[index] / 256;
      tmp = SaturaLH(tmp, -32768, 32767);
      Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
      Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
    }
  }
  else
  {
    if (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)
    {
      for (index = 0; index < (recbufsize / 2U); index++)
      {
        tmp = Audio_DigMic2RecBuff[index] / 256;
        tmp = SaturaLH(tmp, -32768, 32767);
        Audio_In_Ctx[0].pBuff[2U * index]        = (uint8_t) tmp;
        Audio_In_Ctx[0].pBuff[(2U * index) + 1U] = (uint8_t) ((uint32_t) tmp >> 8);
      }
    }
  }

  /* Invoke the 'HalfTransfer' callback function */
  if(hdfsdm_filter == &haudio_in_dfsdm_filter[0])
  {
    Audio_DmaDigMic1RecHalfBuffCplt = 1;
  }
  else
  {
    Audio_DmaDigMic2RecHalfBuffCplt = 1;
  }

  if (((Audio_DmaDigMic1RecHalfBuffCplt != 0U) && (Audio_DmaDigMic2RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC)) ||
      ((Audio_DmaDigMic1RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC1)) ||
      ((Audio_DmaDigMic2RecHalfBuffCplt != 0U) && (Audio_In_Ctx[0].Device == AUDIO_IN_DIGITAL_MIC2)))
  {
    BSP_AUDIO_IN_HalfTransfer_CallBack(0);
    Audio_DmaDigMic1RecHalfBuffCplt = 0;
    Audio_DmaDigMic2RecHalfBuffCplt = 0;
  }
}

/**
  * @brief  DFSDM filter error callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None.
  */
void HAL_DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  UNUSED(hdfsdm_filter);

  BSP_AUDIO_IN_Error_CallBack(0);
}
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1) */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
