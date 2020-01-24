#include <stdio.h>
#include "stm32l475e_iot01_audio.h"

/**
  * @brief This function handles DFSDM Left DMA interrupt request.
  * @param None
  * @retval None
  */
void AUDIO_DFSDM_DMAx_MIC1_IRQHandler(void) {
    HAL_DMA_IRQHandler(haudio_in_dfsdm_filter[0].hdmaReg);
}
