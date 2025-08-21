#include "microsecond_timer.h"

extern "C" void TIM5_IRQHandler(void) {
  HAL_TIM_IRQHandler(&moteus::MicrosecondTimer::handle_);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM5) {
    moteus::MicrosecondTimer::HandleOverflow();
  }
}
