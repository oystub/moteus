#pragma once

#include "mbed.h"

namespace moteus {

class MicrosecondTimer {
 public:
  MicrosecondTimer() {
    __HAL_RCC_TIM5_CLK_ENABLE();

    handle_.Instance = TIM5;
    handle_.Init.Period = 0xFFFFFFFF;  // TIM5 is 32-bit
    handle_.Init.Prescaler = (SystemCoreClock / 1000000U) - 1; // 1 µs tick
    handle_.Init.ClockDivision = 0;
    handle_.Init.CounterMode = TIM_COUNTERMODE_UP;
    handle_.Init.RepetitionCounter = 0;

    HAL_TIM_Base_Init(&handle_);

    __HAL_TIM_CLEAR_FLAG(&handle_, TIM_FLAG_UPDATE);

    HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);

    HAL_TIM_Base_Start_IT(&handle_);
  }

  static uint64_t read_us64() {
    uint32_t hi1 = high_word_;
    uint32_t lo  = TIM5->CNT;
    uint32_t hi2 = high_word_;
    if (hi1 != hi2) { lo = TIM5->CNT; hi1 = hi2; }
    return (static_cast<uint64_t>(hi1) << 32) | lo;
  }

  static uint64_t read_ms64() {
    return read_us64() / 1000ULL;
  }

  static void HandleOverflow() {
    high_word_++;
  }

  static inline TIM_HandleTypeDef handle_ = {};

 private:
  static inline volatile uint32_t high_word_ = 0;
};

}  // namespace moteus