display /d systemState
print /d systemState

display /d (LED.pGPIOx.ODR >> 5) & 0b1
print /d (LED.pGPIOx.ODR >> 5) & 0b1

display /d ElapsedTime
print /d ElapsedTime

display /d telemetry_status
print /d telemetry_status

display /d heartbeat[0]
print /d heartbeat[0]

display /d TIM_4.pTIMx.CCER & 0b1
print /d TIM_4.pTIMx.PSC  & 0b1
display /d (TIM_4.pTIMx.CCER >> 4) & 0b1
print /d (TIM_4.pTIMx.PSC >> 4) & 0b1

display /d TIM_2.pTIMx.PSC
print /d TIM_2.pTIMx.PSC
display /d TIM_2.pTIMx.ARR
print /d TIM_2.pTIMx.ARR

display /d TIM_4.pTIMx.PSC
print /d TIM_4.pTIMx.PSC
display /d TIM_4.pTIMx.ARR
print /d TIM_4.pTIMx.ARR

info display