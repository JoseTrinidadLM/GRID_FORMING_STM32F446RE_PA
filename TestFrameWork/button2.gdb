set LOOP_SEL.pGPIOx.PUPDR = 0x10000000
break EXTI15_10_IRQHandler
finish
set LOOP_SEL.pGPIOx.PUPDR = 0x00000000
continue
break Protocol_HeartBeat
continue