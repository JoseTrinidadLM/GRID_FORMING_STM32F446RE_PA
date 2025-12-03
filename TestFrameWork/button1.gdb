set LOOP_SEL.pGPIOx.PUPDR = 0x40000000
break EXTI15_10_IRQHandler
finish
set LOOP_SEL.pGPIOx.PUPDR = 0x00000000
continue
break Protocol_HeartBeat
continue