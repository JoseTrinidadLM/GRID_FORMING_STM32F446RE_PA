break main
continue
break TIM2_IRQHandler
commands
    silent
    set $tim2_hits = $tim2_hits + 1
    continue
end
break Protocol_HeartBeat
continue