break main
continue
break TIM3_IRQHandler
commands
    silent
    echo "TIM3 interrupt occurred (heartbeat tick)\n"
    continue
end
break Protocol_HeartBeat
continue