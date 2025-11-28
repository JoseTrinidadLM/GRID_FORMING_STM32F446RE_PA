break Protocol_HeartBeat
commands
    silent
    echo "Heartbeat detected, setting breakpoint in executeCommand()\n"
    break executeCommand
    continue
end

break executeCommand
commands
    silent
    echo "Now in executeCommand()\n"
    print /d command
    finish
end