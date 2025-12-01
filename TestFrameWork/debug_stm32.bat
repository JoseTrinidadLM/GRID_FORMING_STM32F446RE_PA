
@echo off
wt ^
    new-tab --title "ST-LINK GDB Server" cmd /k ^
        C:\ST\STM32CubeCLT_1.20.0\STLink-gdb-server\bin\ST-LINK_gdbserver.exe ^
        -cp C:\ST\STM32CubeCLT_1.20.0\STM32CubeProgrammer\bin ^
        -i 0669FF545383564867214231 ^
        --halt --frequency 4000 ^
        -d ^
    ; new-tab --title "GDB Debug" cmd /k ^
        arm-none-eabi-gdb ^
        %USERPROFILE%\Documents\InternProject\STM32F446RE\GRID_FORMING\build\GRID_FORMING.elf ^
        -ex "target remote localhost:61234" ^
        -ex "monitor halt" ^
        -ex "load" ^
        -ex "break main" ^
        -ex "continue"
