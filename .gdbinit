target remote | openocd -f board/stm32f4discovery.cfg -c "stm32f4x.cpu configure -rtos auto; gdb_port pipe"
set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4
mon reset halt
load
