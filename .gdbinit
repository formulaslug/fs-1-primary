target remote | openocd -f board/st_nucleo_f3.cfg -c "stm32f3x.cpu configure -rtos auto; gdb_port pipe; log_output openocd.log"
set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4
