target extended-remote localhost:3333
monitor reset halt
monitor reset init
monitor targets
set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4
load
b main
continue
