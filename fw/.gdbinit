target extended-remote /dev/ttyACM0
monitor swdp_scan
attach 1
set mem inaccessible-by-default off
set print asm-demangle on
load
