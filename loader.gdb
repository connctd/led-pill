target extended-remote :3333

set print asm-demangle on

monitor reset init
monitor sleep 1000
monitor halt
monitor sleep 1000

# detect unhandled exceptions, hard faults and panics
break HardFault
break rust_begin_unwind

file target/thumbv7m-none-eabi/debug/pill-led

load

# *try* to stop at the user entry point (it might be gone due to inlining)
break main

monitor arm semihosting enable
monitor arm semihosting ioclient 2

# start the process but immediately halt the processor
stepi
