TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  += -O0 -g
ASFLAGS += -g 

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf51822_QFAA.ld

# object files

OBJS =  system_nrf51.o blinky.o 
OBJS += nrf_delay.o

# include common make file

include $(TEMPLATEROOT)/Makefile.common
