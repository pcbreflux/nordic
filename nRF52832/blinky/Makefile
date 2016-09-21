TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  += -O0 -g
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DNRF52832
ASFLAGS += -g 
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DNRF52832

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf52832_QFAA.ld

# object files

OBJS =  system_nrf52.o main.o 

# include common make file

include $(TEMPLATEROOT)/Makefile.common
