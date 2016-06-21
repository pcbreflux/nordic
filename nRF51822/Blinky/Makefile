TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  += -O0 -g
CFLAGS += -DBSP_DEFINES_ONLY
ASFLAGS += -g 
ASMFLAGS += -DBSP_DEFINES_ONLY
#LDLIBS += -lc -lgcc -lrdimon

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf51822.ld

# object files

OBJS =  system_nrf51.o main.o 
OBJS += nrf_delay.o

# include common make file

include $(TEMPLATEROOT)/Makefile.common
