# Project name
PROJECT = roboxylohero

# Toolchain
CC = avr-gcc
OBJCOPY = avr-objcopy

# Processor type
MCU = atmega328p

# Compiler flags
CFLAGS = -mmcu=$(MCU) -DF_CPU=16000000UL -O2 -Wall

# Linker flags
LDFLAGS = -mmcu=$(MCU)

# Source files
SRC = src/main.c \
	  src/robot.c \
	  src/screen.c \
      src/include/HX8357.c \
      src/include/I2C_BNO055.c \
      src/include/I2C.c \
      src/include/LCD_GFX.c \
      src/include/ServoControl.c \
      src/include/TimerHelper.c \
      src/include/uart.c \
# Object files
OBJ = $(SRC:.c=.o)

# Include directories
INCLUDES = -Isrc/include \
		   -I. \
           -I/usr/lib/avr/include/util \

# Build rules
all: $(PROJECT).hex

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(PROJECT).elf: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $@

$(PROJECT).hex: $(PROJECT).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

clean:
	rm -f $(OBJ) $(PROJECT).elf $(PROJECT).hex

.PHONY: all clean
