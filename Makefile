F_CPU = 9600000
SRC = picoUART/
SOURCES := $(wildcard $(SRC)/*.c $(SRC)/*.cpp)


help:
	@echo "Usage:"
	@echo "build: compile hex file"
	@echo "flash: flash hex to MCU (cfg. for ATTiny13, specify port with 'port=/dev/xxxXXX')"

rebuild: clean build

build: link
	@echo "Generating hex..."
	@avr-objcopy -R .eeprom -O ihex output.out output.hex
	@avr-size -C --mcu=attiny13 output.out

link: main 
	@echo "Linking..."
	@avr-gcc -mmcu=attiny13 -std=c++11 -Os -Wall -mcall-prologues -o output.out *.o

main:
	@echo "Compiling main.c..."
	@avr-gcc -mmcu=attiny13 -DF_CPU=$(F_CPU) -std=c++11 -Os -Wall -mcall-prologues -I$(SRC) -c main.cpp $(SOURCES)

flash:
ifdef port
	@avrdude -c stk500v1 -P $(port) -b 19200 -p t13 -U flash:w:output.hex
else
	@echo "Please specify port with 'port=/dev/xxxXXX'"
endif

clean:
	@rm -f *.o
	@rm -f *.hex
	@rm -f *.out