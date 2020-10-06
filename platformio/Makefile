all:
	pio run

check:
	pio check

fuses:
	sudo avrdude -c atmelice_isp -p m64 -U hfuse:w:0xD8:m
	sudo avrdude -c atmelice_isp -p m64 -U lfuse:w:0x7F:m
	sudo avrdude -c atmelice_isp -p m64 -U efuse:w:0xFF:m

upload: all
	sudo avrdude -c atmelice_isp -p m64 -U flash:w:.pio/build/ATmega64/firmware.hex
