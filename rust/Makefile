.EXPORT_ALL_VARIABLES:
AVR_CPU_FREQUENCY_HZ=20_000_000

all:
	cargo build --target avr-atmega64a.json -Z build-std=core --release

check:
	cargo check --target avr-atmega64a.json -Z build-std=core 

fuses:
	sudo avrdude -c atmelice_isp -p m64 -U hfuse:w:0xD9:m
	sudo avrdude -c atmelice_isp -p m64 -U lfuse:w:0xFF:m
	sudo avrdude -c atmelice_isp -p m64 -U efuse:w:0xFF:m

upload: all
	sudo avrdude -c atmelice_isp -p m64 -U flash:w:target/avr-atmega64a/release/rombender.elf

clean:
	cargo clean
