#include <Arduino.h>
#include <SdFat.h>

#include <rom.inc>

#define SD_CS_PIN PB0

SdFat SD;
File sk1ROM;

void set_addr(u16 addr)
{
  PORTF = addr & 255;
  PORTC = addr >> 8;
}

void set_data(byte data)
{
  PORTA = data;
}

void mem_ce(uint8_t state) 
{
  pinMode(PE7, OUTPUT);
  digitalWrite(PE7, state);
}

void mem_oe(uint8_t state) 
{
  pinMode(PE6, OUTPUT);
  digitalWrite(PE6, state);
}

void mem_we(uint8_t state) 
{
  pinMode(PE5, OUTPUT);
  digitalWrite(PE5, state);
}

void write_byte(u16 addr, byte data)
{
  set_addr(addr);
  set_data(data);
  delay(1);
  mem_we(LOW);  //latch addr
  delay(1);
  mem_we(HIGH); //latch data
  delay(1);
}

//AT28C256
void disable_sdp() {
  write_byte(0x5555, 0xAA);
  write_byte(0x2AAA, 0x55);
  write_byte(0x5555, 0x80);
  write_byte(0x5555, 0xAA);
  write_byte(0x2AAA, 0x55);
  write_byte(0x5555, 0x20);
}

//AT28C256
void erase_chip() {
  write_byte(0x5555, 0xAA);
  write_byte(0x2AAA, 0x55);
  write_byte(0x5555, 0x80);
  write_byte(0x5555, 0xAA);
  write_byte(0x2AAA, 0x55);
  write_byte(0x5555, 0x10);
  delay(20);
}

void enable_outputs()
{
  DDRA = 0b11111111; //DATA
  DDRC = 0b01111111; //ADDR_H
  DDRF = 0b11111111; //ADDR_L
}

void disable_outputs()
{
  DDRA = 0b00000000; //DATA
  DDRC = 0b00000000; //ADDR_H
  DDRF = 0b00000000; //ADDR_L
}

void init_rom() {
  //enable the chip
  mem_ce(LOW);
  mem_oe(HIGH);
  for (u16 addr = 0; addr < sk1_rom_bin_len; addr++)
  {
    write_byte(addr, sk1_rom_bin[addr]);
  }
  //lol as a full array it's too big for this avr to address
  write_byte(32767, 0x7c);

  //set CE/OE to tri-state and let the device take over
  //WE doesn't need to go tri-state
  pinMode(PE6, INPUT);
  pinMode(PE7, INPUT);
}

void setup()
{
  mem_we(HIGH);
  Serial.begin(115200);
  enable_outputs();
  delay(20);
  init_rom();
  disable_outputs();

  //turn these on for now
  mem_ce(LOW);
  mem_oe(LOW);
  randomSeed(analogRead(PE1)); //PE1 and PE2 should both be floating
}

void loop()
{
  //just read back one random address every second and see if we get data
  delay(1000);
  u16 rand_addr = random(32767);
  set_addr(rand_addr);
  Serial.print(rand_addr);
  Serial.print(" = ");
  Serial.println(PORTA);
}