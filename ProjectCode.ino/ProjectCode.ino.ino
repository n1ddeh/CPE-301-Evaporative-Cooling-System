// INITIALIZE DHT TEMPERATURE / HUMIDITY SENSOR
#include "DHT.h"
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ANALOG
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADCH_DATA = (unsigned int*) 0x79;
volatile unsigned int* my_ADCL_DATA = (unsigned int*) 0x78;

unsigned char WATER_LEVEL_PORT = 0;

void setup() {
  adc_init();
  Serial.begin(9600);

  dht.begin();
}

void loop() {
  delay(2000);
  unsigned int adc_reading = adc_read(WATER_LEVEL_PORT);
  Serial.println(adc_reading);

  float f = temperatureRead(true);
  float h = humidity();

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(f);
}

void adc_init()
{
  // Register A
  *my_ADCSRA |= 0x80; // Set Bit 7 to 1
  *my_ADCSRA &= 0b11011111; // Clear Bit 5
  *my_ADCSRA &= 0b11110111; // Clear Bit 3
  *my_ADCSRA &= 0b11111000; // Clear bits 2-0

  // Register B
  *my_ADCSRB &= 0b11110111; // Clear bit 3
  *my_ADCSRB &= 0b11111000; // Clear bit 2-0

  // MUX
  *my_ADMUX &= 0b01111111; // Clear bit 7
  *my_ADMUX |= 0b01000000; // Set bit 6
  *my_ADMUX &= 0b11011111; // Clear bit 5. Right adjusted result.
  *my_ADMUX &= 0b11100000; // Clear Bits 4-0
}

// Water Level
int read_water_level()
{
  adc_read(WATER_LEVEL_PORT);
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // Clear Analog channel selection bits
  *my_ADMUX &= 0b11100000;
  *my_ADMUX &= 0b11011111;

  if (adc_channel_num > 7)
  {
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000;
  }
  *my_ADMUX += adc_channel_num;

  *my_ADCSRA |= 0b01000000;

  while ((*my_ADCSRA & 0x40) != 0);

  return pow(2 * (*my_ADCH_DATA & (1 << 0)), 8) + pow(2 * (*my_ADCH_DATA & (1 << 1)), 9) + *my_ADCL_DATA; 
}

float temperatureRead(bool F) {
  float t;
  if (F) {
    t = dht.readTemperature(true);
  }
  else {
    t = dht.readTemperature();
  }
  if (isnan(t)) {
    Serial.println(F("Failed to read temperature from DHT sensor!"));
  }
  return t;
}

float humidity() {
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println(F("Failed to read humidity from DHT sensor!"));
  }
  return h;
}
