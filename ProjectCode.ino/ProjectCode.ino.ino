// INITIALIZE DHT TEMPERATURE / HUMIDITY SENSOR
#include "DHT.h"
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// INITIALIZE LCD 
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 7, 6, 5, 4, 3);

// DIGITAL PORT B REGISTERS
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

// ANALOG
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADCH_DATA = (unsigned int*) 0x79;
volatile unsigned int* my_ADCL_DATA = (unsigned int*) 0x78;

// WATER LEVEL ANALOG PORT A0
unsigned char WATER_LEVEL_PORT = 0;

// THRESHOLDS
#define TEMPERATURE_THRESHOLD_F 80.0000
#define TEMPERATURE_THRESHOLD_C 26.6667
#define WATER_LEVEL_THRESHOLD 100

// Flags depicting what state we are in.
const enum state {
   off = 0,
   idle = 1,
   temp = 2,
   water = 3
};

// Begin in off state.
enum state stat = off;

void setup() {
  //Initialize analog ports
  adc_init();

  // Initialzie Serial Port
  Serial.begin(9600);

  // Initialize LCD
  lcd.begin(16, 2);

  // Initialize DHT
  dht.begin();

  // Set PB7 as input and PB6, PB5, PB4, PB3, and PB2 as output
  *ddr_b &= 0b01111111;
  *ddr_b |= 0b01111110;
  // PB7 will be the button
  // PB6 will be IDLE LED (Green)
  // PB5 will be Error (red)
  // PB4 will be temp (blue)
  // PB3 will be Disabled state (yellow)
  // PB2 will be fan
}

void loop() {
  delay(2000);
        
  unsigned int w = adc_read(WATER_LEVEL_PORT);
  float f = temperatureRead(true);
  float h = humidity();

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(f);
  Serial.print(F(" Water: "));
  Serial.print(w);
  Serial.print('\n');

  // Choose State Space
  switch(stat) {
    case off:
      Serial.println("Disabled State");
      disabled_state();
      break;
    case idle:
      Serial.println("Idle State");
      idle_state();
      break;
    case water:
      Serial.println("Error State");
      error_state();
      break;
    case temp:
      Serial.println("Running State");
      running_state();
      break;
    default:
      break;
  }
}

/*///////////////////
  MACHINE STATES
///////////////////*/
void disabled_state() { // Or off state
  lcd.clear();
  lcd.noDisplay();

  *port_b &= 0b10000001; // Turn off all LEDs
  *port_b |= 0b00001000; // Turn on Yellow LED
  
  // Listen to PB7 and await high signal
  while ( (*pin_b & (1 << 7)) == 0) { }

  // Start button pressed and initialize idle state.
  stat = idle;
  lcd.display();
}

void idle_state() {
  *port_b |= 0b01000000; // Turn on green LED
  *port_b &= 0b01000000; // Turn off other LEDs & fan
  
  // Get water level, temperature, and humidity
  unsigned int w = water_level();
  float t = temperatureRead(true);
  float h = humidity();

  // Display temperature and humidity to screen
  lcd_th(t, h);

  // Check water level.
  if (w < WATER_LEVEL_THRESHOLD) stat = water;
  
  // Check temperature.
  else if (t > TEMPERATURE_THRESHOLD_F) stat = temp;
}

void error_state() {
  *port_b |= 0b00100000; // Turn on red LED
  *port_b &= 0b00100000; // Turn off other LEDs
  
  lcd.clear();
  lcd.print("Low Water");

  unsigned int w = water_level();

  // Wait for water level to increase
  while (w < WATER_LEVEL_THRESHOLD) {
    delay(1000);
    w = water_level();
    lcd.setCursor(0, 1);
    lcd.print("Level:");
    lcd.setCursor(7, 1);
    lcd.print(w);
}
  
  // Water level is now okay
  stat = idle;
  lcd.clear();
}

void running_state()
{
  *port_b |= 0b00010000; // Enable fan and running LED
  *port_b &= 0b00010000; // Disable other LEDs
  float f = temperatureRead(true);
  float h = humidity();

  // Check water level and temperature
  if (water_level() < WATER_LEVEL_THRESHOLD) stat = water;
  else if ( f > TEMPERATURE_THRESHOLD_F ) {
    delay(1000);
    Serial.print("Temp: ");
    Serial.print(f);
    Serial.print('\n');
    lcd_th(f, h);
    running_state();
  }
  else {
    lcd.clear();
    stat = idle;
  }
}

/*///////////////////
 UTILITY FUNCTIONS
///////////////////*/

// GET WATER LEVEL FROM ANALOG PORT 0
unsigned int water_level() {
  return adc_read(WATER_LEVEL_PORT);
}

// GET TEMPERATURE FROM DHT11
float temperatureRead(bool F) {
  float t;
  if (F) t = dht.readTemperature(true); // Fahrenheit
  else t = dht.readTemperature();       // Celsius
  if (isnan(t)) Serial.println(F("Failed to read temperature from DHT sensor!"));
  return t;
}

// GET HUMIDITY FROM DHT11
float humidity() {
  float h = dht.readHumidity();
  if (isnan(h)) Serial.println(F("Failed to read humidity from DHT sensor!"));
  return h;
}

// OUTPUT TEMPERATURE AND HUMIDITY TO LCD
void lcd_th(float t, float h) {
  lcd.setCursor(0, 0);
  lcd.print("Temp:  Humidity:");
  lcd.setCursor(0, 1);
  lcd.print(t);
  lcd.setCursor(7, 1);
  lcd.print(h);
}

/*//////////////////////////////
    ANALOG/DIGITAL CONVERSION 
//////////////////////////////*/

// INITIALIZE THE ADC
void adc_init() {
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

unsigned int adc_read(unsigned char adc_channel_num)
{
  // Clear Analog channel selection bits
  *my_ADMUX &= 0b11100000;
  *my_ADMUX &= 0b11011111;
  if (adc_channel_num > 7) {
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000;
  }
  *my_ADMUX += adc_channel_num;
  *my_ADCSRA |= 0b01000000;
  while ((*my_ADCSRA & 0x40) != 0);
  return pow(2 * (*my_ADCH_DATA & (1 << 0)), 8) + pow(2 * (*my_ADCH_DATA & (1 << 1)), 9) + *my_ADCL_DATA; 
}

ISR(INT7_vect) {
  if (!stat) {
    Serial.print("Turning Off");
    disabled_state();
  }
}
