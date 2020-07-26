//
// Simple example to verify that Platform IO is correctly configured and 
// able to compile code for the Zumo 32U4.
//
// Will blink the red, yellow, and green LEDs in sequence. Writes 
// "Hello, world" on the serial console at startup, and writes the value 
// of loop_counter to the LCD and serial console when loop() is called.
//
#include <Arduino.h>
#include <Zumo32U4.h>

namespace {
  // Delay this many milliseconds each loop()
  const uint32_t DELAY_MS = 100;

  // Count times through loop() (wrap-around is intended)
  uint8_t loop_counter = 0;

  // Robot's LCD display
  Zumo32U4LCD lcd = Zumo32U4LCD{};
} 

// setup() is called *once* at Zumo power-on 
void setup() {
  // Initialize the LCD display. Most libraries have some type of 
  // setup or initialization to put hardware into correct state.
  lcd.init();

  // Set speed of the serial console interface. Your IDE's console
  // monitor needs to match this value otherwise you'll see garbage.
  Serial.begin(115200);
  
  // and hello to you too
  Serial.println("Hello, world");
}

// Repeatedly called as long as robot is powered-on
void loop() {
  uint8_t idx = loop_counter % 3;
  loop_counter++;

  ledRed(idx == 0);
  ledGreen(idx == 1);
  ledYellow(idx == 2);

  Serial.println(loop_counter);

  lcd.clear();
  lcd.print(loop_counter);

  delay(DELAY_MS);
}
