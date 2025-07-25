const int CLOCK_PIN = 12;     // Clock output to meter
const int DATA_PIN = 13;      // Data input from meter (open-drain)
const int BUTTON_PIN = 14;    // Button input (active LOW)

const unsigned long BIT_DELAY_US = 400;  // Approximate delay between clock edges

void setup() {
  Serial.begin(115200);

  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT_PULLUP);  // Meter releases to HIGH (open-drain)
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(CLOCK_PIN, LOW); // Idle state is HIGH
  Serial.println("Ready to read meter on button press...");
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);  // Simple debounce
    while (digitalRead(BUTTON_PIN) == LOW);  // Wait for release

    Serial.println("Reading meter...");
    //set clock pin high, wait for 500ms
    digitalWrite(CLOCK_PIN, HIGH);
    delay(750);

    readMeterFrame();
    
  }
  digitalWrite(CLOCK_PIN, LOW);
}

void readMeterFrame() {
  String output = "";
  const int num_chars = 16;  // Read up to 16 ASCII characters (incl. \r)

  for (int c = 0; c < num_chars; ++c) {
    char ch = readSensusChar();
    output += ch;
    if (ch == '\r') break;
  }

  Serial.print("Meter Response: ");
  Serial.println(output);
}

char readSensusChar() {
  uint8_t byte = 0;

  // Wait for start bit (0)
  while (readBit() != 0) { }

  // Read 7 data bits (LSB first)
  for (int i = 0; i < 7; i++) {
    byte |= (readBit() << i);
  }

  // Parity bit (ignored)
  readBit();

  // Stop bit (should be 1)
  if (readBit() != 1) {
    Serial.println("Warning: Stop bit was not 1!");
  }

  return (char)byte;
}

uint8_t readBit() {
  // Drive clock LOW
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(BIT_DELAY_US);

  // Read data
  uint8_t bit = digitalRead(DATA_PIN);

  // Drive clock HIGH
  digitalWrite(CLOCK_PIN, HIGH);
  delayMicroseconds(BIT_DELAY_US);

  return bit;
}