#include <HardwareSerial.h>

#define BAUD_RATE 1200
#define RX_BUFFER_SIZE 1024  // Adjust size as needed
#define START_UP_DELAY 300   // ms between CLK high and MSG start

HardwareSerial mySerial(1);  // UART1
uint8_t tx_bytes[] = { 0xA5, 0x0F, 0x06, 0x90, 0x1D, 0x00, 0x00, 0x00, 0xF9, 0x28 };
uint8_t rx_buffer[RX_BUFFER_SIZE];
int rx_len = 0;

void gemini_test_msg(void);

const int CLOCK_PIN = 12;   // Clock output to meter
const int DATA_PIN = 13;    // Data input from meter (open-drain)
const int BUTTON_PIN = 14;  // Button input (active LOW)
const int PROTOCOL_PIN = 25;

const unsigned long BIT_DELAY_US = 400;  // Approximate delay between clock edges
#define BIT_DURATION_US (104)

void setup() {
  Serial.begin(115200);

  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT_PULLUP);  // Meter releases to HIGH (open-drain)
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PROTOCOL_PIN, INPUT_PULLUP);

  digitalWrite(CLOCK_PIN, LOW);  // Idle state is HIGH
  Serial.println("Ready to read meter on button press...");
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);  // Simple debounce
    while (digitalRead(BUTTON_PIN) == LOW)
      ;  // Wait for release

    Serial.println("Reading meter...");
    //set clock pin high, wait for 500ms
    digitalWrite(CLOCK_PIN, HIGH);
    //SENSUS READ
    delay(START_UP_DELAY);


    if (digitalRead(PROTOCOL_PIN) == LOW) {
      // THIS IS SENSUS
      readMeterFrame();
    } else {
      // THIS IS GEMINI
      //uart_send_byte(0xA5);
    uint8_t msg[1000] = {0};
      msg[0]=0xA5;
      for(int z=1; z<1000; z++)
      {
        msg[z]=z;
      }
      uart_send_buffer(msg, 1000);
    }

    digitalWrite(CLOCK_PIN, HIGH);

    delay(100);
    //digitalWrite(CLOCK_PIN, LOW);

    // Delay between sensus and gemini
    //delay(1000);

    // Gemini Write / Read
    // digitalWrite(CLOCK_PIN, LOW);
    // delay(500);
    // //gemini_test_msg();
  }
  digitalWrite(CLOCK_PIN, LOW);
}

void readMeterFrame() {
  String output = "";
  const int num_chars = 500;  // Read up to 16 ASCII characters (incl. \r)

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
  while (readBit() != 0) {}

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

void gemini_test_msg(void) {
  // Step 1: Configure DATA_PIN as UART TX, send data
  Serial.println("Sending command...");
  mySerial.begin(BAUD_RATE, SERIAL_8N1, -1, DATA_PIN);  // -1 = no RX, TX only
  delay(10);                                            // small delay for UART startup

  for (size_t i = 0; i < sizeof(tx_bytes); i++) {
    mySerial.write(tx_bytes[i]);
  }
  mySerial.flush();  // wait until data is sent
  delay(10);         // optional delay depending on target device response time
  mySerial.end();

  // Step 2: Reconfigure DATA_PIN as UART RX, receive response
  Serial.println("Waiting for response...");
  mySerial.begin(BAUD_RATE, SERIAL_8N1, DATA_PIN, -1);  // RX only on DATA_PIN, no TX
  delay(100);                                           // wait for device to respond (adjust if needed)

  // Read available bytes
  unsigned long start_time = millis();  // Record the time when we start waiting

  while ((millis() - start_time < 3000) && rx_len < RX_BUFFER_SIZE) {
    if (mySerial.available()) {
      rx_buffer[rx_len++] = mySerial.read();
      start_time = millis();  // reset timer after receiving a byte
    }
  }

  // Print received data
  Serial.print("Received ");
  Serial.print(rx_len);
  Serial.println(" bytes:");
  for (int i = 0; i < rx_len; i++) {
    Serial.printf("0x%02X ", rx_buffer[i]);
  }
  Serial.println();
}



void uart_send_byte(uint8_t data) {
  // Start bit (LOW)
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(BIT_DURATION_US);

  // Send 8 data bits (LSB first)
  for (uint8_t i = 0; i < 8; i++) {
    bool bit = data & 0x01;
    digitalWrite(CLOCK_PIN, bit);
    delayMicroseconds(BIT_DURATION_US);
    data >>= 1;
  }

  // Stop bit (HIGH)
  digitalWrite(CLOCK_PIN, HIGH);
  delayMicroseconds(BIT_DURATION_US);
}

void uart_send_buffer(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    uint8_t byte = data[i];

    // Start bit (LOW)
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(BIT_DURATION_US);

    // Send 8 data bits (LSB first)
    for (uint8_t bit = 0; bit < 8; bit++) {
      digitalWrite(CLOCK_PIN, byte & 0x01);
      delayMicroseconds(BIT_DURATION_US);
      byte >>= 1;
    }

    // Stop bit (HIGH)
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(BIT_DURATION_US);
  }
}