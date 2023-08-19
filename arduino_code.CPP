//the program will use the led to indicate operation, read the code to understand how it works!

// Define the LED pin
const int led = 13;
char data;

void setup() {
  pinMode(led, OUTPUT);  // Set LED pin as an output
  Serial.begin(9600);    // Initialize serial communication at 9600 bps
}

void loop() {
  if (Serial.available() > 0) {  // Check if there's data available in the serial buffer
    data = Serial.read();  // Read the received data
    
    if (data == 'l') {
      digitalWrite(led, HIGH);  // Turn on the LED
      Serial.println("ligado!");  // Send feedback message to serial monitor
    } else if (data == 'd') {
      digitalWrite(led, LOW);  // Turn off the LED
      Serial.println("desligado");  // Send feedback message to serial monitor
    } else {
      // If an invalid command is received
      Serial.println("Invalid command. Use 'l' to turn on the LED and 'd' to turn it off.");
    }
  }
}
