#include <RF24.h>
#define bufferSize 32
const byte address[6] = {0x30, 0x30, 0x30, 0x30, 0x32}; // Indirizzo del pipe
char Received[bufferSize];

RF24 radio(PB0, PB1);                // Imposta CE e nCSN conformemente all'hardware

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address); // Imposta l'indirizzo del pipe 0 di ricezione
  radio.setAutoAck(false);           // Disattiva la richiesta di ACK
  radio.printPrettyDetails();
  radio.startListening();
  Serial.println("Pronto...");
}

void loop() {
  if (radio.available()) {
     radio.read(Received, bufferSize);
     Serial.println(Received);
  }
}
