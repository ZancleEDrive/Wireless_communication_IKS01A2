#include <RF24.h>
#define bufferSize 32
const byte address[6] = {0x30, 0x30, 0x30, 0x30, 0x32}; // Indirizzo del pipe
int counter=0;
char Hello[bufferSize];

RF24 radio(PB0, PB1); // Imposta CE e nCSN conformemente all'hardware
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setAutoAck(false); // Disattiva la richiesta di ACK
  radio.openWritingPipe(address);  // Imposta l'indirizzo del pipe di trasmissione
  radio.stopListening();
  radio.printPrettyDetails();
}

void loop() {
  sprintf(Hello, "Ciao Zed"); // Personalizzare il testo trasmesso
  radio.write(Hello, bufferSize);
  delay(1000);
}
