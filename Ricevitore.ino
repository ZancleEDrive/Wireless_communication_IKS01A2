#include <RF24.h>
#define bufferSize 32
const byte address[6] = {0x30, 0x30, 0x30, 0x30, 0x32}; // Indirizzo del pipe
char Received[bufferSize];
char output[9];

RF24 radio(PB0, PB1);                // Imposta CE e nCSN conformemente all'hardware

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address); // Imposta l'indirizzo del pipe 0 di ricezione
  radio.setAutoAck(false);           // Disattiva la richiesta di ACK
  radio.printPrettyDetails();
  radio.startListening();
  Serial.println("Pronto...");
}

void extract_data(int start,int finish){
    for (int i=start;i<finish;i++){
          output[i-start]=Received[i];
    }
}
void loop() {
  if (radio.available()) {
     radio.read(Received, bufferSize);
     if(Received[0]=='0'){
         Serial.print("Yaw: ");
         extract_data(1,11);
         Serial.print(output);
         Serial.print("  Pitch: ");
         extract_data(11,21);
         Serial.print(output);
         Serial.print("  Roll: ");
         extract_data(21,31);
         Serial.println(output);
     }
     else if(Received[0]=='1'){
         Serial.print("Acc_x[mg]:");
         extract_data(1,11);
         Serial.print(output);
         Serial.print("      Acc_y[mg]:");
         extract_data(11,21);
         Serial.print(output);
         Serial.print("      Acc_z[mg]:");
         extract_data(21,31);
         Serial.println(output);
  
     }
  }
}
