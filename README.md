# Wireless_communication
Comunicazione wireless tra due nrf24l01 connessi a due Nucleo F401-RE e giroscopio-accelerometro IKS01A2.
La Nucleo con il sensore IMU funge da trasmettitore.
L'IMU utilizza la libreria Motion_FX grazie alla quale effettua il sensor fusion di accelerometro e magnetometro 9 assi per ricavare il valore del giroscopio.
Invece di prendere in considerazione il valore in uscita dal sensore, ad ogni iterazione viene calcolata la differenza tra il nuovo valore e il valore campionato al primo istante, in modo tale che ad ogni reset si azzeri il sistema di riferimento.
Libreria: https://www.st.com/resource/en/user_manual/um2220-getting-started-with-motionfx-sensor-fusion-library-in-xcubemems1-expansion-for-stm32cube-stmicroelectronics.pdf


Ogni valore del sensore è codificato con 10 byte, in quanto ogni cifra o carattere corrisponde ad un char (1 char= 1 byte). In particolare ogni valore necessita di un carattere per il segno meno, uno per la virgola, tre per i valori della parte decimale, e cinque per la parte intera. 
I transceiver permettono di inviare pacchetti da masssimo 32byte, perciò i dati del sensore sono divisi in due pacchetti, identificati da un id.
In particolare, il pacchetto con id pari a 0 identifica i dati del giroscopio, il pacchetto con id pari a 1 quelli dell'accelerometro.
In ricezione viene letto il primo carattere del messaggio per identificare il pacchetto ricevuto. 
Conoscendo esattamente la dimensione dei singolo dato nel messaggio inviato (10byte) è facile estrarre il dato usando la funzione extract_data che prende l'indice iniziale del dato nel messaggio e quello finale +1. In trasmissione la funzione dtostrf permettere di convertitore i dati ricavati dal sensor fusion in array di char, e poterli concatenare grazie al codifica fissa di 10 byte.

 * VCC  3.3V
 * MISO PA6
 * MOSI PA7
 * SCK  PA5
 * CE   PB0
 * CSN  PB1

![image](https://user-images.githubusercontent.com/85572398/205714133-96ed2589-9a6d-48c4-b708-07be834c80ef.png)
