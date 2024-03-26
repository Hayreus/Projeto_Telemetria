#include <SPI.h>   // Biblioteca para comunicação SPI
#include <RF24.h>  // Biblioteca para o módulo NRF24L01

// Definições para os pinos do módulo NRF24
#define CE_PIN   9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN); // Cria um objeto para controle do NRF24

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial com baudrate 9600

  // Inicialize o módulo NRF24
  radio.begin(); // Inicializa o NRF24
  radio.setPALevel(RF24_PA_LOW); // Configura a potência de recepção do NRF24 como baixa
  radio.openReadingPipe(1, 0xF0F0F0F0E1LL); // Abre o canal de leitura do NRF24
  radio.startListening(); // Inicia o modo de escuta do NRF24
}

void loop() {
  if (radio.available()) { // Verifica se há dados disponíveis para leitura
    char receivedData[128]; // Cria um array para armazenar os dados recebidos
    radio.read(&receivedData, sizeof(receivedData)); // Lê os dados recebidos do NRF24

    // Exibe os dados recebidos na porta serial
    Serial.print("Dados recebidos: ");
    Serial.println(receivedData);
  }
}
