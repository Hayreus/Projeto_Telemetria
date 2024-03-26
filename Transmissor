#include <Wire.h> // Biblioteca para comunicação I2C
#include <SPI.h>  // Biblioteca para comunicação SPI
#include <SD.h>   // Biblioteca para o cartão SD
#include <RF24.h> // Biblioteca para o módulo NRF24L01
#include <TinyGPS++.h> // Biblioteca para o módulo GPS
#include <MPU9250.h> // Biblioteca para o MPU9250
#include <EEPROM.h> // Biblioteca para a EEPROM

// Definições para os pinos do módulo NRF24
#define CE_PIN   9
#define CSN_PIN 10

// Definições para o módulo SD
#define SD_CS_PIN 4

// Definições para o módulo GPS
#define GPS_SERIAL Serial1 // Define a porta serial utilizada pelo módulo GPS

// Pino analógico para monitoramento do nível da bateria
#define BATTERY_PIN A0

RF24 radio(CE_PIN, CSN_PIN); // Cria um objeto para controle do NRF24
TinyGPSPlus gps; // Cria um objeto TinyGPS++ para processamento dos dados do GPS
MPU9250 IMU(Wire, 0x68); // Cria um objeto para o MPU9250

// Variáveis para armazenar valores de bias
float accelBias[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0};
float magBias[3] = {0, 0, 0};
float magScale[3] = {0, 0, 0};

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial com baudrate 9600
  GPS_SERIAL.begin(9600); // Inicializa a comunicação serial com baudrate 9600 para o módulo GPS
  delay(1000); // Aguarda 1 segundo

  // Inicialize os módulos
  Wire.begin(); // Inicializa a comunicação I2C
  SPI.begin(); // Inicializa a comunicação SPI
  SD.begin(SD_CS_PIN); // Inicializa o cartão SD

  // Inicialize o módulo NRF24
  radio.begin(); // Inicializa o NRF24
  radio.setPALevel(RF24_PA_LOW); // Configura a potência de transmissão do NRF24 como baixa
  radio.openWritingPipe(0xF0F0F0F0E1LL); // Configura o endereço de escrita do NRF24

  // Carrega os valores de bias da EEPROM
  loadCalibrationData();

  // Calibração do sensor MPU9250
  calibrateMPU9250();
}

void loop() {
  // Atualize o objeto TinyGPS++ com os dados do GPS
  while (GPS_SERIAL.available() > 0) { // Enquanto houver dados disponíveis na porta serial do GPS
    if (gps.encode(GPS_SERIAL.read())) { // Se os dados forem válidos e processados pelo TinyGPS++
      break; // Sai do loop
    }
  }

  // Leitura dos dados dos sensores
  float altitude = gps.altitude.meters(); // Lê a altitude em metros do objeto gps
  float latitude = gps.location.lat(); // Lê a latitude do objeto gps
  float longitude = gps.location.lng(); // Lê a longitude do objeto gps
  float velocity = gps.speed.kmph(); // Lê a velocidade em km/h do objeto gps

  // Leitura dos dados do giroscópio
  float gyroX = IMU.getGyroX_rads();
  float gyroY = IMU.getGyroY_rads();
  float gyroZ = IMU.getGyroZ_rads();

  // Leitura do nível da bateria
  float batteryLevel = readBatteryLevel();

  // Envio dos dados via NRF24
  sendDataViaNRF24(altitude, latitude, longitude, velocity, gyroX, gyroY, gyroZ, batteryLevel);

  delay(100); // Aguarde 100 milissegundos antes de ler os sensores novamente
}

// Função para calibrar o sensor MPU9250
void calibrateMPU9250() {
  Serial.println("Iniciando calibração do MPU9250. Por favor, mantenha-o imóvel...");
  
  // Realiza a calibração do MPU9250
  if (IMU.calibrate() == 0) {
    Serial.println("Calibração do MPU9250 completa.");

    // Armazena os valores de desvio na EEPROM
    saveCalibrationData();
  } else {
    Serial.println("Erro durante a calibração do MPU9250. Certifique-se de que o sensor está funcionando corretamente.");
  }
}

// Função para salvar os valores de bias na EEPROM
void saveCalibrationData() {
  EEPROM.put(0, accelBias);
  EEPROM.put(sizeof(accelBias), gyroBias);
  EEPROM.put(sizeof(accelBias) + sizeof(gyroBias), magBias);
  EEPROM.put(sizeof(accelBias) + sizeof(gyroBias) + sizeof(magBias), magScale);
}

// Função para carregar os valores de bias da EEPROM
void loadCalibrationData() {
  EEPROM.get(0, accelBias);
  EEPROM.get(sizeof(accelBias), gyroBias);
  EEPROM.get(sizeof(accelBias) + sizeof(gyroBias), magBias);
  EEPROM.get(sizeof(accelBias) + sizeof(gyroBias) + sizeof(magBias), magScale);
}

// Função para ler o nível da bateria
float readBatteryLevel() {
  // Lê a tensão da bateria
  int rawValue = analogRead(BATTERY_PIN);

  // Converte o valor para volts
  float voltage = rawValue * (5.0 / 1023.0);

  // Ajuste devido a características da bateria ou divisor de tensão
  // Substitua 5.0 e 10.0 pelos valores reais do divisor de tensão, se aplicável
  voltage = voltage * (10.0 / 5.0);

  return voltage;
}

// Função para enviar os dados via NRF24
void sendDataViaNRF24(float altitude, float latitude, float longitude, float velocity, float gyroX, float gyroY, float gyroZ, float batteryLevel) {
  // Construa a string de dados para enviar via NRF24
  String data = "Altitude: " + String(altitude) + "m, Latitude: " + String(latitude, 6) + ", Longitude: " + String(longitude, 6) + ", Velocidade: " + String(velocity) + "km/h, GyroX: " + String(gyro
