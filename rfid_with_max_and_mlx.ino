#define BLYNK_PRINT Serial
#define SPI_SCK 18  // Serial Clock
#define SPI_MISO 19  // Master In Slave Out
#define SPI_MOSI 23  // Master Out Slave In

/* Fill in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL6NWN_4zSJ"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "uwd8260-miG9_J2J66d2MvST5OiUeo9m"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

BlynkTimer timer;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Bhoot";
char pass[] = "anika002";


#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN 4         // Configurable, depending on your wiring
#define SS_PIN 5         // Configurable, depending on your wiring
float heartRate = 0;
String message = "No Presence";
byte block = 2; // Example block number (can be 0 to 3 for sector 0, 4 to 7 for sector 1, etc.)
byte buffer[18]; // Buffer to hold the read data
byte size = sizeof(buffer); // Size of the buffer
MFRC522 rfid(SS_PIN, RST_PIN);  // Create MFRC522 instance
MFRC522::MIFARE_Key key;


#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS 1000

// Initialize custom I2C bus for MLX90614 on SDA = 4, SCL = 5
TwoWire I2C_MLX90614 = TwoWire(1);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();  // No arguments in the constructor

// Initialize MAX30100 on default I2C bus (SDA = 21, SCL = 22)
PulseOximeter pox;

uint32_t tsLastReport = 0;
float temp = 0;
float spo2 = 0;
float objTemperature = 0;
void onBeatDetected() {
  Serial.println("Beat detected!");

  //  if(heartRate >= temp){
  //    temp = heartRate;
  //  }
  //  if(sp02 >= temp){
  //    temp = heartRate;
  //  }
  //  Serial.println(temp);
  //  if(heartRate == 0){
  //    temp = 0;
  //  }
}


// This function sends Arduino's up time every second to Virtual Pin (5).
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
//  Blynk.virtualWrite(V4, objTemperature);
  Blynk.virtualWrite(V5, spo2);
  Blynk.virtualWrite(V6, heartRate);
  
//    Serial.println(message);
}

void setup() {
  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  I2C_MLX90614.begin(21, 22);  // Custom I2C bus for MLX90614

  // Initialize MLX90614 with the custom I2C bus
  if (!mlx.begin(MLX90614_I2CADDR, &I2C_MLX90614)) {  // Pass the address and custom I2C bus object
    Serial.println("Error: Could not connect to MLX90614 sensor. Check wiring.");
    while (1);  // Stop execution if MLX90614 initialization fails
  }

  // Initialize MAX30100 on the default I2C bus
  Wire.begin(16, 17);  // Default I2C bus (you may adjust the pins as needed)
  if (!pox.begin()) {
    Serial.println("Error initializing MAX30100. Check wiring.");
    while (1);  // Stop execution if MAX30100 initialization fails
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
  pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);  // Set IR LED current (same current options as the red LED)
  pox.setOnBeatDetectedCallback(onBeatDetected);


  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SS_PIN);             // Init SPI bus
  rfid.PCD_Init();       // Init MFRC522
  Serial.println("Scan a card to read data.");
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;  // Default key

  // Initialize the custom I2C bus for MLX90614 on pins SDA=4, SCL=5

}

int rfidReadSection() {

  message = "No Presence";
  
    Blynk.virtualWrite(V7, message);
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return 0;

  Serial.println("Card detected!");

  // Authenticate to the block
  MFRC522::StatusCode status;
  status = rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfid.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Authentication failed: ");
    Serial.println(rfid.GetStatusCodeName(status));
    //    return;
  }

  // Read data from the block
  status = rfid.MIFARE_Read(block, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Read failed: ");
    Serial.println(rfid.GetStatusCodeName(status));
  } else {
    Serial.print("Data in block ");
    Serial.print(block);
    Serial.print(": ");

    // Print data as characters
    message.reserve(16);
    message = "";
    for (byte i = 0; i < 16; i++) {
      Serial.write(buffer[i]); // Write each byte to Serial
      message += (char)buffer[i];
    }
    Serial.println();

    Serial.println(message);
    Serial.println(message);
    Serial.println(message);
    Serial.println(message);
    Blynk.virtualWrite(V7, message);
  }
  return 100;
}


void loop() {
  // Specify the block to read
    int rfidStatus = rfidReadSection();
    Serial.println(rfidStatus);


  pox.update();
  // Update MAX30100 data
  // Look for new cards

  //  timer.setInterval(1000L, myTimerEvent);



  while (rfidStatus == 100) {

  pox.update();

    // Display heart rate and SpO2 every second
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
      // Temperature readings from MLX90614
      Serial.print("Ambient Temp: ");
      Serial.print(mlx.readAmbientTempF());
      Serial.print(" °F, Object Temp: ");
      objTemperature = mlx.readObjectTempF();
      Serial.print(objTemperature);
      Serial.println(" °F");

      Serial.print("Heart rate: ");
      heartRate = pox.getHeartRate();
      Serial.print(heartRate);

      Serial.print(" bpm / SpO2: ");
      spo2 = pox.getSpO2();
      Serial.print(spo2);
      Serial.println(" % ");

      Blynk.virtualWrite(V6, heartRate);
      Blynk.virtualWrite(V5, spo2);
//      Blynk.virtualWrite(V4, objTemperature);
      tsLastReport = millis();
      
    }
    
//    timer.setInterval(1000L, myTimerEvent);
//    Blynk.virtualWrite(V6, heartRate);
//    Blynk.virtualWrite(V5, spo2);
//    Blynk.virtualWrite(V4, objTemperature);
    //    }

    //  rfid.PICC_HaltA();
    //  rfid.PCD_StopCrypto1();

    // Setup a function to be called every second
    
    Blynk.run();
//      timer.run(); // Initiates BlynkTimer
    // Small delay to manage I2C timing
  }
}
