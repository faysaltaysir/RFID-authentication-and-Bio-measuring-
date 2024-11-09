#define BLYNK_PRINT Serial
#define SPI_SCK 18  // Serial Clock
#define SPI_MISO 19  // Master In Slave Out
#define SPI_MOSI 23  // Master Out Slave In
#define RST_PIN 4         // Configurable, depending on your wiring
#define SS_PIN 5         // Configurable, depending on your wiring
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
Servo myServo;
LiquidCrystal_I2C lcd(0x27, 20, 4);
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
char ssid[] = "palash";
char pass[] = "palash1234";

#include <SPI.h>
#include <MFRC522.h>


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

#define REPORTING_PERIOD_MS 100

#define REPORTING_PERIOD_MS2 1000
int input = 0;
// Initialize custom I2C bus for MLX90614 on SDA = 4, SCL = 5
TwoWire I2C_MLX90614 = TwoWire(1);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();  // No arguments in the constructor

// Initialize MAX30100 on default I2C bus (SDA = 21, SCL = 22)
PulseOximeter pox;

uint32_t tsLastReport = 0;

uint32_t tsLastReport2 = 0;
float temp = 0;
float spo2 = 0;
float objTemperature = 0;
String profileName = "";

int inputStatusB = 1;
int servoAngle = 0;
int rfidReadSection();
void onBeatDetected() {
  Serial.println("Beat detected!");
}
BLYNK_WRITE(V9) {  // Virtual Pin V1 for receiving data
  input = param.asInt();  // Read the integer value sent from the app
  Serial.print("Received value from Blynk: ");
  //  Serial.println(inputStatusB);
  Serial.println(input);
//  Blynk.virtualWrite(V7, "No Presence");
//  Blynk.virtualWrite(V8, "NULL");
}
BLYNK_WRITE(V0) {  // Virtual Pin V1 for receiving data
  servoAngle = param.asInt();  // Read the integer value sent from the app
  Serial.print("servoAngle Received value from Blynk : ");
  Serial.println(inputStatusB);
  Serial.println(servoAngle);
  myServo.write(servoAngle);
}
void myTimerEvent()
{
  Blynk.virtualWrite(V5, spo2);
  Blynk.virtualWrite(V6, heartRate);
}
void LCD_INIT() {
  Wire.begin(16, 17);
  lcd.init();                      // initialize the lcd
  lcd.init();
  lcd.backlight();
  //  lcd.setCursor(3, 0);
  //  lcd.print("Welcome You All");
  lcd.setCursor(0, 0);
  lcd.print("Swipe Your Card Here");
}
void MAX_INIT() {
  Wire.begin(16, 17);
  if (!pox.begin()) {
    Serial.println("Error initializing MAX30100. Check wiring.");
    while (1);
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
  pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
}
void MLX_INIT() {
  I2C_MLX90614.begin(21, 22);

  if (!mlx.begin(MLX90614_I2CADDR, &I2C_MLX90614)) {
    Serial.println("Error: Could not connect to MLX90614 sensor. Check wiring.");
    while (1);
  }
}
void RFID_INIT() {
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SS_PIN);             // Init SPI bus
  rfid.PCD_Init();       // Init MFRC522
  Serial.println("Scan a card to read data.");
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;  // Default key

  // Initialize the custom I2C bus for MLX90614 on pins SDA=4, SCL=5
}
void setup() {
  myServo.attach(15);
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  LCD_INIT();
  MAX_INIT();
  MLX_INIT();
  RFID_INIT();
  Blynk.virtualWrite(V7, "No Presence");
  Blynk.virtualWrite(V8, "NULL");
}
int count = 1;
void loop() {
  Blynk.run();
  int rfidStatus = rfidReadSection();
  Serial.print(rfidStatus);
  pox.update();

  if (rfidStatus != 100 && count == 1) {
//    Blynk.virtualWrite(V7, "No Presence");
    
    Blynk.virtualWrite(V8, "NULL");
    
    lcd.setCursor(0, 3);
    lcd.print("                   ");
    lcd.setCursor(0, 2);
    lcd.print("                   ");
    lcd.setCursor(0, 1);
    lcd.print("                   ");
    count = 0;
  }

  //  Serial.print("inputStatus1: ");
  //  Serial.println(input);
  //  timer.setInterval(1000L, myTimerEvent);
  while (input == 1 && rfidStatus == 100) {
    Blynk.run();
    Wire.begin(16, 17);
    //    Serial.println("inputStatus2: " + input);
    //    Serial.println(input);
    //    if (input != 1) {
    //      break;  // Exit the loop if button value changes
    //    }
    pox.update();

    // Display heart rate and SpO2 every second
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
      pox.update();
      Serial.print("Heart rate: ");
      heartRate = pox.getHeartRate();
      Serial.print(heartRate);
      Serial.print(" bpm / SpO2: ");
      spo2 = pox.getSpO2();
      Serial.print(spo2);
      Serial.println(" % ");
      Blynk.virtualWrite(V6, heartRate);
      Blynk.virtualWrite(V5, spo2);
      tsLastReport = millis();
    }
    if (millis() - tsLastReport2 > REPORTING_PERIOD_MS2) {


      objTemperature = mlx.readObjectTempF();
      Serial.print(objTemperature);
      Serial.println(" °F");
      //      lcd.init();
      // Print a message to the LCD.
      //      lcd.backlight();
      lcd.setCursor(0, 3);
      lcd.print(objTemperature);
      lcd.print(" F");
      tsLastReport2 = millis();
    }

    //        timer.setInterval(1000L, myTimerEvent);


    Blynk.run();
    count = 1;
    //      timer.run(); // Initiates BlynkTimer
    // Small delay to manage I2C timing

    // Temperature readings from MLX90614
    //      Serial.print("Ambient Temp: ");
    //      Serial.print(mlx.readAmbientTempF());
    //      Serial.print(" °F, Object Temp: ");
    //      objTemperature = mlx.readObjectTempF();
    //      Serial.print(objTemperature);
    //      Serial.println(" °F");
  }
}

void READ_RFID(int block) {
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
    if (block == 2) {
      message.reserve(16);
      message = "";
    }
    if (block == 1) {
      profileName.reserve(16);
      profileName = "";
    }

    if (block == 2) {
      for (byte i = 0; i < 16; i++) {
        Serial.write(buffer[i]); // Write each byte to Serial
        message += (char)buffer[i];
      }
    }
    if (block == 1) {
      for (byte i = 0; i < 16; i++) {
        Serial.write(buffer[i]); // Write each byte to Serial
        profileName += (char)buffer[i];
      }
    }
  }
}
int rfidReadSection() {
  //  message = "No Presence";

  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return 0;

  Serial.println("Card detected!");

  // Authenticate to the block

  READ_RFID(1);
  READ_RFID(2);

  Serial.println();

  Serial.println("message=>" + message);
  Serial.println(profileName);


  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  if (message == "REGISTERED      ") {
    lcd.setCursor(4, 1);
    lcd.print(message);
    lcd.setCursor(6, 2);

    Serial.println(message);
    Serial.println(message);
    Serial.println(profileName);
    Serial.println(profileName);
    lcd.print(profileName);
    Blynk.virtualWrite(V7, message);
    Blynk.virtualWrite(V8, profileName);
    return 100;
  }
  else  {
    lcd.setCursor(4, 1);
    lcd.print("NOT REGISTERED");
    Blynk.virtualWrite(V7, "NOT REGISTERED");
    Blynk.virtualWrite(V8, "NULL");
    return 200;
  }
}
