#include <PN5180.h>
#include <PN5180ISO15693.h>

#define PN5180_NSS  5
#define PN5180_BUSY 16
#define PN5180_RST  17

PN5180ISO15693 nfc(PN5180_NSS, PN5180_BUSY, PN5180_RST);

void setup() {
  Serial.begin(115200);
  Serial.println(F("ESP32 FreeStyle Libre Reader"));

  Serial.println("Inicializando o leitor...");
  nfc.begin();
  Serial.println("Redefinindo o leitor..");
  nfc.reset();
  Serial.println("Habilitando o campo NFC...");
  nfc.setupRF();

  Serial.println("Pronto para utilização...");

  uint8_t firmwareVersion[2];
  nfc.readEEprom(FIRMWARE_VERSION, firmwareVersion, sizeof(firmwareVersion));
  Serial.print(F("Firmware version: "));
  Serial.print(firmwareVersion[1]);
  Serial.print(".");
  Serial.println(firmwareVersion[0]);

  if (firmwareVersion[0] == 0) {
    Serial.println(F("Falha na inicialização!"));
    while (1) {};
  }
}

void loop() {
  Serial.println(F("Aproxime o sensor FreeStyle Libre..."));
  
  uint8_t uid[8];
  ISO15693ErrorCode rc = nfc.getInventory(uid);
  if (rc == ISO15693_EC_OK) {
    Serial.println(F("Sensor não encontrado!"));
    
    uint8_t blockBuffer[8];
    rc = nfc.readSingleBlock(uid, 0x00, blockBuffer, sizeof(blockBuffer));
    
    if (rc == ISO15693_EC_OK) {
      uint8_t sensorStatus = blockBuffer[4];
      
      if (sensorStatus == 0x03) {  // Sensor active and working
        Serial.println(F("Sensor is active"));
        
        // Read glucose data
        rc = nfc.readSingleBlock(uid, 0x03, blockBuffer, sizeof(blockBuffer));
        if (rc == ISO15693_EC_OK) {
          uint16_t currentGlucose = (blockBuffer[0] & 0x1F) << 8 | blockBuffer[1];
          uint8_t trend = blockBuffer[2];
          
          Serial.print(F("Glico se atual: "));
          Serial.print(currentGlucose);
          Serial.println(F(" mg/dL"));
          
          Serial.print(F("Tendência: "));
          switch(trend) {
            case 0x00: Serial.println(F("Estável")); break;
            case 0x01: Serial.println(F("Subindo lentamente")); break;
            case 0x02: Serial.println(F("Subindo")); break;
            case 0x03: Serial.println(F("Subindo rapidamente")); break;
            case 0x04: Serial.println(F("Caindo lentamente")); break;
            case 0x05: Serial.println(F("Caindo")); break;
            case 0x06: Serial.println(F("Caindo rapidamente")); break;
            default: Serial.println(F("Desconhecido")); break;
          }
        }
      } else {
        Serial.println(F("Sensor não está ativo!"));
      }
    }
  } else {
    Serial.println(F("Sensor não encontrado!"));
  }

  delay(1500);
}
