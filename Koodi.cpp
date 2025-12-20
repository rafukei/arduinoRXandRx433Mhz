// Moottoriohjaimen pinnit
const int ENA = 5;
const int IN1 = 4;
const int IN2 = 3;
const int ENB = 6;
const int IN3 = 7;
const int IN4 = 8;

// Painonapit
const int BTN_UP = A0;
const int BTN_DOWN = A1;
const int BTN_LEFT = A2;
const int BTN_RIGHT = A3;

// Moodin valinta
const int MODE_PIN = 2;
bool isTransmitter = true;

// Ohjausdata
struct MotorData {
  int8_t speed;           // -100..+100
  int8_t turn;            // -100..+100
  float currentSpeed = 0.0f;
  float currentTurn = 0.0f;
};

MotorData mData;

// Paketin rakenne
#define SYNC_BYTE1 0xAA
#define SYNC_BYTE2 0x55
#define MAX_DATA_SIZE 10

// Sujuvuus
const float ACCEL = 3.5f;
const float DECEL = 6.0f;
const int MAX_PWM = 200;

// Aika
unsigned long lastUpdate = 0;
const int TX_INTERVAL = 40;   // 25Hz
const int RX_INTERVAL = 20;   // 50Hz

// CRC-16 laskenta (ilman taulukkoa)
uint16_t calculateCRC16(const uint8_t* data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];
    
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;  // Modbus polynomi
      } else {
        crc = crc >> 1;
      }
    }
  }
  
  return crc;
}

// Paketin käsittelyn tila
enum PacketState {
  WAITING_FOR_SYNC1,
  WAITING_FOR_SYNC2,
  WAITING_FOR_LENGTH,
  WAITING_FOR_DATA,
  WAITING_FOR_CRC_LOW,
  WAITING_FOR_CRC_HIGH
};

struct PacketParser {
  PacketState state = WAITING_FOR_SYNC1;
  uint8_t dataBuffer[MAX_DATA_SIZE];
  uint8_t dataLength = 0;
  uint8_t dataReceived = 0;
  uint16_t packetCount = 0;
  uint16_t errorCount = 0;
  
  void reset() {
    state = WAITING_FOR_SYNC1;
    dataLength = 0;
    dataReceived = 0;
  }
};

PacketParser parser;

void setup() {
  Serial.begin(1200);
  
  // Pinnien asetukset
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);
  
  // Alustus
  stopMotors();
  
  isTransmitter = digitalRead(MODE_PIN);
  delay(100);
  
  Serial.println(isTransmitter ? "TX - Lähetin" : "RX - Vastaanotin");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Moodin tarkistus
  static unsigned long lastModeCheck = 0;
  if (currentTime - lastModeCheck > 500) {
    bool newMode = digitalRead(MODE_PIN);
    if (newMode != isTransmitter) {
      isTransmitter = newMode;
      resetControls();
      Serial.println(isTransmitter ? "TX - Lähetin" : "RX - Vastaanotin");
    }
    lastModeCheck = currentTime;
  }
  
  if (isTransmitter) {
    handleTransmitter(currentTime);
  } else {
    handleReceiver(currentTime);
  }
}

void handleTransmitter(unsigned long currentTime) {
  static unsigned long lastTxTime = 0;
  static unsigned long lastProcessTime = 0;
  
  // Päivitä ohjaus 50Hz
  if (currentTime - lastProcessTime >= 20) {
    readButtons();
    updateSmoothValues();
    
    mData.speed = constrain(mData.currentSpeed * 100, -100, 100);
    mData.turn = constrain(mData.currentTurn * 100, -100, 100);
    
    lastProcessTime = currentTime;
  }
  
  // Lähetä data 25Hz
  if (currentTime - lastTxTime >= TX_INTERVAL) {
    sendMotorData();
    lastTxTime = currentTime;
  }
}

void handleReceiver(unsigned long currentTime) {
  static unsigned long lastMotorUpdate = 0;
  
  // Käsittele saapuvat paketit
  if (parsePacket()) {
    // Paketti käsitelty onnistuneesti
    mData.currentSpeed = mData.speed / 100.0f;
    mData.currentTurn = mData.turn / 100.0f;
  }
  
  // Päivitä moottorit 50Hz
  if (currentTime - lastMotorUpdate >= RX_INTERVAL) {
    updateMotors();
    lastMotorUpdate = currentTime;
  }
}

bool parsePacket() {
  while (Serial.available() > 0) {
    uint8_t receivedByte = Serial.read();
    
    switch (parser.state) {
      
      case WAITING_FOR_SYNC1:
        // Etsi ensimmäinen synkkaustavu
        if (receivedByte == SYNC_BYTE1) {
          parser.state = WAITING_FOR_SYNC2;
        }
        break;
        
      case WAITING_FOR_SYNC2:
        // Odota toista synkkaustavua
        if (receivedByte == SYNC_BYTE2) {
          parser.state = WAITING_FOR_LENGTH;
        } else {
          // Virheellinen sekvenssi, aloita alusta
          parser.state = WAITING_FOR_SYNC1;
          parser.errorCount++;
        }
        break;
        
      case WAITING_FOR_LENGTH:
        // Lue datan pituus
        if (receivedByte <= MAX_DATA_SIZE && receivedByte > 0) {
          parser.dataLength = receivedByte;
          parser.dataReceived = 0;
          parser.state = WAITING_FOR_DATA;
        } else {
          // Virheellinen pituus, aloita alusta
          parser.state = WAITING_FOR_SYNC1;
          parser.errorCount++;
        }
        break;
        
      case WAITING_FOR_DATA:
        // Kerää dataa puskuriin
        parser.dataBuffer[parser.dataReceived] = receivedByte;
        parser.dataReceived++;
        
        // Tarkista onko kaikki data saatu
        if (parser.dataReceived >= parser.dataLength) {
          parser.state = WAITING_FOR_CRC_LOW;
        }
        break;
        
      case WAITING_FOR_CRC_LOW:
        // Lue CRC:n ala-tavu
        parser.dataBuffer[parser.dataLength] = receivedByte;  // Väliaikainen tallennus
        parser.state = WAITING_FOR_CRC_HIGH;
        break;
        
      case WAITING_FOR_CRC_HIGH:
        {
          // Lue CRC:n ylä-tavu ja tarkista CRC
          uint16_t receivedCRC = (receivedByte << 8) | parser.dataBuffer[parser.dataLength];
          
          // Laske CRC saadusta datasta
          uint16_t calculatedCRC = calculateCRC16(parser.dataBuffer, parser.dataLength);
          
          if (receivedCRC == calculatedCRC) {
            // CRC OK, käsittele data
            
            // Oletetaan että data sisältää: [speed, turn]
            if (parser.dataLength >= 2) {
              mData.speed = (int8_t)parser.dataBuffer[0];
              mData.turn = (int8_t)parser.dataBuffer[1];
              
              parser.packetCount++;
              
              // Tulosta statistiikkaa ajoittain
              if (parser.packetCount % 100 == 0) {
                Serial.print("Paketteja: ");
                Serial.print(parser.packetCount);
                Serial.print(" Virheitä: ");
                Serial.println(parser.errorCount);
              }
              
              parser.reset();
              return true;
            }
          } else {
            // CRC virhe
            parser.errorCount++;
            
            // Tulosta virhe ajoittain (ei liian usein)
            if (parser.errorCount % 10 == 0) {
              Serial.print("CRC virhe! Paketteja: ");
              Serial.print(parser.packetCount);
              Serial.print(" Virheitä: ");
              Serial.println(parser.errorCount);
            }
          }
          
          // Aloita seuraava paketti
          parser.reset();
        }
        break;
    }
  }
  
  return false;
}

void sendMotorData() {
  // Valmistele lähetettävä data
  uint8_t txData[2];
  txData[0] = (uint8_t)mData.speed;
  txData[1] = (uint8_t)mData.turn;
  
  // Laske CRC
  uint16_t crc = calculateCRC16(txData, 2);
  
  // Lähetä paketti
  Serial.write(SYNC_BYTE1);
  Serial.write(SYNC_BYTE2);
  Serial.write(0x02);           // Datan pituus (2 tavua)
  Serial.write(txData, 2);      // Itse data
  Serial.write(crc & 0xFF);     // CRC ala-tavu
  Serial.write(crc >> 8);       // CRC ylä-tavu
}

void readButtons() {
  bool up = !digitalRead(BTN_UP);
  bool down = !digitalRead(BTN_DOWN);
  bool left = !digitalRead(BTN_LEFT);
  bool right = !digitalRead(BTN_RIGHT);
  
  // Määritä tavoitteet painonappien perusteella
  float targetSpeed = 0.0f;
  float targetTurn = 0.0f;
  
  if (up && !down) {
    targetSpeed = 1.0f;
  } else if (!up && down) {
    targetSpeed = -1.0f;
  }
  
  if (left && !right) {
    targetTurn = -1.0f;
  } else if (!left && right) {
    targetTurn = 1.0f;
  }
  
  // Päivitä sujuvasti kohti tavoitteita
  float speedAccel = (targetSpeed != 0.0f) ? ACCEL : DECEL;
  float turnAccel = (targetTurn != 0.0f) ? ACCEL : DECEL;
  
  mData.currentSpeed += (targetSpeed - mData.currentSpeed) * speedAccel * 0.02f;
  mData.currentTurn += (targetTurn - mData.currentTurn) * turnAccel * 0.02f;
  
  // Deadzone pienten arvojen käsittelyyn
  if (abs(mData.currentSpeed) < 0.05f) mData.currentSpeed = 0.0f;
  if (abs(mData.currentTurn) < 0.05f) mData.currentTurn = 0.0f;
}

void updateSmoothValues() {
  // Sujuvuus käsitellään readButtons()-funktiossa
}

void updateMotors() {
  // Laske moottoritehot
  float leftPower = mData.currentSpeed - (mData.currentTurn * 0.7f);
  float rightPower = mData.currentSpeed + (mData.currentTurn * 0.7f);
  
  // Normalisoi jos jompikumpi ylittää 1.0
  float maxPower = max(abs(leftPower), abs(rightPower));
  if (maxPower > 1.0f) {
    leftPower /= maxPower;
    rightPower /= maxPower;
  }
  
  // Aseta moottorien nopeudet
  setMotor(ENA, IN1, IN2, leftPower * MAX_PWM);
  setMotor(ENB, IN3, IN4, rightPower * MAX_PWM);
}

void setMotor(int enPin, int in1, int in2, int speed) {
  // Rajoita nopeus
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  
  if (speed > 0) {
    // Eteenpäin
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enPin, speed);
  } else if (speed < 0) {
    // Taaksepäin
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enPin, -speed);
  } else {
    // Pysähdy
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enPin, 0);
  }
}

void stopMotors() {
  setMotor(ENA, IN1, IN2, 0);
  setMotor(ENB, IN3, IN4, 0);
}

void resetControls() {
  mData.speed = 0;
  mData.turn = 0;
  mData.currentSpeed = 0.0f;
  mData.currentTurn = 0.0f;
  parser.reset();
  stopMotors();
}
