/*
 * ROBOTTI-AUTON OHJAUSJÄRJESTELMÄ
 * 
 * Tämä ohjelma ohjaa robottiautoa kahdella moottorilla.
 * Sama koodi toimii sekä LÄHETTÄJÄNÄ (TX) että VASTAANOTTIMENA (RX).
 * Lähettäjä lukee painonappeja ja lähettää dataa sarjaportin kautta.
 * Vastaanotin vastaanottaa datan ja ohjaa moottoreita.
 * 
 * Toiminnot:
 * - UP/DOWN: Eteen/taakse liikkuminen
 * - LEFT/RIGHT: Kääntyminen vasemmalle/oikealle
 * - Sujuvat kiihtyvyydet ja hidastukset
 * - Automaattinen pysäytys jos signaali katkeaa
 * - Käännös palautuu nollaan sujuvasti kun auto pysähtyy
 */

// ==================== PINNIEN MÄÄRITYKSET ====================

// Vasemman moottorin pinnit (PWM- ja suuntapinnit)
const int ENA = 5;   // Vasemman moottorin PWM-nopeus
const int IN1 = 4;   // Vasemman moottorin suunta 1
const int IN2 = 3;   // Vasemman moottorin suunta 2

// Oikean moottorin pinnit (PWM- ja suuntapinnit)
const int ENB = 6;   // Oikean moottorin PWM-nopeus
const int IN3 = 7;   // Oikean moottorin suunta 1
const int IN4 = 8;   // Oikean moottorin suunta 2

// Ohjauspainikkeiden pinnit (käytetään sisäistä vetovastusta)
const int BTN_UP = A0;     // Liiku eteenpäin
const int BTN_DOWN = A1;   // Liiku taaksepäin
const int BTN_LEFT = A2;   // Käänny vasemmalle
const int BTN_RIGHT = A3;  // Käänny oikealle

// Moodin valintapinni (määrittää onko laite lähettäjä vai vastaanotin)
const int MODE_PIN = 2;    // HIGH = Lähettäjä, LOW = Vastaanotin
bool isTransmitter = true; // Nykyinen tila

// ==================== OHJAUSDATAN RAKENNE ====================

/**
 * Moottorien ohjausdata
 * Sisältää nopeuden ja käännöksen arvot sekä niiden sujuvat versiot
 */
struct MotorData {
  int8_t speed;           // Nopeus: -100 (taakse) ... +100 (eteen)
  int8_t turn;            // Käännös: -100 (vasen) ... +100 (oikea)
  float currentSpeed = 0.0f;  // Sujuvasti muuttuva nopeus
  float currentTurn = 0.0f;   // Sujuvasti muuttuva käännös
};

MotorData mData; // Pääohjausdata-objekti

// ==================== SARJAPROTOKOLLAN MÄÄRITYKSET ====================

// Synkkaustavut paketin alkuun (paketin tunnistamiseksi)
#define SYNC_BYTE1 0xAA  // Ensimmäinen synkkaustavu
#define SYNC_BYTE2 0x55  // Toinen synkkaustavu
#define MAX_DATA_SIZE 10 // Suurin sallittu datan pituus

// Sujuvuuden parametrit (kiihtyvyys ja hidastus)
const float ACCEL = 3.5f;   // Kiihtyvyys kun liikutaan
const float DECEL = 6.0f;   // Hidastus kun pysäytellään
const int MAX_PWM = 200;    // Maksimi PWM-arvo moottoreille

// Aikaparametrit
unsigned long lastUpdate = 0;        // Viimeisin päivitysaika
unsigned long lastPacketTime = 0;    // Viimeisin vastaanotetun paketin aika
const int TX_INTERVAL = 40;          // Lähetystaajuus: 25Hz (1000ms / 40ms)
const int RX_INTERVAL = 20;          // Vastaanottotaajuus: 50Hz (1000ms / 20ms)
const unsigned long SIGNAL_TIMEOUT = 1200; // Signaalin katkeamisen timeout: 200ms

// ==================== CRC-16 TARKISTUSSUMMA ====================

/**
 * Laskee CRC-16 tarkistussumman datalle (Modbus-polynomi)
 * @param data Datataulukko
 * @param length Datataulukon pituus
 * @return CRC-16 tarkistussumma
 */
uint16_t calculateCRC16(const uint8_t* data, uint16_t length) {
  uint16_t crc = 0xFFFF; // Alustusarvo
  
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i]; // XOR nykyisen tavun kanssa
    
    for (uint8_t bit = 0; bit < 8; bit++) {  
      if (crc & 0x0001) {  // Jos alin bitti on 1
        crc = (crc >> 1) ^ 0xA001;  // Siirrä ja XOR polynomilla
      } else {  
        crc = crc >> 1;  // Vain siirto
      }  
    }
  }
  
  return crc;
}

// ==================== PAKETIN JÄSENNYKSEN TILAT ====================

/**
 * Paketin jäsennystilat
 * Vastaanotettu data käsitellään tilakoneella
 */
enum PacketState {
  WAITING_FOR_SYNC1,     // Odotetaan 1. synkkaustavua
  WAITING_FOR_SYNC2,     // Odotetaan 2. synkkaustavua
  WAITING_FOR_LENGTH,    // Odotetaan datan pituutta
  WAITING_FOR_DATA,      // Odotetaan itse dataa
  WAITING_FOR_CRC_LOW,   // Odotetaan CRC:n alaosaa
  WAITING_FOR_CRC_HIGH   // Odotetaan CRC:n yläosaa
};

/**
 * Paketin jäsentäjä
 * Hallinnoi paketin käsittelyn tilaa ja dataa
 */
struct PacketParser {
  PacketState state = WAITING_FOR_SYNC1; // Nykyinen tila
  uint8_t dataBuffer[MAX_DATA_SIZE];     // Datapuskuri
  uint8_t dataLength = 0;                // Odotettu datan pituus
  uint8_t dataReceived = 0;              // Vastaanotettu datan määrä
  uint16_t packetCount = 0;              // Onnistuneesti käsiteltyjen pakettien määrä
  uint16_t errorCount = 0;               // Virheellisten pakettien määrä
  
  /**
   * Nollaa jäsentäjän tilan
   */
  void reset() {
    state = WAITING_FOR_SYNC1;
    dataLength = 0;
    dataReceived = 0;
  }
};

PacketParser parser; // Pääjäsentäjä-objekti

// ==================== PÄÄSETUP-FUNKTIO ====================

/**
 * Alustusfunktio - suoritetaan kerran käynnistyessä
 */
void setup() {
  Serial.begin(600); // Alusta sarjaportti 9600 baudin nopeudella
  
  // Aseta moottoripinnit lähtötilaan
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Aseta painonapit sisäisellä vetovastuksella
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);
  
  // Alustus: pysäytä moottorit ja aseta moodi
  stopMotors();
  isTransmitter = digitalRead(MODE_PIN); // Lue moodi pinnistä
  lastPacketTime = millis(); // Alusta pakettiajastin
  delay(100); // Lyhyt viive stabiloitua varten
  
  // Tulosta käynnistysviesti
  //Serial.println(isTransmitter ? "TX - Lähetin" : "RX - Vastaanotin");
}

// ==================== PÄÄLOOP-FUNKTIO ====================

/**
 * Pääsilmukka - suoritetaan jatkuvasti
 */
void loop() {
  unsigned long currentTime = millis(); // Nykyinen aika
  
  // Tarkista moodin muutos (500ms välein)
  static unsigned long lastModeCheck = 0;
  if (currentTime - lastModeCheck > 500) {
    bool newMode = digitalRead(MODE_PIN);
    if (newMode != isTransmitter) {
      isTransmitter = newMode;
      resetControls(); // Nollaa ohjausarvot moodin vaihtuessa
      //Serial.println(isTransmitter ? "TX - Lähetin" : "RX - Vastaanotin");
    }
    lastModeCheck = currentTime;
  }
  
  // Suorita joko lähettäjän tai vastaanottimen tehtävät
  if (isTransmitter) {
    handleTransmitter(currentTime);
  } else {
    handleReceiver(currentTime);
  }
}

// ==================== LÄHETTÄJÄN TOIMINNOT ====================

/**
 * Käsittelee lähettäjän toiminnot
 * @param currentTime Nykyinen aika
 */
void handleTransmitter(unsigned long currentTime) {
  static unsigned long lastTxTime = 0;       // Viimeisin lähetysaika
  static unsigned long lastProcessTime = 0;  // Viimeisin prosessointiaika
  
  // Päivitä ohjausarvot 50Hz taajuudella (20ms välein)
  if (currentTime - lastProcessTime >= 20) {
    readButtons();        // Lue painonappien tilat
    updateSmoothValues(); // Päivitä sujuvat arvot
    
    // Muunna sujuvat arvot lähetettäväksi muotoon (-100...+100)
    mData.speed = constrain(mData.currentSpeed * 100, -100, 100);  
    mData.turn = constrain(mData.currentTurn * 100, -100, 100);  
    
    lastProcessTime = currentTime;
  }
  
  // Lähetä data 25Hz taajuudella (40ms välein)
  if (currentTime - lastTxTime >= TX_INTERVAL) {
    sendMotorData(); // Lähetä moottoridata
    lastTxTime = currentTime;
  }
}

// ==================== VASTAANOTTIMEN TOIMINNOT ====================

/**
 * Käsittelee vastaanottimen toiminnot
 * @param currentTime Nykyinen aika
 */
void handleReceiver(unsigned long currentTime) {
  static unsigned long lastMotorUpdate = 0;  // Viimeisin moottoripäivitys
  static unsigned long lastSmoothUpdate = 0; // Viimeisin sujuvuuspäivitys
  static bool signalLost = false;            // Onko signaali katkennut
  
  // Käsittele saapuvat sarjaportin paketit
  if (Serial.available() > 0) {
    if (parsePacket()) { // Jos saatiin validi paketti
      lastPacketTime = currentTime; // Päivitä viimeisen paketin aika
      
      // Aseta suorat arvot nopeudelle ja käännökselle
      mData.currentSpeed = mData.speed / 100.0f;
      mData.turn = mData.turn; // Säilytetään kokonaislukuna
      
      // Jos signaali palasi katkenneen tilan jälkeen
      if (signalLost) {
        //rSerial.println("Signaali palautui");
        signalLost = false;
      }
    }
  }
  
  // Tarkista onko signaali katkennut (yli 200ms ilman paketteja)
  if (currentTime - lastPacketTime > SIGNAL_TIMEOUT) {
    if (!signalLost) {
      emergencyStop(); // Pysäytä moottorit hätätilassa
      signalLost = true;
    }
    return; // Älä päivitä moottoreita kun signaali on katkennut
  }
  
  // Päivitä käännöksen sujuvuus 50Hz taajuudella (20ms välein)
    if (currentTime - lastSmoothUpdate >= RX_INTERVAL) {
    
      float targetSpeed = mData.speed / 100.0f; // -1.0 ... +1.0
      float targetTurn  = mData.turn  / 100.0f;
    
      float speedAccel = (targetSpeed != 0.0f) ? ACCEL : DECEL;
      float turnAccel  = (targetTurn  != 0.0f) ? ACCEL : DECEL;
    
      mData.currentSpeed += (targetSpeed - mData.currentSpeed) * speedAccel * 0.02f;
      mData.currentTurn  += (targetTurn  - mData.currentTurn)  * turnAccel  * 0.02f;
    
      // Deadzone
      if (abs(mData.currentSpeed) < 0.05f) mData.currentSpeed = 0.0f;
      if (abs(mData.currentTurn)  < 0.05f) mData.currentTurn  = 0.0f;
    
      lastSmoothUpdate = currentTime;
    }
  
  // Päivitä moottorit 50Hz taajuudella (20ms välein)
  if (currentTime - lastMotorUpdate >= RX_INTERVAL) {
    updateMotors(); // Päivitä moottorien nopeudet
    lastMotorUpdate = currentTime;
  }
}

// ==================== PAKETIN KÄSITTELY ====================

/**
 * Jäsentää saapuvan paketin sarjaportista
 * @return true jos paketti käsiteltiin onnistuneesti, muuten false
 */
bool parsePacket() {
  bool packetReceived = false; // Palautusarvo
  
  // Käsittele kaikki saatavilla olevat tavut
  while (Serial.available() > 0) {
    uint8_t receivedByte = Serial.read(); // Lue yksi tavu
    
    switch (parser.state) {  
        
      case WAITING_FOR_SYNC1:  
        // Odota ensimmäistä synkkaustavua (0xAA)
        if (receivedByte == SYNC_BYTE1) {
          parser.state = WAITING_FOR_SYNC2; // Siirry seuraavaan tilaan
        }
        break;  
          
      case WAITING_FOR_SYNC2:  
        // Odota toista synkkaustavua (0x55)
        if (receivedByte == SYNC_BYTE2) {
          parser.state = WAITING_FOR_LENGTH; // Siirry pituuden odotukseen
        } else {
          // Virheellinen sekvenssi, aloita alusta
          parser.state = WAITING_FOR_SYNC1;
          parser.errorCount++;
        }
        break;  
          
      case WAITING_FOR_LENGTH:  
        // Lue datan pituus (tavua)
        if (receivedByte <= MAX_DATA_SIZE && receivedByte >= 2) {
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
        // Lue CRC:n ala-tavu (vähiten merkitsevä tavu)
        parser.dataBuffer[parser.dataLength] = receivedByte;  // Väliaikainen tallennus
        parser.state = WAITING_FOR_CRC_HIGH;  
        break;  
          
      case WAITING_FOR_CRC_HIGH:  
        {  
          // Lue CRC:n ylä-tavu (eniten merkitsevä tavu)
          uint16_t receivedCRC = (receivedByte << 8) | parser.dataBuffer[parser.dataLength];  
            
          // Laske CRC saadusta datasta
          uint16_t calculatedCRC = calculateCRC16(parser.dataBuffer, parser.dataLength);  
            
          // Tarkista CRC:n oikeellisuus
          if (receivedCRC == calculatedCRC) {  
            // CRC OK, käsittele data  
              
            // Data sisältää: [speed, turn]  
            if (parser.dataLength >= 2) {  
              mData.speed = (int8_t)parser.dataBuffer[0];  // Nopeus (-100...+100)
              mData.turn = (int8_t)parser.dataBuffer[1];   // Käännös (-100...+100)
                
              parser.packetCount++;  
                
              // Tulosta statistiikkaa joka 100. paketti
              if (parser.packetCount % 100 == 0) {  
               // Serial.print("Paketteja: ");  
               // Serial.print(parser.packetCount);  
               // Serial.print(" Virheitä: ");  
                //Serial.println(parser.errorCount);  
              }  
                
              packetReceived = true; // Merkkaa onnistunut paketti
            }  
          } else {  
            // CRC virhe  
            parser.errorCount++;  
              
            // Tulosta virhe joka 10. virhe
            if (parser.errorCount % 10 == 0) {  
              //Serial.print("CRC virhe! Paketteja: ");  
              //Serial.print(parser.packetCount);  
              //Serial.print(" Virheitä: ");  
              //Serial.println(parser.errorCount);  
            }  
          }  
            
          // Aloita seuraava paketti riippumatta onnistumisesta
          parser.reset();  
        }  
        break;  
    }
  }
  
  return packetReceived;
}

// ==================== DATAN LÄHETYS ====================

/**
 * Lähettää moottoridatan sarjaportin kautta
 */
void sendMotorData() {
  // Valmistele lähetettävä data (2 tavua)
  uint8_t txData[2];
  txData[0] = (int8_t)mData.speed;  // Nopeus
  txData[1] = (int8_t)mData.turn;   // Käännös
  
  // Laske CRC-16 tarkistussumma datalle
  uint16_t crc = calculateCRC16(txData, 2);
  
  // Lähetä paketti:
  // 1. Synkkaustavut
  // 2. Datan pituus
  // 3. Itse data
  // 4. CRC tarkistussumma
  Serial.write(SYNC_BYTE1);        // 1. synkkaustavu
  Serial.write(SYNC_BYTE2);        // 2. synkkaustavu
  Serial.write(0x02);              // Datan pituus (2 tavua)
  Serial.write(txData, 2);         // Itse data
  Serial.write(crc & 0xFF);        // CRC ala-tavu
  Serial.write(crc >> 8);          // CRC ylä-tavu
}

// ==================== PAINONAPPIEN LUKU ====================

/**
 * Lukee painonappien tilat ja päivittää kohdearvot
 */
void readButtons() {
  // Lue painonappien tilat (invertoitu koska INPUT_PULLUP)
  bool up = !digitalRead(BTN_UP);     // Eteenpäin
  bool down = !digitalRead(BTN_DOWN); // Taaksepäin
  bool left = !digitalRead(BTN_LEFT); // Vasemmalle
  bool right = !digitalRead(BTN_RIGHT); // Oikealle
  
  // Alusta kohdearvot
  float targetSpeed = 0.0f;  // Nopeuden kohdearvo
  float targetTurn = 0.0f;   // Käännöksen kohdearvo
  
  // Määritä nopeuden kohdearvo UP/DOWN-nappien perusteella
  if (up && !down) {
    targetSpeed = 1.0f;      // Täysi eteenpäin
  } else if (!up && down) {
    targetSpeed = -1.0f;     // Täysi taaksepäin
  }
  // Molemmat pois tai molemmat päällä -> targetSpeed = 0.0f
  
  // Määritä käännöksen kohdearvo LEFT/RIGHT-nappien perusteella
  if (left && !right) {
    targetTurn = -1.0f;      // Täysi vasemmalle
  } else if (!left && right) {
    targetTurn = 1.0f;       // Täysi oikealle
  }
  // Molemmat pois tai molemmat päällä -> targetTurn = 0.0f
  
  // Päivitä sujuvasti kohti kohdearvoja
  
  // Valitse kiihtyvyys/hidastus arvo sen mukaan onko kohdearvo 0
  float speedAccel = (targetSpeed != 0.0f) ? ACCEL : DECEL;
  float turnAccel = (targetTurn != 0.0f) ? ACCEL : DECEL;
  
  // Päivitä nopeus sujuvasti (20ms aikaväli)
  mData.currentSpeed += (targetSpeed - mData.currentSpeed) * speedAccel * 0.02f;
  
  // Päivitä käännös sujuvasti (20ms aikaväli)
  mData.currentTurn += (targetTurn - mData.currentTurn) * turnAccel * 0.02f;
  
  // Deadzone: nollaa hyvin pienet arvot
  if (abs(mData.currentSpeed) < 0.05f) mData.currentSpeed = 0.0f;
  if (abs(mData.currentTurn) < 0.05f) mData.currentTurn = 0.0f;
}

// ==================== MOOTTORIEN OHJAUS ====================

/**
 * Päivittää moottorien nopeudet nykyisten arvojen perusteella
 */
void updateMotors() {
  // Kun auto on pysähtynyt, palauta käännös sujuvasti nollaan
  if (abs(mData.currentSpeed) < 0.05f) {
    // Käytä hidastusarvoa (DECEL) palauttaessa nollaan
    mData.currentTurn += (0.0f - mData.currentTurn) * DECEL * 0.02f;
    
    // Pienet arvot nollataan
    if (abs(mData.currentTurn) < 0.05f) {
      mData.currentTurn = 0.0f;
    }
  }
  
  // Laske moottorien tehot:
  // - Vasen moottori = nopeus - (käännös * 0.7)
  // - Oikea moottori = nopeus + (käännös * 0.7)
  // Kerroin 0.7 säätää käännöksen voimakkuutta suhteessa nopeuteen
  float leftPower = mData.currentSpeed - (mData.currentTurn * 0.7f);
  float rightPower = mData.currentSpeed + (mData.currentTurn * 0.7f);
  
  // Normalisoi tehot jos jompikumpi ylittää 1.0
  float maxPower = max(abs(leftPower), abs(rightPower));
  if (maxPower > 1.0f) {
    leftPower /= maxPower;
    rightPower /= maxPower;
  }
  
  // Aseta moottorien nopeudet (PWM-arvot)
  setMotor(ENA, IN1, IN2, leftPower * MAX_PWM);
  setMotor(ENB, IN3, IN4, rightPower * MAX_PWM);
}

/**
 * Asettaa yhden moottorin nopeuden ja suunnan
 * @param enPin PWM-pinni (nopeus)
 * @param in1 Suunta-pinni 1
 * @param in2 Suunta-pinni 2
 * @param speed Nopeus (-MAX_PWM...+MAX_PWM)
 */
void setMotor(int enPin, int in1, int in2, int speed) {
  // Rajoita nopeus sallittuun alueeseen
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  
  if (speed > 0) {
    // Eteenpäin
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enPin, MAX_PWM-speed);
  } else if (speed < 0) {
    // Taaksepäin
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enPin, -speed); // Positiivinen arvo
  } else {
    // Pysähdy
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enPin, 0);
  }
}

/**
 * Pysäyttää molemmat moottorit
 */
void stopMotors() {
  setMotor(ENA, IN1, IN2, 0);
  setMotor(ENB, IN3, IN4, 0);
}

// ==================== HÄTÄTILATOIMINNOT ====================

/**
 * Hätäpysäytys - pysäyttää moottorit välittömästi
 * Käytetään kun signaali katkeaa tai muu hätätilanne
 */
void emergencyStop() {
  static unsigned long lastEmergencyStop = 0;
  
  // Estä liian useat tulostukset (max 1 sekunnin välein)
  if (millis() - lastEmergencyStop > 1000) {
    Serial.println("!!! SIGNAL LOST - EMERGENCY STOP !!!");
    lastEmergencyStop = millis();
  }
  
  // Pysäytä moottorit välittömästi
  stopMotors();
  
  // Nollaa kaikki ohjausarvot
  mData.speed = 0;
  mData.turn = 0;
  mData.currentSpeed = 0.0f;
  mData.currentTurn = 0.0f;
  
  // Nollaa paketin jäsentäjä
  parser.reset();
}

// ==================== APUFUNKTIOT ====================

/**
 * Päivittää sujuvat arvot (nykyisessä versiossa tämä on
 * sisällytetty muihin funktioihin, pidetään yhteensopivuuden vuoksi)
 */
void updateSmoothValues() {
  // Sujuvuus käsitellään readButtons()-funktiossa
  // Tämä funktio on jätetty yhteensopivuuden vuoksi
}

/**
 * Nollaa ohjausarvot ja moottorit
 * Käytetään moodin vaihtuessa
 */
void resetControls() {
  mData.speed = 0;
  mData.turn = 0;
  mData.currentSpeed = 0.0f;
  mData.currentTurn = 0.0f;
  parser.reset();
  stopMotors();
}
