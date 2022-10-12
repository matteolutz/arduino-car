#include <SoftwareSerial.h>

#define RX_PIN 10
#define TX_PIN 11

#define SERIAL_T SoftwareSerial

class ComMaster {
private:
  SERIAL_T* _s;

  bool _foundBegin = false;

  size_t _currDataBufIdx;
  uint8_t* _dataBuf;

  void handlePacketHelloWorld(uint8_t from);

  void sendPacketAck(uint8_t to);

  uint8_t handleNextByte();
  uint8_t shiftByte(uint8_t b);

public:
  static const uint8_t PACKET_SYN = 0x1;
  static const uint8_t PACKET_ACK = 0x2;
  static const uint8_t PACKET_HELLO_WORLD = 0x3;

  static const uint8_t PACKET_DATA = 0x4;

  static const uint8_t PACKET_BEGIN_CHAR = 0x1;
  static const uint8_t PACKET_TERMINATION_CHAR = 0x2;

  static const uint8_t MASTER_ID = 0x1;

  static const unsigned long BAUD_RATE = 115200;

  ComMaster(SERIAL_T* s);
  ~ComMaster();

  void sendPacket(uint8_t* buf, size_t len);

  bool init();
  void update();
};

ComMaster::ComMaster(SERIAL_T* s)
  : _s(s), _dataBuf(new uint8_t[0xff]), _currDataBufIdx(0) {}
ComMaster::~ComMaster() {
  delete _dataBuf;
}

bool ComMaster::init() {
  _s->begin(ComMaster::BAUD_RATE);
  return true;
}

uint8_t ComMaster::handleNextByte() {
  _currDataBufIdx--;
  return _dataBuf[_currDataBufIdx] >> 2;
}

uint8_t ComMaster::shiftByte(uint8_t b) {
  return b << 2;
}

void ComMaster::sendPacket(uint8_t* buf, size_t len) {
  size_t packetSize = len + 2;

  uint8_t nBuf[packetSize];
  nBuf[0] = ComMaster::PACKET_BEGIN_CHAR;
  
  for(size_t i = 0; i < len; i++) {
    nBuf[i+1] = shiftByte(buf[i]);
  }

  nBuf[packetSize - 1] = ComMaster::PACKET_TERMINATION_CHAR;

  _s->write(nBuf, packetSize);
}

void ComMaster::handlePacketHelloWorld(uint8_t from) {
  uint8_t num = handleNextByte();

  Serial.print("Hello World!, from: ");
  Serial.print(from, HEX);
  Serial.print(" , with data: ");
  Serial.println(num, HEX);
}

void ComMaster::sendPacketAck(uint8_t to) {
  uint8_t buf[] = { ComMaster::MASTER_ID, to, ComMaster::PACKET_ACK };
  sendPacket(buf, 3);
}

void ComMaster::update() {
  if(_s->available()) {
    uint8_t c = _s->read();
    
    if(_foundBegin) {
      if(c == ComMaster::PACKET_TERMINATION_CHAR) {
        size_t bytesRead = _currDataBufIdx;

        uint8_t packetType = handleNextByte();
        uint8_t packetReceiver = handleNextByte();

        if(packetReceiver != ComMaster::MASTER_ID) {
          size_t packetSize = bytesRead + 2;

          uint8_t nBuf[packetSize];
          nBuf[0] = ComMaster::PACKET_BEGIN_CHAR;
          memcpy(nBuf + 1, _dataBuf, bytesRead);
          nBuf[packetSize - 1] = ComMaster::PACKET_TERMINATION_CHAR;

          _s->write(nBuf, packetSize);
          goto reset;
        }

        uint8_t packetSender = handleNextByte();

        switch (packetType) {
          case ComMaster::PACKET_SYN:
            Serial.print("New slave, welcome! Sending ACK to: ");
            Serial.println(packetSender, HEX);
            sendPacketAck(packetSender);
            break;
          case ComMaster::PACKET_HELLO_WORLD:
            uint8_t msg1 = handleNextByte();
            uint8_t msg2 = handleNextByte();
            uint8_t msg = msg1 | (msg2 << 5);
            Serial.print("Got Hello, World!, from slave ");
            Serial.print(packetSender, HEX);
            Serial.print(", with information ");
            Serial.println(msg, HEX);
            break;
          default:
            Serial.print("Unknown packet type: ");
            Serial.print(packetType);
            Serial.println(", ignoring...");
            goto reset;
            break;
        }

        if(_currDataBufIdx != 0) {
          Serial.println("Databuffer not read completely! Ignoring...");
        }

reset:
        _foundBegin = false;
        _currDataBufIdx = 0;
        memset(_dataBuf, 0, 0xff);
        return;
      }

      _dataBuf[_currDataBufIdx] = c;
      _currDataBufIdx++;
      return;
    }

    if(c == ComMaster::PACKET_BEGIN_CHAR) {
      _foundBegin = true;
    }
  }
}

SoftwareSerial s(RX_PIN, TX_PIN);

ComMaster com(&s);

long last;
bool ledState = false;

long last2;
bool ledState2 = false;


void setup() {
  Serial.begin(115200);

  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);

  if (!com.init()) {
    Serial.println("Failed to init ComMaster, exiting...");
    for (;;) {}
  }

  Serial.println("Init done, running...");

  last = millis();
  last2 = millis();
}

void loop() {
  com.update();

  if((millis() - last) >= 1000) {
    ledState = !ledState;
    uint8_t buf[] = {
      ledState, ComMaster::MASTER_ID, 0x7, ComMaster::PACKET_DATA
    };
    com.sendPacket(buf, 4);
    last = millis();
  }

  if((millis() - last2) >= 500) {
    ledState2 = !ledState2;
    uint8_t buf[] = {
      ledState2, ComMaster::MASTER_ID, 0x8, ComMaster::PACKET_DATA
    };
    com.sendPacket(buf, 4);
    last2 = millis();
  }
}