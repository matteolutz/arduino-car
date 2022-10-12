#include <SoftwareSerial.h>

#define ID 0x7

#define RX_PIN 10
#define TX_PIN 11

#define SERIAL_T SoftwareSerial

class ComSlave {
private:
  SERIAL_T* _s;

  bool _foundBegin = false;

  size_t _currDataBufIdx;
  uint8_t* _dataBuf;
  uint8_t _id;

  bool _synSent = false;
  bool _ackRecv = false;

  bool handlePacketAck();

  void sendPacketSyn();

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

  ComSlave(SERIAL_T* s, uint8_t id);
  ~ComSlave();

  void sendPacket(uint8_t* buf, size_t len);

  bool init();
  void update();
};

ComSlave::ComSlave(SERIAL_T* s, uint8_t id)
  : _s(s), _dataBuf(new uint8_t[0xff]), _currDataBufIdx(0), _id(id) {
  }
ComSlave::~ComSlave() {
  delete _dataBuf;
}

bool ComSlave::init() {
  _s->begin(ComSlave::BAUD_RATE);
  return true;
}

uint8_t ComSlave::handleNextByte() {
  _currDataBufIdx--;
  return _dataBuf[_currDataBufIdx] >> 2;
}

uint8_t ComSlave::shiftByte(uint8_t b) {
  return b << 2;
}

void ComSlave::sendPacket(uint8_t* buf, size_t len) {
  size_t packetSize = len + 2;

  uint8_t nBuf[packetSize];
  nBuf[0] = ComSlave::PACKET_BEGIN_CHAR;
  
  for(size_t i = 0; i < len; i++) {
    nBuf[i+1] = shiftByte(buf[i]);
  }

  nBuf[packetSize - 1] = ComSlave::PACKET_TERMINATION_CHAR;

  _s->write(nBuf, packetSize);
}

bool ComSlave::handlePacketAck() {
  uint8_t idByte = handleNextByte();

  return idByte == _id;
}

void ComSlave::sendPacketSyn() {
  uint8_t buf[] = { _id, ComSlave::MASTER_ID, ComSlave::PACKET_SYN };
  sendPacket(buf, 3);
}

void ComSlave::update() {
  if (!_synSent) {
    Serial.println("Sending SYN to master...");
    sendPacketSyn();
    _synSent = true;
    return;
  }

  if (_s->available()) {
    uint8_t c = _s->read();

    if (_foundBegin) {
      if (c == ComSlave::PACKET_TERMINATION_CHAR) {
        uint8_t bytesRead = _currDataBufIdx;

        uint8_t packetType = handleNextByte();
        uint8_t packetReceiver = handleNextByte();

        if (packetReceiver != _id) {
          size_t packetSize = bytesRead + 2;

          uint8_t nBuf[packetSize];
          nBuf[0] = ComSlave::PACKET_BEGIN_CHAR;
          memcpy(nBuf + 1, _dataBuf, bytesRead);
          nBuf[packetSize - 1] = ComSlave::PACKET_TERMINATION_CHAR;

          _s->write(nBuf, packetSize);
          goto reset;
        }

        uint8_t packetSender = handleNextByte();

        switch (packetType) {
          case ComSlave::PACKET_ACK:
            _ackRecv = packetSender == ComSlave::MASTER_ID;
            if(_ackRecv) {
              Serial.print("Received ACK from master (");
              Serial.print(packetSender);
              Serial.println("), greeting him...");

              uint8_t buf[] = {
                0x3, 0x9, _id, packetSender, ComSlave::PACKET_HELLO_WORLD
              };
              sendPacket(buf, 5);
            }
            break;
          case ComSlave::PACKET_DATA:
            Serial.println("data packet");
            uint8_t data = handleNextByte();
            digitalWrite(12, data != 0);
            break;
          default:
            Serial.print("Unknown packet type: ");
            Serial.print(packetType);
            Serial.println(", ignoring...");
            goto reset;
        }

        if (_currDataBufIdx != 0) {
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

    if (c == ComSlave::PACKET_BEGIN_CHAR) {
      _foundBegin = true;
    }
  }
}

SoftwareSerial s(RX_PIN, TX_PIN);

ComSlave com(&s, ID);

void setup() {
  Serial.begin(115200);

  pinMode(12, OUTPUT);

  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);


  if (!com.init()) {
    Serial.println("Failed to init ComSlave, exiting...");
    for (;;) {}
  }

  Serial.println("Init done, running...");
}

void loop() {
  com.update();
}