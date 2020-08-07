/*
 *Формат команд, принимаемых прогой: 
 * Флаг начала пакета - 2 байта, это всегда строка TS (54-53 или 0x36 0x35)
 * Код команды - 1 байт
 * Длина данных - 1 байт
 * Данные - 0-255 байт
 */

#include <Arduino.h>

#define SERIAL_PORT             Serial1
#define SERIAL_SPEED            9600
#define SERIAL_COMM_COUNT       16
#define SERIAL_COMM_BUFF_LENGTH 128
#define RESET_MILLIS            120
#define SEND_ACK                true
#define ACK_MESSAGE             "$&"

enum serMode:byte {
  SERIAL_OFF,
  INPUT_WAIT,
  ENABLING_WAIT,
  WAIT_COMMAND,
  WAIT_COMM_LENGTH,
  RECEIVE_PAYLOAD
};

enum command:byte {
    COMM_NONE,
    COMM_STATUS,
    COMM_MOTOR_GET_TICKS,
    COMM_MOTOR_SET_MOVEMENT,
    COMM_MOTOR_GET_SPEED,
    COMM_LED_SET,
    COMM_LED_SET_COLOR_ALL,
    COMM_FLASHLIGHT,
    COMM_RANGE_GET, 
    COMM_BATTERY_VOLTAGE_GET,
};

struct SerialCommandCallback {
   command commId = COMM_NONE;
   void (*function)();
};  

byte          serialReceiveMode           = SERIAL_OFF;
byte          currentCommand              = 0;
byte          commandPayloadLength        = 0;
byte          commandBufferCursorPosition = 0;
unsigned long lastPacketReceiveTime       = 0;
unsigned long waitStartTime               = 0;
byte commandBuffer[SERIAL_COMM_BUFF_LENGTH];

SerialCommandCallback serialCommandCallbacks[SERIAL_COMM_COUNT];
void (*comm_recv_basic)();

void ClearCommandBuffer() {
  commandBufferCursorPosition = 0;
  for (int i=0;i<SERIAL_COMM_BUFF_LENGTH;i++) {
    commandBuffer[i] = 0;
  }
}

void PrepareComm() {   
  ClearCommandBuffer(); 
  SERIAL_PORT.begin(SERIAL_SPEED);
  lastPacketReceiveTime = millis();  
  serialReceiveMode = INPUT_WAIT;
}

void ParseCommand() {
  if ( comm_recv_basic != NULL ) {
    comm_recv_basic();
  }
  for (byte i=0;i<SERIAL_COMM_COUNT;i++) {
    if ( serialCommandCallbacks[i].commId == currentCommand ) {
        serialCommandCallbacks[i].function();
        break;
    }
  }
  ClearCommandBuffer();
  currentCommand       = 0;
  commandPayloadLength = 0;
}

void RegSerialCommand(command id, void(*function)()) {
    for (byte i = 0; i < SERIAL_COMM_COUNT; i++) {
        auto curCommand = serialCommandCallbacks[i];
        if ( curCommand.commId == COMM_NONE ) {
            curCommand.commId = id;
            curCommand.function = function;
            serialCommandCallbacks[i] = curCommand;
            break;
        }
    }   
}

void CheckComm() {
  if (serialReceiveMode == SERIAL_OFF) {
    return;
  }

   if (SERIAL_PORT.available() > 0) {
      lastPacketReceiveTime = millis();
   } else {
    return;
   }
  
  if ( serialReceiveMode != INPUT_WAIT && lastPacketReceiveTime - waitStartTime > RESET_MILLIS) {
    serialReceiveMode = INPUT_WAIT;
    waitStartTime     = lastPacketReceiveTime;
    ClearCommandBuffer();
    return;
  }

  if (serialReceiveMode == INPUT_WAIT) {
      while (SERIAL_PORT.available() > 0) {
        char in = SERIAL_PORT.read();
        if (in == 54) {
          waitStartTime     = millis();
          serialReceiveMode = ENABLING_WAIT;
          return;
        }
      }
      return;
  }
  if (serialReceiveMode == ENABLING_WAIT) {
    char in = SERIAL_PORT.read();
    if (in == 53) {
      serialReceiveMode = WAIT_COMMAND;
      return;
    } else {
      serialReceiveMode = INPUT_WAIT;
      return;
    }
  }

  if (serialReceiveMode == WAIT_COMMAND) {
    currentCommand    = SERIAL_PORT.read();
    serialReceiveMode = WAIT_COMM_LENGTH;
    return;
  }
  
  if (serialReceiveMode == WAIT_COMM_LENGTH) {
    commandPayloadLength = SERIAL_PORT.read();
    serialReceiveMode    = RECEIVE_PAYLOAD;
    ClearCommandBuffer();
    return;
  }

  if (serialReceiveMode == RECEIVE_PAYLOAD) {
        while (SERIAL_PORT.available()) {
          commandBuffer[commandBufferCursorPosition] = SERIAL_PORT.read();
          commandBufferCursorPosition++;
        }
        if ( commandBufferCursorPosition == commandPayloadLength ) {
          commandBufferCursorPosition = 0;
          serialReceiveMode = INPUT_WAIT;
          ParseCommand();
          delay(2);
          SERIAL_PORT.println(ACK_MESSAGE);
          
        } else {
          return;
        }
  }
}
