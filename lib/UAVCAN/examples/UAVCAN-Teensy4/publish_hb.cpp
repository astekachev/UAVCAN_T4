 /**************************************************************************************
  * INCLUDE
  **************************************************************************************/

#include <Arduino.h>

#include "../lib/UAVCAN/src/ArduinoUAVCAN.h"
#include "../lib/FlexCAN_T4/FlexCAN_T4.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const NODE_ID  = 123;                    // 1 to 127
static int const LED_BLINK_PIN = 2;                 // UAVCAN Detector
static int const SILENT_CAN_PIN  = 21;              // S-pin TJA1051T/3,118
static CanardPortID const BIT_PORT_ID  = 1620U;     //
static byte const DEBUG = 0;                        // Debug for serial monitor

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

//CAN
void    can_normal       ();
void    can_silent       ();

//UAVCAN
bool    transmitCanFrame(CanardFrame const &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

ArduinoUAVCAN uc(NODE_ID, transmitCanFrame);
Heartbeat_1_0<> hb;

volatile bool blinkFlag = true;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(9600);

  /* Setup CAN access */
  can_normal();
  Can1.begin();
  Can1.setClock(CLK_60MHz);
  Can1.setBaudRate(1000000);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();

  /* Configure initial heartbeat */
  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::ADVISORY;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 7;
}

void loop()
{
  /* Update the heartbeat object */
  hb.data.uptime = millis() / 1000;
  hb = Heartbeat_1_0<>::Mode::MAINTENANCE;

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  unsigned long const now = millis();
  if(now - prev > 1000) {
    uc.publish(hb);
    prev = now;
    blinkFlag = !blinkFlag;
  }

  /* Transmit all enqeued CAN frames */
  while(uc.transmitCanFrame()) { }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void can_normal()
{
  pinMode (SILENT_CAN_PIN, OUTPUT);
  digitalWrite(SILENT_CAN_PIN, LOW);
}

void can_silent()
{
  pinMode (SILENT_CAN_PIN, OUTPUT);
  digitalWrite(SILENT_CAN_PIN, HIGH);
}

void led_blink()
{
  if (blinkFlag) {
  pinMode (LED_BLINK_PIN, OUTPUT);
  analogWrite(LED_BLINK_PIN, 1);
}
  else {
    pinMode (LED_BLINK_PIN, OUTPUT);
    digitalWrite(LED_BLINK_PIN, LOW);
  }
}

bool transmitCanFrame(CanardFrame const & frame)
{
  CAN_message_t msg;

  msg.id = frame.extended_can_id;
  msg.timestamp = frame.timestamp_usec;
  msg.flags.extended = true;
	std::copy(reinterpret_cast<uint8_t const *>(frame.payload), reinterpret_cast<uint8_t const *>(frame.payload) + static_cast<uint8_t const>(frame.payload_size), msg.buf);
  msg.len = static_cast<uint8_t const>(frame.payload_size);

  Can1.write(msg);
  led_blink();

  if (Serial && DEBUG) {
      //Frame decode
  Serial.print("  LEN: "); Serial.print(frame.payload_size);
  Serial.print(" TS: "); Serial.print(frame.timestamp_usec);
  Serial.print(" CAN_ID: "); Serial.print(frame.extended_can_id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < static_cast<uint8_t const>(frame.payload_size); i++ ) {
    Serial.print(reinterpret_cast<uint8_t const *>(frame.payload)[i], HEX); Serial.print(" ");
  } Serial.println();
}

  return true;
}
