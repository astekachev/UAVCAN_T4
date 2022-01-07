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

 static int const NODE_ID  = 124;                    // 1 to 127
 static int const LED_BLINK_PIN = 2;                 // UAVCAN Detector
 static int const SILENT_CAN_PIN  = 21;              // S-pin TJA1051T/3,118
 static CanardPortID const BIT_PORT_ID  = 1620U;     //
 static byte const DEBUG = 0;                        // Debug for serial monitor

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void    can_normal                ();
void    can_silent                ();

void    onReceive                 (const CAN_message_t &);
void    onHeartbeat_1_0_Received  (CanardTransfer const &, ArduinoUAVCAN &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

ArduinoUAVCAN uc(NODE_ID, nullptr);

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
  Can1.onReceive(onReceive);

  /* Subscribe to the reception of Heartbeat message. */
  uc.subscribe<Heartbeat_1_0<>>(onHeartbeat_1_0_Received);

}

void loop()
{

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

 void onReceive(const CAN_message_t &msg)  // received a packet
 {
    CanardFrame const frame       /* create libcanard frame */
     {
       0,//msg.timestamp,             /* timestamp_usec  */
       msg.id,                    /* extended_can_id limited to 29 bit */
       msg.len,                   /* payload_size */
       (const void *) msg.buf     /* payload */
     };

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
     uc.onCanFrameReceived(frame);
}

void onHeartbeat_1_0_Received(CanardTransfer const & transfer, ArduinoUAVCAN & /* uc */)
{
  Heartbeat_1_0<> const hb = Heartbeat_1_0<>::deserialize(transfer);

  if (hb.data.mode.value)
  {
    char msg[64];
    snprintf(msg, 64,
           "ID %02X, Uptime = %lu, Health = %d, Mode = %d, VSSC = %d",
           transfer.remote_node_id, hb.data.uptime, hb.data.health.value, hb.data.mode.value, hb.data.vendor_specific_status_code);

    Serial.println(msg);
  } else {Serial.println("no data");}
}