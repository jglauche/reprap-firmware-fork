/*
 * Class to handle internal communications in the machine via RS485
 *
 * Adrian Bowyer 3 July 2009
 *
 */
  
#if USE_EXTRUDER_CONTROLLER == true

#include "intercom.h"

intercom::intercom()
{
  pinMode(RX_ENABLE_PIN, OUTPUT);
  pinMode(TX_ENABLE_PIN, OUTPUT);
  digitalWrite(RX_ENABLE_PIN, 0); //always listen.
  Serial1.begin(38400);  
}

void intercom::sendPacket(byte address, char* string)
{
  digitalWrite(TX_ENABLE_PIN, 1);
  Serial1.print(address, HEX);
  Serial1.print(MASTER_ADDRESS);
  Serial1.println(string);
  digitalWrite(TX_ENABLE_PIN, 0);
  getPacket(myBuffer, IC_BUFFER);
  if(myBuffer[0] != MASTER_ADDRESS[0] ||  myBuffer[1] != MASTER_ADDRESS[1])
  {
     // Horrible error - what to do?
  } 
}

void intercom::sendPacketWithReply(byte address, char* string, char* reply)
{
  
}

void intercom::getPacket(char* string, int len)
{
  int i = -1;
  ok = true;
  do
  {
    while(!Serial1.available()) delay(1);
    i++;
    
    // Stop runaway buffer overflow
    
    if(i >= len)
      ok = false;
    else  
      string[i] = Serial1.read();
  } while(string[i] != '\n' && ok);
  string[i] = 0;
}

#endif
