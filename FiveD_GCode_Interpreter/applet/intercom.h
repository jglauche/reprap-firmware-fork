/*
 * Class to handle internal communications in the machine via RS485
 *
 * Adrian Bowyer 3 July 2009
 *
 */
 
#ifndef INTERCOM_H
#define INTERCOM_H

#if USE_EXTRUDER_CONTROLLER == true

#define IC_BUFFER 10
#define MASTER_ADDRESS "00"

//our RS485 pins
#define RX_ENABLE_PIN	13
#define TX_ENABLE_PIN	12

 
class intercom
{
  private:
    char myBuffer[IC_BUFFER];
    bool ok;
    void getPacket(char* string, int len);
    
  public:
    intercom();
    void sendPacket(byte address, char* string);
    void sendPacketWithReply(byte address, char* string, char* reply);

};

extern intercom talker;

#endif
#endif
