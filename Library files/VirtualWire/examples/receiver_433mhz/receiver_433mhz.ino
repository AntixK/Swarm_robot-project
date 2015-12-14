// Receiver

#include <VirtualWire.h>
#undef int
#undef abs
#undef double
#undef float
#undef round

void setup()
{
  Serial.begin(115200);	// Debugging only
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_set_rx_pin(5);         // Set receiver data pin number
  vw_setup(2000);	 // Bits per sec
  vw_rx_start();       // Start the receiver PLL running
}

void loop()
{
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  if (vw_get_message(buf, &buflen))
  {
    /*for (byte i = 0; i < buflen; i++)
    {
      Serial.print((char)buf[i]);
    }*/
    char msg = (char)buf[0]+(char)buf[1];
    // Payload is just 2 digits because angle of transmitter 
    // from any corner point with reference to magnetic north 
    // forms a maximum angle of 90 degrees.
    if (msg==45)
    {
      Serial.println(msg);
    }
    else
    {
      msg="";
    }
  }
}
