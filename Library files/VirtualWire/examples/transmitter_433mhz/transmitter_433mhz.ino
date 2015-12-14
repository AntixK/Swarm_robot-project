// Transmitter

#include <VirtualWire.h>
#undef int
#undef abs
#undef double
#undef float
#undef round

void setup()
{
  Serial.begin(115200);	  // Debugging only
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_set_tx_pin(9);         // Set transmitter data pin number
  vw_setup(2000);	 // Bits per sec
}

void loop()
{
  //const char *msg = "Hello!";
  if (Serial.available())
  {
    char val = Serial.read();
    if (val == '\n')
    {
      char *msg = val;
    }
    else
    {
      char *msg = val + *msg;
    }
    vw_send((uint8_t *)msg, strlen(msg));
    vw_wait_tx(); // Wait until the whole message is gone
    Serial.println(msg);
  }
}
