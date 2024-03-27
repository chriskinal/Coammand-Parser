#include <Arduino.h>

bool gotUM982 = false;
bool setUM982 = false;

HardwareSerial* SerialGPS = &Serial7;   //UM982 com1
const int32_t baudGPS = 460800;         //UM982 connection speed
const int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];    //Extra serial tx buffer
const int tmp_serial_buffer_size = 2048;
uint8_t tmpGPSrxbuffer[tmp_serial_buffer_size];    //Extra serial rx buffer
uint8_t tmpGPStxbuffer[tmp_serial_buffer_size];    //Extra serial tx buffer

// Baudrates for detecting UM982 receiver
uint32_t baudrates[]
{
  460800,
  115200
};
const uint32_t nrBaudrates = sizeof(baudrates)/sizeof(baudrates[0]);

void setup() {
  delay(5000);
  
  //Check for UM982
  uint32_t baudrate = 0;
  for (uint32_t i = 0; i < nrBaudrates; i++)
  {
    baudrate = baudrates[i];
    Serial.print(F("Checking for UM982 at baudrate: "));
		Serial.println(baudrate);
    SerialGPS->begin(baudrate);
    // Increase the size of the serial buffer to hold longer UM982 config messages
    SerialGPS->addMemoryForRead(tmpGPSrxbuffer, tmp_serial_buffer_size);
    SerialGPS->addMemoryForWrite(tmpGPStxbuffer, tmp_serial_buffer_size);
    delay(100);
    SerialGPS->write("VERSION\r\n");
    delay(100);
    while (SerialGPS->available())
    {
      char incoming[100];
      SerialGPS->readBytesUntil('\n', incoming, 100);
      //Serial.println(incoming);
      if ( strstr(incoming, "UM982") != NULL)
      {
        if (baudrate != 460800)
          {
            Serial.println("UM982 baudrate wrong for AOG. Setting to 460800 bps for AOG");
            SerialGPS->write("CONFIG COM1 460800\r\n");
            delay(100);
            SerialGPS->begin(baudGPS);
          }
        gotUM982 = true;
        break;
      }
      if ( gotUM982 ) {break;}
    }
    if ( gotUM982 ) {break;}
  }

  // Check and configure UM982
  if ( gotUM982 )
  {
    SerialGPS->write("CONFIG\r\n"); // Request the UM982 Configuration
    delay(200);

    while ( SerialGPS->available() && !setUM982 )
    {
      char incoming[100];
      SerialGPS->readBytesUntil('\n', incoming, 100);
      //Serial.println(incoming);

      //Check the "UM982 configured" flag.
      if ( strstr(incoming, "CONFIG ANTENNADELTAHEN") != NULL)
      {
        Serial.println("Got the config line");
        if ( strstr(incoming, "ANTENNADELTAHEN 0.0099 0.0099 0.0099") != NULL)
        {
          Serial.println("And it is already configured");
          
          // Reset serial buffer size
          SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
          SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);
          setUM982 = true;
        }
        else
        {
          Serial.println("And it is not already configured");

          //Clear out the serial channel
          SerialGPS->write("UNLOGALL\r\n");
          while ( ( SerialGPS->available()))
          {
            SerialGPS->read();
          }
          // Set UM982 operating mode
          Serial.println("Setting mode");
          SerialGPS->write("MODE ROVER AUTOMOTIVE\r\n");
          delay(100);

          // Set heading to tractor
          Serial.println("Setting heading to tractor");
          SerialGPS->write("CONFIG HEADING TRACTOR\r\n");
          delay(100);

          // Set heading smoothing
          Serial.println("Setting heading smoothing");
          SerialGPS->write("CONFIG SMOOTH HEADING 10\r\n");
          delay(100);

          // Set COM1 to 460800
          Serial.println("Setting COM1 to 460800 bps");
          SerialGPS->write("CONFIG COM1 460800\r\n");
          delay(100);

          // Set COM2 to 460800
          Serial.println("Setting COM2 to 460800 bps");
          SerialGPS->write("CONFIG COM2 460800\r\n");
          delay(100);

          // Set COM3 to 460800
          Serial.println("Setting COM3 to 460800 bps");
          SerialGPS->write("CONFIG COM3 460800\r\n");
          delay(100);

          // Set GGA message and rate
          Serial.println("Setting GGA");
          SerialGPS->write("GPGGA COM1 0.1\r\n");
          delay(100);

          // Set VTG message and rate
          Serial.println("Setting VTG");
          SerialGPS->write("GPVTG COM1 0.1\r\n");
          delay(100);

          // Set HPR message and rate
          Serial.println("Setting HPR");
          SerialGPS->write("GPHPR COM1 0.1\r\n");
          delay(100);

          // Setting the flag to signal UM982 is configured for AOG
          Serial.println("Setting UM982 configured flag");
          SerialGPS->write("CONFIG ANTENNADELTAHEN 0.0099 0.0099 0.0099\r\n");
          delay(100);

          // Saving the configuration in the UM982
          Serial.println("Saving the configuration");
          SerialGPS->write("SAVECONFIG\r\n");
          delay(100);

          // Reset the serial buffer size
          SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
          SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

        }
      }
    }

  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("In the loop");
  delay(20);
}