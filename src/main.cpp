#include <Arduino.h>

bool gotUM982 = false;
bool setUM982 = false;

HardwareSerial* SerialGPS = &Serial7;   //UM982 com1
const int32_t baudGPS = 460800;         //UM982 connection speed
constexpr int serial_buffer_size = 2048;
uint8_t GPSrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];    //Extra serial tx buffer

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
    SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);
    delay(10);
    SerialGPS->write("UNLOGALL\r\n");
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
  if ( gotUM982 )
  {
    SerialGPS->write("UNLOGALL\r\n"); // Free up the serial channel
    delay(100);
    SerialGPS->write("MODE\r\n"); // Request the operating mode
    delay(100);
    SerialGPS->write("CONFIG\r\n"); // Request the Configuration
    delay(200);

    while (SerialGPS->available())
    {
      char incoming[100];
      SerialGPS->readBytesUntil('\n', incoming, 100);

      //Check / set the "UM982 configured" flag.
      if ( strstr(incoming, "ANTENNADELTAHEN 0.0099 0.0099 0.0099") != NULL)
      {
        Serial.println("UM982 Already Configured");
        setUM982 = true;
      }
      else
      {
        SerialGPS->write("CONFIG ANTENNADELTAHEN 0.0099 0.0099 0.0099\r\n");
        delay(100);
      }
      
      if ( setUM982 ){break;} // Abort rest of checks if the UM982 is already configured.

      // Check / set UM982 operating mode
      if ( strstr(incoming, "MODE ROVER AUTOMOTIVE") != NULL)
      {
        Serial.println("MODE ROVER AUTOMOTIVE - Already Set");
      }
      else
      {
        SerialGPS->write("MODE ROVER AUTOMOTIVE\r\n");
        delay(100);
      }

      //Check / set heading
      if ( strstr(incoming, "CONFIG HEADING TRACTOR") != NULL)
      {
        Serial.println("CONFIG HEADING TRACTOR - Already Set");
      }
      else
      {
        SerialGPS->write("CONFIG HEADING TRACTOR\r\n");
        delay(100);
      }

      //Check / set smoothing the heading output
      if ( strstr(incoming, "CONFIG SMOOTH HEADING 10") != NULL)
      {
        Serial.println("CONFIG SMOOTH HEADING 10 - Already Set");
      }
      else
      {
        SerialGPS->write("CONFIG SMOOTH HEADING 10\r\n");
        delay(100);
      }

      //Check / set COM1 baud.
      if ( strstr(incoming, "CONFIG COM1 460800") != NULL)
      {
        Serial.println("CONFIG COM1 460800 - Already Set");
      }
      else
      {
        SerialGPS->write("CONFIG COM1 460800\r\n");
        delay(100);
      }

      //Check / set COM2 baud.
      if ( strstr(incoming, "CONFIG COM2 460800") != NULL)
      {
        Serial.println("CONFIG COM2 460800 - Already Set");
      }
      else
      {
        SerialGPS->write("CONFIG COM2 460800\r\n");
        delay(100);
      }

      //Check / set COM3 baud.
      if ( strstr(incoming, "CONFIG COM3 460800") != NULL)
      {
        Serial.println("CONFIG COM3 460800 - Already Set");
      }
      else
      {
        SerialGPS->write("CONFIG COM3 460800\r\n");
        delay(100);
      }

    }

    // Reset data output and save configuration.
    SerialGPS->write("GPGGA COM1 .1\r\n");
    delay(100);
    SerialGPS->write("GPVTG COM1 .1\r\n");
    delay(100);
    SerialGPS->write("GPHPR COM1 .1\r\n");
    delay(100);
    SerialGPS->write("SAVECONFIG\r\n");
    delay(100);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}