#include <BearSSLHelpers.h>
#include <CertStoreBearSSL.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiGratuitous.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiClientSecureAxTLS.h>
#include <WiFiClientSecureBearSSL.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiServerSecureAxTLS.h>
#include <WiFiServerSecureBearSSL.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>


#include <WiFiClient.h>
#include <WiFiServer.h>

#include <WiFiUdp.h>

#include <Wire.h>
#include <PubSubClient.h>
#include <WiFiManager.h> //To not have the hassle of manually hardcoding SSID and a password of wifi everytime

 
#define SDA_PIN 4                                     //GPIO Pin 4 as SDA in ESP 8266
#define SCL_PIN 5                                     //GPIO Pin 5 as Sclk in ESP 8266

#define TW_HOUSE_DEMO 0x20                            // Defining all the command bytes for the Atmega MCU, according to the Vienna Protocol document
#define TW_HOUSE_RGB 0x21
#define TW_HOUSE_R 0x22
#define TW_HOUSE_G 0x23
#define TW_HOUSE_B 0x24
#define TW_FERRIS_ENABLED 0x30
#define TW_METRO_DEMO 0x40
#define TW_METRO_1_DRIVE 0x41
#define TW_METRO_2_DRIVE 0x42

#define TW_DISABLE 0x00                              // Defining all the data bytes for the Atmega MCU, according to the Vienna Protocol document
#define TW_ENABLE 0x01
#define TW_INTO_STATION 0x00
#define TW_INTO_TUNNEL 0x01

const int16_t I2C_MASTER = 0x42;
const int16_t I2C_SLAVE = 0x08;



//Defining parameters of MQTT
const char* ssid = "TP-Link_B218";                     //Wifi SSid
const char* password = "47626881";                     //Wifi Password
const char* mqttServer = "192.168.0.102";              //IP address of Raspberry Pi which has my MQTT broker
const int mqttPort = 1883;
const char* mqttUser = "";                             //MQTT  USER id 
const char* mqttPassword = "";                         //MQTT Password




//Callback function called whenever I get any message from the MQTT broker to my client, i.e the ESP8266
        void callback( char* topic, byte* payload, unsigned int length)
                      {
                        Serial.print("Message arrived in topic: ");
                        Serial.println(topic);
                        Serial.print("Message:");
                        String message = "";
                      
                        for (int i = 0; i< length; i++)          //Printing character by character the message in the payload
                            {
                              Serial.print((char)payload[i]);
                              message+=(char)payload[i];
                            }
                        Serial.println();
                        Serial.println("--------------------");
                        if (String(topic) == "LED")
                        {
                                if (message=="LED BLINK")
                                {
                                    
                                    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
                                    delay(2000);                       // wait for a second
                                    digitalWrite(LED_BUILTIN, LOW);    // turn the LED on (HIGH is the voltage level)
                                    delay(5000);                       // wait for a second

                                    Serial.println("LED is blinking");
                                }
                                else
                                {
                                  Serial.println("To blink LED, MQTT messages should be 'LED BLINK' in topic LED"); //
                                  }

                        }
                         if (String(topic) == "HOUSE_DEMO")
                        {
                                if (message=="DISABLE")                      // MQTT Expected: topic - HOUSE_DEMO, message: DISABLE
                                {
                                     Wire.beginTransmission(111);            // transmit to device at address 111
                                     Wire.write(TW_HOUSE_DEMO);              // sends the corresponding hex value and its corresponding bytes
                                     Wire.write(TW_DISABLE);                 // sends data byte 
                                     Wire.endTransmission(); 
                                }
                                if (message=="ENABLE")                       // MQTT Expected: topic - HOUSE_DEMO, message: ENABLE
                                {
                                     Wire.beginTransmission(111);            // transmit to device at address 111
                                     Wire.write(TW_HOUSE_DEMO);              // sends the corresponding hex value and its corresponding bytes
                                     Wire.write(TW_ENABLE);                  // sends data byte  
                                     Wire.endTransmission();
                                     delay(2600);
                                }

                        }  

                     
                    if (String(topic) == "HOUSE")
                    {
                           if (message=="RGB")                                // MQTT Expected: topic - HOUSE, message: RGB
                            {
                                     Wire.beginTransmission(111);            // transmit to device at address 111
                                     Wire.write(TW_HOUSE_RGB);               // sends the corresponding hex value and its corresponding bytes
                                     Wire.write (0x00);                      // for red channel
                                     Wire.write (0xFF);                      // for green channel
                                     Wire.write (0x00);                      // for blue channel
                                     Wire.endTransmission(); 
                                    }
                        
                           if (message == "RED")                              // MQTT Expected: topic - HOUSE, message: RED
                            {
                                     Wire.beginTransmission(111);             // transmit to device at address 111
                                     Wire.write(TW_HOUSE_R);                  // sends the corresponding hex value and its corresponding bytes  
                                     Wire.write (0xFF);                       // for red channel
                                     Wire.endTransmission(); 
    
                            }
                        
                           if (message == "GREEN")                              // MQTT Expected: topic - HOUSE, message: GREEN
                            {
                                     Wire.beginTransmission(111);               // transmit to device at address 111
                                     Wire.write(TW_HOUSE_G);                    // sends the corresponding hex value and its corresponding bytes  
                                     Wire.write (0xFF);                         // for red channel
                                     Wire.endTransmission(); 
    
                            }
                         
                           if (message == "BLUE")                              // MQTT Expected: topic - HOUSE, message: BLUE
                            {
                                     Wire.beginTransmission(111);              // transmit to device at address 111
                                     Wire.write(TW_HOUSE_B);                   // sends the corresponding hex value and its corresponding bytes  
                                     Wire.write (0xFF);                        // for red channel
                                     Wire.endTransmission(); 
    
                            }
                    }
                        
                     if (String(topic) == "FERRIS")                        // MQTT Expected: topic - FERRIS_ENABLED
                        {
                          if (message=="ENABLE")                           //MQTT Expected:  topic - METRO_1_DRIVE, message: INTO_STATION
                                {
                                     Wire.beginTransmission(111);          // transmit to device at address 111
                                     Wire.write(TW_FERRIS_ENABLED);        // sends the corresponding hex value and its corresponding bytes  
                                     Wire.write (TW_ENABLE);               // corresponding data byte
                                     Wire.endTransmission();
                                }
                        }
                     if (String(topic) == "METRO_DEMO")                     // MQTT Expected: topic - METRO_DEMO
                        {
                          if (message=="ENABLE")                            //MQTT Expected:  topic - METRO_1_DRIVE, message: INTO_STATION
                                {
                                     Wire.beginTransmission(111);           // transmit to device at address 111
                                     Wire.write(TW_METRO_DEMO);              // sends the corresponding hex value and its corresponding bytes  
                                     Wire.write (TW_ENABLE);                      
                                     Wire.endTransmission();

                                }
                          }
   
                          
                      if (String(topic) == "METRO_1_DRIVE")
                        {
                                                  
                                if (message=="INTO_STATION")                       //MQTT Expected:  topic - METRO_1_DRIVE, message: INTO_STATION
                                {
                                     Wire.beginTransmission(111);                       // transmit to device at address 111
                                     Wire.write(TW_METRO_1_DRIVE);                      // sends the corresponding hex value and its corresponding bytes  
                                     Wire.write (TW_INTO_STATION);                      
                                     Wire.endTransmission();

                                }
                                 if (message=="INTO_TUNNEL")                       //MQTT Expected:  topic - METRO_1_DRIVE, message: INTO_TUNNEL
                                {
                                     Wire.beginTransmission(111);                 // transmit to device at address 111
                                     Wire.write(TW_METRO_1_DRIVE);                // sends the corresponding hex value and its corresponding bytes  
                                     Wire.write (TW_INTO_TUNNEL);                 
                                     Wire.endTransmission();

                                }
                         }

                       if (String(topic) == "METRO_2_DRIVE")
                        {
                                                  
                                if (message=="INTO_STATION")                       //MQTT Expected:  topic - METRO_2_DRIVE, message: INTO_STATION
                                {
                                     Wire.beginTransmission(111);                 // transmit to device at address 111
                                     Wire.write(TW_METRO_2_DRIVE);                // sends the corresponding hex value and its corresponding bytes  
                                     Wire.write (TW_INTO_STATION);                 // for red channel
                                     Wire.endTransmission();

                                }
                                 if (message=="INTO_TUNNEL")                        //MQTT Expected:  topic - METRO_2_DRIVE, message: INTO_TUNNEL
                                {
                                     Wire.beginTransmission(111);                 // transmit to device at address 111
                                     Wire.write(TW_METRO_2_DRIVE);                // sends the corresponding hex value and its corresponding bytes  
                                     Wire.write (TW_INTO_TUNNEL);                 
                                     Wire.endTransmission();

                                }
                          }

                          
          
                   }
                      




// Setting up MQTT
WiFiClient espClient;   
//PubSubClient client(espClient);                                       //client object of the PubSubclient library given access to mqtt attributes like mqttServer, mqttPort, callback, espClient 
PubSubClient client(mqttServer, mqttPort, callback, espClient);



void setup() { 
  // put your setup code here, to run once:

    Wire.begin( SDA_PIN, SCL_PIN);                            // join i2c bus with SDA=4 and SCL=5 of NodeMCU 
    
    Serial.begin(115200);                                    // Beginning of printing in the serial monitor, according to the baudrate of NodeMCU i.e. 115200
    WiFi.begin(ssid, password);
    pinMode(LED_BUILTIN, OUTPUT);
          while (WiFi.status() != WL_CONNECTED)              // Connection to Wifi status check
              {
               delay(500);
               Serial.println("Connecting to WiFI.."); 
              }
              Serial.println("Connected to WiFi network");

               client.setServer(mqttServer, mqttPort);      //Setup our MQTT server
               client.setCallback(callback);                //Here, I am calling the callback function, whenever receiving a message from MQTT 

            while (!client.connected())                     //Until Program is not connected to the MQTT server this will not execute further
            
            {
              Serial.println("Connecting to MQTT...");
    
                  if (client.connect("ESPClient8266", mqttUser, mqttPassword)) //to the appropriate topic, mqttUser and password )
                  {
                    Serial.println("Connected to MQTT");
                    client.subscribe("LED");
                  }
     
                  else {
                    Serial.print("failed with state");
                    Serial.print(client.state());
                    delay(2000);
                  }
            }

        //client.publish("esp8266", "Hello Raspberry ");
        // client.subscribe("esp8266");
        
}




void loop() 
{

//  // My MQTT client Code(as in my ESP 8266 is the client to the broker which is in the Raspberry PI)
      //client.publish("esp8266", "Hello From  client"); //My topic name and the message I want to send
      client.subscribe("esp8266");   // esp8266 being the topic subscribed to
      delay(1000);
      client.loop();
      
      
  //I2c connection code with Atmega begins here
    Wire.beginTransmission(111); // Address of my I2C slave i.e. the Atmega1284p 
    Wire.write("Connected to the Atmega board");
    Wire.endTransmission();

    delay(2500);



                 
    Wire.requestFrom(111,4); //since address of the Atmega(slave) is 111 and it expects commands of upto 4 bytes



  //  When I am receiving data or commands from my slave i.e. theAtmega board
   while(Wire.available())
      {
        byte a = Wire.read(); // byte size commands receiving from my slave
        Serial.println(a);
        }

 
}
