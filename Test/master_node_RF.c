#include <SPI.h>
#include <RH_RF95.h>
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     2    // "D"
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define LED 13

void setup() 
{
    pinMode(LED, OUTPUT);     
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    while (!Serial);
    Serial.begin(9600);
    delay(100);

    Serial.println("Feather LoRa RX Test!");
  
    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init()) {
      Serial.println("LoRa radio init failed");
      while (1);
    }
    Serial.println("LoRa radio init OK!");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("setFrequency failed");
      while (1);
    }
    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);
}


void loop()
{
    if (rf95.available())
    {
        // Should be a message for us now   
        uint8_t buff[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buff);

        if (rf95.recv(buff, &len))
        {
            digitalWrite(LED, HIGH);
            char *buf = (char*)buff;
            
            //char *buf = "1/21/10,12/2/2019,25.23,70.1,0.123,-3.13,2.1475,S";
            //char *buf = "1/21/10,3/1/2019,110.32N,0.3E,2000.32,";
            Serial.println(buf);
            if(buf[strlen(buf)-1]!='S'){
            
            int index_table[10];
            int index = 0, count = 0; 
            for(int i=0 ; i<strlen(buf) ; i++){
                if(buf[i]==','){
                    index_table[index] = i;
                    index+=1;
                } 
            }

            char cur_time_converted[11];
            memset(cur_time_converted,0,9);
            char cur_time[index_table[0]+2];
            memcpy( cur_time, &buf[0], index_table[0] );  
            cur_time[sizeof(cur_time)/sizeof(char)-2] = '/';
            cur_time[sizeof(cur_time)/sizeof(char)-1] = '\0';
            index = 0;
            for(int i = 0 ; i<sizeof(cur_time)/sizeof(char) ; i++){
                if(cur_time[i]!='/'){
                    index++; 
                }
                else{
                    if(index==1){
                        char str1[2] = {cur_time[i-1],'\0'};
                        char str2[3] = "0";
                        strcat(str2,str1);
                        str2[2] = '\0';
                        strcat(cur_time_converted,str2);  
                    }else{
                        char str2[3];
                        memcpy( str2, &cur_time[i-2], 2 );
                        str2[2] = '\0';
                        strcat(cur_time_converted,str2);
                    } 
                    index = 0;
                }
            }
            cur_time_converted[8] = '\0';
            Serial.print("The current time is: ");Serial.println(cur_time_converted);


            //parsing date substring
             char cur_date[index_table[1]-index_table[0]], cur_date_converted[11];
            
            char year[5];
            cur_date[sizeof(cur_date)/sizeof(char)-1] = '\0';
            memset( cur_date, 0 , sizeof(cur_date)/sizeof(char)-1);
            memcpy( cur_date, &buf[index_table[0]+1],  index_table[1]-index_table[0]-3);
            memset( cur_date_converted,0,11);
            memset( year, 0, 5);
            memcpy( year, "20", 2);
            char var[2];
            memcpy(var,&buf[index_table[1]-2], 2);
            strcat( year, var);
            year[4] = '\0';
            memcpy(cur_date_converted,year,5);
            index = 0;
            for(int i = 0 ; i<sizeof(cur_date)/sizeof(char) ; i++){
                if(cur_date[i]!='/'){
                    index++; 
                }
                else{
                      if(index==1){
                      char str1[2] = {cur_date[i-1],'\0'};
                      char str2[3] = "0";
                      strcat(str2,str1);
                      str2[2] = '\0';
                      strcat(cur_date_converted,str2);  
               
                      }else{
                        char str2[3];
                        memcpy( str2, &cur_date[i-2], 2 );
                        str2[2] = '\0';
                        strcat(cur_date_converted,str2);
                      } 
                       index = 0;
                  }
            }
            cur_date_converted[10] = '\0';
            Serial.print("The date is: ");Serial.println(cur_date_converted);

            //parsing location
            char latitude[index_table[2]-index_table[1]-1], latitude_converted[index_table[2]-index_table[1]];
            memcpy(latitude, &buf[index_table[1]+1],index_table[2]-index_table[1]-2);
            latitude[sizeof(latitude)/sizeof(char)-1] = '\0';
            memset(latitude_converted,0 , sizeof(latitude_converted)/sizeof(char));
            if(buf[index_table[2]-1]=='S'){
                memcpy( latitude_converted, "-", 1);
            }
            strcat(latitude_converted, latitude);
            latitude_converted[sizeof(latitude_converted)/sizeof(char)] = '\0';

            char longitude[index_table[3]-index_table[2]-1], longitude_converted[index_table[3]-index_table[2]];
            memcpy(longitude, &buf[index_table[2]+1],index_table[3]-index_table[2]-2);
            longitude[sizeof(longitude)/sizeof(char)-1] = '\0';
            memset(longitude_converted,0 , sizeof(longitude_converted)/sizeof(char));
            if(buf[index_table[3]-1]=='W'){
                memcpy( longitude_converted, "-", 1);
            }
            strcat(longitude_converted, longitude);
            longitude_converted[sizeof(longitude_converted)/sizeof(char)] = '\0';
            Serial.print("The location is: ");Serial.print(latitude_converted);
            Serial.print(", ");Serial.println(longitude_converted);
  
            //parsing height
            char height[10];
            memset(height, 0, 10);
            memcpy(height, &buf[index_table[3]+1], index_table[4]-index_table[3]-1);
            height[sizeof(height)/sizeof(char)-1]='\0';
            Serial.print("The height is: ");Serial.println(height);
            }
            
            //packet G ends and S starts

            
            else{
            int index_table[10]; 

            int index = 0, count = 0; 
            for(int i=0 ; i<strlen(buf) ; i++){
                if(buf[i]==','){
                    index_table[index] = i;
                    index+=1;
                } 
            }

            char cur_time_converted[9];
            memset(cur_time_converted,0,9);
            char cur_time[index_table[0]+2];
            memcpy( cur_time, &buf[0], index_table[0] );  
            cur_time[sizeof(cur_time)/sizeof(char)-2] = '/';
            cur_time[sizeof(cur_time)/sizeof(char)-1] = '\0';
            index = 0;
            for(int i = 0 ; i<sizeof(cur_time)/sizeof(char) ; i++){
                if(cur_time[i]!='/'){
                    index++; 
                }
                else{
                    if(index==1){
                        char str1[2] = {cur_time[i-1],'\0'};
                        char str2[3] = "0";
                        strcat(str2,str1);
                        str2[2] = '\0';
                        strcat(cur_time_converted,str2);  
                    }else{
                        char str2[3];
                        memcpy( str2, &cur_time[i-2], 2 );
                        str2[2] = '\0';
                        strcat(cur_time_converted,str2);
                    } 
                    index = 0;
                }
            }
            cur_time_converted[8] = '\0';
            Serial.print("The current time is: ");Serial.println(cur_time_converted);


            //parsing date substring
            char cur_date[index_table[1]-index_table[0]], cur_date_converted[11];
            
            char year[5];
            cur_date[sizeof(cur_date)/sizeof(char)-1] = '\0';
            memset( cur_date, 0 , sizeof(cur_date)/sizeof(char)-1);
            memcpy( cur_date, &buf[index_table[0]+1],  index_table[1]-index_table[0]-3);
            memset( cur_date_converted,0,11);
            memset( year, 0, 5);
            memcpy( year, "20", 2);
            char var[2];
            memcpy(var,&buf[index_table[1]-2], 2);
            strcat( year, var);
            year[4] = '\0';
            memcpy(cur_date_converted,year,5);
            index = 0;
            for(int i = 0 ; i<sizeof(cur_date)/sizeof(char) ; i++){
                if(cur_date[i]!='/'){
                    index++; 
                }
                else{
                      if(index==1){
                      char str1[2] = {cur_date[i-1],'\0'};
                      char str2[3] = "0";
                      strcat(str2,str1);
                      str2[2] = '\0';
                      strcat(cur_date_converted,str2);  
               
                      }else{
                        char str2[3];
                        memcpy( str2, &cur_date[i-2], 2 );
                        str2[2] = '\0';
                        strcat(cur_date_converted,str2);
                      } 
                       index = 0;
                  }
            }
            cur_date_converted[10] = '\0';
            Serial.print("The date is: ");Serial.println(cur_date_converted);

                  
            //parsing temperautre
            char temperature[8];
            memset(temperature, 0, 8);
            memcpy(temperature, &buf[index_table[1]+1], index_table[2]-index_table[1]-1);
            temperature[sizeof(temperature)/sizeof(char)-1]='\0';
            Serial.print("The temperature is: ");
            Serial.print(temperature);
            Serial.println(" Celcius degree");

            //parsing humidity
            char humidity[8];
            memset(humidity, 0, 8);
            memcpy(humidity, &buf[index_table[2]+1], index_table[3]-index_table[2]-1);
            humidity[sizeof(humidity)/sizeof(char)-1]='\0';
            Serial.print("The Humidity is : ");
            Serial.print(humidity);
            Serial.println("%");

            //parsing axis x, y, z
            char axis_x[10];
            memset(axis_x, 0, 10);
            memcpy(axis_x, &buf[index_table[3]+1], index_table[4]-index_table[3]-1);
            axis_x[9]='\0';

            char axis_y[10];
            memset(axis_y, 0, 10);
            memcpy(axis_y, &buf[index_table[4]+1], index_table[5]-index_table[4]-1);
            axis_y[9]='\0';

            char axis_z[10];
            memset(axis_z, 0, 10);
            memcpy(axis_z, &buf[index_table[5]+1], index_table[6]-index_table[5]-1);
            axis_z[sizeof(axis_z)/sizeof(char)-1]='\0';
            Serial.print("the axis of the tilt is x: ");  Serial.print(axis_x);
            Serial.print(" y: ");Serial.print(axis_y);
            Serial.print(" z: ");Serial.println(axis_z);
            }
            
            Serial.println("Transmission done");
            Serial.println(" ");
            delay(1000);
            uint8_t data[] = "And hello back to you";
            rf95.send(data, sizeof(data));
            rf95.waitPacketSent();
            digitalWrite(LED, LOW);
            }
        
        else
        {
            Serial.println("Receive failed");
        }
    }
}

