/*
 Visual Counter V20190319-1.0

 To use, telnet to your device's IP address and type.
 You can see the client's input in the serial monitor as well.
 Using an Arduino Wiznet Ethernet shield.

 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13

 Cara kerja: Jika client (PC) mengirim permintaan berupa
   "N" -> Alat ini akan mengirim value dari 'entranceMunitionCntr'
   "X" -> Alat ini akan mengirim value dari 'exitMunitionCntr'
   "R" -> Semua counter di-reset
   
 created 19 Mar 2019
 by Samudra P. Buana
 */

#include <SPI.h>
#include <Ethernet.h>
#include <Arduino_FreeRTOS.h>
void TaskCountEntrance( void *pvParameters );
void TaskCountExit( void *pvParameters );

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.
// gateway and subnet are optional:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

byte byArray[8] = {0xAB,0xAC,0xCD,0x22,0x3F,0xA5,0xA8,0x7B};
char cArr[6] = "";

char receivedCmd[8];
String sReceived[8];
const int sensorEntrance = 2;
const int sensorExit = 3;
int entranceFlag;
int currentEntranceFlag;
int exitFlag, charCnt;
int currentExitFlag;
double entranceMunitionCntr = 0;
double exitMunitionCntr = 0;
double differenceMunitionCntr = 0;

IPAddress ip(192, 168, 0, 177);
IPAddress myDns(192,168,0, 1);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

// telnet defaults to port 23
EthernetServer server(10002);
boolean alreadyConnected = false; // whether or not the client was connected previously

void setup() {
  xTaskCreate(
    TaskCountEntrance
    ,  (const portCHAR *) "Count Entrance Munitions"
    ,  128 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  1  // priority
    ,  NULL );

  xTaskCreate(
    TaskCountExit
    ,  (const portCHAR *) "Count Exit Munitions"
    ,  128 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  2  // priority
    ,  NULL );

  // Read data terakhir dari SDCard
  
  pinMode(sensorEntrance, INPUT);
  pinMode(sensorExit, INPUT);
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  // initialize the ethernet device
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  // start listening for clients
  server.begin();
  // Open serial communications and wait for port to open:
  Serial.print("Chat server address:");
  Serial.println(Ethernet.localIP());
}

void loop() {
  // wait for a new client:
  EthernetClient client = server.available();

  // when the client sends the first byte, say hello:
  if (client) {
    if (!alreadyConnected) {
      // clear out the input buffer:
      client.flush();
      Serial.println("We have a new client");
      alreadyConnected = true;
    }

    if (client.available() > 0) {
      // read the bytes incoming from the client:
      char thisChar = client.read();
      
      // jumlah Command standar = 5-charakter
      // MASUK, KLUAR, dan RESET
      if((charCnt >= 5))  charCnt=0; //R 
      cArr[charCnt] = thisChar;
      
      Serial.print("  Terima =  ");
      Serial.print(cArr);
      Serial.println("");
      charCnt++;
    
      // jika cmd = "MASUK", maka ...
      if((cArr[0] == 'M') & (cArr[1] == 'A') &
      (cArr[2] == 'S') &(cArr[3] == 'U') &
      (cArr[4] == 'K')){ // "N"  
        client.println(entranceMunitionCntr);
      }
      else // jika cmd = "KLUAR", maka ...
      if((cArr[0] == 'K') & (cArr[1] == 'L') &
      (cArr[2] == 'U') &(cArr[3] == 'A') &
      (cArr[4] == 'R')){ // "N"  
        client.println(exitMunitionCntr);
        // echo the bytes to the server as well:
        //Serial.write(thisChar);
      }
      else // jika cmd = "RESET", maka ...
      if((cArr[0] == 'R') & (cArr[1] == 'E') &
      (cArr[2] == 'S') &(cArr[3] == 'E') &
      (cArr[4] == 'T')){ // "RESET"  
        // Reset all counters
        entranceMunitionCntr = 0;
        exitMunitionCntr = 0;
        differenceMunitionCntr = 0;
        client.println(differenceMunitionCntr);
        // echo the bytes to the server as well:
        //Serial.write(thisChar);
      }
    }
  }
}


void array_to_string(byte array[], unsigned int len, char buffer[])
{
  for(unsigned int i=0; i<len; i++)
  {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i*2+0] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
    buffer[i*2+1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
  }
  buffer[len*2]='\0';
}

void TaskCountEntrance(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    entranceFlag = digitalRead(sensorEntrance);
    if((entranceFlag == HIGH) & (currentEntranceFlag == LOW))
    { 
      entranceMunitionCntr++;
      Serial.print("Entrance Munition Cntr: ");
      Serial.println(entranceMunitionCntr);
      differenceMunitionCntr = entranceMunitionCntr - exitMunitionCntr;
      Serial.print("Diff: ");
      Serial.println(differenceMunitionCntr);
    }
    currentEntranceFlag = entranceFlag;  
    vTaskDelay( 50 / portTICK_PERIOD_MS );
  }
}

void TaskCountExit(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    exitFlag = digitalRead(sensorExit);
    if((exitFlag == HIGH) & (currentExitFlag == LOW))
    { 
      exitMunitionCntr++;
      Serial.print("Exit Munition Cntr: ");
      Serial.println(exitMunitionCntr);
      differenceMunitionCntr = entranceMunitionCntr - exitMunitionCntr;
      Serial.print("Diff: ");
      Serial.println(differenceMunitionCntr);
    }
    currentExitFlag = exitFlag;
     vTaskDelay( 50 / portTICK_PERIOD_MS );  
  }
}



