/************************************************************************************
ESP32 - Cam based Video Doorbell (no sound) by Craig Hoffmann

Note this code was based on RTOS Multi-Client MJPEG Streaming From ESP32 by Anatoli Arkhipenko
https://www.hackster.io/anatoli-arkhipenko/multi-client-mjpeg-streaming-from-esp32-47768f
https://github.com/arkhipenko/esp32-mjpeg-multiclient-espcam-drivers

The code has been considerably restructured to minimise redundant frame bufferering, add mqtt,
add stream parameters in url, etc, specifically focussed on use as a doorbell camera with home assistant.

I found this to be the most robust code for this project and being multiclient allows stream recording
at the same time as multiple clients watching - within reason
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
COMPILE WITH ARDUINO SETTINGS BELOW: (Important)
===============================================
  Board: ESP32 Dev Module       <- Code has only been tested with the ESP32-cam module using this setting
  CPU Freq: 240
  Flash Freq: 80
  Flash mode: QIO
  Flash Size: 4Mb  Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)    <- This results in higher FPS
updated:
  Flash Size: Minimal SPIFFS (1.9MB App with OTA/190kB SPIFFS)    <- Changed to this when adding OTA
  PSRAM: Enabled
  Core Debug Level: None

**************************************************************************************
*/


#pragma GCC optimize ("-Ofast")

#include "UserConfig.h"      //  <--- This is the only file you need to adjust for personal setup

#include "src/sensor.h"
#include "src/esp_camera.h"
#include "src/ov2640.h"

//#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <WiFiManager.h>     // https://github.com/tzapu/WiFiManager 
#include <ArduinoJson.h>
#include <esp_wifi.h>
#include <Update.h>


// ****************************************************************
// Select camera model:
//    CAMERA_MODEL_AI_THINKER
//    CAMERA_MODEL_WROVER_KIT
//    CAMERA_MODEL_ESP_EYE
//    CAMERA_MODEL_M5STACK_PSRAM
//    CAMERA_MODEL_M5STACK_WIDE
// ****************************************************************

#define CAMERA_MODEL_AI_THINKER        // ONLY TESTED WITH THIS MODEL
#include "src/camera_pins.h"


// ****************************************************************
// General config options - get it working first before changing
// ****************************************************************

#define MAX_CLIENTS   4
#define FPS 12
#define FRAME_BUFFERS 40
#define JPEG_QUALITY 20

#define MQTT_CONN_RETRY_ms (10*1000)            // Only try to reconnect every 10 seconds
#define MQTT_PERIOD_mS (120*1000)               // Send MQTT status updates every minute
#define OTHER_PERIOD_mS (10*1000)               // Do Other stuff every 10 seconds
#define CAPTURE_PERIOD_mS (1000/FPS)            // Capture frames every 
#define VALID_DATA_CODE_ADDR 0                  // eeprom address used to store int value code
#define VALID_DATA_CODE ((int)12137)            // just a value used to flag if eeprom has been written before
#define SETUP_DATA_ADDR 4                       // Setup Data Structure starting address in flash/eeprom


// ****************************************************************
// The following pins are specific to the ESP32-cam board
// change if you use a different (untested) board 
// ****************************************************************

#define INPUT1_PIN 2      // This is GPIO2 pin - 47k pullup onboard - THIS ONE IS USED FOR WIFI CONFIG (CAN ALSO BE USED AS DOORBELL BUTTON)
#define INPUT2_PIN 15     // This is GPIO15 pin - 47k pullup onboard
#define INPUT3_PIN 13     // This is GPIO13 pin - 47k pullup onboard
#define OUTPUT1_PIN 12    // This is GPIO12 pin - no pullup onboard
#define OUTPUT2_PIN 14    // This is GPIO12 pin - no pullup onboard
#define ON_BOARD_LED 33   // This is GPIO33 pin - LED with 1k to 3.3V
#define CAMERA_FLASH 4    // This is GPIO4 pin - Bright white LED drive circuit


// ****************************************************************
// Global variables
// ****************************************************************

unsigned long PreviousMQTTMillis = 0;
unsigned long PreviousOtherMillis = 0;
unsigned long LastMQTTReconnectMillis = 0;
unsigned long PreviousCaptureMillis = 0;

char HostName[32]="";

char doorbell_ip_topic[MAX_TOPIC_STR_LEN+10];
char doorbell_input1_topic[MAX_TOPIC_STR_LEN+10];
char doorbell_input2_topic[MAX_TOPIC_STR_LEN+10];
char doorbell_input3_topic[MAX_TOPIC_STR_LEN+10];
char doorbell_setup_topic[MAX_TOPIC_STR_LEN+10];

uint8_t noActiveClients;              // number of active client streams


// ****************************************************************
// EEPROM data structure
// ****************************************************************

// The camera settings
struct EEPROMCameraSettingsStruct
{
  int brightness;
  int contrast;
  int saturation;
  int effect;
  int whitebal;
  int awb_gain;
  int wb_mode;
  int exposure_ctrl;
  int aec2;
  int ae_level;
  int aec_value;
  int gain_ctrl;
  int agc_gain;
  int gainceiling;
  int bpc;
  int wpc;
  int raw_gma;
  int lenc;
  int hmirror;
  int vflip;
  int dcw;
  int colorbar;
} CameraSettings = { 0,0,0,0,1,1,0,1,1,1,300,1,5,1,1,1,1,1,0,0,0,0 };

// The MQTT Settings
struct EEPROMDataStruct
{
  char MQTTHost[MAX_SERVER_STR_LEN+2];       // mqtt host ip address
  char MQTTUser[MAX_USER_STR_LEN+2];       // mqtt user name
  char MQTTPassword[MAX_PASSWORD_STR_LEN+2];    // mqtt password
  char MQTTTopic[MAX_TOPIC_STR_LEN+2];    // mqtt password
  int MQTTPort;       // mqtt port number
} SetupData = { "0.0.0.0", "user", "secret", "doorcam/", 1883 };


WebServer server(80);
WiFiClient mqttClient;
PubSubClient client(mqttClient);


// ****************************************************************
// RTOS Task Handle
// ****************************************************************

TaskHandle_t tCam;     // handles getting picture frames from the camera and storing them locally


//***************************************************************************
// camCB - RTOS thread to grab frames from camera - started by mjpegCB
//
//***************************************************************************

volatile uint32_t frameNumber;  // Current frame number
volatile uint32_t ifb = 0;  // index for internal frames ring buffer input
volatile camera_fb_t* fb[FRAME_BUFFERS];    //  Grab a frame from the camera and query its size
volatile uint32_t fb_num[FRAME_BUFFERS];


void camCB(void* pvParameters) 
{
  unsigned long LastTime;     
  float FrameTime;           

  TickType_t xLastWakeTime;    // Last time thread was active
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FPS);  // A running interval associated with currently desired frame rate

  uint32_t fbcount = 0;  // how many frames are in the buffer
  uint32_t nextifb = FRAME_BUFFERS;  // index for internal frames ring buffer input

  frameNumber = 0;
  ifb = 0;
  
  // Thread Loop
  xLastWakeTime = xTaskGetTickCount();
  PreviousCaptureMillis = millis();
  Serial.println("camCB started");
  Serial.println(xFrequency);


  for (;;) 
  {

    nextifb=ifb+1;
    if (nextifb >= FRAME_BUFFERS)
    {
      nextifb = 0;
    }

    fbcount++;
    if (fbcount>FRAME_BUFFERS)
    {
      esp_camera_fb_return((camera_fb_t*) fb[nextifb]);
      fbcount--;
      //taskYIELD();
    }

    fb[nextifb] = esp_camera_fb_get();
    frameNumber++;
    fb_num[nextifb] = frameNumber;
    ifb=nextifb;

    //FrameTime = millis() - LastTime;            // **** CRAIG - FOR TESTING ****
    //LastTime = millis();
    //Serial.print("ms: ");
    //Serial.println(FrameTime);
    //Serial.print("   FPS: ");
    //Serial.println(1000.0/(float)FrameTime);
    
    if ( noActiveClients == 0 ) 
    {
      //Serial.println("No clients connected - suspend cam capturing");
      //Serial.printf("mjpegCB: free heap           : %d\n", ESP.getFreeHeap());
      //Serial.printf("mjpegCB: min free heap)      : %d\n", ESP.getMinFreeHeap());
      //Serial.printf("mjpegCB: max alloc free heap : %d\n", ESP.getMaxAllocHeap());
      //Serial.printf("mjpegCB: tCam stack wtrmark  : %d\n", uxTaskGetStackHighWaterMark(tCam));
      //Serial.flush();
      //vTaskSuspend(NULL);  // passing NULL means "suspend yourself"
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);

  }
}


//***************************************************************************
// RTOS thread to handle mjpeg connection streams
//
//***************************************************************************


// ==== STREAMING ======================================================
// "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n" 
const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Cache-Control: no-store, no-cache\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);


struct streamInfo {
  uint32_t        frame;
  WiFiClient      client;
  TaskHandle_t    task;
  char            RequestedFPS;
  char            FramesBack;
};

// ==== Handle connection request from clients ===============================
void handleJPGSstream(void)
{
  if ( noActiveClients >= MAX_CLIENTS )
  {
    String message = "\n\n*** Too many clients connected - try again later ***\n";
    server.send(200, "text / plain", message);
   
    return;
  }

  int i=FPS;
  String ArgsString = "";
  if (server.arg("fps") != "")
  {
    ArgsString = server.arg("fps");     // Gets the value of the query parameter
    i = ArgsString.toInt();
    if ((i<1) || (i>FPS))               // if requested fps is out of range select minimum
    {
      i = FPS;   
    }
  }

  int priority = 0;
  ArgsString = "";
  if (server.arg("priority") != "")
  {
    ArgsString = server.arg("priority");   //Gets the value of the query parameter
    priority = ArgsString.toInt();
    if ((priority!=1))                     // if priority not 1 then set to 0
    {
      priority=0;   
    }
  }

  int FramesBack = 0;
  ArgsString = "";
  if (server.arg("back") != "")
  {
    ArgsString = server.arg("back");     //Gets the value of the query parameter
    FramesBack = ArgsString.toInt();
    if (FramesBack<0)                         // make sure frames back is within allowed range
    {
      FramesBack=0;   
    }
    else if (FramesBack>(FRAME_BUFFERS*2/3))  // make sure frames back is within allowed range
    {
      FramesBack=(FRAME_BUFFERS*2/3);   
    }
  }

  taskYIELD();

  streamInfo* info = new streamInfo;

  info->frame = frameNumber;
  info->client = server.client();
  info->RequestedFPS = i;
  info->FramesBack = FramesBack;


  noActiveClients++;
  Serial.print("New stream connection, total ");
  Serial.println(noActiveClients);


  //  Creating task to push the stream to all connected clients
  int rc = xTaskCreatePinnedToCore(
             streamCB,
             "strmCB",
             3 * 1024,             
             (void*) info,
             5,                //Priority
             &info->task,
             tskNO_AFFINITY);     
  if ( rc != pdPASS ) {
    Serial.printf("handleJPGSstream: error creating RTOS task. rc = %d\n", rc);
    Serial.printf("handleJPGSstream: free heap  : %d\n", ESP.getFreeHeap());
    delete info;
  }

  // Wake up streaming tasks, if they were previously suspended:
  if ( eTaskGetState( tCam ) == eSuspended )
  {
    Serial.println("Wake up camera task from suspended state");
    vTaskResume( tCam );
  }
}


int NextStreamID = 100;

// ==== Actually stream content to all connected clients ========================
void streamCB(void * pvParameters) 
{
  char buf[16];
  TickType_t xLastWakeTime;
  TickType_t xFrequency;
  unsigned long LastTime=0;     
  unsigned long StartTime=0;     
  float FrameTime=0.0;
  int FrameCount=0;
  uint32_t ofb=0;           // index for internal frames ring buffer output

  // For reporting info only
  if ((noActiveClients == 1) || (NextStreamID == 199))
  {
    NextStreamID = 100;
  }
  int ThisStreamID = NextStreamID;
  NextStreamID++;

  //ofb=(ifb-1)&FRAME_MASK;
  streamInfo* info = (streamInfo*) pvParameters;
  

  if ( info == NULL ) {
    Serial.println("streamCB: a NULL pointer passed");
  }
  Serial.println("Stream started");

  ofb = ifb;  
  info->frame = fb_num[ofb];

  //  Immediately send this client a header
  info->client.write(HEADER, hdrLen);
  info->client.write(BOUNDARY, bdrLen);


  taskYIELD();


  xLastWakeTime = xTaskGetTickCount();                     
  xFrequency = pdMS_TO_TICKS(1000 / (info->RequestedFPS));

  LastTime = millis();

  for (;;) 
  {
    //  Only bother to send anything if there is someone watching
    if ( info->client.connected() ) 
    {

      ofb = ifb;
      
//      if ( info->frame != fb_num[ofb]) 
      if(1==1)
      {


        // Take note of the latest cam frame number for next time round the loop
        info->frame = fb_num[ofb];

        
        // If requested go back in time and get older frames
        if (info->FramesBack > 0)
        {
          if (ofb < info->FramesBack)
          {
            ofb = FRAME_BUFFERS + ofb - info->FramesBack;
          }
          else
          {
            ofb = ofb - info->FramesBack;
          }
        }
      
        info->client.write(CTNTTYPE, cntLen);
        sprintf(buf, "%u\r\n\r\n", fb[ofb]->len);
        info->client.write(buf, strlen(buf));

        info->client.write((char*) fb[ofb]->buf, (fb[ofb]->len));                   // (size_t)fb[ofb]->len);

        // FrameTime = millis() - StartTime;
        // Serial.print(ThisStreamID);
        // Serial.print("ProcessTime  ms: ");
        // Serial.print(FrameTime);
        // Serial.print(" buflen: ");
        // Serial.println(fb[ofb]->len);

        info->client.write(BOUNDARY, bdrLen);

        // Information code block to report average FPS served over 10 frames per connection
/*        #define AVERAGE_OVER 10
        FrameCount++;
        if (FrameCount>=AVERAGE_OVER)
        {
          FrameCount=0;
          FrameTime = millis() - LastTime;
          LastTime = millis();
          Serial.print(ThisStreamID);
          Serial.print(" Av FPS: ");
          Serial.print(1000.0*AVERAGE_OVER/(float)FrameTime);
          long rssi = WiFi.RSSI();
          Serial.print("  RSSI:");
          Serial.println(rssi);
        }
*/

//***        taskYIELD();    
        if ((xTaskGetTickCount()-xLastWakeTime)>xFrequency)    // We have overrun an entire frame so give system a bit extra time to recover
        {
          //Serial.printf("e");
          xLastWakeTime = xTaskGetTickCount();                     
//***          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
          vTaskDelayUntil(&xLastWakeTime, (xFrequency*3/4));
        }
        else
        {
          vTaskDelayUntil(&xLastWakeTime, xFrequency);     
        }

     
      }
      else
      {
        taskYIELD();
        xLastWakeTime = xTaskGetTickCount();                     
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));

      }
    }
    else 
    {
      // client disconnected - clean up.
      noActiveClients--;
      Serial.print("Stream closed, ");
      Serial.println(noActiveClients);
      Serial.flush();
      info->client.flush();
      info->client.stop();
      delete info;
      info = NULL;
      vTaskDelete(NULL);
      taskYIELD();
    }
  }
}



const char JHEADER[] = "HTTP/1.1 200 OK\r\n" \
                       "Content-disposition: inline; filename=capture.jpg\r\n" \
                       "Content-type: image/jpeg\r\n\r\n";
const int jhdLen = strlen(JHEADER);

// ==== Serve up one JPEG frame =============================================
void handleJPG(void)
{
  WiFiClient client = server.client();
  uint32_t ofb=0;  // index for internal frames ring buffer output

  Serial.println("jpg request");

  
  if (!client.connected()) return;
//  if (noActiveClients>=1)   // If already streaming use the latest capured frame
  {
    ofb=ifb;
    client.write(JHEADER, jhdLen);
    client.write((char*) fb[ofb]->buf, (size_t)fb[ofb]->len);
  }
/*  else
  { 
    camera_fb_t* fb_new = esp_camera_fb_get();
    client.write(JHEADER, jhdLen);
    client.write((char*)fb_new->buf, fb_new->len);
    esp_camera_fb_return(fb_new);
  }*/
}


// ==== Handle invalid URL requests ============================================
void handleNotFound()
{
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text / plain", message);
}


// ****************************************************************
// Handle MQTT Setup Web Page Request
// ****************************************************************

void HandleMQTTSetupPath()
{
  char TempStr[20];
  server.sendContent("HTTP/1.1 200 OK\r\n");    // start the page header
  server.sendContent("Content-Type: text/html\r\n");
  server.sendContent("Connection: close\r\n");  // the connection will be closed after completion of the response
  server.sendContent("\r\n");                   // this separates header from content that follows
  server.sendContent("<!DOCTYPE HTML>");
  server.sendContent("<html>");
  server.sendContent("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><title>MQTT Setup</title></head>");
  server.sendContent("<body style=\"font-family:Verdana;text-align:center;min-width:340px;\"><h1>Doorbell MQTT Setup</h1>");
  server.sendContent("by Craig Hoffmann<p>");

  server.sendContent("<p><div>");
  server.sendContent("<p><form action=\"/mqttConfirmSave\" method=\"post\">");

  server.sendContent(String("<p>Server: <br><input type=\"text\" name=\"Server\" value=\"") + SetupData.MQTTHost + "\" size=\"" + String(MAX_SERVER_STR_LEN) + "\">");
  server.sendContent(String("<p>User: <br><input type=\"text\" name=\"User\" value=\"") + SetupData.MQTTUser + "\" size=\"" + String(MAX_USER_STR_LEN) + "\">");
  server.sendContent(String("<p>Password: <br><input type=\"password\" name=\"Password\" value=\"XXXXXX\" size=\"") + String(MAX_PASSWORD_STR_LEN) + "\">");
  //server.sendContent(String("<p>") + SetupData.MQTTPassword + "<p>");   // ** test code only **
  dtostrf(SetupData.MQTTPort, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Port: <br><input type=\"text\" name=\"Port\" value=\"") + TempStr + "\" min=\"0\" max=\"65535\">");
  server.sendContent(String("<p>Common Topic: <br><input type=\"text\" name=\"Topic\" value=\"") + SetupData.MQTTTopic + "\" size=\"" + String(MAX_TOPIC_STR_LEN) + "\">");

  server.sendContent("<p><input type=\"submit\" value=\"Update\">");
  server.sendContent("</form>");
  server.sendContent("</div>");
 
  server.sendContent("</body>");
  server.sendContent("</html>");
  server.sendContent("\r\n");
  server.client().stop(); // Stop is needed because we sent no content length
}


// ****************************************************************
// Handle MQTT Save Confirmation POST Request
// ****************************************************************

void HandleMQTTSaveConfirmation()
{
  int i;
  char TempStr[20];
  String ArgsString = "";

  Serial.println("Save Setup...");

  if (server.hasArg("Server"))
  {
    ArgsString = server.arg("Server");     //Gets the value of the query parameter
    i = ArgsString.length()+1;
    if (i < MAX_SERVER_STR_LEN)
    {
      ArgsString.toCharArray(SetupData.MQTTHost,i);
    }
  }

  if (server.hasArg("User"))
  {
    ArgsString = server.arg("User");     //Gets the value of the query parameter
    i = ArgsString.length()+1;
    if (i < MAX_USER_STR_LEN)
    {
      ArgsString.toCharArray(SetupData.MQTTUser,i);
    }
  }

  if (server.hasArg("Password"))
  {
    ArgsString = server.arg("Password");     //Gets the value of the query parameter
    i = ArgsString.length()+1;
    if ((i < MAX_PASSWORD_STR_LEN) && (ArgsString != "XXXXXX")) 
    {
      ArgsString.toCharArray(SetupData.MQTTPassword,i);
    }
  }


  if (server.hasArg("Port"))
  {
    if (server.arg("Port") != "")
    {
      ArgsString = server.arg("Port");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=1) && (i<=65535))
      {
        SetupData.MQTTPort = i;
      }
    }
  }

  if (server.hasArg("Topic"))
  {
    ArgsString = server.arg("Topic");     //Gets the value of the query parameter
    i = ArgsString.length()+1;
    if (i < MAX_USER_STR_LEN)
    {
      ArgsString.toCharArray(SetupData.MQTTTopic,i);
    }
  }

  Serial.println("EEPROM.put...");
  EEPROM.put(SETUP_DATA_ADDR + sizeof(CameraSettings)+1,SetupData);
  
  Serial.println("EEPROM.commit...");
  noInterrupts();
  EEPROM.commit();
  interrupts();

  Serial.println("EEPROM done...");

  server.sendContent("HTTP/1.1 200 OK\r\n");
  server.sendContent("Content-Type: text/html\r\n");
  server.sendContent("Connection: close\r\n");  // the connection will be closed after completion of the response
  server.sendContent("\r\n");                   // this separates header from content that follows
  server.sendContent("<!DOCTYPE HTML>");
  server.sendContent("<html>");
  server.sendContent("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><title>MQTT Setup Saved</title></head>");
  server.sendContent("<body style=\"font-family:Verdana;\"><center><h1>Save Complete</h1>");
  server.sendContent("<p>");

  server.sendContent(String("<p>Server: <br><input type=\"text\" name=\"Server\" value=\"") + SetupData.MQTTHost + "\" size=\"" + String(MAX_SERVER_STR_LEN) + "\" readonly>");
  server.sendContent(String("<p>User: <br><input type=\"text\" name=\"User\" value=\"") + SetupData.MQTTUser + "\" size=\"" + String(MAX_USER_STR_LEN) + "\" readonly>");
  server.sendContent(String("<p>Password: <br><input type=\"password\" name=\"Password\" value=\"XXXXXX\" size=\"") + String(MAX_PASSWORD_STR_LEN) + "\" readonly>");
  //server.sendContent(String("<p>") + SetupData.MQTTPassword + "<p>");   // ** test code only **
  dtostrf(SetupData.MQTTPort, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Port: <br><input type=\"text\" name=\"Port\" value=\"") + TempStr + "\" min=\"0\" max=\"65535\" readonly>");
  server.sendContent(String("<p>Topic prefix: <br><input type=\"text\" name=\"Topic\" value=\"") + SetupData.MQTTTopic + "\" size=\"" + String(MAX_TOPIC_STR_LEN) + "\" readonly>");
  server.sendContent("<p>RESETTING ESP32....<p>");

  //server.sendContent("<p><form><input type=\"button\" value=\"Return\" onclick=\"window.location.href='/'\"/></form>");
  server.sendContent("</center></body>");
  server.sendContent("</html>");
  server.sendContent("\r\n");
  server.client().stop(); // Stop is needed because we sent no content length

  Serial.println("Resetting!");
  delay(5000);
  ESP.restart();

}


// ****************************************************************
// Camera setup web page
// ****************************************************************

void HandleCameraSetup()
{
  int i;
  char TempStr[20];
  String ArgsString = "";

  sensor_t* s = esp_camera_sensor_get();

  if (server.hasArg("brightness"))
  {
    if (server.arg("brightness") != "")
    {
      ArgsString = server.arg("brightness");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=-2) && (i<=2))
      {
        CameraSettings.brightness = i;
        s->set_brightness(s, i); 
      }
    }
  }

  if (server.hasArg("contrast"))
  {
    if (server.arg("contrast") != "")
    {
      ArgsString = server.arg("contrast");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=-2) && (i<=2))
      {
        CameraSettings.contrast = i;
        s->set_contrast(s, i); 
      }
    }
  }

  if (server.hasArg("saturation"))
  {
    if (server.arg("saturation") != "")
    {
      ArgsString = server.arg("saturation");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=-2) && (i<=2))
      {
        CameraSettings.saturation = i;
        s->set_saturation(s, i); 
      }
    }
  }

  
  if (server.hasArg("effect"))
  {
    if (server.arg("effect") != "")
    {
      ArgsString = server.arg("effect");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=6))
      {
        CameraSettings.effect = i;
        s->set_special_effect(s, i); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
      }
    }
  }


  if (server.hasArg("whitebal"))
  {
    if (server.arg("whitebal") != "")
    {
      ArgsString = server.arg("whitebal");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.whitebal = i;
        s->set_whitebal(s, i); 
      }
    }
  }

  if (server.hasArg("awb_gain"))
  {
    if (server.arg("awb_gain") != "")
    {
      ArgsString = server.arg("awb_gain");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.awb_gain = i;
        s->set_awb_gain(s, i); 
      }
    }
  }

  if (server.hasArg("wb_mode"))
  {
    if (server.arg("wb_mode") != "")
    {
      ArgsString = server.arg("wb_mode");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=4))
      {
        CameraSettings.wb_mode = i;
        s->set_wb_mode(s, i); 
      }
    }
  }

  if (server.hasArg("exposure_ctrl"))
  {
    if (server.arg("exposure_ctrl") != "")
    {
      ArgsString = server.arg("exposure_ctrl");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.exposure_ctrl = i;
        s->set_exposure_ctrl(s, i); 
      }
    }
  }

  if (server.hasArg("aec2"))
  {
    if (server.arg("aec2") != "")
    {
      ArgsString = server.arg("aec2");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.aec2 = i;
        s->set_aec2(s, i); 
      }
    }
  }

  if (server.hasArg("ae_level"))
  {
    if (server.arg("ae_level") != "")
    {
      ArgsString = server.arg("ae_level");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=-2) && (i<=2))
      {
        CameraSettings.ae_level = i;
        s->set_ae_level(s, i); 
      }
    }
  }

  if (server.hasArg("aec_value"))
  {
    if (server.arg("aec_value") != "")
    {
      ArgsString = server.arg("aec_value");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1200))
      {
        CameraSettings.aec_value = i;
        s->set_aec_value(s, i); 
      }
    }
  }

  if (server.hasArg("gain_ctrl"))
  {
    if (server.arg("gain_ctrl") != "")
    {
      ArgsString = server.arg("gain_ctrl");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.gain_ctrl = i;
        s->set_gain_ctrl(s, i); 
      }
    }
  }

  if (server.hasArg("agc_gain"))
  {
    if (server.arg("agc_gain") != "")
    {
      ArgsString = server.arg("agc_gain");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=30))
      {
        CameraSettings.agc_gain = i;
        s->set_agc_gain(s, i); 
      }
    }
  }

  if (server.hasArg("gainceiling"))
  {
    if (server.arg("gainceiling") != "")
    {
      ArgsString = server.arg("gainceiling");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=30))
      {
        CameraSettings.gainceiling = i;
        s->set_gainceiling(s, (gainceiling_t)i); 
      }
    }
  }


  if (server.hasArg("bpc"))
  {
    if (server.arg("bpc") != "")
    {
      ArgsString = server.arg("bpc");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.bpc = i;
        s->set_bpc(s, i); 
      }
    }
  }

  if (server.hasArg("wpc"))
  {
    if (server.arg("wpc") != "")
    {
      ArgsString = server.arg("wpc");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.wpc = i;
        s->set_wpc(s, i); 
      }
    }
  }

  if (server.hasArg("raw_gma"))
  {
    if (server.arg("raw_gma") != "")
    {
      ArgsString = server.arg("raw_gma");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.raw_gma = i;
        s->set_raw_gma(s, i); 
      }
    }
  }

  if (server.hasArg("lenc"))
  {
    if (server.arg("lenc") != "")
    {
      ArgsString = server.arg("lenc");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.lenc = i;
        s->set_lenc(s, i); 
      }
    }
  }

  if (server.hasArg("hmirror"))
  {
    if (server.arg("hmirror") != "")
    {
      ArgsString = server.arg("hmirror");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.hmirror = i;
        s->set_hmirror(s, i); 
      }
    }
  }

  if (server.hasArg("vflip"))
  {
    if (server.arg("vflip") != "")
    {
      ArgsString = server.arg("vflip");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.vflip = i;
        s->set_vflip(s, i); 
      }
    }
  }

/**** effects the resolution - remove
  if (server.hasArg("dcw"))
  {
    if (server.arg("dcw") != "")
    {
      ArgsString = server.arg("dcw");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.dcw = i;
        s->set_dcw(s, i); 
      }
    }
  }

  if (server.hasArg("colorbar"))
  {
    if (server.arg("colorbar") != "")
    {
      ArgsString = server.arg("colorbar");     //Gets the value of the query parameter
      i = ArgsString.toInt();
      if ((i>=0) && (i<=1))
      {
        CameraSettings.colorbar = i;
        s->set_colorbar(s, i); 
      }
    }
  }
****/

  // Save settings to EEPROM - KEEP THIS LAST
  if (server.hasArg("savesettings"))
  {
    if (server.arg("savesettings") != "")
    {
      ArgsString = server.arg("savesettings");     //Gets the value of the query parameter
      if (ArgsString=="Yes")
      {
        Serial.println("EEPROM.put...");
        EEPROM.put(VALID_DATA_CODE_ADDR,VALID_DATA_CODE);
        EEPROM.put(SETUP_DATA_ADDR,CameraSettings);

        Serial.println("EEPROM.commit...");
        noInterrupts();
        EEPROM.commit();
        interrupts();

        Serial.println("EEPROM.done...");
      }
    }
  }

  server.sendContent("HTTP/1.1 200 OK\r\n");    // start the page header
  server.sendContent("Content-Type: text/html\r\n");
  server.sendContent("Connection: close\r\n");  // the connection will be closed after completion of the response
  server.sendContent("\r\n");                   // this separates header from content that follows
  server.sendContent("<!DOCTYPE HTML>");
  server.sendContent("<html>");
  server.sendContent("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><title>Camera Setup</title></head>");
  server.sendContent("<body style=\"font-family:Verdana;text-align:center;min-width:340px;\"><h1>Doorbell Camera Setup</h1>");
  server.sendContent("by Craig Hoffmann<p>");

  server.sendContent("<p><div>");
  server.sendContent("<p><form action=\"/camerasetup\" method=\"post\">");

  dtostrf(s->status.brightness, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Brightness (-2 to 2): <br><input type=\"text\" name=\"brightness\" value=\"") + TempStr + "\" min=\"-2\" max=\"2\">");
  dtostrf(s->status.contrast, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Contrast (-2 to 2): <br><input type=\"text\" name=\"contrast\" value=\"") + TempStr + "\" min=\"-2\" max=\"2\">");
  dtostrf(s->status.saturation, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Saturation (-2 to 2): <br><input type=\"text\" name=\"saturation\" value=\"") + TempStr + "\" min=\"-2\" max=\"2\">");
  dtostrf(s->status.special_effect, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Effect (0 to 6): <br><input type=\"text\" name=\"effect\" value=\"") + TempStr + "\" min=\"0\" max=\"6\">");
  dtostrf(s->status.awb, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Auto White balance (0 or 1): <br><input type=\"text\" name=\"whitebal\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
  dtostrf(s->status.awb_gain, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Auto White Bal Gain (0 or 1): <br><input type=\"text\" name=\"awb_gain\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
  dtostrf(s->status.wb_mode, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>White Bal Mode (0 to 4): <br><input type=\"text\" name=\"wb_mode\" value=\"") + TempStr + "\" min=\"0\" max=\"4\">");
  dtostrf(s->status.aec, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Auto Exposure Control (0 or 1): <br><input type=\"text\" name=\"exposure_ctrl\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
  dtostrf(s->status.aec2, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>AEC2 DSP (0 or 1): <br><input type=\"text\" name=\"aec2\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
  dtostrf(s->status.ae_level, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>AE Level (-2 to 2): <br><input type=\"text\" name=\"ae_level\" value=\"") + TempStr + "\" min=\"-2\" max=\"2\">");
  dtostrf(s->status.aec_value, -8, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>AEC Value (0 to 1200): <br><input type=\"text\" name=\"aec_value\" value=\"") + TempStr + "\" min=\"0\" max=\"1200\">");
  dtostrf(s->status.agc, -4, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Auto Gain Control (0 or 1): <br><input type=\"text\" name=\"gain_ctrl\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
  dtostrf(s->status.agc_gain, -6, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>AGC Gain (0 to 30): <br><input type=\"text\" name=\"agc_gain\" value=\"") + TempStr + "\" min=\"0\" max=\"30\">");
  dtostrf((int)s->status.gainceiling, -6, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Gain Ceiling (0 to 6): <br><input type=\"text\" name=\"gainceiling\" value=\"") + TempStr + "\" min=\"0\" max=\"6\">");

  dtostrf(s->status.bpc, -6, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Black pixel correction (0 or 1): <br><input type=\"text\" name=\"bpc\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
  dtostrf(s->status.wpc, -6, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>White pixel correction (0 or 1): <br><input type=\"text\" name=\"wpc\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
  dtostrf(s->status.raw_gma, -6, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Raw gma (0 or 1): <br><input type=\"text\" name=\"raw_gma\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
  dtostrf(s->status.lenc, -6, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Lens correction (0 or 1): <br><input type=\"text\" name=\"lenc\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
  dtostrf(s->status.hmirror, -6, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Horiz mirror (0 or 1): <br><input type=\"text\" name=\"hmirror\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
  dtostrf(s->status.vflip, -6, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Vert flip (0 or 1): <br><input type=\"text\" name=\"vflip\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
/**** effects the resolution - remove
  dtostrf(s->status.dcw, -6, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>DCW (0 or 1): <br><input type=\"text\" name=\"dcw\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
  dtostrf(s->status.colorbar, -6, 0, TempStr);  // negative width means left justify
  server.sendContent(String("<p>Color bars (0 or 1): <br><input type=\"text\" name=\"colorbar\" value=\"") + TempStr + "\" min=\"0\" max=\"1\">");
****/
  server.sendContent("<p><input type=\"checkbox\" name=\"savesettings\" value=\"Yes\"> Save to EEPROM");

  server.sendContent("<p><input type=\"submit\" value=\"Update Camera Settings\">");
  server.sendContent("</form>");
  server.sendContent("</div>");
 
  server.sendContent("</body>");
  server.sendContent("</html>");
  server.sendContent("\r\n");
  server.client().stop(); // Stop is needed because we sent no content length

}


// ****************************************************************
// Over The Air update webpage
// ****************************************************************

/* Server Index Page */
String OtaUpdatePage = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";



// ****************************************************************
// Process incoming MQTT
// ****************************************************************

void MQTTcallback(char* topic, byte* payload, unsigned int length)
{
  StaticJsonDocument<500> doc;
  int tempInt = 0;

  Serial.println("Received mqtt");

  //Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, payload);
  // Test if parsing succeeds.
  if (error) 
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  sensor_t* s = esp_camera_sensor_get();
  
  JsonVariant brightness = doc["brightness"];
  if (!brightness.isNull())
  {
    tempInt = (int)brightness;
    if ((tempInt<-2) || (tempInt>2)) 
    {
      tempInt=0; 
    }
    CameraSettings.brightness = tempInt;
    s->set_brightness(s, tempInt);     // -2 to 2
  }

  JsonVariant contrast = doc["contrast"];
  if (!contrast.isNull())
  {
    tempInt = (int)contrast;
    if ((tempInt<-2) || (tempInt>2)) 
    {
      tempInt=0; 
    }
    CameraSettings.contrast = tempInt;
    s->set_contrast(s, tempInt);     // -2 to 2
  }

  JsonVariant saturation = doc["saturation"];
  if (!saturation.isNull())
  {
    tempInt = (int)saturation;
    if ((tempInt<-2) || (tempInt>2)) 
    {
      tempInt=0; 
    }
    CameraSettings.saturation = tempInt;
    s->set_saturation(s, tempInt);     // -2 to 2
  }

  JsonVariant effect = doc["effect"];
  if (!effect.isNull())
  {
    tempInt = (int)effect;
    if ((tempInt<0) || (tempInt>6)) 
    {
      tempInt=0; 
    }
    CameraSettings.effect = tempInt;
    s->set_special_effect(s, tempInt); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  }

  JsonVariant whitebal = doc["whitebal"];
  if (!whitebal.isNull())
  {
    tempInt = (int)whitebal;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.whitebal = tempInt;
    s->set_whitebal(s, tempInt);     // 0 = disable , 1 = enable
  }

  JsonVariant awb_gain = doc["awb_gain"];
  if (!awb_gain.isNull())
  {
    tempInt = (int)awb_gain;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.awb_gain = tempInt;
    s->set_awb_gain(s, tempInt);     // 0 = disable , 1 = enable
  }

  JsonVariant wb_mode = doc["wb_mode"];
  if (!wb_mode.isNull())
  {
    tempInt = (int)wb_mode;
    if ((tempInt<0) || (tempInt>4)) 
    {
      tempInt=0; 
    }
    CameraSettings.wb_mode = tempInt;
    s->set_wb_mode(s, tempInt);     // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  }

  JsonVariant exposure_ctrl = doc["exposure_ctrl"];
  if (!exposure_ctrl.isNull())
  {
    tempInt = (int)exposure_ctrl;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.exposure_ctrl = tempInt;
    s->set_exposure_ctrl(s, tempInt);     // 0 = disable , 1 = enable
  }

  JsonVariant aec2 = doc["aec2"];
  if (!aec2.isNull())
  {
    tempInt = (int)aec2;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.aec2 = tempInt;
    s->set_aec2(s, tempInt);     // 0 = disable , 1 = enable
  }

  JsonVariant ae_level = doc["ae_level"];
  if (!ae_level.isNull())
  {
    tempInt = (int)ae_level;
    if ((tempInt<-2) || (tempInt>2)) 
    {
      tempInt=0; 
    }
    CameraSettings.ae_level = tempInt;
    s->set_ae_level(s, tempInt);     // -2 to 2
  }

  JsonVariant aec_value = doc["aec_value"];
  if (!aec_value.isNull())
  {
    tempInt = (int)aec_value;
    if ((tempInt<0) || (tempInt>1200)) 
    {
      tempInt=300; 
    }
    CameraSettings.aec_value = tempInt;
    s->set_aec_value(s, tempInt);     // 0 to 1200 
  }

  JsonVariant gain_ctrl = doc["gain_ctrl"];
  if (!gain_ctrl.isNull())
  {
    tempInt = (int)gain_ctrl;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.gain_ctrl = tempInt;
    s->set_gain_ctrl(s, tempInt);     // 0 = disable , 1 = enable
  }

  JsonVariant agc_gain = doc["agc_gain"];
  if (!agc_gain.isNull())
  {
    tempInt = (int)agc_gain;
    if ((tempInt<0) || (tempInt>30)) 
    {
      tempInt=5; 
    }
    CameraSettings.agc_gain = tempInt;
    s->set_agc_gain(s, tempInt);     // 0 to 30 (0)  Higher number brighter image
  }

  JsonVariant gainceiling = doc["gainceiling"];
  if (!gainceiling.isNull())
  {
    tempInt = (int)gainceiling;
    if ((tempInt<0) || (tempInt>6)) 
    {
      tempInt=1; 
    }
    CameraSettings.gainceiling = tempInt;
    s->set_gainceiling(s, (gainceiling_t)tempInt);     // 0 to 6  3 for low light
  }

  JsonVariant bpc = doc["bpc"];
  if (!bpc.isNull())
  {
    tempInt = (int)bpc;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.bpc = tempInt;
    s->set_bpc(s, tempInt);     
  }

  JsonVariant wpc = doc["wpc"];
  if (!wpc.isNull())
  {
    tempInt = (int)wpc;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.wpc = tempInt;
    s->set_wpc(s, tempInt);     
  }

  JsonVariant raw_gma = doc["raw_gma"];
  if (!raw_gma.isNull())
  {
    tempInt = (int)raw_gma;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.raw_gma = tempInt;
    s->set_raw_gma(s, tempInt);     
  }

  JsonVariant lenc = doc["lenc"];
  if (!lenc.isNull())
  {
    tempInt = (int)lenc;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.lenc = tempInt;
    s->set_lenc(s, tempInt);     
  }

  JsonVariant hmirror = doc["hmirror"];
  if (!hmirror.isNull())
  {
    tempInt = (int)hmirror;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.hmirror = tempInt;
    s->set_hmirror(s, tempInt);     
  }

  JsonVariant vflip = doc["vflip"];
  if (!vflip.isNull())
  {
    tempInt = (int)vflip;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.vflip = tempInt;
    s->set_vflip(s, tempInt);     
  }

/**** Effects the resolution - remove
  JsonVariant dcw = doc["dcw"];
  if (!dcw.isNull())
  {
    tempInt = (int)dcw;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.dcw = tempInt;
    s->set_dcw(s, tempInt);     
  }

  JsonVariant colorbar = doc["colorbar"];
  if (!colorbar.isNull())
  {
    tempInt = (int)colorbar;
    if ((tempInt<0) || (tempInt>1)) 
    {
      tempInt=0; 
    }
    CameraSettings.colorbar = tempInt;
    s->set_colorbar(s, tempInt);     
  }
****/


  // Save Settings if requested - KEEP THIS ONE LAST
  JsonVariant SaveToEEPROM = doc["savesettings"];
  if (!SaveToEEPROM.isNull())
  {
    tempInt = (int)SaveToEEPROM;
    if (tempInt==1) 
    {
      Serial.println("EEPROM.put...");
      EEPROM.put(VALID_DATA_CODE_ADDR,VALID_DATA_CODE);
      EEPROM.put(SETUP_DATA_ADDR,CameraSettings);

      Serial.println("EEPROM.commit...");
      noInterrupts();
      EEPROM.commit();
      interrupts();

      Serial.println("EEPROM.done...");
    }
  }


  //s->set_brightness(s, -2);     // -2 to 2
  //s->set_contrast(s, 0);       // -2 to 2
  //s->set_saturation(s, 0);     // -2 to 2
  //s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  //s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  //s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  //s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  //s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  //s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  //s->set_ae_level(s, 2);       // -2 to 2 (0)
  //s->set_aec_value(s, 300);    // 0 to 1200 (300)
  //s->set_gain_ctrl(s, 0);      // 0 = disable , 1 = enable (1)
  //s->set_agc_gain(s, 5);       // 0 to 30 (0)  Higher number brighter image
  //s->set_gainceiling(s, (gainceiling_t)1);  // 0 to 6  3 for low light
  //s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  //s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  //s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  //s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  //s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  //s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  //s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  //s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

}


// ****************************************************************
// Get the "non factory" part of the ESP32 MAC address
// ****************************************************************

#include "esp_system.h"
String getMacAddressPart(void) 
{
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  //sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  sprintf(baseMacChr, "%02X%02X%02X", baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}


// ****************************************************************
// Reconnect MQTT if necessary
// ****************************************************************

void ReconnectMQTT() 
{
  
  // Only attempt to reconnect every MQTT_CONN_RETRY_ms
  if ((millis() - LastMQTTReconnectMillis) >= MQTT_CONN_RETRY_ms)
  {
    if (SetupData.MQTTHost[0] == '0')   // If IP address begins with 0 then ignore setup of MQTT
    {
      Serial.println("Z");  // Just for debugging  Z - configutre mqtt ip address
      LastMQTTReconnectMillis = millis();
    }
    else
    {
      Serial.println("X");  // Just for debugging  X - attempt to connect mqtt
      //Serial.println(SetupData.MQTTHost);
      //Serial.println(SetupData.MQTTUser);
      //Serial.println(SetupData.MQTTPort);
      // Attempt to connect
      if (client.connect(HostName, SetupData.MQTTUser, SetupData.MQTTPassword)) 
      {
        Serial.println("Y");    // Just for debugging  Y - mqtt connection successful
        Serial.println(HostName);
        Serial.println(SetupData.MQTTHost);
        Serial.println(SetupData.MQTTUser);
        Serial.println(SetupData.MQTTPort);
        client.subscribe(doorbell_setup_topic);  // Success connecting so subscribe to topic

        // List the mqtt topics
        Serial.println(SetupData.MQTTTopic);
        Serial.println(doorbell_setup_topic);
        Serial.println(doorbell_input1_topic);
        Serial.println(doorbell_input2_topic);
        Serial.println(doorbell_input3_topic);
      } 
      else 
      {
        // Failed - Reset reconnect timeout so wait a while before retrying
        LastMQTTReconnectMillis = millis();
      }
    }
  }
}


// ****************************************************************
// Setup
// ****************************************************************

void setup()
{
  int i;
  //#include "soc/soc.h"           // Disable brownout problems
  //#include "soc/rtc_cntl_reg.h"  // Disable brownout problems  
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  // Setup Serial connection:
  Serial.begin(115200);
  delay(1000); // wait for a second
  Serial.printf("setup: free heap  : %d\n", ESP.getFreeHeap());
  Serial.printf("setup: PSRAM size  : %d\n", ESP.getPsramSize());
  Serial.printf("setup: PSRAM free  : %d\n", ESP.getFreePsram());

  sprintf(HostName, "%s%s", MY_HOSTNAME,getMacAddressPart());    // Append lower bits of chipid to host name as hex num

  // Load the doorbell user settings from EEPROM/Flash
  EEPROM.begin(512);
  EEPROM.get(VALID_DATA_CODE_ADDR,i);
  if (i==VALID_DATA_CODE)  // Does it look like data has been written to the eeprom before??
  {
    EEPROM.get(SETUP_DATA_ADDR,CameraSettings);
    EEPROM.get(SETUP_DATA_ADDR + sizeof(CameraSettings)+1,SetupData);
  }
  else
  {
    EEPROM.put(VALID_DATA_CODE_ADDR,VALID_DATA_CODE);
    EEPROM.put(SETUP_DATA_ADDR,CameraSettings);
    EEPROM.put(SETUP_DATA_ADDR + sizeof(CameraSettings)+1,SetupData);

    noInterrupts();
    EEPROM.commit();
    interrupts();
  }
  strcpy(doorbell_ip_topic, SetupData.MQTTTopic);
  strcat(doorbell_ip_topic, "ip/");
  strcpy(doorbell_input1_topic, SetupData.MQTTTopic);
  strcat(doorbell_input1_topic, "input1/");
  strcpy(doorbell_input2_topic, SetupData.MQTTTopic);
  strcat(doorbell_input2_topic, "input2/");
  strcpy(doorbell_input3_topic, SetupData.MQTTTopic);
  strcat(doorbell_input3_topic, "input3/");
  strcpy(doorbell_setup_topic, SetupData.MQTTTopic);
  strcat(doorbell_setup_topic, "setup");

  pinMode(INPUT1_PIN, INPUT_PULLUP);   // Use this one to reset wifi settings
  pinMode(INPUT2_PIN, INPUT_PULLUP);
  pinMode(INPUT3_PIN, INPUT_PULLUP);
  
  pinMode(OUTPUT1_PIN, OUTPUT);
  pinMode(OUTPUT2_PIN, OUTPUT);

  digitalWrite(OUTPUT1_PIN, LOW);
  digitalWrite(OUTPUT2_PIN, LOW);

  static camera_config_t camera_config = {
    .pin_pwdn       = PWDN_GPIO_NUM,
    .pin_reset      = RESET_GPIO_NUM,
    .pin_xclk       = XCLK_GPIO_NUM,
    .pin_sscb_sda   = SIOD_GPIO_NUM,
    .pin_sscb_scl   = SIOC_GPIO_NUM,
    .pin_d7         = Y9_GPIO_NUM,
    .pin_d6         = Y8_GPIO_NUM,
    .pin_d5         = Y7_GPIO_NUM,
    .pin_d4         = Y6_GPIO_NUM,
    .pin_d3         = Y5_GPIO_NUM,
    .pin_d2         = Y4_GPIO_NUM,
    .pin_d1         = Y3_GPIO_NUM,
    .pin_d0         = Y2_GPIO_NUM,
    .pin_vsync      = VSYNC_GPIO_NUM,
    .pin_href       = HREF_GPIO_NUM,
    .pin_pclk       = PCLK_GPIO_NUM,
//    .xclk_freq_hz     = 18500000,
    .xclk_freq_hz     = 19700000,
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,
    .pixel_format   = PIXFORMAT_JPEG,        // was PIXFORMAT_JPEG
    .frame_size     = FRAMESIZE_XGA,         // was XGA
    .jpeg_quality   = JPEG_QUALITY,
    .fb_count       = FRAME_BUFFERS+1
  };

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

#if defined(CAMERA_FLASH)
  pinMode(CAMERA_FLASH,OUTPUT);
  digitalWrite(CAMERA_FLASH,0);
#endif



  if (esp_camera_init(&camera_config) != ESP_OK) {
    Serial.println("Error initializing the camera - Restart");
    //delay(10000);
    ESP.restart();
  }

  //  Configure and connect to WiFi

  
  // Make sure AP mode not active
  WiFi.softAPdisconnect();
  delay(10);

  // WiFiManager for configuring/managing the wifi settings
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);     // 3min (180second) timeout for Access Point configuration
  wifiManager.setDebugOutput(false);   // Note WiFiManager prints password on serial if debug enabled!! 

  // Check if wifi config needs to be reset - forcing Access Point Config portal
  if (digitalRead(INPUT1_PIN) == LOW)
  {
    Serial.println("INPUT1 LOW - check if held for 5 seconds");
    for (i=0;i<5000;i++)
    {
      if (digitalRead(INPUT1_PIN) != LOW)
      {
        Serial.println("INPUT1 HIGH - released early - continue");
        i = 9999;  // Abort testing button released early
      }
      delay(1);
    }
    if (i<9000)
    {
      Serial.println("INPUT1 LOW - 5 seconds - reset wifi settings");
      wifiManager.resetSettings();
    }
  }

  // Connect using wifimanager
  if(!wifiManager.autoConnect(HostName,AP_PASSWORD)) 
  {
    Serial.println("Failed to connect - resetting");
    ESP.restart();  //reset and try again
  }

  // Made it past wifimanager so must be connected
  Serial.println("WiFi connected");
  WiFi.softAPdisconnect();     // ensure softAP is stopped so mDNS works reliably
  delay(10);
  esp_wifi_set_ps(WIFI_PS_NONE);

  IPAddress ip;
  ip = WiFi.localIP();
  
  Serial.println("");
  Serial.print("Stream Link: http://");
  Serial.print(ip);
  Serial.println("/mjpeg/1");
  
  Serial.print("Snapshot Link: http://");
  Serial.print(ip);
  Serial.println("/jpg");

  Serial.print("ESP MAC Address: ");
  Serial.println(WiFi.macAddress());


  // Start the MQTT
  client.setServer(SetupData.MQTTHost, SetupData.MQTTPort);
  client.setCallback(MQTTcallback);
  PreviousMQTTMillis = millis();


  sensor_t * s = esp_camera_sensor_get();

  s->set_brightness(s, CameraSettings.brightness);     // -2 to 2
  s->set_contrast(s, CameraSettings.contrast);         // -2 to 2
  s->set_saturation(s, CameraSettings.saturation);     // -2 to 2
  s->set_special_effect(s, CameraSettings.effect);     // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, CameraSettings.whitebal);         // 0 = disable , 1 = enable
  s->set_awb_gain(s, CameraSettings.awb_gain);         // 0 = disable , 1 = enable
  s->set_wb_mode(s, CameraSettings.wb_mode);           // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, CameraSettings.exposure_ctrl);  // 0 = disable , 1 = enable  
  s->set_aec2(s, CameraSettings.aec2);                 // 0 = disable , 1 = enable
  s->set_ae_level(s, CameraSettings.ae_level);         // -2 to 2 
  s->set_aec_value(s, CameraSettings.aec_value);       // 0 to 1200 (300)
  s->set_gain_ctrl(s, CameraSettings.gain_ctrl);       // 0 = disable , 1 = enable (1)
  s->set_agc_gain(s, CameraSettings.agc_gain);         // 0 to 30 (0)  Higher number brighter image
  s->set_gainceiling(s, (gainceiling_t)CameraSettings.gainceiling);  // 0 to 6  3 for low light
  s->set_bpc(s, CameraSettings.bpc);            // 0 = disable , 1 = enable
  s->set_wpc(s, CameraSettings.wpc);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, CameraSettings.raw_gma);        // 0 = disable , 1 = enable
  s->set_lenc(s, CameraSettings.lenc);           // 0 = disable , 1 = enable
  s->set_hmirror(s, CameraSettings.hmirror);        // 0 = disable , 1 = enable
  s->set_vflip(s, CameraSettings.vflip);          // 0 = disable , 1 = enable
//  s->set_dcw(s, CameraSettings.dcw);            // 0 = disable , 1 = enable         **** Effects the resolution??? ****
//  s->set_colorbar(s, CameraSettings.colorbar);       // 0 = disable , 1 = enable

  Serial.printf("Num task priorities: %d\n", configMAX_PRIORITIES);

  // Creating a new RTOS task for grabbing frames from the camera
  xTaskCreatePinnedToCore(
    camCB,        // callback
    "cam",        // name
    1 * 1024,       // stacj size    
    NULL,         // parameters
    4,            // priority  was 2
    &tCam,        // RTOS task handle
    tskNO_AFFINITY);     //  tskNO_AFFINITY); PRO_CPU);    // core    

  Serial.println("Prefill frame buffers");
  delay(2000);
  Serial.println("Ready");


  // Registering webserver handling routines
  server.on("/mjpeg/1", HTTP_GET, handleJPGSstream);                       // mjpeg stream handler
  server.on("/jpg", HTTP_GET, handleJPG);                                  // single jpg snapshot handler
  server.on("/mqttsetup", HandleMQTTSetupPath);                            // For setting up the MQTT server details
  server.on("/mqttConfirmSave", HTTP_POST, HandleMQTTSaveConfirmation);    // Note the device will reset after a save to adopt new settings
  server.on("/camerasetup", HandleCameraSetup);                            // For changing the camera settings 
  server.onNotFound(handleNotFound);

  // Registering webserver for OTA updates
  server.on("/otaupdate", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", OtaUpdatePage);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  // Starting webserver
  server.begin();

  // Initially no clients connected
  noActiveClients = 0;

  Serial.printf("\nmjpegCB: free heap (start)  : %d\n", ESP.getFreeHeap());

  
  Serial.printf("setup complete: free heap  : %d\n", ESP.getFreeHeap());
  Serial.printf("setup: PSRAM free  : %d\n", ESP.getFreePsram());

  PreviousOtherMillis = millis();

}


// ****************************************************************
// MAIN LOOP
// ****************************************************************


int LastInput1State = 0;
int LastInput2State = 0;
int LastInput3State = 0;

int testing = 0;

void loop() 
{
  // Check if MQTT is connected
  if (!client.connected()) 
  {
    ReconnectMQTT();
  }
  else
  {
    client.loop();
    LastMQTTReconnectMillis = millis();      // This is to ensure the last reconnect doesnt roll over   

    // MQTT Publish any changes to the inputs
    // Main loop has a delay so inputs effectively have a debounce of same time.
    // INPUT1
    if ((digitalRead(INPUT1_PIN)==LOW) && (LastInput1State != LOW))
    {
      client.publish(doorbell_input1_topic, "ON", false);    // we don't want this retained
      LastInput1State = LOW;
    }
    else if ((digitalRead(INPUT1_PIN)==HIGH) && (LastInput1State != HIGH))
    {
      client.publish(doorbell_input1_topic, "OFF", false);    // we don't want this retained
      LastInput1State = HIGH;
    }

    // INPUT2
    if ((digitalRead(INPUT2_PIN)==LOW) && (LastInput2State != LOW))
    {
      client.publish(doorbell_input2_topic, "ON", false);    // we don't want this retained
      LastInput2State = LOW;
    }
    else if ((digitalRead(INPUT2_PIN)==HIGH) && (LastInput2State != HIGH))
    {
      client.publish(doorbell_input2_topic, "OFF", false);    // we don't want this retained
      LastInput2State = HIGH;
    }

    // INPUT3
    if ((digitalRead(INPUT3_PIN)==LOW) && (LastInput3State != LOW))
    {
      client.publish(doorbell_input3_topic, "ON", false);    // we don't want this retained
      LastInput3State = LOW;
    }
    else if ((digitalRead(INPUT3_PIN)==HIGH) && (LastInput3State != HIGH))
    {
      client.publish(doorbell_input3_topic, "OFF", false);    // we don't want this retained
      LastInput3State = HIGH;
    }
    

    // The following block only gets processed approx every MQTT_PERIOD_mS
    if ((millis() - PreviousMQTTMillis) >= MQTT_PERIOD_mS) 
    {
      PreviousMQTTMillis += MQTT_PERIOD_mS;            // Prepare for the next MQTT update time
      // Do MQTT Updates here

      //client.publish(doorbell_ip_topic, IPAddressStr, false);    // we don't want this retained

    }
  }


  // The following block only gets processed approx every OTHER_PERIOD_mS
  if ((millis() - PreviousOtherMillis) >= OTHER_PERIOD_mS) 
  {
    PreviousOtherMillis += OTHER_PERIOD_mS;            // Prepare for the next cycle time

    // Do stuff here
  }

  // Handle any incoming web/url requests
  server.handleClient();
  
  // Let some other tasks do some work until time to loop again
  vTaskDelay(50 / portTICK_PERIOD_MS);        // Delay for 50ms

}
