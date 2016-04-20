/*GPRS working April 2016*/
/*
  Web client

 This sketch connects to a website 
 using Wi-Fi functionality on MediaTek LinkIt platform.

 Change the macro WIFI_AP, WIFI_PASSWORD, WIFI_AUTH and SITE_URL accordingly.

 created 13 July 2010
 by dlf (Metodo2 srl)
 modified 31 May 2012
 by Tom Igoe
 modified 20 Aug 2014
 by MediaTek Inc.

 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <vmsock.h>
//#include "mbedtls\net.h"
//#include <mtk.h>

#include <signal.h>
#include <limits.h>
/*#include "aws_iot_mqtt_interface.h"
#include "aws_iot_version.h"
#include "aws_iot_shadow_interface.h"
#include "aws_iot_shadow_json_data.h"
#include "aws_iot_json_utils.h"
#include "aws_iot_log.h"*/
#include "linkit_aws_header.h"
#include "aws_mtk_iot_config.h"
#ifdef connect
#undef connect
#endif
#include <LTask.h>
#include <LWiFi.h>
#include <LWiFiClient.h>
#include <LGPRS.h>
#include <LGPS.h>
#include <LGSM.h>
#include <LGPRSClient.h>
#include <LGPRSServer.h>
#include <LGPRSUdp.h>

#include <LGATT.h>
#include <LGATTClient.h>
#include <LGATTServer.h>


#define BLE_ON  0

#define  TEMP_EN    0

/* temperature example by using analogRead to read sensor data */
#if TEMP_EN
const int B=4275;                 // B value of the thermistor
const int R0 = 100000;            // R0 = 100k
const int pinTempSensor = A0;     // Grove - Temperature Sensor connect to A5
#endif

float getTemp()
{
#if TEMP_EN
    float t = analogRead(pinTempSensor);
    Serial.print("read temp value is ");
    Serial.println(t);
    return t;
#else
    return 21.7;
#endif
}

#if BLE_ON

int i =0;

// create a uuid for app, this uuid id is to identify a client
static LGATTUUID test_uuid("B4B4B4B4-B4B4-B4B4-B4B4-B4B4B4B4B4B4");

// specified / prefered service uuid
// 128bit format for your cared service uuid  
LGATTUUID uuidService("E20A39F4-73F5-4BC4-A12F-17D1AD07A961");

// 16bit format for your cared service uuid
// LGATTUUID uuidService = 0x180F; // battery service

LGATTClient gatt_client;

boolean descInProcessing(const LGATTUUID &serviceUUID, 
            boolean isPrimary, 
            const LGATTUUID& characteristicUUID
            )
{
    boolean ret1;
    LGATTUUID descriptorUUID;

    // get descriptor for current characteristic
    ret1 = gatt_client.getDescriptorInfo(serviceUUID, isPrimary, characteristicUUID, descriptorUUID);
    if (ret1)
    {
        Serial.print("[LGATTC ino]descriptor uuid :");
        Serial.print(descriptorUUID);
        Serial.println();
    }
    else
        return false;
    return true;
}

void charInProcessing(const LGATTUUID &serviceUUID, 
            boolean isPrimary, 
            const LGATTUUID& characteristicUUID, 
            LGATT_CHAR_PROPERTIES properties
            )
{
    LGATTAttributeValue attrValue = {0};
    // notification or indication
    if ((properties & LGATT_CHAR_PROP_NOTIFY) == LGATT_CHAR_PROP_NOTIFY||
    (properties & LGATT_CHAR_PROP_INDICATE) == LGATT_CHAR_PROP_INDICATE)
    {
        // enable notification first
        if (!gatt_client.setupNotification(true, serviceUUID, isPrimary, characteristicUUID))
        {
            Serial.println("[LGATTC ino]notification already enabled");
        }

        // query if the notification has come.
        LGATTAttributeValue value = {0};
        Serial.println("[LGATTC ino]query notification data");
        if (gatt_client.queryNotification(serviceUUID, isPrimary, characteristicUUID, value))
        {
            Serial.printf("[LGATTC ino]notification data[%s][%d]", value.value, value.len);
            Serial.println();
        }
        
    }

    // read characteristic
    if ((properties & LGATT_CHAR_PROP_READ) == LGATT_CHAR_PROP_READ)
    {
        memset(&attrValue, 0, sizeof(attrValue));
        if (gatt_client.readCharacteristic(serviceUUID, isPrimary, characteristicUUID, attrValue))
        {
            Serial.print("[LGATTC ino]read char :");
            Serial.print((char*)attrValue.value);
            Serial.println();
        }
        else
        {
            Serial.print("[LGATTC ino]read char [FAILED]");
            Serial.println();
        }
    }
    
    // write characteristic
    if ((properties & LGATT_CHAR_PROP_WRITE) == LGATT_CHAR_PROP_WRITE)
    {
        /* you can prepare data for the specified characteristic with write property here */
        char szbuf[] = "im a central"; // somedata will be sent to prepherial
        memset(&attrValue, 0, sizeof(attrValue));
        memcpy(attrValue.value, szbuf, strlen(szbuf));
        attrValue.len = strlen(szbuf);
        if (gatt_client.writeCharacteristic(serviceUUID, isPrimary, characteristicUUID, attrValue))
        {
            Serial.print("[LGATTC ino]written data :");
            Serial.print((char*)attrValue.value);
            Serial.println();
        }
        else
        {
            Serial.print("[LGATTC ino]write data [FAILED]");
            Serial.println();
        }
    }
    
    


}


void serviceInProcessing(const LGATTUUID &inputServiceUUID)
{
    // query all services, to find if your specified service exist or not. 
    int num = gatt_client.getServiceCount();
    // enum all of the services
    for (i = 0; i < num; i++)
    {
        LGATTUUID serviceUUID; 
        boolean isPrimary = false;
        
        Serial.print("[LGATTC ino]serviceInProcessing service uuid :");
        Serial.print(inputServiceUUID);
        Serial.println();

        // service uuid matched
        if (gatt_client.getServiceInfo(i, serviceUUID, isPrimary) && 
            inputServiceUUID == serviceUUID)
        {
            Serial.print("[LGATTC ino]found service uuid :");
            Serial.print(serviceUUID);
            Serial.println();
            boolean ret;

            while (1)
            {
                delay(50);    
                LGATTUUID characteristicUUID;
                LGATT_CHAR_PROPERTIES properties = 0;
                
                // polling all characteristics of current service
                ret = gatt_client.getCharacteristicInfo(serviceUUID, isPrimary, characteristicUUID, properties);
                if (ret)
                {
                    // characteristic processing here
                    charInProcessing(serviceUUID, isPrimary, characteristicUUID, properties);
                    while (1)
                    {
                        // polling all descriptors for current characteristic
                        if (!descInProcessing(serviceUUID, isPrimary, characteristicUUID))
                            break;
                    }
                }
                else
                    break;
                

            };
            break;
        }
        delay(50);    
    }

}

boolean hasFoundSpencifiedBleDev(const LGATTDeviceInfo &info)
{
    // you can check your known remote device address here, or else the first descovered ble device will be connected.
    {
       return true; 
    }
    return false;
}


#endif // BLE_ON

/**
 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
 */
char HostAddress[255] = AWS_IOT_MQTT_HOST;
/**
 * @brief Default MQTT port is pulled from the aws_iot_config.h
 */
VMINT port = AWS_IOT_MQTT_PORT;

char cafileName[] = AWS_IOT_ROOT_CA_FILENAME;
char clientCRTName[] = AWS_IOT_CERTIFICATE_FILENAME;
char clientKeyName[] = AWS_IOT_PRIVATE_KEY_FILENAME;

#define ROOMTEMPERATURE_UPPERLIMIT 32.0f
#define ROOMTEMPERATURE_LOWERLIMIT 25.0f
#define STARTING_ROOMTEMPERATURE ROOMTEMPERATURE_LOWERLIMIT
#define MAX_LENGTH_OF_UPDATE_JSON_BUFFER 200

static void simulateRoomTemperature(float *pRoomTemperature){
	static float deltaChange;

	if(*pRoomTemperature >= ROOMTEMPERATURE_UPPERLIMIT){
		deltaChange = -0.5f;
	}else if(*pRoomTemperature <= ROOMTEMPERATURE_LOWERLIMIT){
		deltaChange = 0.5f;
	}

	*pRoomTemperature+= deltaChange;
}

QoSLevel qos = QOS_0;
int32_t i;
IoT_Error_t rc;

LWiFiClient c;

typedef struct {
  double temperature;
  bool windowOpen;
} ShadowReported;

ShadowReported reported;

typedef struct {
  bool windowOpen;
} ShadowDesired;

ShadowDesired desired;

char shadowTxBuffer[256];
char deltaBuffer[256];
boolean nativeLoop(void* user_data);

void ShadowUpdateStatusCallback(const char *pThingName, ShadowActions_t action, Shadow_Ack_Status_t status,
		const char *pReceivedJsonDocument, void *pContextData) {

//	if (pReceivedJsonDocument != NULL) {
//		DEBUG("Received JSON %s\n", pReceivedJsonDocument);
//	}
	if (status == SHADOW_ACK_TIMEOUT) {
		Serial.println("Update Timeout--");
	} else if (status == SHADOW_ACK_REJECTED) {
		Serial.println("Update RejectedXX");
	} else if (status == SHADOW_ACK_ACCEPTED) {
		Serial.println("Update Accepted !!");
	}
}

void windowActuate_Callback(const char *pJsonString, uint32_t JsonStringDataLen, jsonStruct_t *pContext) {
	if (pContext != NULL) {
	    Serial.print("Delta - Window state changed to ");
            Serial.println(*(bool *)(pContext->pData));
	}
}


MQTTClient_t mqttClient;
char *pJsonStringToUpdate;
float temperature = 0.0;
char JsonDocumentBuffer[MAX_LENGTH_OF_UPDATE_JSON_BUFFER];
size_t sizeOfJsonDocumentBuffer;

bool windowOpen = false;
jsonStruct_t windowActuator;
jsonStruct_t temperatureHandler;
ShadowParameters_t sp;

/*GPS information updated to cloud */
double latitude;
double longitude;
jsonStruct_t latitudeHandler;
jsonStruct_t longitudeHandler;


// invoked in main thread context
void bearer_callback(VMINT handle, VMINT event, VMUINT data_account_id, void *user_data)
{
    if (VM_BEARER_WOULDBLOCK == g_bearer_hdl)
    {
        g_bearer_hdl = handle;
    }
    
    switch (event)
    {
        case VM_BEARER_DEACTIVATED:
            break;
        case VM_BEARER_ACTIVATING:
            break;
        case VM_BEARER_ACTIVATED:
              LTask.post_signal();
              break;
        case VM_BEARER_DEACTIVATING:
            break;
        default:
            break;
    }
}

boolean  mqtt_start(void* ctx)
{      
    rc = NONE_ERROR;
    i = 0;
    aws_iot_mqtt_init(&mqttClient);
  
    sizeOfJsonDocumentBuffer = sizeof(JsonDocumentBuffer) / sizeof(JsonDocumentBuffer[0]);
  
    windowActuator.cb = windowActuate_Callback;
    windowActuator.pData = &windowOpen;
    windowActuator.pKey = "windowOpen";
    windowActuator.type = SHADOW_JSON_BOOL;
    
    temperatureHandler.cb = NULL;
    temperatureHandler.pKey = "temperature";
    temperatureHandler.pData = &temperature;
    temperatureHandler.type = SHADOW_JSON_FLOAT;

  /* GPRS Position updated to the AWS cloud */
    latitudeHandler.cb = NULL;
    latitudeHandler.pKey = "latitude";
    latitudeHandler.pData = &latitude;
    latitudeHandler.type = SHADOW_JSON_DOUBLE;
  
    longitudeHandler.cb = NULL;
    longitudeHandler.pKey = "longitude";
    longitudeHandler.pData = &longitude;
    longitudeHandler.type = SHADOW_JSON_DOUBLE;

    sp = ShadowParametersDefault;
    sp.pMyThingName = AWS_IOT_MY_THING_NAME;
    sp.pMqttClientId = AWS_IOT_MQTT_CLIENT_ID;
    sp.pHost = HostAddress;
    sp.port = port;
    sp.pClientCRT = AWS_IOT_CERTIFICATE_FILENAME;
    sp.pClientKey = AWS_IOT_PRIVATE_KEY_FILENAME;
    sp.pRootCA = AWS_IOT_ROOT_CA_FILENAME;

    Serial.print("  . Shadow Init... ");
    rc = aws_iot_shadow_init(&mqttClient);
    if (NONE_ERROR != rc) {
      Serial.println("Error in connecting...");
    }
    Serial.println("ok");
    
    rc = aws_iot_shadow_connect(&mqttClient, &sp);

    if (NONE_ERROR != rc) {
  		Serial.println("Shadow Connection Error");
    }

    rc = aws_iot_shadow_register_delta(&mqttClient, &windowActuator);
    
    if (NONE_ERROR != rc) {
      Serial.println("Shadow Register Delta Error");
    }

    temperature = STARTING_ROOMTEMPERATURE;
    
    Serial.println("  . mqtt_start finished...ok");
//              if (NONE_ERROR != rc) {
//		Serial.println("An error occurred in the loop.");
//	      }
//
//	      Serial.println("Disconnecting");
//	      rc = aws_iot_shadow_disconnect(&mqttClient);
//
//	      if (NONE_ERROR != rc) {
//		ERROR("Disconnect error");
//	      }
  return true;
}

boolean bearer_open(void* ctx){
    if (WIFI_USED)
        g_bearer_hdl = vm_bearer_open(VM_BEARER_DATA_ACCOUNT_TYPE_WLAN ,  NULL, bearer_callback);
    else
        g_bearer_hdl = vm_bearer_open(VM_APN_USER_DEFINE ,  NULL, bearer_callback);
    if(g_bearer_hdl >= 0)
        return true;
    return false;
}

/* Resolve IP address for AWS server */
VMINT wifiResolveCallback(vm_soc_dns_result *pDNS)
{
  //C_ADDRESS = (const char*)&pDNS->address[0];
  IN_ADDR addr;
  addr.S_un.s_addr = pDNS->address[0];
  CONNECT_IP_ADDRESS = inet_ntoa(addr);
//  Serial.println("wifiResolveCallback");
//  Serial.print("ip address is ");
//  Serial.println(CONNECT_IP_ADDRESS);
  LTask.post_signal();
  return 0;
}


boolean wifiResolveDomainName(void *userData)
{
  VMCHAR *domainName = (VMCHAR *)userData;
  vm_soc_dns_result dns;
  IN_ADDR addr;
  
//  Serial.print("in wifiResolveDomainName, host name is ");
//	Serial.println(domainName);

  VMINT resolveState;
  if (WIFI_USED){
        resolveState = vm_soc_get_host_by_name(VM_TCP_APN_WIFI,
                                 domainName,
                                 &dns,
                                 &wifiResolveCallback);
      Serial.flush();
  }
  else{
      Serial.flush();
        resolveState = vm_soc_get_host_by_name(6,
                                 domainName,
                                 &dns,
                                 &wifiResolveCallback);
      Serial.flush();
  }
                           
  if (resolveState > 0)
  {
    // not done yet
    return false;
  }

  switch (resolveState)
  {
  case VM_E_SOC_SUCCESS:  // Get IP address successfully, result is filled.
    addr.S_un.s_addr = dns.address[0];
    CONNECT_IP_ADDRESS = inet_ntoa(addr);
    Serial.print("ip address is ");
    Serial.println(CONNECT_IP_ADDRESS);

    return true;
  case VM_E_SOC_WOULDBLOCK:  // wait response from network, result could be gotten from callback.
    // need to wait, return directly
    // so MMI message loop may continue.
    return false;
  case VM_E_SOC_INVAL:  // invalid arguments: null domain_name, etc.
  case VM_E_SOC_ERROR:  // unspecified error
  case VM_E_SOC_LIMIT_RESOURCE:  // socket resources not available
  case VM_E_SOC_INVALID_ACCOUNT:  // invalid data account id
    return true;
  }
}

/* GPRS Specific Variable Begin */
#define SITE_URL "www.google.com"
LGPRSClient client;
gpsSentenceInfoStruct info;
char buff[256];
/* GPRS Specific Variable End */

/* SMS specific variable*/
char recv[161]="On";

/* GPS Related Function */


static unsigned char getComma(unsigned char num,const char *str)
{
  unsigned char i,j = 0;
  int len=strlen(str);
  for(i = 0;i < len;i ++)
  {
     if(str[i] == ',')
      j++;
     if(j == num)
      return i + 1; 
  }
  return 0; 
}

static double getDoubleNumber(const char *s)
{
  char buf[10];
  unsigned char i;
  double rev;
  
  i=getComma(1, s);
  i = i - 1;
  strncpy(buf, s, i);
  buf[i] = 0;
  rev=atof(buf);
  return rev; 
}

static double getIntNumber(const char *s)
{
  char buf[10];
  unsigned char i;
  double rev;
  
  i=getComma(1, s);
  i = i - 1;
  strncpy(buf, s, i);
  buf[i] = 0;
  rev=atoi(buf);
  return rev; 
}

void parseGPGGA(const char* GPGGAstr)
{
  /* Refer to http://www.gpsinformation.org/dale/nmea.htm#GGA
   * Sample data: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
   * Where:
   *  GGA          Global Positioning System Fix Data
   *  123519       Fix taken at 12:35:19 UTC
   *  4807.038,N   Latitude 48 deg 07.038' N
   *  01131.000,E  Longitude 11 deg 31.000' E
   *  1            Fix quality: 0 = invalid
   *                            1 = GPS fix (SPS)
   *                            2 = DGPS fix
   *                            3 = PPS fix
   *                            4 = Real Time Kinematic
   *                            5 = Float RTK
   *                            6 = estimated (dead reckoning) (2.3 feature)
   *                            7 = Manual input mode
   *                            8 = Simulation mode
   *  08           Number of satellites being tracked
   *  0.9          Horizontal dilution of position
   *  545.4,M      Altitude, Meters, above mean sea level
   *  46.9,M       Height of geoid (mean sea level) above WGS84
   *                   ellipsoid
   *  (empty field) time in seconds since last DGPS update
   *  (empty field) DGPS station ID number
   *  *47          the checksum data, always begins with *
   */

  int tmp, hour, minute, second, num ;
  if(GPGGAstr[0] == '$')
  {
    tmp = getComma(1, GPGGAstr);
    hour     = (GPGGAstr[tmp + 0] - '0') * 10 + (GPGGAstr[tmp + 1] - '0');
    minute   = (GPGGAstr[tmp + 2] - '0') * 10 + (GPGGAstr[tmp + 3] - '0');
    second    = (GPGGAstr[tmp + 4] - '0') * 10 + (GPGGAstr[tmp + 5] - '0');
    
    
    tmp = getComma(2, GPGGAstr);
    latitude = getDoubleNumber(&GPGGAstr[tmp]);
    tmp = getComma(4, GPGGAstr);
    longitude = getDoubleNumber(&GPGGAstr[tmp]);
    
    tmp = getComma(7, GPGGAstr);
    num = getIntNumber(&GPGGAstr[tmp]);    

    if (num > 0)
    {
      sprintf(buff, "UTC timer %2d-%2d-%2d", hour, minute, second);
      Serial.println(buff);
  
      sprintf(buff, "latitude = %10.4f, longitude = %10.4f", latitude, longitude);
      Serial.println(buff); 
      
      sprintf(buff, "satellites number = %d", num);
      Serial.println(buff); 
    }
      
  }
  else
  {
    Serial.println("Not get data"); 
  }
}



/* Main setup function */
void setup()
{
  delay(5000);  
  LTask.begin();
  int i=0;
      // Begin Serial Port 
  Serial.begin(115200);
  while(!Serial)
    delay(100);

      // Start Shadow app
  Serial.println("Starting Shadwo App on LinkedItOne");  
  // keep retrying until connected to AP
  if (WIFI_USED){
    LWiFi.begin();
    Serial.print("  . Connecting to AP...");
    Serial.flush();
    while (0 == LWiFi.connect(WIFI_AP, LWiFiLoginInfo(WIFI_AUTH, WIFI_PASSWORD)))
    {
      delay(1000);
    }
  }
  else{  
    Serial.println("  . Connecting to GPRS...");
    Serial.flush();
    while (!LGPRS.attachGPRS(GPRS_APN, GPRS_USERNAME, GPRS_PASSWORD))
    {
      i++;      
      delay(500);
      sprintf(buff,  "Waiting (%d)... ZZZ", i);
      Serial.println(buff);      
    }
    
    sprintf(buff, " Connected. (%d)", i);  
    Serial.println(buff);      

    i=0;
    while (!LSMS.ready())
    {
      delay(1000);
    }

    LSMS.beginSMS("17327715310");
    LSMS.println("Hello from LinkIt!");
    LSMS.endSMS();     
    
  }

  Serial.println("ok");
  
  LTask.remoteCall(&wifiResolveDomainName, (void*)HostAddress);
 
//  CONNECT_IP_ADDRESS = IP_ADDRESS;
  CONNECT_PORT = port;
  
  LTask.remoteCall(&bearer_open, NULL);
  LTask.remoteCall(&mqtt_start, NULL);

  Serial.println("Setup Done");  
}

/* for analogRead or other API, some may not be able to called in remoteCall. 
You may need to call it in loop function and then pass the parameter to nativeLoop through a pointer. */
void loop()
{
    int aa[1]; /* Read and store the analog data in this variable and pass the parameter to nativeLoop through a pointer */
    
    char buf[20]; // SMS Buffer
    int v; // read the SMS character
    
    Serial.println("LGPS loop"); 
    LGPS.getData(&info);
    Serial.println((char*)info.GPGGA); 
    parseGPGGA((const char*)info.GPGGA);

    if (LSMS.available()) // Check if there is new SMS
    {
      int j=0;
      Serial.println("There is new message.");
      LSMS.remoteNumber(buf, 20); // display Number part
      Serial.print("Number:");
      Serial.println(buf);
  
      Serial.print("Content:"); // display Content part
      while (true)
      {
        v = LSMS.read();
        if (v < 0)
          break;
        Serial.print((char)v);
        recv[j++]=(char)v;
      }
      Serial.println();
      
      recv[j]='\n';
      Serial.println(recv);
      
      LSMS.flush(); // delete message  
    }

//    aa[0] =analogRead(A0);
    Serial.flush();
#if TEMP_EN
    updateTemp();
#endif
    LTask.remoteCall(nativeLoop, (void*)aa);

#if BLE_ON

    // scan device
    Serial.println("[LGATTC ino]loop::start to scan.");
    int num = 0;
    LGATTDeviceInfo info = {0};
    while (1)
    {
        // scan ble devices
        num = gatt_client.scan(6);
        int found = 0;
        Serial.printf("scan num [%d]", num);
        Serial.println();

        // polling all found devices
        for (i = 0; i < num; i++)
        {
            gatt_client.getScanResult(i, info);
            Serial.printf("[LGATTC ino]dev address : [%x:%x:%x:%x:%x:%x] rssi [%d]", 
            info.bd_addr.addr[5], info.bd_addr.addr[4], info.bd_addr.addr[3], info.bd_addr.addr[2], info.bd_addr.addr[1], info.bd_addr.addr[0],
            info.rssi);
            Serial.println();

            // check if the current ble device is the one you prefered.
            if (hasFoundSpencifiedBleDev(info))
            {
                found = 1;
                break;
            }
            
        }
        if (found)
        {
            break;
        }
        delay(500);
    }
    
    Serial.println("[LGATTC ino]loop::start to connect.");

    if (!gatt_client.connect(info.bd_addr)) // search all services till timeout or searching done.
    {
        Serial.println("[LGATTC ino]begin() failed to connect.");
        delay(0xffffffff);
    }
    else
        Serial.println("[LGATTC ino]begin() success to connect.");

    // central working here, serviceInProcessing is a function query all characteristics. 
    // it make a chance for you to do some actions to the one or more characteristics. 
    // such as read / write them.
    int times = 3;
    while (times--)
    {
        serviceInProcessing(uuidService);
        delay(1000);
    }

    // disconnect the remote
    gatt_client.disconnect(info.bd_addr);
    Serial.println("[LGATTC ino] Disconnected");

    // finished all
    if (!gatt_client.end())
    {
        Serial.println("[LGATTC ino] failed to end");
    }
    Serial.println("[LGATTC ino] ended");
    
    delay(1000);

    // re-start again
    if (!gatt_client.begin(test_uuid))
    {
        Serial.println("[LGATTC ino] failed to begin");
    }    
    Serial.println("[LGATTC ino] Begined");

#endif 


    delay(2000);
}

/* message could be passed value to publish_Shadow */
int publish_Shadow(char * topic, char * message) {
    int rc = NONE_ERROR;
    
    rc = aws_iot_shadow_yield(&mqttClient, 1000);   //please don't try to put it lower than 1000, otherwise it may going to timeout easily and no response  
    delay(1000);
    Serial.println("=======================================================================================");
    Serial.print("On Device: window state ");
    if (windowOpen)
        Serial.println("true");
    else
        Serial.println("false");
        // increment temperature randomly
    simulateRoomTemperature(&temperature);

    if (temperature > 20)
         windowOpen = true;
    rc = aws_iot_shadow_init_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
    if (rc == NONE_ERROR) {
        //rc = aws_iot_shadow_add_reported(JsonDocumentBuffer, sizeOfJsonDocumentBuffer, 2, &temperatureHandler, &windowActuator);
        rc = aws_iot_shadow_add_reported(JsonDocumentBuffer, sizeOfJsonDocumentBuffer, 4, &temperatureHandler, &windowActuator, &latitudeHandler, &longitudeHandler);
        if (rc == NONE_ERROR) {
	    rc = aws_iot_finalize_json_document(JsonDocumentBuffer, sizeOfJsonDocumentBuffer);
            if (rc == NONE_ERROR){
	        Serial.print("Update Shadow: ");
                Serial.println(JsonDocumentBuffer);
		rc = aws_iot_shadow_update(&mqttClient, AWS_IOT_MY_THING_NAME, JsonDocumentBuffer, ShadowUpdateStatusCallback, NULL, 4, true);
                Serial.print(" rc for aws_iot_shadow_update is ");
                Serial.println(rc);
            }
        }
     }
     Serial.println("*****************************************************************************************");
     return rc;
}

char mqtt_message[2048];
boolean nativeLoop(void* user_data) {
    
    int *bb = (int*)user_data;
    sprintf(mqtt_message, "%s : your passing message", *bb);
    
    publish_Shadow("mtkTopic5", mqtt_message);
    Serial.flush();
}

/* if enabled, update temperature value from sensor */
void updateTemp()
{
    static unsigned long timer_t = millis();
    
    // update per 2000ms
    if(millis()-timer_t > 2000)
    {
        timer_t = millis();
        temperature = getTemp();
        Serial.print("update temp: ");
        Serial.println(temperature);
    }
}
