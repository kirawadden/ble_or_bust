//#include <ESP32_Servo.h>

/**
 * @file    MYO_EMG
 * @author  Kira Wadden
 * @date    August 2018
 * @brief   Communicating between the Myo armband and ESP32 via BLE to receive EMG notifications
 */

#include <BLEDevice.h>
#include <time.h>

//#include <Servo.h>
///variables for the linear actuator
//#define LINEARPIN 10
//#define LINEAR_MIN 1050   //max & min pulses in microseconds for the linear actuator
//#define LINEAR_MAX 2000 
//
//Servo LINEAR;
//int linear50Value = 1500; //current positional value being sent to the linear actuator
bool bicepFlexed = false;
//bool extended = false;

const int interval = 500;

int track;
int tracker;
double threshold;
int tot[interval + 5][8];
int triggercount;
bool lasttriggered;
bool triggerPattern[3];
int triggerPatternTail;
int doubleFlexCount;
int changeCounter; // counts # of changes after last double flex

bool hasDebounced;
bool isActive;




//myo ble variables
// The remote service we wish to connect to.
static BLEUUID serviceUUID("d5060001-a904-deb9-4748-2c7f4a124842");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("d5060401-a904-deb9-4748-2c7f4a124842");

// EMG service UUID
static BLEUUID    emgSUUID("d5060005-a904-deb9-4748-2c7f4a124842");
// EMG characteristic UUID 0
static BLEUUID    emgCUUID("d5060105-a904-deb9-4748-2c7f4a124842");
// EMG characteristic UUID 2
static BLEUUID    emgC2UUID("d5060305-a904-deb9-4748-2c7f4a124842");

static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.print("Notify callback for EMG Data Characteristic: ");
    Serial.println(pBLERemoteCharacteristic->getUUID().toString().c_str());
    uint8_t emgData;
    for ( int i = 0; i < length; i ++)
    {
      //Serial.println((uint8_t)pData[i]);
      doubleFlex(pData);
    }
}

bool checkDoubleFlex() {
  if (triggerPattern[findIndexInTriggerPattern(triggerPatternTail - 1)] && 
      !triggerPattern[findIndexInTriggerPattern(triggerPatternTail - 2)] && 
      triggerPattern[findIndexInTriggerPattern(triggerPatternTail - 3)])
    return true;
  
  return false;
}

int findIndexInTriggerPattern(int index){
  if (index >= 0) return index;
  else {
    return 3 + index;
  }
}


//void DataCollector(uint8_t* emgSamples) { //maybe make emgSamples a pointer
//  
//  for (int i = 0; i < 8; i++) {
//      tot[track][i] = abs(static_cast<int>(emgSamples[i]));
//  }
//  track++;
//  tracker++;
//}

    
void print(uint8_t* emgSamples)
{
  int sum = 0;

  int average [8];
  for (int j = 0; j < 8; j++) {
    for (int i = 0; i < interval; i++) {
      sum += tot[i][j];
    }
    average[j] = sum / 8;
    sum = 0;
  }

  for (size_t i = 0; i < sizeof(emgSamples)/sizeof(uint8_t); i++) {
    sum += average[i];
  }
  sum = sum / static_cast<int>(sizeof(emgSamples)/sizeof(uint8_t));

  // print average EMG data
  for (size_t i = 0; i < sizeof(emgSamples)/sizeof(uint8_t); i++) {
    Serial.println(average[i]);
  }

  if (sum >= threshold) {
    if (!lasttriggered) triggercount++;
    Serial.println("TRIGGERED");
    lasttriggered = 1;
  }
  else  {
    Serial.println("NO ACTION");
    lasttriggered = 0;
  }

  if (lasttriggered != triggerPattern[triggerPatternTail == 0 ? 
      sizeof(triggerPattern) - 1 : (triggerPatternTail - 1)]) {
      if (!hasDebounced) {
        hasDebounced = true;
        return;
      }

      isActive = true;

      triggerPattern[triggerPatternTail] = lasttriggered;
      triggerPatternTail = (triggerPatternTail + 1) % sizeof(triggerPattern);
      changeCounter++;
      if (checkDoubleFlex() && changeCounter > 2) {
        doubleFlexCount++;
        changeCounter = 0;
        bicepFlexed = true;
      }
      hasDebounced = false;
  }

  Serial.println("trigger count: "); 
  Serial.print(triggercount);
  Serial.println("double flex count: ");
  Serial.print(doubleFlexCount);
}


void doubleFlex(uint8_t* pData)
{
  track = 0;
  threshold = 20;
  tot[interval + 5][8];
  triggercount = 0;
  lasttriggered = false;
  triggerPattern[0] = 0;
  triggerPattern[1] = 0;
  triggerPattern[2] = 0;

  doubleFlexCount = 0;
  triggerPatternTail = 1;
  hasDebounced = 0;
  isActive = 0;
  changeCounter = 0;
  hasDebounced = 0;

  
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {

    for (int i = 0; i < 8; i++) {
      tot[track][i] = abs(static_cast<int>(pData[i]));
    }
    track++;
    tracker++;
    

    //Myo Sampling rate: 200 Hz
    if (tracker >= 600) {
      changeCounter = 0;
      
      if (!isActive) {
        triggerPatternTail = 1;
        triggerPattern[0] = triggerPattern[2];
        triggerPattern[1] = 0;
        triggerPattern[2] = 0;
      }

      tracker = 0;
      isActive = false;
    }
     
    print(pData);
    track = 0;

    // If a standard exception occurred, we print out its message
    } catch (int e){
        Serial.println("Error");
        Serial.println(e);
    }
}
    

bool connectToServer(BLEAddress pAddress) {
    Serial.print("Forming a connection to ");
    Serial.println(pAddress.toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    // Connect to the remove BLE Server.
    pClient->connect(pAddress);
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our service");
    
    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our characteristic");

    // set sleep mode 
    uint8_t sleepPkt[3] = {0x09, 0x01, 0x01};
    pRemoteCharacteristic->writeValue(sleepPkt, 3, true);
    delay(500);

    // set EMG mode to send filtered
    uint8_t emgPkt[5] = {0x01, 0x03, 0x02, 0x00, 0x00 }; 
    pRemoteCharacteristic->writeValue(emgPkt, 5, true);
    delay(500);

    const uint8_t notificationOn[] = {0x01, 0x00};

    // Obtain reference to EMG service UUID
    pRemoteService = pClient->getService(emgSUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(emgSUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our EMG service");
    Serial.println(emgSUUID.toString().c_str());
    
// Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(emgCUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(emgCUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our EMG characteristic");
    Serial.println(emgCUUID.toString().c_str());
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(emgC2UUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(emgC2UUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our EMG characteristic");
    Serial.println(emgC2UUID.toString().c_str());
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {

      // 
      Serial.print("Found our device!  address: "); 
      advertisedDevice.getScan()->stop();

      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void setup() {
  Serial.begin(115200);
//  LINEAR.attach(LINEARPIN, LINEAR_MIN, LINEAR_MAX);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
//  LINEAR.writeMicroseconds(LINEAR_MAX);
  delay(1000);
} // End of setup.


// This is the Arduino main loop function
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }
//  if(bicepFlexed){
//        if(extended){
//          LINEAR.writeMicroseconds(LINEAR_MIN);
//          delay(10000);
//          }
//         else{
//           LINEAR.writeMicroseconds(LINEAR_MAX);
//           delay(10000);
//         }
//  }
  delay(1000);
} // End of loop
