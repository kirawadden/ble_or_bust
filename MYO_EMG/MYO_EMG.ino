#include <ESP32Servo.h>

/**
 * @file    MYO_EMG
 * @author  Kira Wadden
 * @date    August 2018
 * @brief   Communicating between the Myo armband and ESP32 via BLE to receive EMG notifications
 */

#include <BLEDevice.h>

//variables for the linear actuator
#define LINEARPIN 18
#define LINEAR_MIN 1050   //max & min pulses in microseconds for the linear actuator
#define LINEAR_MAX 2000

Servo LINEAR;
//int linear50Value = 1500; //current positional value being sent to the linear actuator
bool handClosed = false; //use to keep track of the position of the actuator, start off with hand being assumed open


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
// static BLEUUID    emgC2UUID("d5060305-a904-deb9-4748-2c7f4a124842");

static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

//variables for the rolling window approach


int circularQueue[20];
int tail = 0;
double threshold = 20;
int totalSingleFlexed = 0; //counts the number of triggered values in the circular queue
bool triggered = false; //bool to say if 80% of countTotal is above threshold then trigger


static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.print("Notify callback for EMG Data Characteristic: ");
    Serial.println(pBLERemoteCharacteristic->getUUID().toString().c_str());
    int8_t emgData;
    double sum = 0;
    int counter = 0;


    for ( int i = 0; i < length; i ++){
      if((int8_t)pData[i]>10){ //arbitrary value of 10 to get rid of smaller EMG noise values
        sum += abs((int8_t)pData[i]);
        counter++;
      }
    }
    Serial.print("The sum of that line of characteristics is: ");
    Serial.println(sum/length);
    if(sum/counter >= threshold) {  //checks if that 16 byte array of EMG values is above threshold, if so increment the count
      circularQueue[tail]=1; //adds it to circularQueue
      }
    else circularQueue[tail]=0;
    tail = (tail+1)%20;
    for(int i=0; i<20;i++){
      totalSingleFlexed += circularQueue[i];
      }
    if(totalSingleFlexed>=16){
     if(handClosed && LINEAR.read()==0){ //the read function retruns the current pulse width modulus of the linear actuator
            LINEAR.writeMicroseconds(LINEAR_MAX);
            Serial.println("WRITE TO MAX");
            handClosed = false;
            }
     else if(handclosed! && LINEAR.read()==0){
            LINEAR.writeMicroseconds(LINEAR_MIN);
            Serial.println("WRITE TO MIN YO");
            handClosed = true;
      }
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
//    pRemoteCharacteristic = pRemoteService->getCharacteristic(emgC2UUID);
//    if (pRemoteCharacteristic == nullptr) {
//      Serial.print("Failed to find our characteristic UUID: ");
//      Serial.println(emgC2UUID.toString().c_str());
//      return false;
//    }
//    Serial.println(" - Found our EMG characteristic");
//    Serial.println(emgC2UUID.toString().c_str());
//    pRemoteCharacteristic->registerForNotify(notifyCallback);
//    pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
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
  //LINEAR.attach(LINEARPIN, LINEAR_MIN, LINEAR_MAX);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
  LINEAR.attach(LINEARPIN);
  LINEAR.writeMicroseconds(LINEAR_MAX);
  delay(10000);
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

//    if(triggered){
//        Serial.println("TRIGGERED");
//        if(handClosed){
//            LINEAR.writeMicroseconds(LINEAR_MAX);
//            Serial.println("WRITE TO MAX");
//            delay(10000);
//            handClosed = false;
//            }
//         else{
//            LINEAR.writeMicroseconds(LINEAR_MIN);
//            Serial.println("WRITE TO MIN YO");
//            delay(10000);
//
//            handClosed = true;
//          }
//
//      }

} // End of loop
