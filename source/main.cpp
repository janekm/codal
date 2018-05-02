#include "mbed.h"
#include "Microbot.h"
#include "DynamicPwm.h"
#include "LEDArray.h"

#include "BLE.h"
#include "VL53L0X.h"
static const uint16_t uuid16_list[]        = {GattService::UUID_HEALTH_THERMOMETER_SERVICE};
 

static const char     DEVICE_NAME[]        = "BBC micro:bit";

Microbot microbot;

char eye_16x9[] = \
    "\xb5\xc1\xd4\xe6\xe4\xd7\xdf\xd9\xd0\xc0\xe3\xe3\xe3\xe3\xe0\xde\xbc\xd2"\
    "\xc3\xb6\x81\x76\x8e\x50\x41\x84\x99\x76\x88\xda\xe3\xd0\xbc\xbc\x7a\xc8"\
    "\x70\x17\x20\x1e\x19\x2c\x34\x94\xa2\x6f\xb0\xdc\x9f\x7c\xa3\x9b\x78\x3d"\
    "\x56\x17\x15\x1c\x07\x26\x6d\x9b\xa5\xbe\x7e\xb3\x7c\x78\xc6\x7d\xb6\x00"\
    "\x00\x42\x22\x2b\x0d\x28\x92\xb1\xb7\x6f\x31\xee\xfe\x3e\x82\x24\x13\x4e"\
    "\x26\xa6\xc3\x31\x37\xb0\xb8\x37\x53\xdf\xff\x9a\x3c\x7b\x5e\x36\x8d\xe8"\
    "\xd4\xe4\xbc\xb6\xb4\xc0\xc1\xbc\xd5\xf3\x91\x52\x7e\x8a\xb6\xb8\xc2\xbc"\
    "\xc2\xd1\xc5\xe8\xe0\xdf\xb8\x9b\x80\x63\x88\x8d\x79\x99\xc1\xc4\xc4\xca"\
;

/**
 * This function is called when the ble initialization process has failed
 */
void onBleInitError(BLE &ble, ble_error_t error)
{
    /* Avoid compiler warnings */
    (void) ble;
    (void) error;

    /* Initialization error handling should go here */
}


const uint8_t  MicroBitEventServiceUUID[] = {
    0xe9,0x5d,0x93,0xaf,0x25,0x1d,0x47,0x0a,0xa0,0x62,0xfa,0x19,0x22,0xdf,0xa9,0xa8
};

const uint8_t  MicroBitEventServiceMicroBitEventCharacteristicUUID[] = {
    0xe9,0x5d,0x97,0x75,0x25,0x1d,0x47,0x0a,0xa0,0x62,0xfa,0x19,0x22,0xdf,0xa9,0xa8
};

const uint8_t  MicroBitEventServiceClientEventCharacteristicUUID[] = {
    0xe9,0x5d,0x54,0x04,0x25,0x1d,0x47,0x0a,0xa0,0x62,0xfa,0x19,0x22,0xdf,0xa9,0xa8
};

const uint8_t  MicroBitEventServiceMicroBitRequirementsCharacteristicUUID[] = {
    0xe9,0x5d,0xb8,0x4c,0x25,0x1d,0x47,0x0a,0xa0,0x62,0xfa,0x19,0x22,0xdf,0xa9,0xa8
};

const uint8_t  MicroBitEventServiceClientRequirementsCharacteristicUUID[] = {
    0xe9,0x5d,0x23,0xc4,0x25,0x1d,0x47,0x0a,0xa0,0x62,0xfa,0x19,0x22,0xdf,0xa9,0xa8
};


const uint8_t  MicroBitButtonServiceUUID[] = {
    0xe9,0x5d,0x98,0x82,0x25,0x1d,0x47,0x0a,0xa0,0x62,0xfa,0x19,0x22,0xdf,0xa9,0xa8
};

const uint8_t  MicroBitButtonAServiceDataUUID[] = {
    0xe9,0x5d,0xda,0x90,0x25,0x1d,0x47,0x0a,0xa0,0x62,0xfa,0x19,0x22,0xdf,0xa9,0xa8
}
;
const uint8_t  MicroBitButtonBServiceDataUUID[] = {
    0xe9,0x5d,0xda,0x91,0x25,0x1d,0x47,0x0a,0xa0,0x62,0xfa,0x19,0x22,0xdf,0xa9,0xa8
};

struct EventServiceEvent
{
    uint16_t    type;
    uint16_t    reason;
};

BLE &blei = BLE::Instance();

EventServiceEvent initialValueForClientEventCharacteristic;
EventServiceEvent initialValueForEventCharacteristic;
EventServiceEvent initialValueForClientRequirementstCharacteristic;

uint8_t buttonAstate;
uint8_t buttonBstate;
ReadWriteGattCharacteristic<EventServiceEvent> 
    clientEventCharacteristic(MicroBitEventServiceClientEventCharacteristicUUID, 
            &initialValueForClientEventCharacteristic);
ReadOnlyGattCharacteristic<EventServiceEvent> 
    eventCharacteristic(MicroBitEventServiceMicroBitEventCharacteristicUUID, 
            &initialValueForEventCharacteristic, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
WriteOnlyGattCharacteristic<EventServiceEvent> clientRequirementsCharacteristic(MicroBitEventServiceClientRequirementsCharacteristicUUID, &initialValueForClientRequirementstCharacteristic);

ReadOnlyGattCharacteristic<EventServiceEvent> 
    microbotRequirementsCharacteristic(MicroBitEventServiceMicroBitRequirementsCharacteristicUUID, 
            &initialValueForEventCharacteristic, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

ReadOnlyGattCharacteristic<uint8_t> 
    buttonACharacteristic(MicroBitButtonAServiceDataUUID, 
            &buttonAstate, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
ReadOnlyGattCharacteristic<uint8_t> 
    buttonBCharacteristic(MicroBitButtonBServiceDataUUID, 
            &buttonBstate, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);


void eventCallback(Event evt) {
    EventServiceEvent _sEvt;
    _sEvt.type = evt.source;
    _sEvt.reason = evt.value;
    if(blei.gap().getState().connected == 1) {
        //DMESG("evt type: %d reason: %d\r\n", _sEvt.type, _sEvt.reason);
        blei.gattServer().write(eventCharacteristic.getValueHandle(), 
                (uint8_t *)&_sEvt, sizeof(EventServiceEvent));
    }
}

void onDataWrittenCallback(const GattWriteCallbackParams* params) {
    if (params->handle == clientEventCharacteristic.getValueHandle()) {
        EventServiceEvent* event = (EventServiceEvent *)(params->data); 
        DMESG("Event type: %d reason: %d\r\n", event->type, event->reason);
        Event _cevt(event->type,event->reason);
        microbot.messageBus.send(_cevt);
        /* Do something here to the actuated LED based on the received params. */
    }
    if (params->handle == clientRequirementsCharacteristic.getValueHandle()) {
        EventServiceEvent* event = (EventServiceEvent *)(params->data);
        microbot.messageBus.listen(event->type, event->reason, &eventCallback);
    }
}

/* Restart Advertising on disconnection*/
void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    DMESG("disconnection\r\n");
    microbot.messageBus.ignore(DEVICE_ID_ANY, DEVICE_EVT_ANY, &eventCallback);
    BLE::Instance().gap().startAdvertising();
}
 

/**
 * Callback triggered when the ble initialization process has finished
 */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;
 
    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }
 
    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }
 
    ble.gap().onDisconnection(disconnectionCallback);


    GattCharacteristic *charTable[] = {&clientEventCharacteristic, &eventCharacteristic, &clientRequirementsCharacteristic, &microbotRequirementsCharacteristic};
    GattService         eventService(MicroBitEventServiceUUID, 
        charTable, sizeof(charTable) / sizeof(GattCharacteristic *));
    ble.gattServer().addService(eventService);

    GattCharacteristic *charTable2[] = {&buttonACharacteristic, &buttonBCharacteristic};
    GattService         buttonService(MicroBitButtonServiceUUID, 
        charTable2, sizeof(charTable2) / sizeof(GattCharacteristic *));
    ble.gattServer().addService(buttonService);
    /* Setup primary service. */
    //thermometerServicePtr = new HealthThermometerService(ble, currentTemperature, HealthThermometerService::LOCATION_EAR);
 
    /* setup advertising */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().setDeviceName((const uint8_t *)DEVICE_NAME);
    //ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list));
    //ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::THERMOMETER_EAR);
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(100); /* 1000ms */
    ble.gattServer().onDataWritten(onDataWrittenCallback);
    ble.gap().startAdvertising();    
}
 
//DynamicPwm led(p20);
DynamicPwm bin1(p9);
DynamicPwm bin2(p12);
//DynamicPwm din1(p4);
//DynamicPwm din2(p6);
//DynamicPwm cin1(p11);
//DynamicPwm cin2(p10);
//mbed::Serial pc(p22, p23, 38400);

//mbed::I2C i2c(p27, p28);

LEDArray ledArray(microbot.i2c, microbot.io.i2cAD2);
VL53L0X distanceA(microbot.i2c);
/**
  * A periodic callback invoked by the fiber scheduler idle thread.
  * We use this for any low priority, backgrounf housekeeping.
  *
  */
void idleCallback(Event evt)
{
    //codal_dmesg_flush();
    //this->io.led.setDigitalValue(1);
    //DMESG("idle");
    //microbot_dmesg_flush();
    blei.processEvents();
}



#define PCA_ADDR 0xB0

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD


char read8(char reg) {
  char buffer[1];
  buffer[0] = reg;
  microbot.i2c.write(PCA_ADDR, buffer, 1, true);
  microbot.i2c.read(PCA_ADDR, buffer, 1);
  return buffer[0];
}

void write8(char reg, char d) {
  char buffer[2];
  buffer[0] = reg;
  buffer[1] = d;
  microbot.i2c.write(PCA_ADDR, buffer, 2);
}


void setPWM(uint8_t num, uint16_t on, uint16_t off) {
  //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);
  char buffer[5];
  buffer[0] = LED0_ON_L+4*num;
  buffer[1] = on & 0xFF;
  buffer[2] = on >> 8;
  buffer[3] = off & 0xFF;
  buffer[4] = off >> 8;

  microbot.i2c.write(PCA_ADDR, buffer, 5);
}

void setPin(uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  val = min(val, 4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, val);
    }
  }
}

void buttonCallback(Event evt)
{
    DMESG("Button event source: %d, value: %d\r\n", evt.source, evt.value);
    if (evt.source == 1) {
    if (evt.value == 1) {
        setPin(0, 243, false);
    } else if (evt.value == 2) {
        setPin(0, 330, false);
    }
    } else {
    if (evt.value == 1) {
        setPin(0, 400, false);
    } else if (evt.value == 2) {
        setPin(0, 330, false);
    }        
    }
}

void setPWMFreq(float freq) {
  //Serial.print("Attempting to set freq ");
  //Serial.println(freq);
  freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  char prescale = floor(prescaleval + 0.5);

  
  char oldmode = read8(PCA9685_MODE1);
  //setLED(1, oldmode, 0, 0);
  //updateLED();
  char newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  microbot.sleep(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                          // This is why the beginTransmission below was not working.
  //  Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
}



#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS                0x008a

int main()
{
    int i = 0;
    int render_frame = 1;
    int display_frame = 0;
    uint8_t i2c_buf[2];
    microbot.init();
    microbot.io.led.setDigitalValue(1);
    printf("Hello World\r\n");
    microbot.i2c.frequency(400000);


    write8(PCA9685_MODE1, 0x00);
    setPWMFreq(60.0);
    //for (int i=0; i < 16; i++) {
    //setPin(7, 330, false);

    microbot.messageBus.listen(DEVICE_ID_SCHEDULER, DEVICE_SCHEDULER_EVT_IDLE, &idleCallback, MESSAGE_BUS_LISTENER_IMMEDIATE);
    microbot.messageBus.listen(DEVICE_ID_BUTTON_A, DEVICE_EVT_ANY, &buttonCallback);
    microbot.messageBus.listen(DEVICE_ID_BUTTON_B, DEVICE_EVT_ANY, &buttonCallback);
    blei.init(bleInitComplete);
    
    /* SpinWait for initialization to complete. This is necessary because the
     * BLE object is used in the main loop below. */
    while (blei.hasInitialized()  == false) 
    { 
        microbot.sleep(10);
        /* spin loop */ 
    }
    
    DMESG("has inited\r\n");
    //while(true) {
        //blei.waitForEvent();
    //    microbot.sleep(10);
    //};
    bin1.setPeriodUs(50);
    bin2.setPeriodUs(50);
    //din1.setPeriodUs(50);
    //din2.setPeriodUs(50);
    //cin1.setPeriodUs(50);
    //cin2.setPeriodUs(50);
    bin2.write(0.6);
    bin1.write(0.0);
    //cin2.write(0.6);
    //cin1.write(0.0);
    //din2.write(0.6);
    //din1.write(0.0);
    microbot.io.i2cAD1.setDigitalValue(1);
    microbot.io.i2cAD2.setDigitalValue(0);
    microbot.io.i2cADB1.setDigitalValue(0);
    microbot.io.i2cADB2.setDigitalValue(0);
    //}
    DMESG("WHOAMI: %x\r\n", distanceA.readReg(0xC0));
    i2c_buf[0] = 0xC0;
    microbot.i2c.write(0x52, (char*)i2c_buf, 1, true);
    microbot.i2c.read(0x52, (char*)i2c_buf, 2, false);

    DMESG("I2C read: %x, %x\r\n", i2c_buf[0], i2c_buf[1]);
    if (i2c_buf[0]!= 0xEE) {
        //NVIC_SystemReset();
    }

    i2c_buf[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;
    i2c_buf[1] = 0x54>>1;
    microbot.i2c.write(0x52, (char*)i2c_buf, 2, false);

    i2c_buf[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;
    i2c_buf[1] = 0x54>>1;
    microbot.i2c.write(0x52, (char*)i2c_buf, 2, false);

    //microbot.io.i2cAD1.setDigitalValue(0);
    microbot.io.i2cADB1.setDigitalValue(1);

    i2c_buf[0] = 0xC0;
    microbot.i2c.write(0x52, (char*)i2c_buf, 1, true);
    microbot.i2c.read(0x52, (char*)i2c_buf, 2, false);

    DMESG("I2C read: %x, %x\r\n", i2c_buf[0], i2c_buf[1]);
    if (i2c_buf[0]!= 0xEE) {
        //NVIC_SystemReset();
    }

    //microbot.io.i2cAD1.setDigitalValue(0);
    //microbot.io.i2cADB1.setDigitalValue(1);

    i2c_buf[0] = 0xC0;
    microbot.i2c.write(0x54, (char*)i2c_buf, 1, true);
    microbot.i2c.read(0x54, (char*)i2c_buf, 2, false);

    DMESG("I2C read: %x, %x\r\n", i2c_buf[0], i2c_buf[1]);
    if (i2c_buf[0]!= 0xEE) {
        //NVIC_SystemReset();
    }

    i2c_buf[0] = 0x92;
    microbot.i2c.write(0x39<<1, (char*)i2c_buf, 1, true);
    microbot.i2c.read(0x39<<1, (char*)i2c_buf, 1, false);

    DMESG("APDS_id: %x", i2c_buf[0]);
    if (i2c_buf[0]!= 0xAB) {
        //NVIC_SystemReset();
    }
    ledArray.selectFrame(1);
    ledArray.sendImage((char *)eye_16x9);

    ledArray.displayFrame(1);
    microbot.sleep(5000);
    
    while(1)
    {
        //microbot.io.led.setDigitalValue(1);
        //microbot.sleep(100);
        //microbot.io.led.setDigitalValue(0);
        //microbot.sleep(100);
        
        microbot.sleep(1);
        render_frame = render_frame == 1 ? 0 : render_frame + 1;
        display_frame = display_frame == 1? 0 : display_frame + 1;
        ledArray.displayFrame(display_frame);
        ledArray.selectFrame(render_frame);
        microbot.io.led.setDigitalValue(0);
        ledArray.renderPlasma(i * 0.05);
        i++;

        for(int i = 0; i < 4; i++) {
            microbot.apa_spi.write(0x00);
        }
        microbot.apa_spi.write(0xE1);
        microbot.apa_spi.write(ledArray.is31_frame_buffer[3]>>0);
        microbot.apa_spi.write(ledArray.is31_frame_buffer[5]>>0);
        microbot.apa_spi.write(ledArray.is31_frame_buffer[7]>>0);

        microbot.apa_spi.write(0xE1);
        microbot.apa_spi.write(ledArray.is31_frame_buffer[9]>>0);
        microbot.apa_spi.write(ledArray.is31_frame_buffer[11]>>0);
        microbot.apa_spi.write(ledArray.is31_frame_buffer[13]>>0);
        for (int i = 0; i < 4; i++) {
            microbot.apa_spi.write(0xFF);
        }
        
        //ble.waitForEvent();
    }
}