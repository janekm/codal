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


#define MAX_X 15
#define MAX_Y 8

#define FRAME_SLEEP 1000

#define MAX_SNAKE_LENGTH 32
#define SNAKE_COLOUR 32
#define BACKGROUND_COLOUR 0
#define FOOD_COLOUR 10

typedef struct  __attribute__((packed)) {uint8_t x; uint8_t y;} position_t;

position_t snake_array[MAX_SNAKE_LENGTH];

int snake_x = 4;
int snake_y = 4;
int snake_direction = 0;

int i_snake_head = 0;
int i_snake_tail = 0;
int speed = 300;

static void move_head(int x, int y) {
    i_snake_head = (i_snake_head+1) % MAX_SNAKE_LENGTH;
    snake_array[i_snake_head] = {x, y};
    ledArray.setPixel(x, y, SNAKE_COLOUR);
}

static void move_tail() {
    position_t pos = snake_array[i_snake_tail];
    ledArray.setPixel(pos.x, pos.y, BACKGROUND_COLOUR);
    i_snake_tail = (i_snake_tail + 1) % MAX_SNAKE_LENGTH;

}

static void put_food(void) {
    char color = FOOD_COLOUR;
    int food_x = rand()%16;
    int food_y = rand()%9;
    while(ledArray.getPixel(food_x, food_y) != BACKGROUND_COLOUR) {
        food_x = rand()%16;
        food_y = rand()%9;        
    }
    ledArray.setPixel(food_x, food_y, FOOD_COLOUR);
}

static void snake_init(void) {
    DMESG("snake init");
    snake_x = 6;
    snake_y = 4;
    snake_direction = 0;
    snake_array[0] = {4, 4};
    snake_array[1] = {5, 4};
    snake_array[2] = {6, 4};
    i_snake_head = 2;
    i_snake_tail = 0;
    for (int y = 0; y < 9; y++) {
        for (int x = 0; x < 16; x++) {
            ledArray.setPixel(x, y, BACKGROUND_COLOUR);
        }
    }
    for (int i = i_snake_tail; i <= i_snake_head; i++) {
        position_t pos = snake_array[i];
        ledArray.setPixel(pos.x, pos.y, SNAKE_COLOUR);
    }
    put_food();

    ledArray.pushFrame();
    speed = 300;
    //DMESG("snake init done");
}



bool check_collision(void) {
    if(snake_x > MAX_X || snake_x < 0 || snake_y > MAX_Y || snake_y < 0) {
        return true;
    }
    if(ledArray.getPixel(snake_x, snake_y) == SNAKE_COLOUR) {
        return true;
    } else {
        return false;
    }
}

bool check_food(void) {
    if(ledArray.getPixel(snake_x, snake_y) == FOOD_COLOUR) {
        return true;
    } else {
        return false;
    }
}

void advance_snake(void) {
    switch(snake_direction) {
        case 0:
            snake_x += 1;
            break;
        case 1:
            snake_y += 1;
            break;
        case 2:
            snake_x -= 1;
            break;
        case 3:
            snake_y -= 1;
            break;
    }
    if(check_collision()) {
        DMESG("COLL");
        snake_init();
        return;
    }
    if(check_food()) {
        DMESG("FOOD");
        speed -= 10;
        put_food();
    } else {
        move_tail();    
    }
    move_head(snake_x, snake_y);
    ledArray.pushFrame();
}

void buttonCallback(Event evt)
{
    DMESG("Button event source: %d, value: %d\r\n", evt.source, evt.value);
    if (evt.source == 1) {
    if (evt.value == 1) {
        snake_direction = (snake_direction - 1) % 4;
        if (snake_direction < 0) {
            snake_direction = 3;
        }
    } else if (evt.value == 2) {
    }
    } else {
    if (evt.value == 1) {
        snake_direction = (snake_direction + 1) % 4;
    } else if (evt.value == 2) {
    }        
    }
    if (evt.value == 13) snake_direction = 2;
    if (evt.value == 15) snake_direction = 0;
    if (evt.value == 9) snake_direction = 3;
    if (evt.value == 11) snake_direction = 1;
}

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

    microbot.messageBus.listen(DEVICE_ID_SCHEDULER, DEVICE_SCHEDULER_EVT_IDLE, &idleCallback, MESSAGE_BUS_LISTENER_IMMEDIATE);
    microbot.messageBus.listen(DEVICE_ID_BUTTON_A, DEVICE_EVT_ANY, &buttonCallback);
    microbot.messageBus.listen(DEVICE_ID_BUTTON_B, DEVICE_EVT_ANY, &buttonCallback);
    microbot.messageBus.listen(1104, DEVICE_EVT_ANY, &buttonCallback);
    
    blei.init(bleInitComplete);
    
    /* SpinWait for initialization to complete. This is necessary because the
     * BLE object is used in the main loop below. */
    while (blei.hasInitialized()  == false) 
    { 
        microbot.sleep(10);
        /* spin loop */ 
    }
    
    DMESG("has inited\r\n");

    microbot.io.i2cAD1.setDigitalValue(1);
    microbot.io.i2cAD2.setDigitalValue(0);
    microbot.io.i2cADB1.setDigitalValue(0);
    microbot.io.i2cADB2.setDigitalValue(0);
    //ledArray.selectFrame(1);
    //ledArray.sendImage((char *)eye_16x9);

    //ledArray.displayFrame(1);
    //microbot.sleep(1000);
    
    snake_init();
    while(1)
    {

        microbot.sleep(speed);
        advance_snake();
       

    }
}