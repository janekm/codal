#include "mbed.h"
#include "Microbot.h"
#include "DynamicPwm.h"
#include "LEDArray.h"
#include "MicrobitRadio.h"
#include "MicrobotMotor.h"

Microbot microbot;

MicroBitRadio radio; 
//DynamicPwm led(p20);
DynamicPwm bin1(p9);
DynamicPwm bin2(p12);
//DynamicPwm din1(p4);
//DynamicPwm din2(p6);
//DynamicPwm cin1(p11);
//DynamicPwm cin2(p10);
//mbed::Serial pc(p22, p23, 38400);

//mbed::I2C i2c(p27, p28);

LEDArray ledArray(i2c, microbot.io.i2cAD2);

VL53L0X distanceA(microbot.i2c);
/**
  * A periodic callback invoked by the fiber scheduler idle thread.
  * We use this for any low priority, backgrounf housekeeping.
  *
  */
void idleCallback(Event evt)
{
}

void buttonCallback(Event evt)
{
    DMESG("Button event source: %d, value: %d\r\n", evt.source, evt.value);
    uint16_t buffer[2] = {evt.source, evt.value};
    PacketBuffer pBuffer((uint8_t *)&buffer, 4);
    radio.datagram.send(pBuffer);
}

void onData(Event evt)
{
    //DMESG("Button event source: %d, value: %d\r\n", evt.source, evt.value);
    uint16_t *buffer = (uint16_t *)radio.datagram.recv().getBytes();
    DMESG("packet: %d, %d\r\n", buffer[0], buffer[1]);
    if (buffer[0] == 1) {
        if (buffer[1] == 1) {
            motor_driver_set(0, 400);
        } else if (buffer[1] == 2) {
            motor_driver_set(0, 0);
        }
    } else if (buffer[0] == 2) {
        if (buffer[1] == 1) {
            motor_driver_set(2, 400);
        } else if (buffer[1] == 2) {
            motor_driver_set(2, 0);
        }        
    }
}

int main()
{
    microbot.init();
    microbot.io.led.setDigitalValue(0);
    radio.enable();
    motor_driver_init();
    printf("Hello World\r\n");

    microbot.messageBus.listen(DEVICE_ID_SCHEDULER, DEVICE_SCHEDULER_EVT_IDLE, &idleCallback, MESSAGE_BUS_LISTENER_IMMEDIATE);
    microbot.messageBus.listen(DEVICE_ID_BUTTON_A, DEVICE_EVT_ANY, &buttonCallback);
    microbot.messageBus.listen(DEVICE_ID_BUTTON_B, DEVICE_EVT_ANY, &buttonCallback);
    microbot.messageBus.listen(DEVICE_ID_RADIO, DEVICE_RADIO_EVT_DATAGRAM, onData);
    DMESG("has inited\r\n");
    while(true) {
        //blei.waitForEvent();
        microbot.sleep(10);
    };
    bin1.setPeriodUs(50);
    bin2.setPeriodUs(50);
    //din1.setPeriodUs(50);
    //din2.setPeriodUs(50);
    //cin1.setPeriodUs(50);
    //cin2.setPeriodUs(50);
    bin2.write(0.6);
    bin1.write(0.0);
    
}