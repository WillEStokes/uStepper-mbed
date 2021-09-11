#include "debug.h"
#include "Stepper.h"
#include "MotorController.h"
#include "mbed.h"
#include "EthernetInterface.h"

Stepper stepper(
    PTC12,
    LED1,
    D12,
    D13);

int main(int, char**) {
    
    // Run
    stepper.run();
}

