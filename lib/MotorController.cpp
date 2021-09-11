#include "mbed.h"
#include "MotorController.h"

/*! Parameterised constructor */
MotorController::MotorController(
        PinName step,
        PinName dir)
        : _step(step),
        _dir(dir) { //
    _step = 0;
    _dir = 0;
    
    // Initialise non-constant variables
    _flowConfigured = false;
}

/*! Configure hardware */
void MotorController::setFlowConfig(float stepPeriod) {
    _stepPeriod = stepPeriod;
    _flowConfigured = true;
}

void MotorController::setMotorDirection(int direction) {
    _dir = direction;
}

bool MotorController::runMotor() {
    if (!_flowConfigured) {
        return false;
    }
    
    if (_axisState == RUNNING) {
        return true;
    }
    
    _stepsPerformed = 0;
    _tickerMotor.attach(callback(this, &MotorController::flipStepPin), std::chrono::milliseconds(static_cast<int>(_stepPeriod * 1000 / 2.0f))); //0.002
    setAxisState(RUNNING);
    return true;
}

void MotorController::stopMotor() {
    _tickerMotor.detach();
    setAxisState(IDLE);
    _step = 0;
}

/*! Stepper Functions */
void MotorController::flipStepPin() {
    _step = !_step;
    if (_step == 1) {
        _stepsPerformed++;
    }
}

int MotorController::getStepsPerformed() {
    return _stepsPerformed;
}

float MotorController::getStepPeriod() {
    return _stepPeriod;
}

int MotorController::getAxisState() {
    return _axisState;
}

/*! Setting the pump state */
void MotorController::setAxisState(int state) {
    _axisState = state;
}
