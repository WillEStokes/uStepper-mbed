#include "../debug.h"
#include "Stepper.h"
#include <string.h>
#include <math.h>

MotorController AxisX(D2, D5); // step, dir
MotorController AxisY(D3, D6); // step, dir
MotorController AxisZ(D4, D7); // step, dir

/*! Initialise list of responding functions */
const Stepper::ComMessage Stepper::comMessages[] = {
    {FID_GET_STATUS, (Stepper::messageHandlerFunc)&Stepper::getStatus},
    {FID_RUN_STEPPER, (Stepper::messageHandlerFunc)&Stepper::runStepper},
    {FID_STOP_STEPPER, (Stepper::messageHandlerFunc)&Stepper::stopStepper},
    {FID_RETURN_HOME, (Stepper::messageHandlerFunc)&Stepper::returnToHome},
    {FID_RUN_TO_NEXT, (Stepper::messageHandlerFunc)&Stepper::runToNext},
    {FID_RUN_TO_PREVIOUS, (Stepper::messageHandlerFunc)&Stepper::runToPrevious},
    {FID_GET_SYS_INFO, (Stepper::messageHandlerFunc)&Stepper::getSysInfo},
    {FID_SET_CONFIG, (Stepper::messageHandlerFunc)&Stepper::setMotorConfig},
};

/*! Parameterised constructor */
Stepper::Stepper(
        PinName enable,
        PinName redLED,
        PinName home,
        PinName port)
        : _enable(enable),
        _redLED(redLED),
        _home(home),
        _port(port),
        _fidCount(sizeof (comMessages) / sizeof (ComMessage)), // constant
        _msgHeaderLength(sizeof (MessageHeader)) { //
    
    _enable = 0;
    // Turn LED ON by default
    _redLED = 0;
    _home.mode(PullUp);
    _port.mode(PullUp);
    
    // Initialise non-constant variables
    _flowConfigured = false;
    _motorConfig = new MotorConfig;
}

/*! Get status */
void Stepper::getStatus(const SetMotorSelect* data) { 
    static SystemStatus status; // static is needed to avoid memory allocation every time the function is called
    
    status.header.packetLength = sizeof(SystemStatus);
    status.header.fid = FID_GET_STATUS;
    
    if  ((data->motorSelect.axis < 0) || (data->motorSelect.axis > 2)) {
        status.header.error = MSG_ERROR_INVALID_AXIS;
        _socket->send((char*) &status, sizeof(SystemStatus));
        return;
    }

    getAxisState(data);
    
    switch (data->motorSelect.axis) {
        case AXIS_X:
            _stepsPerformed = AxisX.getStepsPerformed();
            _stepPeriod = AxisX.getStepPeriod();
            break;
        case AXIS_Y:
            _stepsPerformed = AxisY.getStepsPerformed();
            _stepPeriod = AxisY.getStepPeriod();
            break;
        case AXIS_Z:
            _stepsPerformed = AxisZ.getStepsPerformed();
            _stepPeriod = AxisZ.getStepPeriod();
            break;
    }
    
    status.axisState = _axisState;
    status.boardState = _boardState;
    status.home = _home.read();
    status.port = _port.read();
    
    status.suppliedVolume_ml = _stepsPerformed / STEPS_PER_ML;
    if (_boardState == PUMP_RUNNING) {
        status.flowRate_mlmin = (60.0f / _stepPeriod) / STEPS_PER_ML;
    } else {
        status.flowRate_mlmin = 0.0f;
    }
    
    _socket->send((char*) &status, sizeof(SystemStatus));
}

/*! Configure hardware */
void Stepper::setMotorConfig(const SetMotorConfig* data) {
    if  ((data->motorConfig.stepPeriod <= 0.0027f) || (data->motorConfig.stepPeriod > 0.27f)) {
        // One of the parameters is invalid
        comReturn(data, MSG_ERROR_INVALID_PARAMETER);
        return;
    }
    
    if  ((data->motorConfig.axis < 0) || (data->motorConfig.axis > 2)) {
        // One of the parameters is invalid
        comReturn(data, MSG_ERROR_INVALID_AXIS);
        return;
    }
    
    _stepPeriod = data->motorConfig.stepPeriod;
    
    switch (data->motorConfig.axis) {
        case AXIS_X:
            AxisX.setFlowConfig(_stepPeriod);
            break;
        case AXIS_Y:
            AxisY.setFlowConfig(_stepPeriod);
            break;
        case AXIS_Z:
            AxisZ.setFlowConfig(_stepPeriod);
            break;
    }
    
    comReturn(data, MSG_OK);
}

void Stepper::runStepper(const SetMotorSelect* data) {
    if  ((data->motorSelect.axis < 0) || (data->motorSelect.axis > 2)) {
        // One of the parameters is invalid
        comReturn(data, MSG_ERROR_INVALID_AXIS);
        _axisStarted = false;
        return;
    }
    
    switch (data->motorSelect.axis) {
        case AXIS_X:
            _flowConfigured = AxisX.runMotor();
            break;
        case AXIS_Y:
            _flowConfigured = AxisY.runMotor();
            break;
        case AXIS_Z:
            _flowConfigured = AxisZ.runMotor();
            break;
    }
    
    if (!_flowConfigured) {
        comReturn(data, MSG_ERROR_FLOW_NOT_CONFIGURED);
        _axisStarted = false;
        return;
    }
    
    setBoardState(PUMP_RUNNING);
    _axisStarted = true;
    comReturn(data, MSG_OK);
}

void Stepper::stopStepper(SetMotorSelect* data) {
    if  ((data->motorSelect.axis < 0) || (data->motorSelect.axis > 2)) {
        // One of the parameters is invalid
        comReturn(data, MSG_ERROR_INVALID_AXIS);
        return;
    }
    
    switch (data->motorSelect.axis) {
        case AXIS_X:
            AxisX.stopMotor();
            setStepperDirection(data, 0);
            break;
        case AXIS_Y:
            AxisY.stopMotor();
            setStepperDirection(data, 0);
            break;
        case AXIS_Z:
            AxisZ.stopMotor();
            setStepperDirection(data, 0);
            break;
    }
    
    if (AxisX.getAxisState() == 0 && AxisY.getAxisState() == 0 && AxisZ.getAxisState() == 0) {
        setBoardState(IDLE);
    }

    _stepsPerformed = 0;
    comReturn(data, MSG_OK);
}

void Stepper::getAxisState(const SetMotorSelect* data) {
    
    switch (data->motorSelect.axis) {
        case AXIS_X:
            _axisState = AxisX.getAxisState();
            break;
        case AXIS_Y:
            _axisState = AxisY.getAxisState();
            break;
        case AXIS_Z:
            _axisState = AxisZ.getAxisState();
            break;
    }
}

void Stepper::returnToHome(SetMotorSelect* data) {
    if  ((data->motorSelect.axis < 0) || (data->motorSelect.axis > 2)) {
        // One of the parameters is invalid
        comReturn(data, MSG_ERROR_INVALID_AXIS);
        return;
    }
    
    if (!(_home.read() == 0 && _port.read() == 0)) {
        _setMotorSelect = *data;
        setStepperDirection(&_setMotorSelect, 1);
        
        switch (data->motorSelect.axis) {
        case AXIS_X:
            _flowConfigured = AxisX.runMotor();
            break;
        case AXIS_Y:
            _flowConfigured = AxisY.runMotor();
            break;
        case AXIS_Z:
            _flowConfigured = AxisZ.runMotor();
            break;
        }
    
        if (!_flowConfigured) {
            setStepperDirection(&_setMotorSelect, 0);
            comReturn(data, MSG_ERROR_FLOW_NOT_CONFIGURED);
            return;
        }
        
        setBoardState(PUMP_RUNNING);
        _flag = HOME;
        
        _positionSignal = 1;
        _tickerStepper.attach(callback(this, &Stepper::detectFallingEdge), 100ms);
    }
    
    comReturn(data, MSG_OK);
}

void Stepper::runToNext(SetMotorSelect* data) {
    if  ((data->motorSelect.axis < 0) || (data->motorSelect.axis > 2)) {
        // One of the parameters is invalid
        comReturn(data, MSG_ERROR_INVALID_AXIS);
        return;
    }
    
    switch (data->motorSelect.axis) {
        case AXIS_X:
            _flowConfigured = AxisX.runMotor();
            break;
        case AXIS_Y:
            _flowConfigured = AxisY.runMotor();
            break;
        case AXIS_Z:
            _flowConfigured = AxisZ.runMotor();
            break;
    }
    
    if (!_flowConfigured) {
        comReturn(data, MSG_ERROR_FLOW_NOT_CONFIGURED);
        return;
    }
    
    _setMotorSelect = *data;
    setBoardState(PUMP_RUNNING);
    _flag = PORT;
    
    _positionSignal = _port.read();
    
    comReturn(data, MSG_OK);
    _tickerStepper.attach(callback(this, &Stepper::detectFallingEdge), 100ms);
}

void Stepper::runToPrevious(SetMotorSelect* data) {
    if  ((data->motorSelect.axis < 0) || (data->motorSelect.axis > 2)) {
        // One of the parameters is invalid
        comReturn(data, MSG_ERROR_INVALID_AXIS);
        return;
    }
    
    _setMotorSelect = *data;
    setStepperDirection(&_setMotorSelect, 1);
    
    switch (data->motorSelect.axis) {
        case AXIS_X:
            _flowConfigured = AxisX.runMotor();
            break;
        case AXIS_Y:
            _flowConfigured = AxisY.runMotor();
            break;
        case AXIS_Z:
            _flowConfigured = AxisZ.runMotor();
            break;
    }
    
    if (!_flowConfigured) {
        setStepperDirection(&_setMotorSelect, 0);
        comReturn(data, MSG_ERROR_FLOW_NOT_CONFIGURED);
        return;
    }
    
    setBoardState(PUMP_RUNNING);
    _flag = PORT;
    
    _positionSignal = _port.read();
    _tickerStepper.attach(callback(this, &Stepper::detectFallingEdge), 100ms);
    
    comReturn(data, MSG_OK);
}

void Stepper::detectFallingEdge() {
    bool edgeDetected = false;

    switch (_flag) {
        case HOME:
            if (_home.read() < 1 && _port.read() < 1) {
                edgeDetected = true; }
            break;
        case PORT:
            if (_positionSignal == 1 && _port.read() < 1) {
                edgeDetected = true; }
            _positionSignal = _port.read();
            break;
    }
    
    getAxisState(&_setMotorSelect);
    
    if (_axisState != AXIS_RUNNING) {
        _tickerStepper.detach(); }
    
    if (edgeDetected) {
        _tickerStepper.detach();
        switch (_setMotorSelect.motorSelect.axis) {
            case AXIS_X:
                AxisX.stopMotor();
                setStepperDirection(&_setMotorSelect, 0);
                break;
            case AXIS_Y:
                AxisY.stopMotor();
                setStepperDirection(&_setMotorSelect, 0);
                break;
            case AXIS_Z:
                AxisZ.stopMotor();
                setStepperDirection(&_setMotorSelect, 0);
                break;
        }
        
        if (AxisX.getAxisState() == 0 && AxisY.getAxisState() == 0 && AxisZ.getAxisState() == 0) {
            setBoardState(IDLE);
        }
    }
}

void Stepper::setStepperDirection(SetMotorSelect* data, int direction) {
    switch (data->motorSelect.axis) {
        case AXIS_X:
            AxisX.setMotorDirection(direction);
            break;
        case AXIS_Y:
            AxisY.setMotorDirection(direction);
            break;
        case AXIS_Z:
            AxisZ.setMotorDirection(direction);
            break;
    }
}

/*! Get system info */
void Stepper::getSysInfo(const MessageHeader* data) {
    static SystemInfo systemInfo;
    
    systemInfo.header.packetLength = sizeof(SystemInfo);
    systemInfo.header.fid = FID_GET_SYS_INFO;
    
    strcpy(systemInfo.fwVersion, FW_VERSION);
    strcpy(systemInfo.pumpId, PUMP_ID);
    strcpy(systemInfo.ipAddr, _ipAddr.get_ip_address());
    strcpy(systemInfo.macAddr, _eth.get_mac_address());
    
    _socket->send((char*) &systemInfo, sizeof(SystemInfo));
}

/*! LED Functions */
void Stepper::flipRedLED() {
    _redLED = !_redLED;
}

/*! Initialising Ethernet */
void Stepper::initEthernet() {
    // Set static IP
    _eth.set_network(IP_ADDRESS, NETW_MASK, GATEWAY);

    // Bring up the ethernet interface
    _eth.connect();

    // Show the network address
    _eth.get_ip_address(&_ipAddr);

    // Open a socket on the network interface, and create a TCP connection to mbed.org
    _server.open(&_eth);
    _server.bind(7851);
    _server.listen(1);
    _server.set_blocking(true);
    _server.set_timeout(-1);
}

/*! Getting a function pointer based on the FID */
const Stepper::ComMessage* Stepper::getComFromHeader(const MessageHeader* header) {

    if (header->fid >= _fidCount) { //Prevent getting out of an array
        return NULL;
    }

    return &comMessages[header->fid];
}

void Stepper::comReturn(const void* data, const int errorCode) {
    MessageHeader *message = (MessageHeader*) data;
    message->packetLength = _msgHeaderLength;
    message->error = errorCode;
    _socket->send((char*) message, _msgHeaderLength);
}

/*! Setting the board state */
void Stepper::setBoardState(int state) {
    
    _boardState = state;
    
    switch(_boardState) {
        case WAIT_FOR_CONNECTION:
            // Blink red LED
            _tickerStepper.attach(callback(this, &Stepper::flipRedLED), 500ms);
            break;
        case CONNECTED:
            break;
        case IDLE:
            // Solid red LED
            _tickerStepper.detach();
            _redLED = 0;
            break;
        case PUMP_RUNNING:
            break;
        default:
            _tickerStepper.detach();
            _redLED = 1;
            break;
    }
}

/*! Main function */
void Stepper::run() {
    // Indicate initialising state of a system
    setBoardState(WAIT_FOR_CONNECTION);
    
    initEthernet();

    // Initialise data buffer (receive)
    char data[256];
    // Create pointer to header data
    MessageHeader* header;

    while (true) {
        // Indicate state of a system
        setBoardState(WAIT_FOR_CONNECTION);
        
        _socket = _server.accept();
        _socket->getpeername(&_clientAddr);
        
        // Indicate state of a system
        setBoardState(IDLE);
        
        while(true) {
            // Wait for a header
            if (_socket->recv(data, _msgHeaderLength) <= 0) break;
            header = (MessageHeader*)data;
            
            if (header->packetLength != _msgHeaderLength) {
                if (_socket->recv(data + _msgHeaderLength, header->packetLength - _msgHeaderLength) <= 0) break;
            }
            
            const ComMessage* comMessage = getComFromHeader(header);
            
            if(comMessage != NULL && comMessage->replyFunc != NULL) {
                // Allow only pump stop and status commands when pump is running
                (this->*comMessage->replyFunc)((void*)data);
            } else {
                comReturn(data, MSG_ERROR_NOT_SUPPORTED);
            }
        }
        
        // Client disconnected         
        _socket->close();
        
        // Indicate disconnected state
        setBoardState(WAIT_FOR_CONNECTION);
    }
}

