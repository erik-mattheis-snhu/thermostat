/*
 * implementation of a thermostat for the CC3220S-LAUNCHXL development kit <https://www.ti.com/tool/CC3220S-LAUNCHXL>
 *
 * - a hardware timer is used to drive the main loop at 10 Hz (configurable)
 * - the onboard UART is used to provide bi-directional communication to a server via the debug serial port
 * - the onboard temperature sensor is used to measure ambient temperature <https://www.ti.com/product/TMP116>
 * - the side buttons on the board represent up and down buttons that one might find on a thermostat for adjusting temperature
 *   - pressing button 0 lowers the desired temperature
 *   - pressing button 1 raises the lowered temperature
 *   - pressing both buttons simultaneously (within one polling cycle) toggles a remote lock mode
 * - the onboard red LED is used to simulate a connected heating system controlled via GPIO
 *
 * the thermostat compares the set desired temperature with the measured ambient temperature each cycle to update the state
 * of the heating system
 *
 * a simple text protocol is used to send thermostat state to a server connected over the USB serial port whenever state changes
 *
 * commands from the server can be used to request the current state on-demand and remotely set the desired temperature
 *
 * Erik Mattheis <erik.mattheis@snhu.edu>
 */

/* standard libraries */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* chipset drivers and generated configuration */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART.h>
#include "ti_drivers_config.h"

/* thermostat configuration */
const float         MIN_DESIRED_TEMP              = 4.0f;
const float         MAX_DESIRED_TEMP              = 33.0;
const float         BUTTON_PRESS_VALUE            = 0.5f;

/* timer configuration */
const uint32_t      TIMER_HZ = 10;

/* button configuration */
const uint_least8_t DOWN_BUTTON_GPIO_INDEX        = CONFIG_GPIO_BUTTON_0;
const uint_least8_t UP_BUTTON_GPIO_INDEX          = CONFIG_GPIO_BUTTON_1;

/* heater configuration */
const uint_least8_t HEATER_GPIO_INDEX             = CONFIG_GPIO_LED_0;
const unsigned int  HEATER_GPIO_ON_VALUE          = CONFIG_GPIO_LED_ON;
const unsigned int  HEATER_GPIO_OFF_VALUE         = CONFIG_GPIO_LED_OFF;

/* temperature sensor configuration */
const uint_least8_t TEMP_SENSOR_I2C_CONFIG_INDEX  = CONFIG_I2C_0;
const uint8_t       TEMP_SENSOR_I2C_SLAVE_ADDRESS = 0x0041;
const uint8_t       TEMP_SENSOR_I2C_RESULT_REG    = 0x0001;
const I2C_BitRate   TEMP_SENSOR_I2C_BIT_RATE      = I2C_400kHz;
const float         TEMP_SENSOR_RESOLUTION_C      = 0.0078125f;
const unsigned int  TEMP_SENSOR_SENSITIVITY       = 10;

/* temperature sensor variables */
I2C_Handle          tempSensorHandle;
I2C_Transaction     tempSensorTransaction;
uint8_t             tempSensorTxBuffer[1];
uint8_t             tempSensorRxBuffer[2];

/* thermostat variables */
enum HeaterState {
    HEATER_OFF,
    HEATER_ON
};
enum HeaterState    heaterState                   = HEATER_OFF;
float               desiredTemperature            = 20.0f;
float               ambientTemperature            = 0.0f;
unsigned char       preventRemoteSet              = 0;

/* server communications variables */
char                serverInput[64];
char                serverOutput[64];

/* driver handles */
Timer_Handle        timer;
UART_Handle         uart;

/* callback variables */
volatile bool       timerFlag                     = false;
volatile bool       downButtonFlag                = false;
volatile bool       upButtonFlag                  = false;
volatile bool       serverInputFlag               = false;
volatile bool       temperatureFlag               = false;
volatile int16_t    temperatureReading            = false;

/*
 * handle asynchronous notification from timer
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    timerFlag = true; // raise flag for main event loop
}

/*
 * handle asynchronous notification of down button press
 */
void downButtonCallback(uint_least8_t index) {
    downButtonFlag = true; // raise flag for main event loop
}

/*
 * handle asynchronous notification of up button press
 */
void upButtonCallback(uint_least8_t index) {
    upButtonFlag = true; // raise flag for main event loop
}

/*
 * handle asynchronous notification of new input line from server
 */
void serverCallback(UART_Handle handle, void *buf, size_t count) {
    serverInput[count - 1] = 0; // replace new line with null
    serverInputFlag = true;     // raise flag for main event loop
}

/*
 * handle asynchronous temperature readings over the I2C bus
 */
void temperatureCallback(I2C_Handle handle, I2C_Transaction *transaction, bool transferStatus) {
    if (transferStatus) {                                                         // if the bytes were transfered
        int16_t reading = (tempSensorRxBuffer[0] << 8) | (tempSensorRxBuffer[1]); // read 16-bit value from buffer
        int16_t diff = abs(temperatureReading - reading);                         // calculate how much this reading differs from the last reading

        if (diff > TEMP_SENSOR_SENSITIVITY) { // if this reading differs from the last reading by more than the sensitivity setting
            temperatureReading = reading;     // update temperature
            temperatureFlag = true;           // raise flag for main event loop
        }
    }
}

/*
 * ask the temperature sensor for a new measurement over the I2C bus
 */
void requestTemperature() {
    I2C_transfer(tempSensorHandle, &tempSensorTransaction);
}

/*
 * configure the temperature sensor via the I2C bus
 */
void initializeTemperatureSensor() {
    I2C_Params i2cParams;

    I2C_init();
    I2C_Params_init(&i2cParams);

    i2cParams.bitRate                  = TEMP_SENSOR_I2C_BIT_RATE;
    i2cParams.transferMode             = I2C_MODE_CALLBACK;
    i2cParams.transferCallbackFxn      = temperatureCallback;

    tempSensorTransaction.slaveAddress = TEMP_SENSOR_I2C_SLAVE_ADDRESS;
    tempSensorTransaction.writeBuf     = tempSensorTxBuffer;
    tempSensorTransaction.writeCount   = 1;
    tempSensorTransaction.readBuf      = tempSensorRxBuffer;
    tempSensorTransaction.readCount    = 2;

    tempSensorHandle = I2C_open(TEMP_SENSOR_I2C_CONFIG_INDEX, &i2cParams);

    if (tempSensorHandle == NULL) {
        while (1);
    }

    tempSensorTxBuffer[0] = TEMP_SENSOR_I2C_RESULT_REG;
}

/*
 * ask the UART controller to read the next line into serverInput and callback when done
 */
void readFromServer() {
    UART_read(uart, &serverInput, 63);
}

/*
 * setup the UART driver to communicate with the server over a serial port
 */
void initializeServerConnection() {
    UART_Params uartParams;

    UART_init();
    UART_Params_init(&uartParams);

    uartParams.readMode    = UART_MODE_CALLBACK;
    uartParams.readEcho     = UART_ECHO_OFF;
    uartParams.readCallback = serverCallback;

    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        while (1);
    }

    readFromServer();
}

/*
 * setup a timer to throttle the main event loop
 */
void initializeTimer() {
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);

    params.periodUnits   = Timer_PERIOD_HZ;
    params.period        = TIMER_HZ;
    params.timerMode     = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer = Timer_open(CONFIG_TIMER_0, &params);

    if (timer == NULL || Timer_start(timer) == Timer_STATUS_ERROR) {
        while (1);
    }
}

/*
 * configure the heater and button GPIO interfaces
 */
void initializeHeaterAndButtons() {
    // initialize GPIO driver
    GPIO_init();

    // configure down button GPIO pin
    GPIO_setConfig(DOWN_BUTTON_GPIO_INDEX, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(DOWN_BUTTON_GPIO_INDEX, downButtonCallback);
    GPIO_enableInt(DOWN_BUTTON_GPIO_INDEX);

    // configure up button GPIO pin
    GPIO_setConfig(UP_BUTTON_GPIO_INDEX, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(UP_BUTTON_GPIO_INDEX, upButtonCallback);
    GPIO_enableInt(UP_BUTTON_GPIO_INDEX);

    // configure heater GPIO pin
    GPIO_setConfig(HEATER_GPIO_INDEX, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
}

/*
 * set the desired temperature, ensuring that it falls between
 * minimum and maximum bounds, clamping the value to the nearest
 * multiple of the button press value, and only applying the new
 * value if the change is greater than the thermostat sensitivity
 *
 * returns true if the desired temperature was changed
 */
bool setDesiredTemperature(float temperature) {
    if (temperature < MIN_DESIRED_TEMP) {
        temperature = MIN_DESIRED_TEMP;
    } else if (temperature > MAX_DESIRED_TEMP) {
        temperature = MAX_DESIRED_TEMP;
    } else {
        float reciprical = pow(BUTTON_PRESS_VALUE, -1);
        temperature = round(temperature * reciprical) / reciprical;
    }

    if (fabs(desiredTemperature - temperature) > TEMP_SENSOR_RESOLUTION_C * TEMP_SENSOR_SENSITIVITY) {
        desiredTemperature = temperature;
        return true;
    }

    return false;
}

/*
 * write the current state to the server in the following format:
 *
 *   D:20.000000,A:25.187500,H:0,L:0
 *
 * where the fields are as follows:
 *
 *   D: desired temperature (degrees C)
 *   A: ambient temperature (degrees C)
 *   H: heater state (0 = off, 1 = on)
 *   L: remote lock (0 = off, 1 = on)
 */
void sendStateToServer() {
    int len = snprintf(serverOutput, 64, "D:%f,A:%f,H:%d,L:%d\n", desiredTemperature, ambientTemperature, heaterState, preventRemoteSet);
    UART_write(uart, &serverOutput, len);
}

/*
 * parse the server input buffer for supported actions
 *
 * supported actions include a single 'U' character indicating an update request
 * or a command to set the desired temperature in the form:
 *
 *   D:25.000000
 *
 * return true if state should be sent to the server
 */
bool handleServerInput() {
    size_t len = strlen(serverInput);

    if (len > 0) {
        switch (serverInput[0]) {
            case 'U': // update request
                return true;
            case 'D': // set desired temperature
                if (len > 2 && serverInput[1] == ':' && !preventRemoteSet) { // ensure buffer is long enough and remote set is not disallowed
                    float temperature;
                    if (sscanf(&serverInput[2], "%f", &temperature)) { // if a float was successfully parse
                        return setDesiredTemperature(temperature);     // set the desired temperature
                    }
                }
        }
    }

    return false;
}

/*
 * set the desired heater state - return true if state changed
 */
bool setHeaterState(enum HeaterState state) {
    if (heaterState != state) {
        switch (state) {
            case HEATER_ON:
                GPIO_write(HEATER_GPIO_INDEX, HEATER_GPIO_ON_VALUE);
                break;
            case HEATER_OFF:
                GPIO_write(HEATER_GPIO_INDEX, HEATER_GPIO_OFF_VALUE);
                break;
        }
        heaterState = state;
        return true;
    }

    return false;
}

/*
 * decreases desired temperature by one button press
 *
 * returns true if desired temperature changed
 */
bool decreaseDesiredTemperature() {
    return setDesiredTemperature(desiredTemperature - BUTTON_PRESS_VALUE);
}

/*
 * increases desired temperature by one button press
 *
 * returns true if desired temperature changed
 */
bool increaseDesiredTemperature() {
    return setDesiredTemperature(desiredTemperature + BUTTON_PRESS_VALUE);
}

/*
 * the main execution thread invoked by the NoRTOS system
 */
void *mainThread(void *arg0) {
    bool firstTempReceived = false;

    initializeServerConnection();
    initializeHeaterAndButtons();
    initializeTemperatureSensor();
    initializeTimer();

    while(1) { // loop forever
        while (!timerFlag); // wait for timer flag to be raised
        bool stateChanged = false;
        if (serverInputFlag) {                   // if server input flag was raised
            stateChanged |= handleServerInput(); // handle input and set state change indicator accordingly
            serverInputFlag = false;             // lower flag
            readFromServer();                    // request next line from server
        }
        if (downButtonFlag && upButtonFlag) {      // if both down and up button flags were raised
            preventRemoteSet = !preventRemoteSet;  // toggle remote temperature lock
            downButtonFlag = upButtonFlag = false; // lower both flags
            stateChanged = true;
        } else if (downButtonFlag) {                      // if down button flag was raised
            stateChanged |= decreaseDesiredTemperature(); // decrease desired temperature and set state change indicator accordingly
            downButtonFlag = false;                       // lower flag

        } else if (upButtonFlag) {                        // if up button flag was raised
            stateChanged |= increaseDesiredTemperature(); // increase desired temperature and set state change indicator accordingly
            upButtonFlag = false;                         // lower flag
        }
        if (temperatureFlag) {                                                  // if temperature flag was raised
            ambientTemperature = temperatureReading * TEMP_SENSOR_RESOLUTION_C; // multiply temperature reading by sensor resolution to set ambient temperature
            temperatureFlag = false;                                            // lower flag
            firstTempReceived = true;
            stateChanged = true;
        }
        if (ambientTemperature < desiredTemperature) {        // if ambient temperature is less than desired temperature
            stateChanged |= setHeaterState(HEATER_ON);        // make sure heater is on and set state change indicator accordingly
        } else if (ambientTemperature > desiredTemperature) { // if ambient temperature is greater than desired temperature
            stateChanged |= setHeaterState(HEATER_OFF);       // make sure heater is off and set state change indicator accordingly
        }
        if (firstTempReceived && stateChanged) { // if a temperature reading has been made and state has changed
            sendStateToServer();                 // send state to server
        }
        requestTemperature(); // request a temperature reading
        timerFlag = false;    // lower the flag and execute the loop again
    }
}
