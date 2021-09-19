# Thermostat Implementation for [CC3220S-LAUNCHXL](https://www.ti.com/tool/CC3220S-LAUNCHXL)

* a hardware timer is used to drive the main loop at 10 Hz (configurable)
* the onboard UART is used to provide bi-directional communication to a server via the debug serial port
* the [onboard temperature](https://www.ti.com/product/TMP116) sensor is used to measure ambient temperature
* the side buttons on the board represent up and down buttons that one might find on a thermostat for adjusting temperature
 * pressing button 0 lowers the desired temperature
 * pressing button 1 raises the lowered temperature
 * pressing both buttons simultaneously (within one polling cycle) toggles a remote lock mode
* the onboard red LED is used to simulate a connected heating system controlled via GPIO

The thermostat compares the set desired temperature with the measured ambient temperature each cycle to update the state of the heating system.

A simple text protocol is used to send thermostat state to a server connected over the USB serial port whenever state changes.

Commands from the server can be used to request the current state on-demand and remotely set the desired temperature.
