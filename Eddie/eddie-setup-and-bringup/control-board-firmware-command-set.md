# Control Board Firmware Command Set

The Eddie Control Board is a complete robot controller and sensor-interface solution. Parallax’s ready-to-go Eddie Control Board firmware, designed for the Eddie Robot Platform provides an easy-to-use serial command interface to control and manage all of the on-board peripheral electronics such as motor drivers, digital I/O, and analog to digital converter (ADC) channels.

The Eddie Control Board communicates over USB; and when connected to a PC, the board enumerates as a serial COM port. Configure the COM port to use these settings of 115.2 kBaud, 8-bit character size, 1 stop bit and no parity.

All commands adhere to the same general format which is shown below:

**Input:** $\textcolor{OliveGreen}{\< cmd \>}[\textcolor{BrickRed}{\< WS \>}\textcolor{OliveGreen}{\< param1 \>}...\textcolor{BrickRed}{\< WS \>}\textcolor{OliveGreen}{\< paramN \>}]\textcolor{BrickRed}{\< CR \>}$

**Response (Success):** $[\textcolor{OliveGreen}{\< param1 \>}...\textcolor{BrickRed}{\< WS \>}\textcolor{OliveGreen}{\< paramN \>}]\textcolor{BrickRed}{\< CR \>}$

**Response (Failure):** $\textcolor{RoyalBlue}{Error}[\textcolor{BrickRed}{\< SP \>}-\textcolor{BrickRed}{\< SP \>}\textcolor{OliveGreen}{\< verbose\\_reason \>}]\textcolor{BrickRed}{\< CR \>}$


In this format:

* $\textcolor{OliveGreen}{\< cmd \>}$ represents the command mnemonic.
* $\textcolor{OliveGreen}{\< param1 \>}...\textcolor{OliveGreen}{\< paramN \>}$ are optional parameters required by the command or mode. Numbers are always entered as hexadecimal values, and signed values use two’s complement.
* $\textcolor{BrickRed}{\< WS \>}$ refers to one or more whitespace characters, which can be spaces (ASCII 32) or tabs (ASCII 9).
* $\textcolor{BrickRed}{\< CR \>}$ is a single carriage-return character (ASCII 13).
* $\textcolor{BrickRed}{\< SP \>}$ is a single space character (ASCII 32).
* $\textcolor{OliveGreen}{\< verbose\\_reason \>}$ is an optional error message displayed when verbose mode is enabled (using the VERB command).

Allowed characters are in the ASCII range from 32 to 126, except for carriage return (ASCII 13) and tab (ASCII 9). Commands can have up to 254 characters, including the terminating carriage return character. Anything beyond this limit is ignored as an invalid command. The command handler only processes and responds after receiving the carriage return character.

## Table 1: Eddie Command Set Description: _Interface_

<table data-full-width="false"><thead><tr><th width="109.19999999999999">Cmd</th><th width="134">Input Params</th><th width="117">Return Params</th><th width="224">Values</th><th>Description</th></tr></thead><tbody><tr><td>HWVER</td><td></td><td>&#x3C;version></td><td>version=0..FFFF</td><td>Get hardware version</td></tr><tr><td>VER</td><td></td><td>&#x3C;version></td><td>version=0..FFFF</td><td>Get firmware version</td></tr><tr><td>VERB</td><td>&#x3C;mode></td><td></td><td>mode= 0(off), 1(on)</td><td>Set verbose mode</td></tr><tr><td>WATCH</td><td>&#x3C;mode></td><td></td><td>mode= 0(off), 1(on)</td><td>Set watch mode</td></tr><tr><td>BLINK</td><td>&#x3C;pin>&#x3C;rate></td><td></td><td><p>pin=0.. 1F </p><p>rate=0..FFFF</p></td><td>Toggle pin at a specified rate in increments of 0.1Hz</td></tr></tbody></table>

## Table 2: Eddie Command Set Description: _I/O Control_

<table data-full-width="false"><thead><tr><th width="102.19999999999999">Cmd</th><th width="141">Input Params</th><th width="147">Return Params</th><th>Values</th><th>Description</th></tr></thead><tbody><tr><td>IN</td><td>&#x3C;bitmask></td><td></td><td>bitmask=0..7FFFF</td><td>Set GPIO pins in bitmask to inputs</td></tr><tr><td>OUT</td><td>&#x3C;bitmask></td><td></td><td>bitmask=0..7FFFF</td><td>Set GPIO pins in bitmask to outputs</td></tr><tr><td>LOW</td><td>&#x3C;bitmask></td><td></td><td>bitmask=0..7FFFF</td><td>Set GPIO pins in bitmask to low (only applies to output pins)</td></tr><tr><td>HIGH</td><td>&#x3C;bitmask></td><td></td><td>bitmask=0..7FFFF</td><td>Set GPIO pins in bitmask to high (only applies to output pins)</td></tr><tr><td>INS</td><td></td><td>&#x3C;bitmask></td><td>bitmask=0..7FFFF</td><td>Get GPIO pins currently set as inputs</td></tr><tr><td>OUTS</td><td></td><td>&#x3C;bitmask></td><td>bitmask=0..7FFFF</td><td>Get GPIO pins currently set as outputs</td></tr><tr><td>LOWS</td><td></td><td>&#x3C;bitmask></td><td>bitmask=0..7FFFF</td><td>Get GPIO pins currently set as low</td></tr><tr><td>HIGHS</td><td></td><td>&#x3C;bitmask></td><td>bitmask=0..7FFFF</td><td>Get GPIO pins currently set as high</td></tr><tr><td>READ</td><td></td><td>&#x3C;bitmask></td><td>bitmask=0..7FFFF</td><td>Get current state (high/low) of all GPIO pins</td></tr></tbody></table>

## Table 3: Eddie Command Set Description: _Sensor Interfacing_

<table data-full-width="false"><thead><tr><th width="125.19999999999999">Cmd</th><th width="138">Input Params</th><th width="203">Return Params</th><th width="189">Values</th><th>Description</th></tr></thead><tbody><tr><td>SPNG</td><td>&#x3C;bitmask></td><td></td><td>bitmask=0..FFFF</td><td>Set pins in bitmask to act as GPIO pins</td></tr><tr><td>SGP</td><td>&#x3C;bitmask></td><td></td><td>bitmask=0..7FFFF</td><td>Set pins in bitmask to act as GPIO pins</td></tr><tr><td>PING</td><td></td><td><p>&#x3C;value1>[&#x3C;value2></p><p>...&#x3C;valueN>]</p></td><td>value=0,12..B54</td><td>Get PING))) sensor sonar measurements (one 12-bit value per sensor)</td></tr><tr><td>ADC</td><td></td><td>&#x3C;value1>...&#x3C;value8></td><td>value=0..FFF</td><td>Get all ADC values (12-bit values)</td></tr></tbody></table>

## Table 4: Eddie Command Set Description: _Motor Control_

<table data-full-width="false"><thead><tr><th width="110.19999999999999">Cmd</th><th width="135">Input Params</th><th width="144">Return Params</th><th>Values</th><th>Description</th></tr></thead><tbody><tr><td>GO</td><td>&#x3C;left>&#x3C;right></td><td></td><td>left/right=80..7F</td><td>Set motor power (signed byte)</td></tr><tr><td>GOSPD</td><td>&#x3C;left>&#x3C;right></td><td></td><td>left/right=8000..7FFF</td><td>Set motor speed (signed word)</td></tr><tr><td>STOP</td><td>&#x3C;dist></td><td></td><td>dist=0..FFFF</td><td>Slow to a stop over specified distance</td></tr><tr><td>TRVL</td><td>&#x3C;dist>&#x3C;speed></td><td></td><td><p>dist=8000..7FFF </p><p>speed=1..7F or 1..FF</p></td><td>Travel a specified distance in a straight line, ramping up to a maximum specified speed</td></tr><tr><td>TURN</td><td>&#x3C;angle>&#x3C;speed></td><td></td><td><p>dist=8000..7FFF </p><p>speed=1..7F or 1..FF</p></td><td>Rotate in place by a specified angle, ramping up to a maximum specified speed</td></tr><tr><td>STOP</td><td>&#x3C;rate></td><td></td><td>rate=1..7F or 1..FF</td><td>Set rate of acceleration/deceleration</td></tr><tr><td>SPD</td><td></td><td>&#x3C;left>&#x3C;right></td><td>left/right=8000..7FFF</td><td>Get the current average speed (positions per second) for both wheels</td></tr><tr><td>HEAD</td><td></td><td>&#x3C;angle></td><td>angle=0..168 (decimal 0..359)</td><td>Get the current heading (in degrees) relative to start</td></tr><tr><td>DIST</td><td></td><td>&#x3C;left>&#x3C;right></td><td>left/right=80000000.. 7FFFFFFF</td><td>Get the current average speed (positions per second) for both wheels</td></tr><tr><td>RST</td><td></td><td></td><td></td><td>Reset the distance and heading values to 0</td></tr></tbody></table>

