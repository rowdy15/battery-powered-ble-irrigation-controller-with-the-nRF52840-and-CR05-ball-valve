# Battery Powered BLE Irrigation Controller Project Using the nRF52832

Battery powered irrigation project to interface with the ESP32 microcontroller (see [this](https://github.com/rowdy15/ESP32-as-the-Central-BLE-Gateway-for-irrigation-controller) project).

The aim of this project is to create a solution for smart watering of plants that can be turned on from your phone when not at home, but that the irrigation controller only talks to an MQTT broker (in my case via an ESP32) which is on the home network and so the irrigation controller is not directly exposed to the cloud (internet).

The folders and files contained within this repo should be placed in the {sdk root}/examples/ble_peripheral/{name of your project} folder of the nRF5 SDK folder (see below for download). e.g. I have mine in a folder like so:

/{nRF5 sdk root}/examples/ble_peripheral/battery-powered-irrigation-controller/{repo files and folders}

## In order to run this code you will need:

* Segger Embedded Studio (v4.12 is recommended because the UART Logging is known to work in this version.  All versions (up to 4.22) since haven't got this feature working yet. Not sure why.) ([see here](https://www.segger.com/downloads/embedded-studio/))
* nRF52832 microcontroller (see [here](https://www.aliexpress.com/item/4000049368888.html?spm=a2g0o.productlist.0.0.3c8316a3AqcRmq&algo_pvid=faa7c330-631b-47f1-8113-9c7cae6a8b03&algo_expid=faa7c330-631b-47f1-8113-9c7cae6a8b03-7&btsid=64071e58-2167-46e3-9be9-fc9df3a6b57e&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_55) or [here](https://www.aliexpress.com/item/32819293925.html?spm=a2g0o.productlist.0.0.3c8316a3AqcRmq&algo_pvid=faa7c330-631b-47f1-8113-9c7cae6a8b03&algo_expid=faa7c330-631b-47f1-8113-9c7cae6a8b03-3&btsid=64071e58-2167-46e3-9be9-fc9df3a6b57e&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_55)).
* nRF5 SDK ([see here](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK))
* Segger JTAG programmer ([see here](https://www.aliexpress.com/item/32868047097.html?spm=a2g0o.productlist.0.0.2b9f4654Zxgx5e&algo_pvid=9feb38c2-1adf-40f9-830f-0493f3195c1a&algo_expid=9feb38c2-1adf-40f9-830f-0493f3195c1a-1&btsid=d3b98b2c-8529-492c-8289-0554e1065193&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_55)) (depending on nRF52832 board actually... some have the cp2102 serial programmer chip onboard)

NOTE: only the second suggested nRF52832 requires the JTAG (J-Link) programmer actually. And if the JTAG programmer is not being recognised, you may need to install some software drivers so your operating system recognises the device ([see here](https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack)).

## Brief Outline of the code:

In addition to the Generic Access Profile (GAP) and the Generic Attribute Profile (GATT), there are three services, each containing one characteristic.

One of the services is a Battery service, providing the value of the battery as a percentage value (but in hexadecimal format). e.g. 0x64 corresponds to 100 (%).

Two of the services are custom services.  

The characteristic in the first service holds the value for if the tap is on or not (state of tap). A value of 0x00 means the tap is off. a value between 0x01 (1) and 0x3C (60) corresponds to the number of minutes that the tap will be stay turned on for. 

The characteristic in the second custom service holds the frequency with which the microcontroller will measure the battery level.  The code for battery measurement is in the main.c file (near the top after all the variable declarations).

The code is setup so that the nRF52832 (peripheral) pairs and stores bonding info for the first device that connects to it, when it is turned on and will only connect/reconnect this paired central device after this point.  This is reset every time button 1 (pin 14) is pressed for 1 second.  In the working solution, the ESP32 is the "central" device, however you can use any BLE enabled device (phone) to do testing.

Some of the logic for the incoming BLE commands are in the ble_cus.c (tap state) and ble_opt.c (time left) files.

so the main files to manipulate are:

* main.c
* ble_cus.c
* ble_cus.h
* ble_opt.c
* ble_opt.h
* sdk_config.h

if you want to change the pin assignments on your dev board that connect to the DC motor, then change the _TAP_1_ and _TAP_2_ variables in the ble_cus.c file.

## NOTE: to achieve truly low power in the device, you need to disable the following in the sdk_config.h file (by defining them as zero):

* \#define NRFX_UART_ENABLED 0
* \#define UART_ENABLED 0
* \#define NRF_LOG_BACKEND_UART_ENABLED 0

in the codes current state, these are disabled and the nrf_drv_uart.c driver file has been removed and any reference to NRF logging has been commented out so it will compile.

My initial testing seems to show that the chip is using 1.9μA when idle. :sunglasses::+1:

## Brief Outline of the Hardware setup:

If you buy an nRF52832 with an onboard programmer (e.g. you probably have an onboard programmer if you have a microUSB input on the board), then you don't need the Segger JTAG programmer.  However if you want to ensure that you are going to get low power usage, then getting a nRF52832 microcontroller without the onboard programmer might be the way to go. In which case you need to wire it up according to the following diagram where you wire up VCC, GND, SWDIO and SWCLK pins. NOTE: the programmer shown is a cortex SWD programmer but the pinout is exactly the same for the JTAG programmer (20 pin).

![alt text](https://github.com/rowdy15/battery-powered-ble-irrigation-controller-project-with-the-nRF52832/blob/master/BLE_programmingCircuit.jpg)

After programming the microcontroller you can setup the project like the below picture.  The DC motor driver can be purchased from [here](https://www.aliexpress.com/item/32973027142.html?spm=a2g0o.productlist.0.0.f19c2aa05FORu1&algo_pvid=de17072b-b9b5-4913-98e7-d3bace021786&algo_expid=de17072b-b9b5-4913-98e7-d3bace021786-0&btsid=2383cccd-ba69-4950-902d-104b5d5163a3&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_55)
The water timer I used in my project was from [here](https://www.aliexpress.com/item/33032587105.html?spm=a2g0o.productlist.0.0.38b93649F4PbR1&algo_pvid=4fa2c5e4-8a69-4fb9-85c2-7e92f5897566&algo_expid=4fa2c5e4-8a69-4fb9-85c2-7e92f5897566-6&btsid=ef032522-faf7-4365-9972-348be95f5d27&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_55).
However if I had to do it again (which I probably will) I will use a motorised ball valve such as [this](https://www.aliexpress.com/item/32803710399.html?spm=2114.12010615.8148356.70.5da75d4aithFna). Which would mean I would need to change the two AAA batteries to maybe two LiFePO<sub>4</sub> batteries.  And if I did this I would probably replace the two AA batteries to a single LiFePO<sub>4</sub> battery while I'm at it.

![alt text](https://github.com/rowdy15/battery-powered-ble-irrigation-controller-project-with-the-nRF52832/blob/master/final_BLE_Circuit.png)
