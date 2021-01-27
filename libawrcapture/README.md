# libawrcapture
This tiny header-only library contains code to interface with Texas Instruments' [DCA1000EVM](https://www.ti.com/lit/ug/spruij4a/spruij4a.pdf) capture card. It can be used to interface with and capture data from the `xWR1xxx` family of Radar sensors. Data will be streamed over Ethernet and requires proper setup of the network adapter connected to the DCA1000EVM. For further information, consult Texas Instruments' [documentation](https://www.ti.com/lit/ug/spruij4a/spruij4a.pdf).

## Device setup
We currently only support 16-bit sample mode. Please make sure that the following switches are toggled on your DCA1000EVM:
* `12BIT_OFF`
* `14BIT_OFF`
* `16BIT_ON`
* `SW_CONFIG`

Note that this library has only been tested with `AWR1243` devices so far.

## Usage

```c++
#include <radar/radar.h>
#include <radar/units.h>
#include <awrcapture/awrcapture.h>

// A control port is required to send commands to the FPGA.
awrcapture::ControlPort controlPort;

// A data port enables us to receive Radar frames from the sensor.
awrcapture::DataPort dataPort;

int main() {
    // We can query the FPGA version like this:
    std::cout
        << "FPGA version: "
        << controlPort.readFPGAVersion().str()
        << std::endl;
    
    // Inform the DCA1000EVM that we want to establish a connection.
    controlPort.systemConnect();

    // Instruct the DCA1000EVM to configure the FPGA. In particular:
    // 1) Enable raw mode instead of multi mode
    // 2) Choose the correct device family (here: AWR1243)
    // 3) Use LVDS capture to obtain data
    // 4) Stream data over Ethernet instead of storing on SD card
    // 5) Enable 16-bit sample resolution
    controlPort.configFPGAGen(awrcapture::DeviceFamily::AWR1243);

    // Configures settings required for the data port.
    controlPort.configPacketData();

    // Instructs the DCA1000EVM to start capturing radar data.
    controlPort.recordStart();

    // Configure the dimensions of Radar frames.
    radar::FrameConfig frameConfig;
    frameConfig.chirpCount      = 128;
    frameConfig.samplesPerChirp = 256;
    frameConfig.channelCount    = 4;
    
    dataPort.configureFrame(frameConfig);

    // Start application loop
    while (true) {
        // Do some fancy event loop for your application here.

        // Check for new data
        dataPort.poll([](radar::Frame<> &frame) {
            // This callback will be called once a frame is available.

            // Do whatever you wish with the radar frame here!
            // (e.g. visualize it, process it or just store it on disk)
        });
    }
}
```

## Installation
This library uses CMake as build system and requires a C++11 capable compiler. It can be used as a dependency of other libraries and applications by using the CMake `ADD_SUBDIRECTORY` command. This library has no dependencies. Since the underlying socket APIs vary between platforms, we can not guarantee that this library will compile on all platforms. It has only been tested on macOS 10.15 and Ubuntu 18 so far, there is no support for Windows yet.
