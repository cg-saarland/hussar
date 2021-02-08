#ifndef AWRLib_
#define AWRLib_

#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <iostream>
#include <functional>
#include <memory>

#include <fcntl.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdarg>
#define inet_aton(addr, buf) inet_pton(AF_INET, addr, buf)
static inline int ioctl(int socket, unsigned long request, ...) {
    va_list arg;
    va_start(arg, request);
    unsigned long* mode = va_arg(arg, unsigned long*);
    int res = ioctlsocket(socket, request, mode);
    va_end(arg);
    return res;
}
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <net/if.h>
#endif

#include <radar/radar.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"

namespace awrcapture {

struct DeviceFamily {
    enum Enum {
        AWR1243 = 1,
        AWR1642 = 2
    };
};

/**
 * @brief Represents a binary communication with the AWR device over Ethernet.
 * 
 * As per Texas Instrument's DCA1000EVM documentation, this assumes that the
 * host has IP address '192.168.33.30' and the capture device has IP address
 * '192.168.33.180'. The port and buffer size that is used depends on the type
 * of communication channel (control or data), which are implemented as subclasses
 * of this class.
 * 
 * @see ControlPort
 * @see DataPort
 */
template<size_t BufferSize>
class Channel {
protected:
    int sock;
    sockaddr_in addrRadar;
    
    Channel(int port) {
        sock = socket(AF_INET, SOCK_DGRAM, 0);

        sockaddr_in addrListen;
        inet_aton("192.168.33.30", &addrListen.sin_addr);
        addrListen.sin_port = htons(port);
        addrListen.sin_family = AF_INET;
        
        int result = bind(sock, (sockaddr *)&addrListen, sizeof(addrListen));
        _assert(result != -1, "could not bind socket");
        
        inet_aton("192.168.33.180", &addrRadar.sin_addr);
        addrRadar.sin_port = htons(port);
        addrRadar.sin_family = AF_INET;
    }
    
    /// The buffer used for reading and writing packets.
    char buffer[BufferSize];
    /// Points to the first unused byte in the buffer.
    char *head;
    
    void _assert(bool v, const std::string &msg) {
        if (!v) {
            std::cerr << msg << std::endl;
            std::cerr << strerror(errno) << std::endl;
            //assert(false);
        }
    }
    
    /// Returns the amount of used bytes in the buffer.
    size_t pos() const {
        return head - buffer;
    }
    
    /**
     * @brief Appends data of arbitrary type to the buffer.
     * 
     * @warning This does not check for buffer overflows.
     * @warning This assumes both sides of the channel agree on the in-memory representation of
     *          the data type T. As long as only integer types are sent from a little-endian machine,
     *          this will not constitute a problem.
     */
    template<typename T>
    void write(T data) {
        *(T *)head = data;
        head += sizeof(T);
    }
    
    /**
     * @brief Reads data of arbitrary type from the buffer.
     * 
     * @warning This does not check for buffer overflows.
     * @warning This assumes both sides of the channel agree on the in-memory representation of
     *          the data type T. As long as only integer types are sent from a little-endian machine,
     *          this will not constitute a problem.
     */
    template<typename T>
    T read() {
        auto data = *(T *)head;
        head += sizeof(T);
        return data;
    }
    
    /**
     * @brief Receives a packet from the network.
     * 
     * On success, 'head' is set to the beginning of the packet.
     * 
     * @param required Signifies that the presence of a packet is expected (e.g. as response to a command) and that
     *                 an error should be thrown if no packet is available.
     * 
     * @note The channel is configured to be non-blocking, i.e. this function will not wait until a packet is available.
     * 
     * @returns The length of the packet, or -1 if no packet was available.
     */
    ssize_t receive(bool required = true) {
        struct sockaddr_storage src_addr;
        socklen_t src_addr_len = sizeof(src_addr);
        ssize_t recv = (int)recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&src_addr, &src_addr_len);

        if (required)
            _assert(recv > 0, "packet could not be received");
        
        head = buffer;
        return recv;
    }
};

/**
 * @brief Represents the control port that is used to issue commands on the FPGA of AWR devices.
 */
class ControlPort : public Channel<512> {
public:
    struct FPGAVersion {
        uint8_t major, minor;
        uint8_t recordBit;
        
        FPGAVersion(uint16_t v) {
            major     = (v & ((1<< 7)-1))>> 0;
            minor     = (v & ((1<<14)-1))>> 7;
            recordBit = (v & ((1<<15)-1))>>14;
        }
        
        std::string str() {
            return
                std::to_string(major)
                + "."
                + std::to_string(minor)
                + (recordBit == 0 ? " [RECORD]" : "")
            ;
        }
    };

    ControlPort(int port = 4096) : Channel(port) {}
    
    void resetFPGA() {
        startPacket(RESET_FPGA_CMD_CODE);
        endPacket();
    }
    
    FPGAVersion readFPGAVersion() {
        startPacket(READ_FPGA_VERSION_CMD_CODE);
        endPacket(false);
        
        return FPGAVersion(read<uint16_t>());
    }
    
    void systemConnect() {
        startPacket(SYSTEM_CONNECT_CMD_CODE);
        endPacket();
    }
    
    void configFPGAGen(DeviceFamily::Enum device) {
        startPacket(CONFIG_FPGA_GEN_CMD_CODE);
        write<uint8_t>(1); // 1- raw mode, 2- multi mode
        write<uint8_t>(device); // 1- AR1243, 2- AR1642
        write<uint8_t>(1); // 1- LVDS capture, 2- DMM playback
        write<uint8_t>(2); // 1- SD card, 2- Ethernet
        write<uint8_t>(3); // 1- 12bit, 2- 14bit, 3- 16bit
        write<uint8_t>(30); // timer info in seconds /// @todo what is this?
        endPacket();
    }
    
    void configPacketData() {
        startPacket(CONFIG_PACKET_DATA_CMD_CODE);
        write<uint16_t>(1472); // packet size, 48-1472
        write<uint16_t>(3125); // ethernet delay, 625-62500; usec*1000/8
        endPacket();
    }
    
    void recordStart() {
        startPacket(RECORD_START_CMD_CODE);
        endPacket();
    }
    
    void recordStop() {
        startPacket(RECORD_STOP_CMD_CODE);
        endPacket();
    }
    
private:
    static const uint16_t HEADER = 0xA55A;
    static const uint16_t FOOTER = 0xEEAA;
    
    enum Command {
        RESET_FPGA_CMD_CODE = 0x01,
        RESET_AR_DEV_CMD_CODE = 0x02,
        CONFIG_FPGA_GEN_CMD_CODE = 0x03,
        CONFIG_EEPROM_CMD_CODE = 0x04,
        RECORD_START_CMD_CODE = 0x05,
        RECORD_STOP_CMD_CODE = 0x06,
        PLAYBACK_START_CMD_CODE = 0x07,
        PLAYBACK_STOP_CMD_CODE = 0x08,
        SYSTEM_CONNECT_CMD_CODE = 0x09,
        SYSTEM_ERROR_CMD_CODE = 0x0A,
        CONFIG_PACKET_DATA_CMD_CODE = 0x0B,
        CONFIG_DATA_MODE_AR_DEV_CMD_CODE = 0x0C,
        INIT_FPGA_PLAYBACK_CMD_CODE = 0x0D,
        READ_FPGA_VERSION_CMD_CODE = 0x0E
    };
    
    void startPacket(Command cmd) {
        head = buffer;
        
        write<uint16_t>(HEADER);
        write<uint16_t>(cmd);
        
        write<uint16_t>(0); // placeholder for data size
    }
    
    void endPacket(bool checkSuccess = true) {
        uint16_t cmd = *(uint16_t *)(buffer+2);
        
        // insert data size
        uint16_t dataSize = pos() - 6;
        *(uint16_t *)(buffer+4) = dataSize;
        
        // insert footer
        write<uint16_t>(0xEEAA);
        
        // send packet
        ssize_t len = head - buffer;
        ssize_t sent = sendto(sock, buffer, len, 0, (sockaddr *)&addrRadar, sizeof(addrRadar));
        _assert(sent == len, "packet could not be sent");
        
        // receive response
        
        receive();
        
        _assert(read<uint16_t>() == HEADER, "header expected in response packet");
        _assert(read<uint16_t>() == cmd, "expected response cmd to match request cmd");
        
        if (checkSuccess) {
            _assert(read<uint16_t>() == 0, "board responded failure");
        }
    }
};

/**
 * @brief Represents the data port that is used to receive captured radar data.
 *
 * @param Allocator The allocator used to allocate and release the data storage of the radar Frame.
 * This is required to create radar frames that reside in CPU/GPU unified memory.
 */
 template<typename Allocator>
class DataPort : public Channel<1472> {
public:
    using FrameCallback = std::function<void (radar::Frame<Allocator> &)>;

    /// Signifies that frames should first be transformed into frequency space before being handed to the user.
    bool performFFT = true;
    
    DataPort(int port = 4098) : Channel(port) {
        int opt = 1;
        ioctl(sock, FIONBIO, &opt);
    }

    void configureFrame(const radar::FrameConfig &config) {
        frameBuffer.configure(config, 16);
    }
    
    void poll(FrameCallback cb) {
        ssize_t psize;
        while ((psize = receive(false)) > 0) {
            auto seqNo = read<uint32_t>();
            auto bytes = read<uint32_t>() | (uint64_t(read<uint16_t>()) << 32);
            
            if (seqNo != lastSeqNo + 1) {
                std::cerr << "dropped packets (seq no jumped from " << lastSeqNo << " to " << seqNo << ")" << std::endl;
            }
            
            lastSeqNo = seqNo;
            
            frameBuffer.append(head, psize - pos(), cb, performFFT);
        }
    }

private:
    uint32_t lastSeqNo = 0;
    
    struct FrameBuffer {
    protected:
        size_t idx = 0, size = 0;
        std::unique_ptr<char> buffer;
        float scale = 1.f;
        
        radar::Frame<Allocator> frame;
        
    public:
        void configure(const radar::FrameConfig &frameConfig, int bitDepth) {
            frame.configure(frameConfig);
            scale = 1 << (bitDepth-1);
            
            auto samples = frame.sampleCount();
            size = samples * sizeof(uint16_t) * 2; /// times two since we have a real and an imaginary part
            
            idx = 0;
            buffer.reset(new char[size]);
        }
        
        void append(const char *data, ssize_t len, FrameCallback cb, bool performFFT) {
            while (len > 0) {
                ssize_t available = std::min<ssize_t>(len, size - idx);
                memcpy(buffer.get()+idx, data, available);
                
                idx += available;
                data += available;
                len -= available;
                
                if (idx >= size) {
                    process(performFFT);
                    cb(frame);
                    
                    idx = 0;
                }
            }
        }
        
    protected:
        void process(bool performFFT) {
            auto samples = frame.sampleCount();
            int cc = frame.config().channelCount;
            
            for (size_t s = 0; s < samples; s += cc) {
                for (int c = 0; c < cc; ++c) {
                    for (int comp = 0; comp < 2; ++comp) {
                        float v = ((int16_t *)buffer.get())[s*2 + c + comp * cc] / scale;
                        ((float *)(&frame(s+c)))[comp] = v;
                    }
                }
            }
            
            if (performFFT) {
                frame.fft();
                
                float dftScale = std::sqrt(scale);
                for (size_t s = 0; s < samples; ++s) {
                    frame(s) /= dftScale;
                }
            }
        }
    } frameBuffer;
};

}

#pragma GCC diagnostic pop
#endif
