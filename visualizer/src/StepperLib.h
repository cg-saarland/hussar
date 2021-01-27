#ifndef StepperLib_h
#define StepperLib_h

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cerrno>
#include <cstring>

#ifdef _WIN32
#include <termiWin.h>
#define IXANY 0
#define O_NOCTTY 0
#define O_NDELAY 0
#else
#define readFromSerial read
#define writeToSerial write
#define selectSerial select
#define openSerial open
#define closeSerial close
#define ioctlSerial ioctl
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

template<int S>
class StepperLib {
public:
    static constexpr int STEPPER_COUNT = S;
    
    enum {
      CMD_CALIBRATE = 0x00,
      CMD_TARGET    = 0x01,
      CMD_SPEED     = 0x02,

      CMD_STATUS    = 0x80,
      CMD_ERROR     = 0xFF
    };
    
    struct Stepper {
        int id;
        int target = 0;
        int state = 0;
        
        StepperLib *owner;
        
        void sendTarget() {
            owner->send(CMD_TARGET, id, target);
        }
        
        void calibrate() {
            target = 0;
            state = 0;
            
            owner->send(CMD_CALIBRATE, id, target);
        }
    } steppers[STEPPER_COUNT];

    int fd = -1;
    char buffer[4];

    StepperLib() {
        for (int i = 0; i < STEPPER_COUNT; ++i) {
            steppers[i].id = i;
            steppers[i].owner = this;
        }
    }
    
    void connect(const std::string &portname) {
        fd = openSerial(portname.c_str(), O_RDWR | O_NOCTTY);
        if (fd < 0) {
            printf("error %d opening %s: %s\n", errno, portname.c_str(), strerror(errno));
            exit(-1);
        }
    
        set_interface_attribs(fd, B9600);
    }
    
    void send(uint8_t cmd, uint8_t id, int16_t data) {
        if (!connected())
            return;
        
        buffer[0] = cmd;
        buffer[1] = id;
        ((int16_t *)buffer)[1] = data;
        
        if (writeToSerial(fd, buffer, 4) < 4) {
            printf("error %d writing data: %s\n", errno, strerror(errno));
            exit(-1);
        }
    }
    
    bool connected() const {
        return fd >= 0;
    }
    
    void poll() {
        if (!connected())
            return;
        
        while (true) {
            int count;
            ioctlSerial(fd, FIONREAD, &count);
            if (count < 4)
                break;
            
            ssize_t i = readFromSerial(fd, buffer, 4);
            if (i != 4) {
                printf("incomplete read from serial port!\n");
                return;
            }
            
            uint8_t cmd = buffer[0];
            uint8_t id = buffer[1];
            int16_t data = ((int16_t *)buffer)[1];
            
            switch (cmd) {
            case CMD_STATUS:
                steppers[id].state = data / 5;
                //printf(".");
                break;
            default:
                readFromSerial(fd, buffer, 1); // jitter alignment hoping we come back in sync
                ;//printf("received: %d\n", cmd);
            }
        }
    }
    
private:
    int set_interface_attribs(int fd, int speed) {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            printf("error %d from tcgetattr", errno);
            return -1;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
                                         // enable reading
        tty.c_cflag &= ~PARENB;          // odd parity
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            printf("error %d from tcsetattr", errno);
            return -1;
        }
        return 0;
    }
};

#endif /* StepperLib_hpp */
