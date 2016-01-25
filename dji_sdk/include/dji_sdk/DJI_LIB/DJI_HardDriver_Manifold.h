#ifndef __DJI_HARDDRIVER_MANIFOLD_H__
#define __DJI_HARDDRIVER_MANIFOLD_H__


#include <stdio.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include "lib/inc/DJI_Type.h"
#include "lib/inc/DJI_HardDriver.h"

namespace DJI {

namespace onboardSDK {

class HardDriver_Manifold : public HardDriver {

    public:
        HardDriver_Manifold(std::string device, unsigned int baudrate) {
            m_device = device;
            m_baudrate = baudrate;
            m_memLock = PTHREAD_MUTEX_INITIALIZER;
            m_msgLock = PTHREAD_MUTEX_INITIALIZER;
        }


        ~HardDriver_Manifold() {
            _serialClose();
        }


        void init() {
            API_LOG(this, STATUS_LOG, "going to open device %s with baudrate %u...\n", 
                    m_device.c_str(), m_baudrate);
            if( _serialStart(m_device.c_str(), m_baudrate) < 0 ) {
                _serialClose();
                API_LOG(this, ERROR_LOG, "...fail to start serial\n");
            } else {
                API_LOG(this, STATUS_LOG, "...succeed to start serial\n");
            }
        }


        /**
         * @brief Implement a USB hand-shaking protocol for SDK
         */
        void usbHandshake(std::string device) {
            _serialStart(device.c_str(), 38400);
            _serialStart(device.c_str(), 19200);
            _serialStart(device.c_str(), 38400);
            _serialStart(device.c_str(), 19200);
        }


        void setBaudrate(unsigned int baudrate) {
            m_baudrate = baudrate;
        }


        void setDevice(std::string device) {
            m_device = device;
        }


        time_t getTimeStamp() {
            return (unsigned int)time(NULL);
        }


        size_t send(const uint8_t *buf, size_t len) {
            return _serialWrite(buf, len);
        }


        size_t readall(uint8_t *buf, size_t maxlen) {
            return _serialRead(buf, maxlen);
        }


        void lockMemory() {
            pthread_mutex_lock(&m_memLock);
        }


        void freeMemory() {
            pthread_mutex_unlock(&m_memLock);
        }


        void lockMSG() {
            pthread_mutex_lock(&m_msgLock);
        }


        void freeMSG() {
            pthread_mutex_unlock(&m_msgLock);
        }


    private:
        std::string m_device;
        unsigned int m_baudrate;
        pthread_mutex_t m_memLock;
        pthread_mutex_t m_msgLock;

        int m_serial_fd;
        fd_set m_serial_fd_set;


        bool _serialOpen(const char* dev) {
            m_serial_fd = open(dev, O_RDWR | O_NOCTTY);
            if(m_serial_fd < 0) {
                API_LOG(this, ERROR_LOG, "cannot open device %s\n", dev);
                return false;
            }
            return true;
        }


        bool _serialClose() {
            close(m_serial_fd);
            m_serial_fd = -1;
            return true;
        }


        bool _serialFlush() {
            if(m_serial_fd < 0) {
                API_LOG(this, ERROR_LOG, "flushing fail because no device is opened\n");
                return false;
            } else {
                tcflush(m_serial_fd, TCIFLUSH);
                return true;
            }
        }


        bool _serialConfig(int baudrate, char data_bits, char parity_bits, char stop_bits) {
            int st_baud[] = {
                B4800,
                B9600,
                B19200,
                B38400,
                B57600,
                B115200,
                B230400
            };
            int std_rate[] = {
                4800,
                9600,
                19200,
                38400,
                57600,
                115200,
                230400,
                1000000,
                1152000,
                3000000,
            };

            int i,j;
            struct termios newtio, oldtio;
            /* save current port parameter */
            if (tcgetattr(m_serial_fd, &oldtio) != 0) {
                API_LOG(this, ERROR_LOG, "fail to save current port\n");
                return false;
            }
            memset(&newtio, 0, sizeof(newtio));

            /* config the size of char */
            newtio.c_cflag |= CLOCAL | CREAD;
            newtio.c_cflag &= ~CSIZE;

            /* config data bit */
            switch (data_bits) {
                case 7:
                    newtio.c_cflag |= CS7;
                    break;
                case 8:
                    newtio.c_cflag |= CS8;
                    break;
            }
            /* config the parity bit */
            switch (parity_bits) {
                /* odd */
            case 'O':
            case 'o':
                newtio.c_cflag |= PARENB;
                newtio.c_cflag |= PARODD;
                break;
                /* even */
            case 'E':
            case 'e':
                newtio.c_cflag |= PARENB;
                newtio.c_cflag &= ~PARODD;
                break;
                /* none */
            case 'N':
            case 'n':
                newtio.c_cflag &= ~PARENB;
                break;
            }
            /* config baudrate */
            j = sizeof(std_rate) / 4;
            for(i = 0; i < j; ++i) {
                if(std_rate[i] == baudrate) {
                    /* set standard baudrate */
                    cfsetispeed(&newtio, st_baud[i]);
                    cfsetospeed(&newtio, st_baud[i]);
                    break;
                }
            }
            /* config stop bit */
            if( stop_bits == 1 )
                newtio.c_cflag &=  ~CSTOPB;
            else if ( stop_bits == 2 )
                newtio.c_cflag |=  CSTOPB;

            /* config waiting time & min number of char */
            newtio.c_cc[VTIME]  = 1;
            newtio.c_cc[VMIN] = 1;

            /* using the raw data mode */
            newtio.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);
            newtio.c_oflag  &= ~OPOST;

            /* flush the hardware fifo */
            tcflush(m_serial_fd,TCIFLUSH);

            /* activite the configuration */
            if((tcsetattr(m_serial_fd,TCSANOW,&newtio))!=0) {
                API_LOG(this, ERROR_LOG, "fail to active configuration\n");
                return false;
            }
            return true;
        }


        int _serialStart(const char *dev_name, int baud_rate) {
            const char *ptemp;
            if(dev_name == NULL) {
                ptemp = "/dev/ttyUSB0";
            } else {
                ptemp = dev_name;
            }
            if(true == _serialOpen(ptemp) 
                    && true == _serialConfig(baud_rate,8,'N',1)) {

                FD_ZERO(&m_serial_fd_set);
                FD_SET(m_serial_fd, &m_serial_fd_set);
                return m_serial_fd;

            }
            return -1;
        }


        int _serialWrite(const unsigned char *buf, int len) {
            return write(m_serial_fd, buf, len);
        }


        int _serialRead(unsigned char *buf, int len) {
            int saved = 0;
            int ret = -1;

            if(NULL == buf) {
                return -1;
            } else {
                for(; saved < len ;) {
                    ret = read(m_serial_fd,buf + saved,len - saved);
                    if(ret > 0)
                        saved += ret;
                    else
                        break;
                }
                return saved;
            }
        }


};

}

}


#endif
