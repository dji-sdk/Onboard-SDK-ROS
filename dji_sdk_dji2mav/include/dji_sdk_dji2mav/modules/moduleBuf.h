/*****************************************************************************
 * @Brief     Module buf manager of the mavlink module. ROS-free and mav-free
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/12/06
 * @Modified  2015/12/29
 *****************************************************************************/

#ifndef _MAV2DJI_MODULEBUF_H_
#define _MAV2DJI_MODULEBUF_H_


#include <new>
#include <string>
#include <mutex>

namespace dji2mav{

    class ModuleBuf {
        public:
            ModuleBuf(uint16_t bufSize) {
                DJI2MAV_DEBUG("Going to construct ModuleBuf with bufSize " 
                        "%u...", bufSize);

                m_bufSize = bufSize;
                m_head = 0;
                m_bufUsedAmount = 0;

                try {
                    m_buf = new uint8_t[m_bufSize];
                    memset(m_buf, 0, sizeof(uint8_t) * m_bufSize);
                } catch(std::bad_alloc& m) {
                    DJI2MAV_FATAL( "Failed to alloc memory for moduleBuf! "
                            "Exception: %s!", m.what() );
                    exit(EXIT_FAILURE);
                }

                DJI2MAV_DEBUG("...finish constructing ModuleBuf.");

            }


            ~ModuleBuf() {
                if(NULL != m_buf) {
                    delete []m_buf;
                    m_buf = NULL;
                }
            }


            /**
             * @brief  Get the max size of buffer
             * @return The max size of buffer
             */
            inline uint16_t getBufSize() {
                return m_bufSize;
            }


            /**
             * @brief  Get the used amount of buffer
             * @return The used amount of buffer
             */
            inline uint16_t getBufUsedAmount() {
                return m_bufUsedAmount;
            }


            /**
             * @brief   Write data to the buffer. Multi-thread supported
             * @param   src : The pointer to the source memory
             * @param   len : The length that is going to write to the buffer
             * @return  True if succeed or false if fail
             */
            bool writeBuf(const uint8_t *src, uint16_t len) {

                if( m_bufUsedAmount + len > m_bufSize ) {
                    return false;
                } else {
                    m_bufMutex.lock();
                    uint16_t tail = (m_head + m_bufUsedAmount) % m_bufSize;
                    if(len + tail < m_bufSize) {
                        memcpy(m_buf + tail, src, len);
                    } else {
                        uint16_t bSize = m_bufSize - tail;
                        memcpy(m_buf + tail, src, bSize);
                        memcpy(m_buf, src + bSize, len - bSize);
                    }
                    m_bufUsedAmount += len;
                    m_bufMutex.unlock();
                    return true;
                }

            }


            /**
             * @brief  Read data from the buffer. Multi-thread supported
             * @param  dest : The pointer to the destination memory
             * @param  len  : The length that is going to read from the buffer
             * @return True if succeed or false if fail
             */
            bool readBuf(uint8_t *dest, uint16_t len) {

                if( len > m_bufUsedAmount ) {
                    return false;
                } else {
                    m_bufMutex.lock();
                    if(m_head + len < m_bufSize) {
                        memcpy(dest, m_buf + m_head, len);
                        m_head += len;
                    } else {
                        memcpy(dest, m_buf + m_head, m_bufSize - m_head);
                        memcpy(dest + m_bufSize - m_head, m_buf, 
                                len + m_head - m_bufSize);
                        m_head -= m_bufSize - len;
                    }
                    m_bufUsedAmount -= len;
                    m_bufMutex.unlock();
                    return true;
                }
            }


            /*
             * @brief Display the buffer contents. For debug use
             */
            void display() {
                DJI2MAV_INFO("In ModuleBuf head: %u, used: %u, buf: ", 
                        m_head, m_bufUsedAmount);
                for(int i = 0; i < m_bufSize; ++i) {
                    printf("%02x", m_buf[i]);
                }
                printf("\n");
                DJI2MAV_INFO("--- End of display ---");
            }


            /**
             * @brief Clear the buffer. Set used amount 0 but not head pointer
             */
            void clear() {
                DJI2MAV_DEBUG("Going to clear the buffer with used amount "
                        "%u...", m_bufUsedAmount);
                m_bufMutex.lock();
                m_bufUsedAmount = 0;
                m_bufMutex.unlock();
                DJI2MAV_DEBUG("...finish clearing the buffer. Head point to "
                        "%u.", m_head);
            }


        private:
            uint8_t *m_buf;
            uint16_t m_bufSize;
            uint16_t m_head;
            uint16_t m_bufUsedAmount;
            std::mutex m_bufMutex;
    };

}

#endif
