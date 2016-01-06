/*! @brief
 *  @file DJI_Pro_APP.cpp
 *  @version 1.0
 *
 *  @abstract
 *
 *  @attention
 *  Project configuration:
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for QT\STM32\ROS\Cmake
 *  -* @date Nov 11, 2015
 *  -* @author william.wu
 *
 *  -* @version V2.0
 *  -* DJI-onboard-SDK for QT-windows
 *  -* @date Sep 8, 2015
 *  -* @author wuyuwei
 * */

#include <string.h>
#include <stdio.h>
#include "DJI_App.h"
#include "DJI_API.h"

using namespace DJI::onboardSDK;

inline void passData(uint16_t flag, uint16_t enable, void *data, unsigned char *buf,
                     size_t datalen, size_t &offset)
{
    //! @todo new algorithm
    if ((flag & enable))
    {
        memcpy((unsigned char *)data, (unsigned char *)buf + offset, datalen);
        offset += datalen;
    }
}

unsigned char getCmdSet(Header *header)
{
    unsigned char *ptemp = ((unsigned char *)header) + sizeof(Header);
    return *ptemp;
}

unsigned char getCmdCode(Header *header)
{
    unsigned char *ptemp = ((unsigned char *)header) + sizeof(Header);
    ptemp++;
    return *ptemp;
}

BroadcastData DJI::onboardSDK::CoreAPI::getBroadcastData() const { return broadcastData; }

BatteryData DJI::onboardSDK::CoreAPI::getBatteryCapacity() const
{
    return broadcastData.capacity;
}

CtrlInfoData DJI::onboardSDK::CoreAPI::getCtrlInfo() const { return broadcastData.ctrl_info; }

void DJI::onboardSDK::CoreAPI::broadcast(Header *header)
{
    unsigned char *pdata = ((unsigned char *)header) + sizeof(Header);
    unsigned short *enableFlag;
    driver->lockMSG();
    pdata += 2;
    enableFlag = (unsigned short *)pdata;
    broadcastData.dataFlag = *enableFlag;
    size_t len = MSG_ENABLE_FLAG_LEN;

    //! @todo better algorithm
    passData(*enableFlag, HAS_TIME, &broadcastData.timeStamp, pdata, sizeof(TimeStampData),
             len);
    passData(*enableFlag, HAS_Q, &broadcastData.q, pdata, sizeof(QuaternionData), len);
    passData(*enableFlag, HAS_A, &broadcastData.a, pdata, sizeof(CommonData), len);
    passData(*enableFlag, HAS_V, &broadcastData.v, pdata, sizeof(VelocityData), len);
    passData(*enableFlag, HAS_W, &broadcastData.w, pdata, sizeof(CommonData), len);
    passData(*enableFlag, HAS_POS, &broadcastData.pos, pdata, sizeof(PossitionData), len);
    passData(*enableFlag, HAS_MAG, &broadcastData.mag, pdata, sizeof(MagnetData), len);
    passData(*enableFlag, HAS_RC, &broadcastData.rc, pdata, sizeof(RadioData), len);
    passData(*enableFlag, HAS_GIMBAL, &broadcastData.gimbal, pdata, sizeof(GimbalData), len);
    passData(*enableFlag, HAS_STATUS, &broadcastData.status, pdata, sizeof(uint8_t), len);
    passData(*enableFlag, HAS_BATTERY, &broadcastData.capacity, pdata, sizeof(BatteryData),
             len);
    passData(*enableFlag, HAS_DEVICE, &broadcastData.ctrl_info, pdata, sizeof(CtrlInfoData),
             len);

    driver->freeMSG();

    if (broadcastCallback.callback)
        broadcastCallback.callback(this, header, broadcastCallback.userData);
}

void DJI::onboardSDK::CoreAPI::recvReqData(Header *header)
{
    unsigned char buf[100] = { 0, 0 };

    uint8_t ack = *((unsigned char *)header + sizeof(Header) + 2);
    if (getCmdSet(header) == SET_BROADCAST)
    {
        switch (getCmdCode(header))
        {
            case CODE_BROADCAST:
                broadcast(header);
                break;
            case CODE_FROMMOBILE:
                if (fromMobileCallback.callback)
                {
                    API_LOG(driver, STATUS_LOG, "Recevie data from mobile\n")
                    fromMobileCallback.callback(this, header, fromMobileCallback.userData);
                }
                break;
            case CODE_LOSTCTRL:
                API_LOG(driver, STATUS_LOG, "onboardSDK lost contrl\n");
                Ack param;
                if (header->sessionID > 0)
                {
                    buf[0] = buf[1] = 0;
                    param.session_id = header->sessionID;
                    param.seq_num = header->sequence_number;
                    param.need_encrypt = header->enc_type;
                    param.buf = buf;
                    param.length = 2;
                    ackInterface(&param);
                }
                break;
            case CODE_MISSION:
                //! @todo add mission session decode
                switch (ack)
                {
                    case MISSION_MODE_A:
                        break;
                    case MISSION_WAYPOINT:
                        if (wayPointData)
                        {
                            if (wayPointCallback.callback)
                                wayPointCallback.callback(this, header,
                                                          wayPointCallback.userData);
                            else
                                API_LOG(driver, STATUS_LOG, "Mode waypoint \n");
                        }
                        break;
                    case MISSION_HOTPOINT:
                        if (hotPointData)
                        {
                            if (hotPointCallback.callback)
                                hotPointCallback.callback(this, header,
                                                          hotPointCallback.userData);
                            else
                                API_LOG(driver, STATUS_LOG, "Mode HP \n");
                        }
                        break;
                    case MISSION_FOLLOW:
                        if (followData)
                        {
                            if (followCallback.callback)
                                followCallback.callback(this, header, followCallback.userData);
                            else
                                API_LOG(driver, STATUS_LOG, "Mode Follow \n");
                        }
                        break;
                    case MISSION_IOC:
                        API_LOG(driver, STATUS_LOG, "Mode IOC \n");
                        break;
                    default:
                        API_LOG(driver, ERROR_LOG, "unkown mission code 0x%X \n", ack);
                        break;
                }
                break;
            case CODE_WAYPOINT:
                //! @todo add waypoint session decode
                break;
            default:
                API_LOG(driver, STATUS_LOG, "error, unknown BROADCAST command code\n");
                break;
        }
    }
    else
        API_LOG(driver, DEBUG_LOG, "receive unknown command\n");
    if (recvCallback.callback)
        recvCallback.callback(this, header, recvCallback.userData);
}

void CoreAPI::setFromMobileCallback(CallBackHandler FromMobileEntrance)
{
    fromMobileCallback = FromMobileEntrance;
}

void CoreAPI::setBroadcastCallback(CallBack handler, UserData userData)
{
    broadcastCallback.callback = handler;
    broadcastCallback.userData = userData;
}
