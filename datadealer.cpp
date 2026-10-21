// Copyright (C) 2012 Denis Shienkov <denis.shienkov@gmail.com>
// Copyright (C) 2012 Laszlo Papp <lpapp@kde.org>
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "datadealer.h"
#include <qdebug.h>

DataDealer::DataDealer()
{
    m_angle = 0;
    m_velocity = 0;
    m_current = 0;
    m_poleNum = 14;
    pointcount = 0;
    m_angleVec.reserve(200);
    m_velocityVec.reserve(200);
    m_currentVec.reserve(200);
}

void DataDealer::CalculateData(QByteArray& data,double circleNum,double velFullScale,double currentScale)
{


    if(!isCanFD){
        m_id = data[0];
        int pos_h =  (int)(data[1]);
        int pos_l = (int)(data[2]);
        if(pos_h<0){
            pos_h+=256;
        }
        if(pos_l<0){
            pos_l+=256;
        }
        m_angle = (((pos_h*256.0 + pos_l) - 32768.0)/32768.0)*360*circleNum;


        char vel_h = data[3];
        int vel_h_high = (vel_h&0xf0)>>4;
        int vel_h_low = vel_h&0x0f;
        int vel_l = (data[4]&0xf0)>>4;
        //qDebug()<<vel_h_high<<vel_h_low<<vel_l;
        m_velocity = ((double(vel_h_high*16*16+ vel_h_low*16+ vel_l) - 2048.0)/2048.0)*velFullScale;

        int cur_h = (data[4]&0x0f)<<8;
        int cur_l = data[5];
        if(cur_l<0){
            cur_l+=256;
        }
        m_current = ((double(cur_h + cur_l) - 2048.0)/2048.0)*currentScale;
    }
    else{
        int64_t pos = 0;
        if(data[0]>>7==0){

            uint64_t fe = (0x00000000ffffffff&(uint64_t)(data[1])<<24);
//            int64_t t = ((int64_t)(0x000000)<<40)|((int64_t)(data[0])<<32)|((unsigned char)(data[1])<<24)|((unsigned char)(data[2])<<16)|((unsigned char)(data[3])<<8);
            pos = ((int64_t)(0x000000)<<40)|
                        ((0x000000ffffffffff&(uint64_t)(data[0])<<32))|
                        ((0x00000000ffffffff&(uint64_t)(data[1])<<24))|
                        ((0x0000000000ffffff&(uint64_t)(data[2])<<16))|
                        ((0x000000000000ffff&(uint64_t)(data[3])<<8))|
                        ((0x00000000000000ff&(uint64_t)(data[4])));

        }else{
//            pos = ((int64_t)(0xFFFFFF)<<40)|
//                        ((uint64_t)(data[0])<<32)|
//                        ((unsigned char)(data[1])<<24)|
//                        ((unsigned char)(data[2])<<16)|
//                        ((unsigned char)(data[3])<<8)|
//                        ((unsigned char)(data[4]));
            pos = ((int64_t)(0xFFFFFF)<<40)|
                    ((0x000000ffffffffff&(uint64_t)(data[0])<<32))|
                    ((0x00000000ffffffff&(uint64_t)(data[1])<<24))|
                    ((0x0000000000ffffff&(uint64_t)(data[2])<<16))|
                    ((0x000000000000ffff&(uint64_t)(data[3])<<8))|
                    ((0x00000000000000ff&(uint64_t)(data[4])));


        }


        m_singlePosition = pos%1048576;
        m_circleNum = pos/1048576;
        m_angle = pos/1048576.;

        int32_t vel = 0;
        if(data[5]>>7==0){
            vel =
                    ((int32_t)(data[5])<<24)|
                    ((unsigned char)(data[6])<<16)|
                    ((unsigned char)(data[7])<<8)|
                    ((unsigned char)(data[8]));
        }else{
            vel =
                    ((int32_t)(data[5])<<24)|
                    ((unsigned char)(data[6])<<16)|
                    ((unsigned char)(data[7])<<8)|
                    ((unsigned char)(data[8]));
        }


        m_freq = vel/8388608.*1000;
        m_RPM = m_freq*60/m_poleNum;
        m_velocity = m_freq;

        int16_t current = ((int16_t)(data[9])<<8)|
                ((unsigned char)(data[10]));

        m_current = current/327.68;

        m_mode = (int)data[11];
        m_runstate = (int)data[12];

    }
//    if(m_angleVec.size()>200){
//        m_angleVec.erase(m_angleVec.begin());
//        m_velocityVec.erase(m_angleVec.begin());
//        m_currentVec.erase(m_currentVec.begin());
//        m_angleVec.push_back(m_angle);
//        m_velocityVec.push_back(m_velocity);
//        m_currentVec.push_back(m_current);
//    }
//    else{
//        m_angleVec.push_back(m_angle);
//        m_velocityVec.push_back(m_velocity);
//        m_currentVec.push_back(m_current);
//    }
//    yposmin = 1000000000000; yposmax = -1000000000000;
//         yvelmin = 1000000000000; yvelmax = -1000000000000;
//         ycurmin = 1000000000000; ycurmax = -1000000000000;
//    for(int i = 0;i<(int)m_angleVec.size();i++){
//        double thisAngle = m_angleVec[i];
//        double thisVel = m_velocityVec[i];
//        double thisCurrent = m_currentVec[i];

//        if(thisAngle<yposmin){
//            yposmin = thisAngle;
//        }
//        if(thisAngle>yposmax){
//            yposmax = thisAngle;
//        }
//        if(thisVel<yvelmin){
//            yvelmin = thisVel;
//        }
//        if(thisVel>yvelmax){
//            yvelmax = thisVel;
//        }
//        if(thisCurrent<ycurmin){
//            ycurmin = thisCurrent;
//        }
//        if(thisCurrent>ycurmax){
//            ycurmax = thisCurrent;
//        }
//    }


}

void DataDealer::clear()
{
    yposmin = 1000000000000; yposmax = -1000000000000;
         yvelmin = 1000000000000; yvelmax = -1000000000000;
         ycurmin = 1000000000000; ycurmax = -1000000000000;
         m_angleVec.clear();
         m_velocityVec.clear();
         m_currentVec.clear();
}
