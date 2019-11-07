#ifndef _BNO080_PLUGIN_H_
#define _BNO080_PLUGIN_H_

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

namespace BNO0809DOF
{   
    class Plugin
    {
        BNO080 imu_;
        public:
            Plugin():
            imu_()
            {
                initialize();
            }

            void initialize()
            {
                Wire.begin();
                imu_.begin();
                Wire.setClock(400000);
                imu_.enableLinearAccelerometer(8);
                imu_.enableGyro(8); 
                imu_.enableMagnetometer(8);
            }

            float readGyroscopeX()
            {
                return imu_.getGyroX();
            }

            float readGyroscopeY()
            {
                return imu_.getGyroY();
            }

            float readGyroscopeZ()
            {
                return imu_.getGyroZ();
            }

            float readAccelerometerX()
            {
                return imu_.getAccelX();
            }

            float readAccelerometerY()
            {
                return imu_.getAccelY();
            }

            float readAccelerometerZ()
            {
                return imu_.getAccelZ();
            }

            float readMagnetometerX()
            {
                return imu_.getMagX();                
            }

            float readMagnetometerY()
            {
                return imu_.getMagY();                
            }

            float readMagnetometerZ()
            {
                return imu_.getMagZ();
            }
    };
}

#endif

