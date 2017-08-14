/**
 *  @autor tekatod
 *  @date 8/9/17
 */

#include <iostream>
#include "JointData.h"
#include "MX28.h"
#include "CM730.h"
#include <chrono>
#include <thread>
#include <time.h>

int main() {
    Robot::CM730 cm730;
    Robot::CM730 cm7301(cm730.get_client_id(), "#0");
    double angle = 15;
    while(true) {
        int params[Robot::JointData::NUMBER_OF_JOINTS * Robot::MX28::PARAM_BYTES];
        int num = 0;
        int jointNum = 0;
        angle *= -1;
        for (size_t i = 1; i < Robot::JointData::NUMBER_OF_JOINTS; ++i) {
            params[num++] = i;
            params[num++] = 0;
            params[num++] = 0;
            params[num++] = 32;
            params[num++] = 0;
            params[num++] = Robot::CM730::GetLowByte(Robot::MX28::Angle2Value(angle));
            params[num++] = Robot::CM730::GetHighByte(Robot::MX28::Angle2Value(angle));
            jointNum++;
        }
        while (num < Robot::JointData::NUMBER_OF_JOINTS * Robot::MX28::PARAM_BYTES) {
            params[num++] = 0;
        }

        int params1[Robot::JointData::NUMBER_OF_JOINTS * Robot::MX28::PARAM_BYTES];
        int num1 = 0;
        int jointNum1 = 0;
        double angle1 = 15;
        for (size_t i = 1; i < Robot::JointData::NUMBER_OF_JOINTS; ++i) {
            params1[num1++] = i;
            params1[num1++] = 0;
            params1[num1++] = 0;
            params1[num1++] = 32;
            params1[num1++] = 0;
            params1[num1++] = Robot::CM730::GetLowByte(Robot::MX28::Angle2Value(angle));
            params1[num1++] = Robot::CM730::GetHighByte(Robot::MX28::Angle2Value(angle));
            jointNum1++;
        }
        while (num1 < Robot::JointData::NUMBER_OF_JOINTS * Robot::MX28::PARAM_BYTES) {
            params1[num1++] = 0;
        }

        cm730.SyncWrite(Robot::MX28::P_D_GAIN, Robot::MX28::PARAM_BYTES, jointNum, params);
        cm7301.SyncWrite(Robot::MX28::P_D_GAIN, Robot::MX28::PARAM_BYTES, jointNum1, params1);
        int ret;
        int val;
    int adr = Robot::CM730::P_GYRO_Z_L;
    int adr1 = Robot::CM730::P_GYRO_Y_L;
    int adr2 = Robot::CM730::P_GYRO_X_L;
//        int adr = Robot::JointData::ID_HEAD_TILT;
//        cm730.ReadWord(0, adr, &val, &ret);
//        cm730.ReadWord(0, adr1, &val, &ret);
//        cm730.ReadWord(0, adr2, &val, &ret);
        clock_t tStart = clock();
        cm730.BulkRead();
        printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
//        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    int ret;
    int val;
//    int adr = Robot::CM730::P_ACCEL_Z_L;
    int adr = Robot::JointData::ID_HEAD_TILT;
    cm730.ReadWord(0, adr, &val, &ret);

    return 0;
}