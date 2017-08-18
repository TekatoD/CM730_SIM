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

bool ComputeLegForwardKinematics(float* out, float pelvis, float tight_roll,
                                             float tight_pitch, float knee_pitch,
                                             float ankle_pitch, float ankle_roll) {

    const float CAMERA_DISTANCE = 33.2; //mm
    const float EYE_TILT_OFFSET_ANGLE = 40.0; //degree
    const float LEG_SIDE_OFFSET = 37.0; //mm
    const float THIGH_LENGTH = 93.0; //mm
    const float CALF_LENGTH = 93.0; //mm
    const float ANKLE_LENGTH = 33.5; //mm
    const float LEG_LENGTH = 219.5; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)


    const float s1 = (float) sin(pelvis);
    const float c1 = (float) cos(pelvis);

    const float s2 = (float) sin(tight_roll);
    const float c2 = (float) cos(tight_roll);

    const float s3 = (float) sin(tight_pitch);
    const float c3 = (float) cos(tight_pitch);

    const float s4 = (float) sin(knee_pitch);
    const float c4 = (float) cos(knee_pitch);

    const float s5 = (float) sin(ankle_pitch);
    const float c5 = (float) cos(ankle_pitch);

    const float s6 = (float) sin(ankle_roll);
    const float c6 = (float) cos(ankle_roll);


    const float s1c3 = s1 * c3;
    const float c4c5 = c4 * c5;
    const float s4s5 = s4 * s5;
    const float s1s3 = s1 * s3;
    const float s4c5 = s4 * c5;
    const float c4s5 = c4 * s5;
    const float mc4c5 = -c4 * c5;
    const float ms4c5 = -s4 * c5;
    const float c1c2c3 = c1 * c2 * c3;
    const float ms3c1c2 = -s3 * c1 * c2;
    const float c1s2 = c1 * s2;

    const float ms2s3 = -s2 * s3;
    const float s2c3 = s2 * c3;

    float r11 = (c1c2c3 - s1s3) * (c4c5 - s4s5) * c6 + (ms3c1c2 - s1c3) * (s4c5 + c4s5) * c6 - c1s2 * s6;
    float r12 = (c1c2c3 - s1s3) * (mc4c5 + s4s5) * s6 + (ms3c1c2 - s1c3) * (ms4c5 - c4s5) * s6 - c1s2 * c6;
    float r13 = (c1c2c3 - s1s3) * (c4s5 + s4c5) + (ms3c1c2 - s1c3) * (s4s5 - c4c5);

    float r31 = (s2c3 * c6) * (c4c5 - s4s5) + ms2s3 * (s4c5 + c4s5) * c6 + c2 * s6;
    float r32 = (s2c3 * s6) * (mc4c5 + s4s5) + ms2s3 * (ms4c5 - c4s5) * s6 + c2 * c6;
    float r33 = s2c3 * (c4s5 + s4c5) + ms2s3 * (s4s5 - c4c5);

    float r21 = r12 * r33 - r13 * r32;
    float r22 = r13 * r31 - r11 * r33;
    float r23 = r11 * r32 - r12 * r31;

    float px = s2c3 * (c4 * CALF_LENGTH + THIGH_LENGTH) +
               ms2s3 * (s4 * CALF_LENGTH) +
               r13 * ANKLE_LENGTH;
    float py = (s1 * c2 * c3 + c1 * s3) * (c4 * CALF_LENGTH + THIGH_LENGTH) +
               (-s3 * s1 * c2 + c1 * c3) * (s4 * CALF_LENGTH) +
               r12 * ANKLE_LENGTH;
    float pz = (c1c2c3 - s1s3) * (c4 * CALF_LENGTH + THIGH_LENGTH) +
                                  (ms3c1c2 - s1c3) * (s4 * CALF_LENGTH) +
                                  r11 * ANKLE_LENGTH;

    std::cout << px << " " << py << " " << pz << std::endl;
}

void ComputeHeadForwardKinematics(float pan, float tilt) {
    const float CAMERA_OFFSET_X = 33.2;
    const float CAMERA_OFFSET_Z = 34.4;
    // todo check order of angles
    const float s1 = sinf(pan);
    const float c1 = cosf(pan);
    const float s2 = sinf(tilt);
    const float c2 = cosf(tilt);

    const float r11 = c1 * c2;
    const float r12 = -c1 * s2;
    const float r13 = -s1;

    const float r31 = s1 * c2;
    const float r32 = -s1 * s2;
    const float r33 = c1;

    const float r21 = -s2;
    const float r22 = -c2;
    const float r23 = 0.0f;

    const float px = r11 * CAMERA_OFFSET_X + r13 * CAMERA_OFFSET_Z;
    const float py = r21 * CAMERA_OFFSET_X + r23 * CAMERA_OFFSET_Z;
    const float pz = r31 * CAMERA_OFFSET_X + r33 * CAMERA_OFFSET_Z;

    std::cout << px << " " << py << " " << pz << std::endl;
}

int main() {
    Robot::CM730 cm730;
//    Robot::CM730 cm7301(cm730.get_client_id(), "#0");
    while(true) {
        int joint_to_test = 20;
        double angle = 30;
        for (size_t i = 0; i < 2; ++i) {
            int params[Robot::JointData::NUMBER_OF_JOINTS * Robot::MX28::PARAM_BYTES];
            int num = 0;
            int jointNum = 0;
            angle *= -1;
            for (size_t i = joint_to_test; i < joint_to_test + 1; ++i) {
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
            cm730.SyncWrite(Robot::MX28::P_D_GAIN, Robot::MX28::PARAM_BYTES, jointNum, params);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
//    while(true) {
//        int params[Robot::JointData::NUMBER_OF_JOINTS * Robot::MX28::PARAM_BYTES];
//        int num = 0;
//        int jointNum = 0;
//        angle *= -1;
//        for (size_t i = 1; i < Robot::JointData::NUMBER_OF_JOINTS; ++i) {
//            params[num++] = i;
//            params[num++] = 0;
//            params[num++] = 0;
//            params[num++] = 32;
//            params[num++] = 0;
//            params[num++] = Robot::CM730::GetLowByte(Robot::MX28::Angle2Value(angle));
//            params[num++] = Robot::CM730::GetHighByte(Robot::MX28::Angle2Value(angle));
//            jointNum++;
//        }
//        while (num < Robot::JointData::NUMBER_OF_JOINTS * Robot::MX28::PARAM_BYTES) {
//            params[num++] = 0;
//        }
//
//        cm730.SyncWrite(Robot::MX28::P_D_GAIN, Robot::MX28::PARAM_BYTES, jointNum, params);
//
//
//    int adr = Robot::CM730::P_ACCEL_Z_L;
//    int adr1 = Robot::CM730::P_ACCEL_Y_L;
//    int adr2 = Robot::CM730::P_ACCEL_X_L;
//        cm730.BulkRead();
//        std::cout << "Gyro z: " << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(adr) << std::endl;
//        std::cout << "Gyro y: " << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(adr1) << std::endl;
//        std::cout << "Gyro x: " << cm730.m_BulkReadData[Robot::CM730::ID_CM].ReadWord(adr2) << std::endl;
//
//
//        std::this_thread::sleep_for(std::chrono::milliseconds(300));
//    }


    cm730.BulkRead();
    ComputeLegForwardKinematics(NULL,
                                (Robot::MX28::Value2Angle(cm730.m_BulkReadData[Robot::JointData::ID_L_HIP_YAW ].ReadWord(Robot::MX28::P_PRESENT_POSITION_L)) * M_PI) / 180,
                                (Robot::MX28::Value2Angle(cm730.m_BulkReadData[Robot::JointData::ID_L_HIP_ROLL].ReadWord(Robot::MX28::P_PRESENT_POSITION_L)) * M_PI) / 180,
                                                          (Robot::MX28::Value2Angle(cm730.m_BulkReadData[Robot::JointData::ID_L_HIP_PITCH].ReadWord(Robot::MX28::P_PRESENT_POSITION_L)) * M_PI) / 180,
                                                                                    (Robot::MX28::Value2Angle(cm730.m_BulkReadData[Robot::JointData::ID_L_KNEE].ReadWord(Robot::MX28::P_PRESENT_POSITION_L)) * M_PI) / 180,
                                                                                                              (Robot::MX28::Value2Angle(cm730.m_BulkReadData[Robot::JointData::ID_L_ANKLE_PITCH].ReadWord(Robot::MX28::P_PRESENT_POSITION_L)) * M_PI) / 180,
                                ((Robot::MX28::Value2Angle(cm730.m_BulkReadData[Robot::JointData::ID_L_ANKLE_ROLL].ReadWord(Robot::MX28::P_PRESENT_POSITION_L))) * M_PI) / 180);

    ComputeHeadForwardKinematics((Robot::MX28::Value2Angle(cm730.m_BulkReadData[Robot::JointData::ID_HEAD_PAN].ReadWord(Robot::MX28::P_PRESENT_POSITION_L)) * M_PI) / 180,
                                 (Robot::MX28::Value2Angle(cm730.m_BulkReadData[Robot::JointData::ID_HEAD_TILT ].ReadWord(Robot::MX28::P_PRESENT_POSITION_L)) * M_PI) / 180);

    int ret;
    int val;
    std::cout << "Privet" << std::endl;
    std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
    for(size_t i = 0; i < 100; ++i) {
        std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
        cm730.BulkRead();
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tp);
        std::cout << "Time taken: " << ms.count() << std::endl;
    }
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tp);
    std::cout << "Time taken: " << ms.count() << std::endl;

    std::cout << "Poka" << std::endl;

    return 0;
}