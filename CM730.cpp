#define MAX_EXT_API_CONNECTIONS 255
#define NON_MATLAB_PARSING
extern "C" {
    #include "extApi.h"
    #include "extApi.c"
    #include "extApiPlatform.h"
    #include "extApiPlatform.c"
}

#include <iostream>
#include <chrono>
#include <thread>
#include "CM730.h"
#include "JointData.h"

Robot::BulkReadData::BulkReadData()
        :
        start_address(0),
        length(MX28::MAXNUM_ADDRESS),
        error(-1) {
    for (int i = 0; i < MX28::MAXNUM_ADDRESS; i++)
        table[i] = 0;
}


int Robot::BulkReadData::ReadByte(int address) {
    if (address >= start_address && address < (start_address + length))
        return (int) table[address];

    return 0;
}


int Robot::BulkReadData::ReadWord(int address) {
    if (address >= start_address && address < (start_address + length))
        return CM730::MakeWord(table[address], table[address + 1]);

    return 0;
}

Robot::CM730::CM730(std::string server_ip, int server_port, int client_id, std::string device_postfix) {
    if(client_id =- -1) {
        m_client_id = simxStart((simxChar *) server_ip.c_str(), server_port, true, true, 5000, 5);
//        simxSynchronous(client_id, true);
    }
    else {
        m_client_id = client_id;
    }
    std::cout << "Can't connect with sim" << std::endl;
    m_device_postfix = device_postfix;
    init_devices();
    for (int i = 0; i < ID_BROADCAST; i++) {
        m_BulkReadData[i] = BulkReadData();
    }
//    this->BulkRead();
}

Robot::CM730::CM730(int client_id, std::string device_postfix) {
    std::string server_ip("127.0.0.1");
    int server_port = 19999;
    if(client_id == -1) {
        m_client_id = simxStart((simxChar *) server_ip.c_str(), server_port, true, true, 5000, 5);
//        simxSynchronous(client_id, true);
    } else {
        m_client_id = client_id;
    }
    if (m_client_id == -1) {
        std::cout << "Can't connect with sim" << std::endl;
    }
    m_device_postfix = device_postfix;
    init_devices();
    for (int i = 0; i < ID_BROADCAST; i++) {
        m_BulkReadData[i] = BulkReadData();
    }
//    this->BulkRead();
}

void Robot::CM730::init_devices() {
    m_emu_devices.reserve(22);
    for(size_t i = P_GYRO_Z_L; i < P_ACCEL_X_H; ++i) {
        m_emu_devices.emplace(std::make_pair(i, this->connect_device("Gyro"  + m_device_postfix)));
    }

    for(size_t i = P_ACCEL_X_H; i < P_VOLTAGE; ++i) {
        m_emu_devices.emplace(std::make_pair(i, this->connect_device("Accelerometer_forceSensor"  + m_device_postfix)));
    }

    m_emu_devices.emplace(std::make_pair(JointData::ID_L_SHOULDER_PITCH, this->connect_device("j_shoulder_l" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_R_SHOULDER_PITCH, this->connect_device("j_shoulder_r" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_R_SHOULDER_ROLL, this->connect_device("j_high_arm_r" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_L_SHOULDER_ROLL, this->connect_device("j_high_arm_l" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_L_ELBOW, this->connect_device("j_low_arm_l" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_R_ELBOW, this->connect_device("j_low_arm_r" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_L_HIP_YAW, this->connect_device("j_pelvis_l" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_R_HIP_YAW, this->connect_device("j_pelvis_r" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_L_HIP_ROLL, this->connect_device("j_thigh1_l" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_R_HIP_ROLL, this->connect_device("j_thigh1_r" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_L_HIP_PITCH, this->connect_device("j_thigh2_l" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_R_HIP_PITCH, this->connect_device("j_thigh2_r" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_L_KNEE, this->connect_device("j_tibia_l" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_R_KNEE, this->connect_device("j_tibia_r" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_L_ANKLE_PITCH, this->connect_device("j_ankle1_l" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_R_ANKLE_PITCH, this->connect_device("j_ankle1_r" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_L_ANKLE_ROLL, this->connect_device("j_ankle2_l" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_R_ANKLE_ROLL, this->connect_device("j_ankle2_r" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_HEAD_PAN, this->connect_device("j_pan" + m_device_postfix)));
    m_emu_devices.emplace(std::make_pair(JointData::ID_HEAD_TILT, this->connect_device("j_tilt" + m_device_postfix)));
}

int Robot::CM730::get_client_id() {
    return m_client_id;
}

int Robot::CM730::connect_device(std::string device_name) {
    int object_handler;
    if(simxGetObjectHandle(m_client_id, (const simxChar*)device_name.c_str(),
                           (simxInt*) &object_handler, simx_opmode_oneshot_wait) == simx_return_ok) {
        if(device_name != ("Gyro" + m_device_postfix) && device_name != ("Accelerometer" + m_device_postfix)) {
//            simxGetJointPosition(m_client_id, object_handler, NULL, simx_opmode_streaming);
        }
        return std::move(object_handler);
    }
    return -1;
}

int Robot::CM730::SyncWrite(int start_addr, int each_length, int number, int *pParam) {
    for(size_t i = 0; i < number * each_length; i += each_length) {
        simxSetObjectIntParameter(m_client_id, m_emu_devices[pParam[i]], 2000, 1, simx_opmode_oneshot);
        simxSetObjectIntParameter(m_client_id, m_emu_devices[pParam[i]], 2001, 1, simx_opmode_oneshot);
        simxSetJointTargetPosition(m_client_id, m_emu_devices[pParam[i]],
                                      (Robot::MX28::Value2Angle(CM730::MakeWord(pParam[i + 5], pParam[i + 6])) * M_PI) / 180, simx_opmode_oneshot);

    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    this->BulkReadJoints();
//    std::this_thread::sleep_for(std::chrono::milliseconds(150));
}

int Robot::CM730::ReadWord(int id, int address, int* pValue, int* error) {
        simxFloat* gyroData = nullptr;
        simxFloat* accelData = nullptr;
        auto get_gyro_data = [this, &gyroData](int ind) {
            simxInt retCount;
            simxCallScriptFunction(m_client_id, "Gyro", sim_scripttype_childscript, "getGyroData", NULL, NULL, NULL, NULL, NULL,
                                   NULL,NULL, NULL, NULL, NULL, &retCount,
            &gyroData, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
            return gyroData[ind];
        };
        auto get_accel_data = [this, &accelData](int ind) {
            simxInt retCount;
            simxCallScriptFunction(m_client_id, "Accelerometer", sim_scripttype_childscript, "getAccelData", NULL, NULL, NULL, NULL, NULL,
                                   NULL,NULL, NULL, NULL, NULL, &retCount,
                                   &accelData, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
            return accelData[ind];
        };
        auto norm_accel = [](double value) {
            return (int)((value + 39.24) / (78.48) * 1023);
        };
        auto norm_gyro = [](double value) {
            return (int)((value + 500) / (1000) * 1023);
        };
        double val = 0;
        switch(address) {
            case P_GYRO_Z_L:
                if(gyroData == nullptr) {
                    *pValue = norm_gyro(get_gyro_data(2));
                }
                else {
                    *pValue = norm_gyro(gyroData[2]);
                }
                break;
            case P_GYRO_Y_L:
                if(gyroData == nullptr) {
                    *pValue = norm_gyro(get_gyro_data(1));
                }
                else {
                    *pValue = norm_gyro(gyroData[1]);
                }
                break;
            case P_GYRO_X_L:
                if(gyroData == nullptr) {
                    *pValue = norm_gyro(get_gyro_data(0));
                }
                else {
                    *pValue = norm_gyro(gyroData[0]);
                }
                break;
            case P_ACCEL_Z_L:
                if(accelData == nullptr) {
                    *pValue = norm_accel(get_accel_data(2));
                }
                else {
                    *pValue = norm_accel(accelData[2]);
                }
                break;
            case P_ACCEL_Y_L:
                if(accelData == nullptr) {
                    *pValue = norm_accel(get_accel_data(1));
                }
                else {
                    *pValue = norm_accel(accelData[1]);
                }
                break;
            case P_ACCEL_X_L:
                if(accelData == nullptr) {
                    *pValue = norm_accel(get_accel_data(0));
                }
                else {
                    *pValue = norm_accel(accelData[0]);
                }
                break;
            default:
                simxFloat pos;
                simxSetObjectIntParameter(m_client_id, m_emu_devices[address], 2000, 1, simx_opmode_oneshot);
                simxSetObjectIntParameter(m_client_id, m_emu_devices[address], 2001, 1, simx_opmode_oneshot);
                while((*error = simxGetJointPosition(m_client_id, m_emu_devices[address], &pos, simx_opmode_oneshot))
                      != simx_return_ok) { };
                pos = (180 * pos) / M_PI;
                *pValue = MX28::Angle2Value(pos);
//                std::cout << pos << std::endl;
                return SUCCESS;
    }
}

int Robot::CM730::BulkRead() {
    for(size_t i = JointData::ID_R_SHOULDER_PITCH; i < JointData::NUMBER_OF_JOINTS; ++i) {
        int value;
        int error;
        this->ReadWord(0, i, &value, &error);
        m_BulkReadData[i].table[MX28::P_PRESENT_POSITION_L] = GetLowByte(value);
        m_BulkReadData[i].table[MX28::P_PRESENT_POSITION_H] = GetHighByte(value);
        if(error == simx_return_ok) {
            m_BulkReadData->error = 0;
        }

    }
    for(size_t i = P_GYRO_Z_L; i < P_VOLTAGE; i += 2) { //TODO:: Check the loop
        int value;
        int error;
        this->ReadWord(0, i, &value, &error);
        m_BulkReadData[ID_CM].table[i] = GetLowByte(value);
        m_BulkReadData[ID_CM].table[i + 1] = GetHighByte(value);
        if(error == simx_return_ok) {
            m_BulkReadData->error = 0;
        }
    }
}

int Robot::CM730::WriteByte(int address, int value, int* error) {
    return SUCCESS;
}

int Robot::CM730::WriteWord(int id, int address, int value, int* error) {
    return SUCCESS;
}

int Robot::CM730::ReadByte(int id, int address, int *pValue, int* error) {
    if(address == MX28::P_VERSION) {
        *pValue = 28;
    }
    return SUCCESS;
}


int Robot::CM730::MakeWord(int lowbyte, int highbyte) {
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int) word;
}


int Robot::CM730::GetLowByte(int word) {
    unsigned short temp;
    temp = word & 0xff;
    return (int) temp;
}


int Robot::CM730::GetHighByte(int word) {
    unsigned short temp;
    temp = word & 0xff00;
    return (int) (temp >> 8);
}

Robot::CM730::~CM730() {
    simxStopSimulation(m_client_id, simx_opmode_oneshot_wait);
}