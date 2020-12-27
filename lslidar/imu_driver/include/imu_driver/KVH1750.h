// KVH1750 serial driver.
// Created by LYC.
// Modified by LarryDong

#ifndef KVH1750_H_
#define KVH1750_H_

#include <iostream>
#include <boost/asio.hpp>

#define Gravity (9.81f)

namespace my_imu{

struct Imu_Data{
    uint8_t valid;
    int sequence_num;
    int16_t temperature;
    float gyro[3];
    float acc[3];
};

using boost::asio::serial_port;
class imu_1750{
public:
    imu_1750() = default;
    explicit imu_1750(std::string dev); // construct func.
    bool get_data(Imu_Data &data);      // decode KVH1750 data;
private:
    // sensor_msgs::Imu imu_data_;         // useful imu_data_
    
    boost::asio::serial_port *sp_{};
    boost::asio::io_service ioSev_;
    std::string portName_;
    boost::system::error_code err_;
    union bag{
        float value;
        uint8_t raw[4];
    }bag1_{},bag2_{};
};

void show_data(Imu_Data);
}

#endif
