// KVH1750 serial driver.
// Created by LYC.
// Modified by LarryDong

#include "KVH1750.h"

namespace my_imu{

imu_1750::imu_1750(std::string dev):portName_(std::move(dev)){
    try{
        sp_ = new serial_port(ioSev_, portName_);
        sp_->set_option(serial_port::baud_rate(921600));
        sp_->set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp_->set_option(serial_port::parity(serial_port::parity::none));
        sp_->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp_->set_option(serial_port::character_size(8));
    }catch (...){
        std::cerr << "Exception Error: " << err_.message() << std::endl;
    }
    
}

bool imu_1750::get_data(Imu_Data &data){
    unsigned char msg[36];
    try{
        while(true){
            read(*sp_, boost::asio::buffer(msg, 1), err_);
            if(msg[0] == 0xFE)
                read(*sp_, boost::asio::buffer(&msg[1], 3), err_);
            if(msg[0] == 0xFE && msg[1] == 0x81 &&msg[2] == 0xFF && msg[3] == 0x55){
                read(*sp_, boost::asio::buffer(&msg[4], 32), err_);
                break;
            }
        }
    }catch (...){
        std::cout << "get data error..." << std::endl;
        std::cerr << "Exception Error: " << err_.message() << std::endl;
        return false;
    }

    // printf("head : %d \n",msg[0]);
    data.valid = msg[28];
    if(data.valid != 0x77)
        return false;
    
    data.sequence_num = msg[29];
    data.temperature = msg[30] << 8 | msg[31];
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 4; j++){
            bag1_.raw[3 - j] = msg[4 + j + i * 4];
            bag2_.raw[3 - j] = msg[16 + j + i * 4];
        }
        data.gyro[i] = bag1_.value;
        data.acc[i] = bag2_.value;
    }
    return true;
}




void show_data(Imu_Data d){
    using namespace std;
    cout << "Data:" << endl;
    cout << "gyro: " << d.gyro[0] << ", " << d.gyro[1] << ", " << d.gyro[2] << endl;
    cout << "acc: " << d.acc[0] << ", " << d.acc[1] << ", " << d.acc[2] << endl;
}

}