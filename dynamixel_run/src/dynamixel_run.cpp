#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/int32.hpp>

class DynamixelController : public rclcpp::Node
{
public:
    DynamixelController() : Node("dynamixel_controller")
    {
        // 시리얼 포트 설정
        try
        {
            serial_.setPort("/dev/ttyUSB0");
            serial_.setBaudrate(1000000);
            serial_.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port");
            rclcpp::shutdown();
            return;
        }

        if (serial_.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            rclcpp::shutdown();
            return;
        }

        // 구독자 생성
        acc_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "acc_topic", 10,
            std::bind(&DynamixelController::acc_callback, this, std::placeholders::_1));
        vel_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "vel_topic", 10,
            std::bind(&DynamixelController::vel_callback, this, std::placeholders::_1));
        pos_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "pos_topic", 10,
            std::bind(&DynamixelController::pos_callback, this, std::placeholders::_1));

        // 초기화 순서
        sendTorqueDisable();
        setDriveMode();     // Drive Mode 먼저 설정
        setOperatingMode(); // Operating Mode 설정
        sendTorqueEnable();

        RCLCPP_INFO(this->get_logger(), "Dynamixel initialization completed");
    }

private:
    serial::Serial serial_;
    int32_t acc_, vel_, pos_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr acc_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pos_sub_;

    void acc_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        acc_ = msg->data;
        updateDynamixel();
    }

    void vel_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        vel_ = msg->data;
        updateDynamixel();
    }

    void pos_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        pos_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received position command: %d", pos_);
        updateDynamixel();
    }

    void sendTorqueDisable()
    {
        uint8_t torque_buffer[14] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x06, 0x00, 0x03, 0x40, 0x00, 0x00};
        uint16_t torque_crc = update_crc(0, torque_buffer, 11);
        torque_buffer[11] = torque_crc & 0xFF;
        torque_buffer[12] = (torque_crc >> 8) & 0xFF;
        serial_.write(torque_buffer, 13);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Torque disabled");
    }

    void setDriveMode()
    {
        uint8_t drive_buffer[14] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x06, 0x00, 0x03, 0x0A, 0x00, 0x00};
        uint16_t drive_crc = update_crc(0, drive_buffer, 11);
        drive_buffer[11] = drive_crc & 0xFF;
        drive_buffer[12] = (drive_crc >> 8) & 0xFF;
        serial_.write(drive_buffer, 13);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Drive mode set");
    }

    void setOperatingMode()
    {
        uint8_t mode_buffer[14] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x06, 0x00, 0x03, 0x0B, 0x00, 0x04};
        uint16_t mode_crc = update_crc(0, mode_buffer, 11);
        mode_buffer[11] = mode_crc & 0xFF;
        mode_buffer[12] = (mode_crc >> 8) & 0xFF;
        serial_.write(mode_buffer, 13);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Operating mode set to Extended Position Control");
    }

    void sendTorqueEnable()
    {
        uint8_t torque_buffer[14] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x06, 0x00, 0x03, 0x40, 0x00, 0x01};
        uint16_t torque_crc = update_crc(0, torque_buffer, 11);
        torque_buffer[11] = torque_crc & 0xFF;
        torque_buffer[12] = (torque_crc >> 8) & 0xFF;
        serial_.write(torque_buffer, 13);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Torque enabled");
    }

    void updateDynamixel()
    {
        //가속도
        uint8_t acceleration_buffer[16] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x09, 0x00, 0x03, 0x6C, 0x00};
        acceleration_buffer[10] = acc_ & 0xFF;
        acceleration_buffer[11] = (acc_ >> 8) & 0xFF;
        acceleration_buffer[12] = (acc_ >> 16) & 0xFF;
        acceleration_buffer[13] = (acc_ >> 24) & 0xFF;
        uint16_t acc_crc = update_crc(0, acceleration_buffer, 14);
        acceleration_buffer[14] = acc_crc & 0xFF;
        acceleration_buffer[15] = (acc_crc >> 8) & 0xFF;
        serial_.write(acceleration_buffer, 16);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // 속도 
        uint8_t velocity_buffer[16] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x09, 0x00, 0x03, 0x70, 0x00};
        velocity_buffer[10] = vel_ & 0xFF;
        velocity_buffer[11] = (vel_ >> 8) & 0xFF;
        velocity_buffer[12] = (vel_ >> 16) & 0xFF;
        velocity_buffer[13] = (vel_ >> 24) & 0xFF;
        uint16_t vel_crc = update_crc(0, velocity_buffer, 14);
        velocity_buffer[14] = vel_crc & 0xFF;
        velocity_buffer[15] = (vel_crc >> 8) & 0xFF;
        serial_.write(velocity_buffer, 16);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // 목표 위치
        uint8_t position_buffer[16] = {0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x09, 0x00, 0x03, 0x74, 0x00};
        position_buffer[10] = pos_ & 0xFF;
        position_buffer[11] = (pos_ >> 8) & 0xFF;
        position_buffer[12] = (pos_ >> 16) & 0xFF;
        position_buffer[13] = (pos_ >> 24) & 0xFF;
        uint16_t pos_crc = update_crc(0, position_buffer, 14);
        position_buffer[14] = pos_crc & 0xFF;
        position_buffer[15] = (pos_crc >> 8) & 0xFF;
        serial_.write(position_buffer, 16);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
    {
        uint16_t i, j;
        static const uint16_t crc_table[256] = {
            0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
            0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
            0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
            0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
            0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
            0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
            0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
            0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
            0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
            0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
            0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
            0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
            0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
            0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
            0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
            0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
            0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
            0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
            0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
            0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
            0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
            0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
            0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
            0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
            0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
            0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
            0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};
        for (j = 0; j < data_blk_size; j++)
        {
            i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
            crc_accum = (crc_accum << 8) ^ crc_table[i];
        }
        return crc_accum;
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelController>());
    rclcpp::shutdown();
    return 0;
}