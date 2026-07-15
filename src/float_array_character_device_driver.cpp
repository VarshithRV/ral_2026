#include <chrono>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class FloatArrayCharacterDeviceDriver : public rclcpp::Node
{
public:
    FloatArrayCharacterDeviceDriver()
    : Node("float_array_character_device_driver")
    {
        device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyACM0");
        channels_ = declare_parameter<int>("channels", 2);

        pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("~/float_array", 10);

        open_serial();

        timer_ = create_wall_timer(
            20ms,
            std::bind(&FloatArrayCharacterDeviceDriver::timer_callback, this)
        );
    }

    ~FloatArrayCharacterDeviceDriver()
    {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

private:
    void open_serial()
    {
        fd_ = open(device_name_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);

        if (fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", device_name_.c_str());
            return;
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "tcgetattr failed");
            return;
        }

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSANOW, &tty);

        RCLCPP_INFO(get_logger(), "Opened serial device %s at 115200 baud", device_name_.c_str());
    }

    bool parse_line(const std::string & line, std::vector<float> & values)
    {
        values.clear();

        std::stringstream ss(line);
        float value;

        while (ss >> value) {
            values.push_back(value);
        }

        return static_cast<int>(values.size()) == channels_;
    }

    void timer_callback()
    {
        if (fd_ < 0) {
            return;
        }

        char read_buffer[512];

        while (true) {
            ssize_t n = read(fd_, read_buffer, sizeof(read_buffer));

            if (n > 0) {
                serial_buffer_.append(read_buffer, n);
            } else {
                break;
            }
        }

        std::vector<float> latest_valid_values;
        bool got_valid_line = false;

        size_t newline_pos;
        while ((newline_pos = serial_buffer_.find('\n')) != std::string::npos) {
            std::string line = serial_buffer_.substr(0, newline_pos);
            serial_buffer_.erase(0, newline_pos + 1);

            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
                continue;
            }

            std::vector<float> values;
            if (parse_line(line, values)) {
                latest_valid_values = values;
                got_valid_line = true;
            } else {
                RCLCPP_WARN(get_logger(), "Invalid line: '%s'", line.c_str());
            }
        }

        if (got_valid_line) {
            std_msgs::msg::Float32MultiArray msg;
            msg.data = latest_valid_values;
            pub_->publish(msg);
        }
    }

    std::string device_name_;
    int channels_;
    int fd_ = -1;

    std::string serial_buffer_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloatArrayCharacterDeviceDriver>());
    rclcpp::shutdown();
    return 0;
}