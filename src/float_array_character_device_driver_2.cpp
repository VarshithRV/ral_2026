#include <chrono>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class FloatArrayCharacterDeviceDriver : public rclcpp::Node
{
public:
    FloatArrayCharacterDeviceDriver()
    : Node("float_array_character_device_driver")
    {
        device_name_ = declare_parameter<std::string>(
            "device_name",
            "/dev/ttyACM0"
        );

        baud_rate_ = declare_parameter<int>("baud_rate", 115200);

        channels_ = declare_parameter<int>("channels", 2);

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/float_array",
            10
        );

        open_serial();

        timer_ = create_wall_timer(
            20ms,
            std::bind(
                &FloatArrayCharacterDeviceDriver::timer_callback,
                this
            )
        );
    }

    ~FloatArrayCharacterDeviceDriver()
    {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

private:
    bool get_termios_baud_rate(
        const int baud_rate,
        speed_t & termios_baud_rate) const
    {
        switch (baud_rate) {
            case 1200:
                termios_baud_rate = B1200;
                break;

            case 2400:
                termios_baud_rate = B2400;
                break;

            case 4800:
                termios_baud_rate = B4800;
                break;

            case 9600:
                termios_baud_rate = B9600;
                break;

            case 19200:
                termios_baud_rate = B19200;
                break;

            case 38400:
                termios_baud_rate = B38400;
                break;

            case 57600:
                termios_baud_rate = B57600;
                break;

            case 115200:
                termios_baud_rate = B115200;
                break;

            case 230400:
                termios_baud_rate = B230400;
                break;

            case 460800:
                termios_baud_rate = B460800;
                break;

            case 500000:
                termios_baud_rate = B500000;
                break;

            case 576000:
                termios_baud_rate = B576000;
                break;

            case 921600:
                termios_baud_rate = B921600;
                break;

            case 1000000:
                termios_baud_rate = B1000000;
                break;

            case 1152000:
                termios_baud_rate = B1152000;
                break;

            case 1500000:
                termios_baud_rate = B1500000;
                break;

            case 2000000:
                termios_baud_rate = B2000000;
                break;

            default:
                return false;
        }

        return true;
    }

    void open_serial()
    {
        speed_t termios_baud_rate;

        if (!get_termios_baud_rate(
                baud_rate_,
                termios_baud_rate))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Unsupported baud rate: %d",
                baud_rate_
            );
            return;
        }

        fd_ = open(
            device_name_.c_str(),
            O_RDONLY | O_NOCTTY | O_NONBLOCK
        );

        if (fd_ < 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to open %s",
                device_name_.c_str()
            );
            return;
        }

        termios tty{};

        if (tcgetattr(fd_, &tty) != 0) {
            RCLCPP_ERROR(
                get_logger(),
                "tcgetattr failed"
            );

            close(fd_);
            fd_ = -1;
            return;
        }

        if (cfsetispeed(&tty, termios_baud_rate) != 0 ||
            cfsetospeed(&tty, termios_baud_rate) != 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to configure baud rate %d",
                baud_rate_
            );

            close(fd_);
            fd_ = -1;
            return;
        }

        // 8N1, no hardware flow control.
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        // Raw input mode.
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
        tty.c_oflag &= ~OPOST;

        // Non-blocking reads.
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        tcflush(fd_, TCIFLUSH);

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(
                get_logger(),
                "tcsetattr failed"
            );

            close(fd_);
            fd_ = -1;
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Opened serial device %s at %d baud",
            device_name_.c_str(),
            baud_rate_
        );
    }

    bool parse_line(
        const std::string & line,
        std::vector<double> & values)
    {
        values.clear();

        std::stringstream ss(line);
        double value;

        while (ss >> value) {
            values.push_back(value);
        }

        return static_cast<int>(values.size()) == channels_;
    }

    double get_epoch_time_seconds() const
    {
        const auto now = std::chrono::system_clock::now();
        const auto duration = now.time_since_epoch();

        return std::chrono::duration<double>(duration).count();
    }

    void timer_callback()
    {
        if (fd_ < 0) {
            return;
        }

        char read_buffer[512];

        // Read all currently available serial bytes.
        while (true) {
            const ssize_t bytes_read = read(
                fd_,
                read_buffer,
                sizeof(read_buffer)
            );

            if (bytes_read > 0) {
                serial_buffer_.append(
                    read_buffer,
                    static_cast<std::size_t>(bytes_read)
                );
            } else {
                break;
            }
        }

        std::vector<double> latest_valid_values;
        bool got_valid_line = false;

        std::size_t newline_pos;

        while (
            (newline_pos = serial_buffer_.find('\n')) !=
            std::string::npos)
        {
            std::string line = serial_buffer_.substr(
                0,
                newline_pos
            );

            serial_buffer_.erase(0, newline_pos + 1);

            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            if (
                line.find_first_not_of(" \t\r\n") ==
                std::string::npos)
            {
                continue;
            }

            std::vector<double> values;

            if (parse_line(line, values)) {
                latest_valid_values = std::move(values);
                got_valid_line = true;
            } else {
                RCLCPP_WARN(
                    get_logger(),
                    "Invalid line: '%s'",
                    line.c_str()
                );
            }
        }

        if (got_valid_line) {
            std_msgs::msg::Float64MultiArray msg;

            msg.data.reserve(latest_valid_values.size() + 1);

            // First element: Unix epoch time in seconds.
            msg.data.push_back(get_epoch_time_seconds());

            // Remaining elements: serial data.
            msg.data.insert(
                msg.data.end(),
                latest_valid_values.begin(),
                latest_valid_values.end()
            );

            pub_->publish(msg);
        }
    }

    std::string device_name_;
    int baud_rate_;
    int channels_;
    int fd_ = -1;

    std::string serial_buffer_;

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray
    >::SharedPtr pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<FloatArrayCharacterDeviceDriver>()
    );

    rclcpp::shutdown();
    return 0;
}