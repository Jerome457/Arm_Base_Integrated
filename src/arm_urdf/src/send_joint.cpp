#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <string>
#include <termios.h>

class JointStateSerial : public rclcpp::Node
{
public:
    JointStateSerial()
    : Node("joint_state_serial")
    {
        while ((serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /dev/ttyACM0...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Configure serial port for 9600 baud, 8N1
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            return;
        }

        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);

        tty.c_cflag &= ~PARENB; // no parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;     // 8 bits per byte
        tty.c_cflag |= CREAD | CLOCAL; // enable read, ignore modem ctrl lines

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO; 
        tty.c_lflag &= ~ECHOE; 
        tty.c_lflag &= ~ISIG; 

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // disable flow control
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1; // timeout 0.1 sec

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
            return;
        }

        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&JointStateSerial::jointCallback, this, std::placeholders::_1)
        );

        // Timer to continuously read from USB
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&JointStateSerial::readSerial, this)
        );
    }

    ~JointStateSerial()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        size_t n = msg->position.size();
        if (n < 3) {
            RCLCPP_WARN(this->get_logger(), "Not enough DOFs");
            return;
        }

        // 3rd last joint
        double third_last_rad = msg->position[n - 3];
        double third_last_deg = third_last_rad * 180.0 / M_PI;

        int third_last_deg_rounded = static_cast<int>(std::round(third_last_deg));
        std::string out = std::to_string(third_last_deg_rounded) + "\n";

        if (serial_fd_ >= 0) {
            write(serial_fd_, out.c_str(), out.size());
        }

        RCLCPP_INFO(this->get_logger(), "TX: %s", out.c_str());
    }

    void readSerial()
    {
        if (serial_fd_ < 0) return;

        char buf[128];
        int bytes_read = read(serial_fd_, buf, sizeof(buf)-1);

        while (bytes_read > 0) {
            buf[bytes_read] = '\0';
            RCLCPP_INFO(this->get_logger(), "RX: %s", buf);
            bytes_read = read(serial_fd_, buf, sizeof(buf)-1);
        }
    }

    int serial_fd_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateSerial>());
    rclcpp::shutdown();
    return 0;
}
