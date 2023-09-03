#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class SerialIMU : public rclcpp::Node
{
    public:
        SerialIMU()
        : Node("serial_imu")
        {
            read_serial();
        }

    private:
        void read_serial()
        {
            const char * portname = "/dev/ttyUSB0";
            int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
            if (fd < 0)
            {
                std::cerr << "Error getting terminal attributes" << std::endl;
                return;
            }

            struct termios tty;
            memset(&tty, 0, sizeof(tty));
            if (tcgetattr(fd, &tty) != 0)
            {
                std::cerr << "Error getting terminal attributes" << std::endl;
                return;
            }

            cfsetospeed(&tty, (speed_t)B1152000);
            cfsetispeed(&tty, (speed_t)B1152000);

            tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
            tty.c_iflag &= ~IGNBRK;
            tty.c_lflag = 0;
            tty.c_oflag = 0;
            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 5;

            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~(PARENB | PARODD);
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;

            if (tcsetattr(fd, TCSANOW, &tty) != 0)
            {
                std::cerr << "Error setting terminal attributes" << std::endl;
                return;
            }

            char buf[256];
            while (true)
            {
                int n = read(fd, buf, sizeof(buf) - 1);
                if (n > 0)
                {
                    buf[n] = '\0';
                    std::string message(buf);
                    std::size_t pos = message.find("Position:");
                    std::size_t orient = message.find(", Orientation: ");

                    if (pos != std::string::npos && orient != std::string::npos)
                    {
                        std::string position = message.substr(pos + 10, orient - (pos + 10));
                        std::string orientation = message.substr(orient + 14);
                    }
                }
            }
        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialIMU>());
    rclcpp::shutdown();
    return 0;
}