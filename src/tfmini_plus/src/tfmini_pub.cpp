#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <mutex>


class TFminiPublisher : public rclcpp::Node
{
public:

    struct SensorConfig
    {
        std::string port;
        std::string topic;
        std::string frame_id;
    };

    TFminiPublisher()
        : Node("tfmini_publisher"),
          running_(true)
    {
        sensors_ = {
            {
                "/dev/ttyUSB0",
                "range/front_right",
                "lidar_front_right"
            },
            {
                "/dev/ttyUSB1",
                "range/front_left",
                "lidar_front_left"
            },
            {
                "/dev/ttyUSB2",
                "range/right_front",
                "lidar_right_front"
            },
            {
                "/dev/ttyUSB3",
                "range/right_rear",
                "lidar_right_rear"
            }
        };

        for (const auto &sensor : sensors_) {

            auto publisher =
                this->create_publisher<sensor_msgs::msg::Range>(
                    sensor.topic,
                    10);

            publishers_.push_back(publisher);

            threads_.emplace_back(
                &TFminiPublisher::sensorThread,
                this,
                sensor,
                publisher);
        }

        RCLCPP_INFO(
            this->get_logger(),
            "TFmini Plus publisher started.");
    }


    ~TFminiPublisher()
    {
        running_ = false;

        for (auto &thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }


private:

    std::vector<SensorConfig> sensors_;

    std::vector<
        rclcpp::Publisher<
            sensor_msgs::msg::Range
        >::SharedPtr
    > publishers_;

    std::vector<std::thread> threads_;

    std::atomic<bool> running_;


    int openSerial(
        const std::string &port)
    {
        int fd = open(
            port.c_str(),
            O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (fd < 0) {

            RCLCPP_ERROR(
                this->get_logger(),
                "Cannot open %s: %s",
                port.c_str(),
                std::strerror(errno));

            return -1;
        }


        struct termios tty{};

        if (tcgetattr(fd, &tty) != 0) {

            RCLCPP_ERROR(
                this->get_logger(),
                "tcgetattr failed for %s: %s",
                port.c_str(),
                std::strerror(errno));

            close(fd);

            return -1;
        }


        /*
         * Baud rate
         */

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);


        /*
         * 8N1
         */

        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;


        /*
         * Enable receiver
         */

        tty.c_cflag |= CREAD;
        tty.c_cflag |= CLOCAL;


        /*
         * Disable software flow control
         */

        tty.c_iflag &= ~IXON;
        tty.c_iflag &= ~IXOFF;
        tty.c_iflag &= ~IXANY;


        /*
         * Raw input
         */

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ISIG;


        /*
         * Raw output
         */

        tty.c_oflag &= ~OPOST;


        /*
         * Read timeout
         */

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;


        if (tcsetattr(
                fd,
                TCSANOW,
                &tty) != 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "tcsetattr failed for %s: %s",
                port.c_str(),
                std::strerror(errno));

            close(fd);

            return -1;
        }


        tcflush(fd, TCIOFLUSH);


        RCLCPP_INFO(
            this->get_logger(),
            "Opened %s",
            port.c_str());

        return fd;
    }


    bool readByte(
        int fd,
        uint8_t &byte)
    {
        while (running_) {

            fd_set read_fds;

            FD_ZERO(&read_fds);
            FD_SET(fd, &read_fds);


            struct timeval timeout{};

            timeout.tv_sec = 0;
            timeout.tv_usec = 100000;


            int result = select(
                fd + 1,
                &read_fds,
                nullptr,
                nullptr,
                &timeout);


            if (result < 0) {

                if (errno == EINTR) {
                    continue;
                }

                return false;
            }


            if (result == 0) {
                continue;
            }


            if (FD_ISSET(fd, &read_fds)) {

                ssize_t n =
                    read(fd, &byte, 1);

                if (n == 1) {
                    return true;
                }

                if (n < 0 && errno != EAGAIN) {
                    return false;
                }
            }
        }

        return false;
    }


    bool readBytes(
        int fd,
        uint8_t *buffer,
        size_t size)
    {
        for (size_t i = 0; i < size; ++i) {

            if (!readByte(fd, buffer[i])) {
                return false;
            }
        }

        return true;
    }


    bool verifyChecksum(
        const uint8_t *data)
    {
        uint16_t checksum = 0;

        /*
         * TFmini frame:
         *
         * 0x59
         * 0x59
         * Dist_L
         * Dist_H
         * Strength_L
         * Strength_H
         * Temp_L
         * Temp_H
         * Checksum
         */

        checksum += 0x59;
        checksum += 0x59;

        for (int i = 0; i < 6; ++i) {
            checksum += data[i];
        }

        return
            static_cast<uint8_t>(checksum)
            == data[6];
    }


    void sensorThread(
        SensorConfig sensor,
        rclcpp::Publisher<
            sensor_msgs::msg::Range
        >::SharedPtr publisher)
    {
        int fd = openSerial(sensor.port);

        if (fd < 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to initialize %s",
                sensor.port.c_str());

            return;
        }


        uint8_t previous_byte = 0;
        uint8_t current_byte = 0;


        while (
            rclcpp::ok() &&
            running_)
        {
            if (!readByte(fd, current_byte)) {
                continue;
            }


            /*
             * Search for TFmini frame header:
             *
             * 0x59 0x59
             */

            if (
                previous_byte == 0x59 &&
                current_byte == 0x59)
            {
                uint8_t data[7];


                if (!readBytes(
                        fd,
                        data,
                        sizeof(data)))
                {
                    continue;
                }


                /*
                 * Check checksum
                 */

                if (!verifyChecksum(data)) {

                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        5000,
                        "Checksum error on %s",
                        sensor.port.c_str());

                    previous_byte = current_byte;

                    continue;
                }


                /*
                 * Distance:
                 *
                 * Dist_L + Dist_H << 8
                 *
                 * Unit = cm
                 */

                uint16_t distance_cm =
                    static_cast<uint16_t>(data[0]) |
                    (static_cast<uint16_t>(data[1]) << 8);


                float distance_m =
                    static_cast<float>(
                        distance_cm) / 100.0f;


                /*
                 * Publish Range
                 */

                auto message =
                    sensor_msgs::msg::Range();


                message.header.stamp =
                    this->get_clock()->now();

                message.header.frame_id =
                    sensor.frame_id;


                message.radiation_type =
                    sensor_msgs::msg::Range::INFRARED;


                message.field_of_view =
                    0.035f;


                message.min_range =
                    0.30f;


                message.max_range =
                    12.0f;


                message.range =
                    distance_m;


                publisher->publish(message);
            }


            previous_byte = current_byte;
        }


        close(fd);


        RCLCPP_INFO(
            this->get_logger(),
            "Closed %s",
            sensor.port.c_str());
    }
};


int main(
    int argc,
    char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<TFminiPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}