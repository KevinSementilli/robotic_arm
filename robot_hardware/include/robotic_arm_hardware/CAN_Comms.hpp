#ifndef ROBOTIC_ARM_CAN_COMMS_HPP
#define ROBOTIC_ARM_CAN_COMMS_HPP

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>
#include <cstring>
#include <iostream>

class CANComms
{
public:

    CANComms(const std::string &interface_name = "can0")
    {
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0)
        {
            perror("Socket creation failed");
            throw std::runtime_error("Failed to create CAN socket");
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ);
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
        {
            perror("Getting interface index failed");
            close(sock_);
            throw std::runtime_error("Failed to get interface index");
        }

        addr_.can_family = AF_CAN;
        addr_.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0)
        {
            perror("Bind failed");
            close(sock_);
            throw std::runtime_error("Failed to bind CAN socket");
        }
    }

    ~CANComms()
    {
        if (sock_ >= 0)
        {
            close(sock_);
        }
    }

    bool send_frame(uint32_t can_id, std::vector<uint8_t> data)
    {
        if (data.size() > 7) {
            std::cerr << "[CANComms] Data too long to append CRC.\n";
            return false;
        }

        uint8_t crc = can_id & 0xFF; // Only 1-byte node ID is used for CRC
        for (const auto& byte : data) crc += byte;
        crc &= 0xFF;
        data.push_back(crc);

        if (!is_valid(can_id, data)) {
            std::cerr << "[CANComms] Invalid CAN frame — not sending.\n";
            return false;
        }

        struct can_frame frame{};
        frame.can_id = can_id;
        frame.can_dlc = data.size();
        std::copy(data.begin(), data.end(), frame.data);

        ssize_t nbytes = write(sock_, &frame, sizeof(frame));
        return (nbytes == sizeof(frame));
    }

    bool receive_frame(struct can_frame &received_frame, int timeout_ms = 100)
    {
        fd_set read_set;
        struct timeval timeout;
        FD_ZERO(&read_set);
        FD_SET(sock_, &read_set);
        timeout.tv_sec = 0;
        timeout.tv_usec = timeout_ms * 1000;

        int ret = select(sock_ + 1, &read_set, nullptr, nullptr, &timeout);
        if (ret > 0 && FD_ISSET(sock_, &read_set))
        {
            ssize_t nbytes = read(sock_, &received_frame, sizeof(received_frame));
            return (nbytes == sizeof(received_frame));
        }
        return false;
    }

private:
    int sock_{-1};
    struct sockaddr_can addr_{};

    bool is_valid(uint32_t can_id, const std::vector<uint8_t> &data)
    {
        // Standard CAN ID must be ≤ 0x7FF (11 bits)
        if (can_id > 0x7FF)
        {
            std::cerr << "[CANComms] Invalid CAN ID: " << std::hex << can_id << "\n";
            return false;
        }

        if (data.size() > 8)
        {
            std::cerr << "[CANComms] Invalid data length: " << data.size() << " (must be ≤ 8)\n";
            return false;
        }
        return true;
    }

};

#endif // ROBOTIC_ARM_CAN_COMMS_HPP