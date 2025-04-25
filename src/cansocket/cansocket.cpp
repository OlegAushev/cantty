#include "cansocket.hpp"
#include <netdb.h>
#include <sys/types.h>

#include <boost/process.hpp>

namespace can {

Socket::Socket(std::ostream& output_stream) : ostream_(output_stream) {
    // check can0: may be interface is already enabled
    // FIND SCRIPT
    ostream_ << "Searching for SocketCAN checking script...\n";
    std::filesystem::path script_path = find_script("socketcan_check.sh");
    if (script_path.empty()) {
        ostream_ << "Failed to find SocketCAN checking script.\n";
        return;
    }

    ostream_ << std::format("Found SocketCAN checking script: {}\n",
                            script_path.string());

    /* RUN SCRIPT */
    std::string cmd = "sh " + script_path.string() + " " + "can0" + " 2>&1";
    ostream_ << std::format(
            "Checking SocketCAN interface can0, executing system command: {}\n",
            cmd);

    auto script_result = boost::process::system(cmd);

    if (script_result == 0) {
        if (create_socket("can0") != Status::ok) {
            socket_ = -1;
        }
    }
}

Socket::~Socket() {
    if (socket_ >= 1) {
        disconnect();
    }
}

Status Socket::create_socket(const std::string& interface) {
    /* CREATE SOCKET */
    ostream_ << "Creating CAN socket...\n";
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
        ostream_ << "Failed to create CAN socket.\n";
        return Status::socket_creation_failed;
    }

    std::strcpy(ifreq_.ifr_name, interface.c_str());
    if (ioctl(socket_, SIOCGIFINDEX, &ifreq_) < 0) {
        ostream_ << "Failed to retrieve CAN interface.\n";
        return Status::interface_retrieving_failed;
    }

    memset(&sockaddr_, 0, sizeof(sockaddr_));
    sockaddr_.can_family = AF_CAN;
    sockaddr_.can_ifindex = ifreq_.ifr_ifindex;

    can_filter filter[1];
    filter[0].can_id = 0;
    filter[0].can_mask = 0x000;
    setsockopt(socket_,
               SOL_CAN_RAW,
               CAN_RAW_FILTER,
               filter,
               sizeof(can_filter));

    if (bind(socket_,
             reinterpret_cast<sockaddr*>(&sockaddr_),
             sizeof(sockaddr_)) < 0) {
        ostream_ << "Failed to bind CAN socket.\n";
        return Status::socket_binding_failed;
    }

    pollfd_.fd = socket_;
    pollfd_.events = POLLIN;

    ostream_ << "Created CAN socket.\n";
    return Status::ok;
}

Status Socket::connect(const std::string& interface,
                       const std::string& bitrate) {
    socket_ = -1;

    if (std::find(interfaces.begin(), interfaces.end(), interface) ==
                interfaces.end() ||
        std::find(bitrates.begin(), bitrates.end(), bitrate) ==
                bitrates.end()) {
        return Status::invalid_argument;
    }

    std::lock_guard<std::mutex> lock1(send_mtx_);
    std::lock_guard<std::mutex> lock2(recv_mtx_);

    /* FIND SCRIPT */
    ostream_ << "Searching for SocketCAN enabling script...\n";

    std::filesystem::path script_path = find_script("socketcan_enable.sh");
    if (script_path.empty()) {
        ostream_ << "Failed to find SocketCAN enabling script.\n";
        return Status::script_not_found;
    }

    ostream_ << std::format("Found SocketCAN enabling script: {}\n",
                            script_path.string());

    /* RUN SCRIPT */
    std::string cmd = "pkexec sh " + script_path.string() + " " + interface +
                      " " + bitrate + " 2>&1";
    ostream_ << std::format(
            "Enabling SocketCAN interface {}, executing system command: {}\n",
            interface,
            cmd);

    auto pkexec_result = boost::process::system(cmd);

    Status error;
    switch (pkexec_result) {
    case 0:
        error = Status::ok;
        break;
    case 1:
        error = Status::invalid_argument;
        break;
    case 2:
        error = Status::device_not_found;
        break;
    case 3:
        error = Status::socketcan_init_failed;
        break;
    default:
        error = Status::script_exec_failed;
        break;
    }

    if (error != Status::ok) {
        ostream_ << std::format(
                "Failed to enable SocketCAN interface. Error code: {}\n",
                std::to_underlying(error));
        return error;
    }

    return create_socket(interface);
}

Status Socket::disconnect() {
    if (socket_ < 0) {
        ostream_ << "Failed to close CAN socket: no socket.\n";
        return Status::socket_closed;
    }

    std::lock_guard<std::mutex> lock1(send_mtx_);
    std::lock_guard<std::mutex> lock2(recv_mtx_);

    if (close(socket_) < 0) {
        ostream_ << "Failed to close CAN socket.\n";
        return Status::socket_closing_failed;
    } else {
        ostream_ << "Closed socket.\n";
        socket_ = -1;
        return Status::ok;
    }
}

std::filesystem::path Socket::find_script(std::filesystem::path name) {
    std::filesystem::path script_path;
    for (auto loc : script_locations) {
        auto absolute_path = std::filesystem::absolute(loc / name);
        if (std::filesystem::exists(absolute_path)) {
            script_path = std::filesystem::canonical(loc / name);
        }
    }
    return script_path;
}

Status Socket::send(const can_frame& frame) {
    if (socket_ < 0) {
        return Status::socket_closed;
    }

    std::lock_guard<std::mutex> lock(send_mtx_);

    if (::write(socket_, &frame, sizeof(can_frame)) != sizeof(can_frame)) {
        return Status::send_error;
    }
    return Status::ok;
}

Status Socket::recv(can_frame& frame) {
    if (socket_ < 0) {
        return Status::socket_closed;
    }

    ssize_t byte_count;

    std::lock_guard<std::mutex> lock(recv_mtx_);

    int ret = poll(&pollfd_, 1, recv_timeout_.count());
    switch (ret) {
    case -1:
        return Status::recv_error;
        break;
    case 0:
        return Status::recv_timeout;
        break;
    default:
        byte_count = ::read(socket_, &frame, sizeof(can_frame));
        if (byte_count < 0) {
            return Status::recv_error;
        } else {
            return Status::ok;
        }
        break;
    }
}

} // namespace can
