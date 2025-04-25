#pragma once

#include <cstring>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <format>
#include <mutex>
#include <ostream>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace can {

enum class Status {
    ok,
    invalid_argument,
    script_not_found,
    device_not_found,
    socketcan_init_failed,
    script_exec_failed,
    socket_creation_failed,
    socket_closing_failed,
    socket_binding_failed,
    socket_closed,
    send_error,
    recv_timeout,
    recv_error,
    interface_retrieving_failed,
};

inline std::vector<std::string_view> const interfaces = {"can0", "can1"};
inline std::vector<std::string_view> const bitrates = {"125000",
                                                       "250000",
                                                       "500000",
                                                       "1000000"};
inline std::set<std::filesystem::path> const script_locations = {"",
                                                                 "scripts",
                                                                 "..",
                                                                 "../scripts"};

class Socket {
private:
    int socket_{-1};
    ifreq ifreq_;
    sockaddr_can sockaddr_;

    pollfd pollfd_;
    static constexpr std::chrono::milliseconds recv_timeout_{1};

    std::mutex send_mtx_;
    std::mutex recv_mtx_;

    std::ostream& ostream_;
public:
    Socket(std::ostream& output_stream);
    ~Socket();
    Socket(Socket const& other) = delete;
    Socket& operator=(Socket const& other) = delete;

    Status connect(const std::string& interface, const std::string& bitrate);
    Status disconnect();

    Status send(can_frame const& frame);
    Status recv(can_frame& frame);

    bool good() const { return socket_ >= 0; }
private:
    Status create_socket(std::string const& interface);
    std::filesystem::path find_script(std::filesystem::path name);
};

} // namespace can
