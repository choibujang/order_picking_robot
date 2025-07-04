#include "controllers/net_controller.hpp"

NetController::NetController(int device_id, std::string server_ip, int server_port)
    : device_id_(device_id),
      server_ip_(server_ip),
      server_port_(server_port) {
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    throw std::runtime_error("Failed to create socket");
  }

  memset(&server_addr_, 0, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(server_port_);
  if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr) <= 0) {
    ::close(sockfd_);
    throw std::runtime_error("Invalid server IP address");
  }
}

NetController::~NetController() {
    if (sockfd_ > 0) {
        ::close(sockfd_);
    }
}

void NetController::sendMjpegData(const std::vector<uint8_t>& mjpeg_data) {
    if (mjpeg_data.empty()) {
        return;
    }

    int total_chunks = (mjpeg_data.size() + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;
    int send_failures = 0;

    for (int i = 0; i < total_chunks; ++i) {
        int offset = i * MAX_CHUNK_SIZE;
        int chunk_size = std::min(MAX_CHUNK_SIZE, static_cast<int>(mjpeg_data.size()) - offset);
        
        std::vector<uint8_t> packet(HEADER_SIZE + chunk_size);
        
        uint32_t net_device_id = htonl(device_id_);
        uint32_t net_frame_id = htonl(frame_id_);
        uint16_t net_chunk_idx = htons(i);
        uint16_t net_total_chunks = htons(total_chunks);

        memcpy(&packet[0], &net_device_id, 4);
        memcpy(&packet[4], &net_frame_id, 4);
        memcpy(&packet[8], &net_chunk_idx, 2);
        memcpy(&packet[10], &net_total_chunks, 2);

        memcpy(&packet[HEADER_SIZE], mjpeg_data.data() + offset, chunk_size);

        ssize_t sent_bytes = sendto(sockfd_, packet.data(), packet.size(), 0,
                                    reinterpret_cast<const struct sockaddr*>(&server_addr_), sizeof(server_addr_));
        
        if (sent_bytes < 0) {
            send_failures++;
            if (send_failures >= kMaxSendFailures) {
                throw std::runtime_error("Network error: Failed to send UDP packet after multiple retries.");
            }
        } else {
            send_failures = 0; 
        }
    }
    frame_id_++;
}
