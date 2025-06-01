#ifndef NET_CONTROLLER_HPP
#define NET_CONTROLLER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <thread>

class NetController {
public:
    NetController(int device_id, std::string server_ip, int server_port);

    /**
     * @brief  하나의 MJPEG 이미지 데이터를 여러 개의 UDP 패킷으로 나누고
     *         각 패킷 앞에 헤더를 붙여 전송. 
     * @param mjpeg_data MJPEG로 인코딩된 이미지 데이터
     * @note 헤더 구조:
     *       [0-3] device_id: 장치 고유 식별자
     *       [4-7] frame_id: 이미지 프레임 고유 식별자
     *       [8-9] chunk_idx: 패킷 인덱스
     *       [10-11] total_chunks: 전체 패킷 수
     * @return 이미지 전체를 전송한 경우 true,
     *         전송 중 네트워크 오류가 5회 이상 발생한 경우 false
     */
    bool sendMjpegData(std::vector<uint8_t> mjpeg_data);
private:
    const int device_id_;

    int sockfd_;
    struct sockaddr_in server_addr_;
    const std::string server_ip_;
    const int server_port_;
    
    int frame_id_ = 0;
    const int HEADER_SIZE = 12;
    const int MAX_UDP_PAYLOAD = 65000;
    const int MAX_CHUNK_SIZE = (MAX_UDP_PAYLOAD - HEADER_SIZE);
};

#endif