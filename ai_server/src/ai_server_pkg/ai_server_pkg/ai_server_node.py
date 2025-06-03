import threading
from queue import Queue

import rclpy
from rclpy.node import Node

from ai_server_pkg.udp_server import UDPServer
from ai_server_pkg.ai_engine import AIEngine

from ros_interfaces.srv import GetDetectedObjects

class AIServerNode(Node):
    def __init__(self):
        """
        ROS2 노드로서 UDP로 전송된 영상 청크를 조립하고 AI 엔진으로 물체를 검출하여
        서비스 요청에 검출 결과를 응답하는 역할을 수행한다.
        """
        super().__init__('ai_server_node')
        # UDP로 수신된 완전한 프레임을 보관할 큐
        # {'device_id': int, 'data': bytes}
        self.frame_queue = Queue(maxsize=5)

        # 각 device_id 별로 최근 검출된 클래스명, 개수, 픽셀 좌표를 저장하는 공유 딕셔너리
        # { device_id : { 'banana' : { 'count' : 2, 'pixels' : [(300, 200), (350, 210)] } } }
        self.detected_objects = {}
        self.lock = threading.Lock()

        self.udp_server = UDPServer(self.frame_queue)
        self.udp_server_thread = threading.Thread(target=self.udp_server.receiver)
        self.udp_server_thread.start()
        self.buf_clean_thread = threading.Thread(target=self.udp_server.buffer_cleaner)
        self.buf_clean_thread.start()

        self.ai_engine = AIEngine(self.frame_queue, self.detected_objects, self.lock)
        self.ai_engine_thread = threading.Thread(target=self.ai_engine.run_detection)
        self.ai_engine_thread.start()

        self.srv = self.create_service(
            GetDetectedObjects,
            'get_detected_objects',
            self.handle_request
        )

    def handle_request(self, request, response):
        """
        GetDetectedObjects 서비스 요청 핸들러

        Args:
            request
                - device_id: 어떤 장치(device_id)에 대한 검출 결과를 요청했는지 식별
            response
                - class_names: 검출된 클래스명 리스트
                - counts: 각 클래스가 검출된 개수 리스트
                - pixel_x, pixel_y: 각 검출 객체의 중심 좌표 리스트
                - pixel_class_indices: pixel_x[i], pixel_y[i]가 어느 클래스(class_names[idx])에 속하는지 인덱스 리스트
        """
        device_id = request.device_id

        with self.lock:
            device_data = self.detected_objects.get(device_id)

        # 해당 device_id에 검출 결과가 없으면 빈 리스트로 응답
        if not device_data:
            response.class_names = []
            response.counts = []
            response.pixel_x = []
            response.pixel_y = []
            response.pixel_class_indices = []
            return response

        class_names = []
        counts = []
        pixel_x = []
        pixel_y = []
        pixel_class_indices = []

        for idx, (class_name, data) in enumerate(device_data.items()):
            class_names.append(class_name)
            counts.append(data['count'])
            for x, y in data['pixels']:
                pixel_x.append(x)
                pixel_y.append(y)
                pixel_class_indices.append(idx)

        response.class_names = class_names
        response.counts = counts
        response.pixel_x = pixel_x
        response.pixel_y = pixel_y
        response.pixel_class_indices = pixel_class_indices
        return response

def main():
    rclpy.init()
    node = AIServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
