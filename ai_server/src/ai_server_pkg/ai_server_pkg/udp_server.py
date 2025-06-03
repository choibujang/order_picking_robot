import socket
from queue import Full
from collections import defaultdict
import time

class UDPServer:
    def __init__(self, frame_queue):
        """
        UDP 기반으로 여러 청크로 분할 전송된 프레임을 재조립하여 frame_queue에 삽입하는 클래스.

        전송된 패킷 헤더에는 device_id, frame_id, chunk_idx, total_chunks 정보가 담겨 있으며
        최대 지정된 대기시간(MAX_WAIT_TIME) 안에 모든 청크가 도착하지 않으면
        일정 비율(RECEIVE_THRESHOLD) 이상 수신된 경우에는 강제로 재조립하여 큐에 넣고
        그렇지 않으면 해당 프레임 데이터를 폐기한다.

        Attributes:
            frame_queue (queue.Queue): 이미지 프레임 데이터를 넣을 큐
                {'device_id': int, 'data': bytes}
        """
        self.port = 8080
        self.frame_queue = frame_queue

        self.HEADER_SIZE = 12
        self.MAX_UDP_PAYLOAD = 65000
        self.MAX_WAIT_TIME = 0.2
        self.RECEIVE_THRESHOLD = 80

        # { (device_id, frame_id) : { 'chunks': {}, 'total': N, 'start': time.time() } }
        self.recv_buffers = defaultdict(lambda: { 'chunks': {}, 'total': None, 'start': time.time() })

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.port))

        print(f"Start UDP server on:{self.port}")

    def receiver(self):
        """
        UDP로 들어오는 청크들을 수신하여 재조립 버퍼에 저장하고
        모든 청크가 모이면 complete_and_enqueue를 호출하여 프레임을 완성한 뒤 큐에 넣는다.
        """
        while True:
            # 최대 MAX_UDP_PAYLOAD 바이트 수신
            data, addr = self.sock.recvfrom(self.MAX_UDP_PAYLOAD)
            if len(data) < self.HEADER_SIZE:
                continue

            # 헤더 파싱
            device_id = int.from_bytes(data[0:4], 'little')
            frame_id = int.from_bytes(data[4:8], 'little')
            chunk_idx = int.from_bytes(data[8:10], 'little')
            total_chunks = int.from_bytes(data[10:12], 'little')
            payload = data[self.HEADER_SIZE:]

            # 재조립 버퍼에 저장
            key = (device_id, frame_id)
            buf = self.recv_buffers[key]

            buf['chunks'][chunk_idx] = payload
            buf['total'] = total_chunks
            buf['start'] = buf.get('start', time.time())

            #  지금까지 수신된 청크 개수가 total_chunks와 같으면 재조립 시도
            if len(buf['chunks']) == total_chunks:
                self.complete_and_enqueue(key)

    def complete_and_enqueue(self, key):
        """
        특정 (device_id, frame_id) 버퍼에 모인 청크를 순서대로 합쳐서
        완전한 이미지 바이트를 만들어 frame_queue에 삽입한다.

        Args:
            key (tuple): (device_id, frame_id) 쌍으로 구성된 키
        """
        buf = self.recv_buffers[key]
        chunks = buf['chunks']

        # 청크 인덱스 순서대로 바이트를 이어붙여 전체 이미지 재조립
        data = b''.join([chunks[i] for i in sorted(chunks.keys())])

        # 큐에 삽입. 이미 큐가 가득 차 있으면 한 개 꺼내고 다시 삽입
        try:
            self.frame_queue.put({ 'device_id': key[0], 'frame_id': key[1], 'data': data }, block=False)
        except Full:
            self.frame_queue.get()
            self.frame_queue.put({ 'device_id': key[0], 'frame_id': key[1], 'data': data }, block=False)

        # 재조립이 완료된 버퍼 삭제
        del self.recv_buffers[key]

    def buffer_cleaner(self):
        """
        일정 주기로 recv_buffers를 스캔하여 MAX_WAIT_TIME 초가 지난 버퍼를 처리한다.
        - 경과시간 > MAX_WAIT_TIME인 경우
            a) 수신된 청크 개수가 전체 청크 개수의 RECEIVE_THRESHOLD(%) 이상이면
               complete_and_enqueue(key)를 호출하여 부분 데이터라도 프레임으로 조립한다.
            b) 그렇지 않으면 해당 버퍼를 삭제한다.
        """
        while True:
            now = time.time()
            expired = []

            # 각 키별 버퍼 정보를 체크
            for key, buf in list(self.recv_buffers.items()):
                elapsed = now - buf['start']
                
                # 버퍼 생성 시간으로부터 경과 시간이 MAX_WAIT_TIME 초 초과한 경우
                if elapsed > self.MAX_WAIT_TIME:
                    if len(buf['chunks']) >= buf['total'] * self.RECEIVE_THRESHOLD:
                        # 청크 비율이 RECEIVE_THRESHOLD 이상이면 재조립
                        self.complete_and_enqueue(key)
                    else:
                        # 재조립 조건 미달, 폐기
                        print(f"Dropped frame {key} (only {len(buf['chunks'])}/{buf['total']})")
                        expired.append(key)

            # drop 대상 버퍼 키 삭제
            for key in expired:
                if key in self.recv_buffers:
                    del self.recv_buffers[key]

            time.sleep(0.05)