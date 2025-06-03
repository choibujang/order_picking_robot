import cv2
import numpy as np
from ultralytics import YOLO
import time
from collections import defaultdict
import copy
from queue import Full

class AIEngine:
    def __init__(self, frame_queue, detected_objects, lock):
        """
        AI 기반 물체 인식을 담당하는 클래스
        - Ultralytics YOLO 모델을 사용하여 전달된 프레임에 대해 실시간 물체 검출을 실행한다.
        - 검출 결과를 device_id별로 공유 딕셔너리에 저장한다.

        Attributes:
            frame_queue (queue.Queue): 이미지 프레임 데이터를 받아올 큐
                {'device_id': int, 'data': bytes}

            detected_objects (dict): 검출 결과를 저장할 공유 딕셔너리. 
                키: device_id (int)
                값: {
                    class_name (str): {
                        'count': int,         # 해당 프레임에서 검출된 객체 수
                        'pixels': [(x,y), ...] # 검출된 객체 중심 좌표 리스트
                    },
                    ...
                }

            lock (threading.Lock): detected_objects 딕셔너리 접근 시 락을 걸기 위한 락 객체.
        """
        self.model = YOLO("../resource/best.pt")
        self.frame_queue = frame_queue
        self.lock = lock
        self.detected_objects = detected_objects

    def run_detection(self):
        """
        frame_queue에서 이미지를 가져와 YOLO 모델로 물체 검출을 수행한 뒤
        검출된 객체 정보를 공유 딕셔너리 detected_objects에 저장한다.
        """
        while True:
            # frame_queue에 새 프레임이 들어올때까지 블로킹 대기
            frame = self.frame_queue.get()
            device_id = frame['device_id']
            image_bytes = frame['data']

            # MJPEG 바이트 데이터를 이미지로 디코딩
            np_arr = np.frombuffer(image_bytes, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is None:
                continue

            # 객체 검출
            try:
                results = self.model.predict(img, verbose=False)
            except Exception as e:
                print(f"[ERROR] YOLO failed: {e}")
                continue
            
            # 검출된 객체들에 대해 클래스 이름, 개수, 중심 좌표를 공유 딕셔너리에 저장
            detections = results[0].boxes

            buffer = defaultdict(lambda: {'count' : 0, 'pixels' : []})

            if len(detections) > 0:
                for det in detections:
                    x1, y1, x2, y2 = map(int, det.xyxy[0])
                    x_center = (x1 + x2) // 2
                    y_center = (y1 + y2) // 2
                    class_id = int(det.cls[0]) 
                    class_name = self.model.names[class_id]

                    buffer[class_name]['count'] += 1
                    buffer[class_name]['pixels'].append((x_center, y_center))

                    cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.circle(img, (x_center, y_center), 3, (0, 0, 255), -1)
                    cv2.putText(img, class_name, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            with self.lock:
                self.detected_objects[device_id] = dict(buffer)

            cv2.imshow("AI Server Detection", img)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        cv2.destroyAllWindows()