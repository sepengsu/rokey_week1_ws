import threading
import queue

class ActionThread(threading.Thread):
    def __init__(self, node):
        super().__init__(daemon=True)
        self.node = node
        self.action_queue = queue.Queue()
        self.running = True

    def run(self):
        while self.running:
            try:
                event = self.action_queue.get(timeout=0.1)
                if event["type"] == "action_status":
                    self.handle_action_status(event)
            except queue.Empty:
                continue

    def handle_action_status(self, event):
        self.node.get_logger().info(f"[Action] Status: {event['status']} for goal {event['goal_index']}")

    def add_event(self, event):
        """큐에 이벤트 추가."""
        self.action_queue.put(event)

    def stop(self):
        self.running = False


class ServiceThread(threading.Thread):
    def __init__(self, node):
        super().__init__(daemon=True)
        self.node = node
        self.service_queue = queue.Queue()
        self.running = True

    def run(self):
        while self.running:
            try:
                event = self.service_queue.get(timeout=0.1)
                if event["type"] == "order_request":
                    self.handle_service_request(event)
            except queue.Empty:
                continue

    def handle_service_request(self, event):
        self.node.get_logger().info(f"[Service] Processing request for Table {event['table_index']}")

    def add_event(self, event):
        """큐에 이벤트 추가."""
        self.service_queue.put(event)

    def stop(self):
        self.running = False
