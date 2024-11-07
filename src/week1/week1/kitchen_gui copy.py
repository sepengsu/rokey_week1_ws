import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append("/home/jaenote/rokey_ros/src/week1")
import rclpy
import threading
from rclpy.node import Node
from ros_msgs.srv import OrderService  # OrderService 서비스 메시지 임포트
from tkinter import messagebox, Toplevel
import tkinter as tk
import sqlite3
from .order_service import MenuNode  # MenuNode를 가져옴
from .data.table_utils import Show
from tkinter import simpledialog

class KitGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("테이블 및 좌석 현황 GUI")
        self.root.geometry("1600x1000")

        # 테이블 주문 현황 표시 프레임
        self.main_frame = tk.Frame(self.root, borderwidth=2, relief="solid")
        self.main_frame.grid(row=0, column=0, padx=10, pady=10)

        # 데이터베이스에서 테이블 주문 정보 불러오기
        self.display_table_orders()

        # 메인 그리드 라벨
        self.main_label = tk.Label(self.root, text="테이블별 주문 현황", font=("Arial", 14))
        self.main_label.grid(row=1, column=0, pady=5)

        # 3x3 숫자 버튼을 위한 프레임
        self.seat_frame = tk.Frame(self.root, borderwidth=2, relief="solid")
        self.seat_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")

        # 좌석 세부 사항 숫자 버튼 생성
        self.create_number_buttons(3, 3)  # 3x3 숫자 버튼

        # 좌석 세부 사항 라벨
        self.seat_label = tk.Label(self.root, text="좌석 세부 사항", font=("Arial", 14))
        self.seat_label.grid(row=1, column=1, pady=5)

    def display_table_orders(self):
        """데이터베이스에서 주문 데이터를 불러와 메인 그리드에 표시합니다."""
        show = Show()
        try :
            orders = show.show_table_orders()
        except sqlite3.OperationalError:
            orders = {i: [] for i in range(1, 10)}
        for i in range(3):
            for j in range(3):
                order_index = i * 3 + j
                order_info = orders[order_index] if order_index < len(orders) else ""
                table_label = tk.Label(self.main_frame, text=order_info,
                                       width=15, height=5, borderwidth=1, relief="solid")
                table_label.grid(row=i, column=j, padx=5, pady=5)

    def create_number_buttons(self, rows, cols):
        """숫자 버튼을 생성합니다."""
        for i in range(rows):
            for j in range(cols):
                num = i * cols + j + 1
                button = tk.Button(self.seat_frame, text=str(num), width=5, height=2,
                                   command=lambda n=num: self.prompt_transport(n))
                button.grid(row=i, column=j, padx=5, pady=5)

    def prompt_transport(self, number):
        """운반 작업 확인 창을 엽니다."""
        response = messagebox.askokcancel("운반 확인", f"{number}번에 운반하겠습니까?")
        if response:
            print(f"{number}번에 운반합니다.")
        '''
        운반 관련 action을 수행하는 코드를 여기에 추가
        '''

    def set_node(self, node):
        """KitNode 인스턴스를 GUI와 연결"""
        self.node = node

    def show_order_popup(self, message):
        """
        주문 요청에 대한 팝업 창을 띄웁니다.
        1. 주문 요청을 받아 GUI에 표시
        2. 주문 요청에 대한 수락/거절을 GUI에서 처리
        3. 주문 수락시 예상 소요 시간을 입력받아 서비스 response 반환
        4. 주문 거절시 거절 사유를 입력받아 서비스 response 반환
        """
        # 팝업 창 생성
        popup = Toplevel(self.root)
        popup.title("새 주문 요청")
        label = tk.Label(popup, text=message, font=("Arial", 12))
        label.pack(pady=10)
        
        # 버튼 프레임 생성
        button_frame = tk.Frame(popup)
        button_frame.pack(pady=5)

        # 수락 버튼
        accept_button = tk.Button(button_frame, text="수락", command=lambda: self.prompt_order(True))
        accept_button.pack(side="left", padx=5)

        # 거절 버튼
        reject_button = tk.Button(button_frame, text="거절", command=lambda: self.prompt_order(False))
        reject_button.pack(side="right", padx=5)
        

    def prompt_order(self, accept: bool):
        '''
        주문 요청에 대한 수락/거절 기반 서비스 return 및 DB 업데이트
        1. 주문 요청에 대한 수락/거절을 GUI에서 처리
        2. 주문 수락시 예상 소요 시간을 입력받아 서비스 response 반환
        3. 주문 거절시 거절 사유를 입력받아 서비스 response 반환
        '''
        if accept:
            # 주문 수락시 예상 소요 시간을 입력받음
            time_input = self.prompt_time_input()
            print(f"[INFO] 주문을 수락하였습니다. 예상 소요 시간: {time_input}")
            '''
            주문 수락 response를 수행하는 코드를 여기에 추가
            '''
        else:
            # 주문 거절시 거절 사유를 입력받음
            reason_input = self.prompt_reason_input()
            print(f"[INFO] 주문을 거절하였습니다. 거절 사유: {reason_input}")
            '''
            주문 거절 action을 수행하는 코드를 여기에 추가
            '''
    
    def prompt_time_input(self):
        """주문 수락시 예상 소요 시간을 입력받는 창을 띄웁니다."""
        time_input = simpledialog.askinteger("예상 소요 시간", "예상 소요 시간을 입력하세요 (분 단위)")
        return time_input
    
    def prompt_reason_input(self):
        """주문 거절시 거절 사유를 입력받는 창을 띄웁니다."""
        reason_input = simpledialog.askstring("거절 사유", "거절 사유를 입력하세요")
        return reason_input


class KitNode(Node):
    def __init__(self, gui: KitGUI):
        super().__init__("kit_node")
        self.gui = gui
        self.gui.set_node(self)

        # 주문 요청을 받는 서비스 서버 생성
        self.order_service = self.create_service(OrderService, 'order_service', self.handle_order_request)

    def handle_order_request(self, request, response):
        """
        MenuNode의 주문 요청을 처리
        1. 주문 요청을 받아 GUI에 표시
        2. 주문 요청에 대한 수락/거절을 GUI에서 처리
        3. 주문 요청 수락/거절에 대한 서비스 response 반환
        """
        # 주문 요청을 받아 GUI에 표시
        order_detail = request.order_detail
        print(f"[INFO] 주문 요청을 받았습니다: {order_detail}")
        self.gui.show_order_popup(order_detail)  # 주문 창을 GUI에서 띄움

        # 주문 요청에 대한 수락/거절을 GUI에서 처리
        response.success = True
        response.message = "Order received and displayed in GUI."
        return response
    
        
    

# GUI 및 ROS 2 노드 실행
def main():
    rclpy.init()

    root = tk.Tk()
    gui = KitGUI(root)
    kit_node = KitNode(gui)  # KitNode 생성 및 GUI에 전달
    menu_node = MenuNode()  # MenuNode 생성

    # MultiThreadedExecutor를 사용하여 두 노드를 동시에 실행
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(kit_node)
    executor.add_node(menu_node)
    
    # 새로운 스레드에서 MultiThreadedExecutor 실행
    thread_executor = threading.Thread(target=executor.spin)
    thread_executor.start()
    
    # GUI 종료 시 모든 노드를 정리
    root.protocol("WM_DELETE_WINDOW", lambda: on_close(kit_node, menu_node, executor))
    root.mainloop()

def on_close(*nodes_and_executor):
    executor = nodes_and_executor[-1]
    nodes = nodes_and_executor[:-1]
    
    for node in nodes:
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == "__main__":
    main()
