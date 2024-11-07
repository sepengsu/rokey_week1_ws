import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append("/home/jaenote/rokey_ros/src/week1")
import rclpy
import threading
from rclpy.node import Node
from ros_msgs.srv import OrderService  # OrderService 서비스 메시지 임포트
from tkinter import Toplevel
import tkinter as tk
from tkinter import simpledialog
from .order_service import MenuNode  # MenuNode를 가져옴
from .data.table_utils import Show, Insert, Delete
import sqlite3
import time


class KitGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("테이블 및 좌석 현황 GUI")
        self.root.geometry("1600x1000")

        self.orders = []  # FIFO 큐: {"table": 테이블 번호, "order_detail": 주문 내용, "eta": 남은 시간}
        self.timers = {}  # 테이블별 타이머 저장
        self.table_labels = {}  # 테이블 번호와 Label 매핑
        self.condition = threading.Condition()  # ROS2 서비스 응답을 위한 조건 변수

        # 테이블 주문 현황 표시 프레임
        self.main_frame = tk.Frame(self.root, borderwidth=2, relief="solid")
        self.main_frame.grid(row=0, column=0, padx=10, pady=10)

        # 데이터베이스에서 테이블 주문 정보 불러오기
        self.display_table_orders()

        # 메인 그리드 라벨
        self.main_label = tk.Label(self.root, text="테이블별 주문 현황", font=("Arial", 14))
        self.main_label.grid(row=1, column=0, pady=5)

        # FIFO 주문 큐 표시
        self.fifo_frame = tk.Frame(self.root, borderwidth=2, relief="solid")
        self.fifo_frame.grid(row=0, column=1, padx=10, pady=10)
        self.fifo_label = tk.Label(self.fifo_frame, text="주문 큐", font=("Arial", 14))
        self.fifo_label.pack()
        self.fifo_listbox = tk.Listbox(self.fifo_frame, height=15, width=30)
        self.fifo_listbox.pack(padx=10, pady=10)

    def display_table_orders(self):
        """테이블 라벨 생성 및 초기화."""
        for i in range(3):
            for j in range(3):
                table_index = i * 3 + j + 1

                # 테이블 라벨 생성 및 저장
                table_label = tk.Label(self.main_frame, text=f"테이블 {table_index}\n주문 없음",
                                       width=15, height=5, borderwidth=1, relief="solid")
                table_label.grid(row=i, column=j, padx=5, pady=5)
                self.table_labels[table_index] = table_label  # 딕셔너리에 저장


    def show_order_popup(self, table_index, message):
        """주문 요청 팝업 창."""
        # 팝업 창 생성
        popup = Toplevel(self.root)
        popup.geometry("400x200")
        popup.title(f"Table {table_index}에서의 새 주문 요청")

        # 주문 메시지 레이블
        label = tk.Label(popup, text=message, font=("Arial", 12))
        label.pack(pady=10)

        # 버튼 프레임
        button_frame = tk.Frame(popup)
        button_frame.pack(pady=10)

        # 수락 버튼 생성 (accept_order 호출)
        accept_button = tk.Button(
            button_frame,
            text="수락",
            command=lambda: self.handle_accept_order(table_index, message, popup)
        )
        accept_button.pack(side="left", padx=5)

        # 거절 버튼 생성 (cancel_order 호출)
        reject_button = tk.Button(
            button_frame,
            text="거절",
            command=lambda: self.handle_cancel_order(table_index, popup))
        reject_button.pack(side="right", padx=5)

    def handle_accept_order(self, table_index, message, popup):
        """주문 수락 처리."""
        popup.destroy()  # 팝업 닫기
        self.accept_order(table_index, message)  # 주문 수락 처리
        
        # ROS2 서비스 응답 설정
        with self.condition:
            self.condition.notify()

    def handle_cancel_order(self, table_index, popup):
        """주문 거절 처리."""
        popup.destroy()  # 팝업 닫기
        self.cancel_order(table_index)  # 주문 취소 처리

    def accept_order(self, table_index, order_detail):
        """주문 수락 및 타이머 시작."""
        if table_index not in self.table_labels:
            print(f"[ERROR] Invalid table index: {table_index}")
            return

        # 주문 정보 GUI 업데이트
        table_label = f"테이블 {table_index}\n{order_detail}\nETA: 10분"
        self.table_labels[table_index].config(text=table_label)

        # 타이머 설정
        timer = self.start_timer(table_index, 10)  # ETA 기본값 10분
        self.timers[table_index] = timer

        # 주문 큐에 추가
        self.orders.append({"table": table_index, "order_detail": order_detail, "eta": 10})
        self.update_fifo_listbox()

        # 주문 정보 데이터베이스에 저장
        insert = Insert()
        insert.insert_table_order(table_index, order_detail, time.strftime('%Y-%m-%d %H:%M:%S'))

        # ROS2 서비스 응답
        self.response = (True, f"테이블 {table_index}의 주문이 수락되었습니다. 예상 소요 시간: 10분.")
        
        # ROS2 서비스 응답 설정
        with self.condition:
            self.condition.notify()

        
    def cancel_order(self, table_index):
        """주문 취소."""
        if table_index not in self.table_labels:
            print(f"[ERROR] Invalid table index: {table_index}")
            return

        # 팝업 창 생성
        popup = Toplevel(self.root)
        popup.geometry("300x200")
        popup.title(f"테이블 {table_index} 주문 취소")

        # 안내 레이블
        label = tk.Label(popup, text="취소 사유를 선택하세요:", font=("Arial", 12))
        label.pack(pady=10)

        # 버튼 프레임
        button_frame = tk.Frame(popup)
        button_frame.pack(pady=10)

        # 취소 사유 리스트
        reasons = ["재료 부족", "고객 요청", "기타"]

        def handle_reason(reason):
            """취소 사유를 처리."""
            popup.destroy()
            # 타이머 제거
            if table_index in self.timers:
                del self.timers[table_index]

            # 테이블 라벨 업데이트
            table_label = self.table_labels[table_index]
            table_label.config(text=f"테이블 {table_index}\n주문 없음")

            # FIFO 큐에서 제거
            self.orders = [order for order in self.orders if order["table"] != table_index]
            self.update_fifo_listbox()

            # 데이터베이스에서 주문 삭제
            delete = Delete()
            delete.delete_table_order(table_index)

            # ROS2 서비스 응답
            self.response = (False, f"테이블 {table_index}의 주문이 취소되었습니다. 사유: {reason}")
            with self.condition:
                self.condition.notify()

        # 취소 사유 버튼 생성
        for reason in reasons:
            reason_button = tk.Button(
                button_frame,
                text=reason,
                command=lambda r=reason: handle_reason(r),
                width=20
            )
            reason_button.pack(pady=5)


    def start_timer(self, table_index, eta):
        """타이머 시작."""
        def countdown():
            nonlocal eta
            eta -= 1
            self.update_order_eta(table_index, eta)
            if eta <= 0:
                self.complete_order(table_index)
            else:
                self.root.after(60000, countdown)

        self.root.after(60000, countdown)
        return countdown

    def update_order_eta(self, table_index, eta):
        """ETA 업데이트."""
        if table_index not in self.table_labels:
            print(f"[ERROR] Invalid table index: {table_index}")
            return

        table_label = self.table_labels[table_index]
        current_text = table_label.cget("text")
        new_text = "\n".join(current_text.split("\n")[:-1]) + f"\nETA: {eta}분"
        table_label.config(text=new_text)

        # FIFO 큐에서도 업데이트
        for order in self.orders:
            if order["table"] == table_index:
                order["eta"] = eta
                break
        self.update_fifo_listbox()

    def complete_order(self, table_index):
        """주문 완료 처리."""
        if table_index not in self.table_labels:
            print(f"[ERROR] Invalid table index: {table_index}")
            return

        table_label = self.table_labels[table_index]
        table_label.config(text=f"테이블 {table_index}\n주문 없음")

    def update_fifo_listbox(self):
        """FIFO 큐 GUI 업데이트."""
        self.fifo_listbox.delete(0, tk.END)
        for order in self.orders:
            self.fifo_listbox.insert(tk.END, f"테이블 {order['table']} - {order['order_detail']} - ETA: {order['eta']}분")


class KitNode(Node):
    def __init__(self, gui: KitGUI):
        super().__init__("kit_node")
        self.gui = gui
        self.order_service = self.create_service(OrderService, 'order_service', self.handle_order_request)

    def handle_order_request(self, request, response):
        """주문 요청 처리."""
        table_index = request.table_index
        order_detail = request.order_detail

        # GUI에서 주문 요청 팝업 표시
        self.gui.show_order_popup(table_index, order_detail)

        # GUI 응답 대기
        with self.gui.condition:
            self.gui.condition.wait()  # 응답이 설정될 때까지 대기

        # 응답 설정
        response.success, response.message = self.gui.response
        print(f"ROS2 응답: {response}")
        return response


def main():
    rclpy.init()

    root = tk.Tk()
    gui = KitGUI(root)
    kit_node = KitNode(gui)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(kit_node)

    thread_executor = threading.Thread(target=executor.spin)
    thread_executor.start()

    root.protocol("WM_DELETE_WINDOW", lambda: on_close(kit_node, executor))
    root.mainloop()


def on_close(node, executor):
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
