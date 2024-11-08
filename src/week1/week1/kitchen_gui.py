import sys
import threading
import tkinter as tk
from tkinter import Toplevel, messagebox
import queue  # GUI와 Node 간 이벤트 전달용 큐
import rclpy
from rclpy.node import Node
from .data.table_utils import Show, Insert, Delete
from ros_msgs.srv import OrderService  # OrderService 서비스 메시지 임포트
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from rclpy.action.client import GoalStatus
from std_msgs.msg import String
from rclpy.action import ActionClient

class KitGUI:
    def __init__(self, root, node, event_queue):
        self.root = root
        self.node = node  # Node 참조
        self.event_queue = event_queue  # 이벤트 큐
        self.root.title("테이블 및 좌석 현황 GUI")
        self.root.geometry("1600x1000")

        self.orders = []  # FIFO 큐
        self.timers = {}  # 테이블별 타이머 저장
        self.table_labels = {}  # 테이블 번호와 Label 매핑

        # 테이블 주문 현황 표시 프레임
        self.main_frame = tk.Frame(self.root, borderwidth=2, relief="solid")
        self.main_frame.grid(row=0, column=0, padx=10, pady=10)
        self.display_table_orders()

        # FIFO 주문 큐 표시
        self.fifo_frame = tk.Frame(self.root, borderwidth=2, relief="solid")
        self.fifo_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")
        self.fifo_label = tk.Label(self.fifo_frame, text="주문 큐", font=("Arial", 14))
        self.fifo_label.pack()
        self.fifo_listbox = tk.Listbox(self.fifo_frame, height=15, width=30)
        self.fifo_listbox.pack(padx=10, pady=10)

        # 숫자 버튼 생성 (3x3)
        self.seat_frame = tk.Frame(self.root, borderwidth=2, relief="solid")
        self.seat_frame.grid(row=0, column=2, padx=10, pady=10, sticky="n")
        self.create_number_buttons(3, 3)

        # "오늘의 매출" 버튼
        self.sales_button = tk.Button(self.root, text="오늘의 매출", font=("Arial", 14), command=self.show_today_sales)
        self.sales_button.grid(row=2, column=0, columnspan=3, pady=10)

        # 주기적으로 이벤트 큐 확인
        self.poll_events()

    def display_table_orders(self):
        """테이블 주문 현황 초기화."""
        for i in range(3):
            for j in range(3):
                table_index = i * 3 + j + 1
                table_label = tk.Label(self.main_frame, text=f"테이블 {table_index}\n주문 없음",
                                       width=15, height=5, borderwidth=1, relief="solid")
                table_label.grid(row=i, column=j, padx=5, pady=5)
                self.table_labels[table_index] = table_label

    def show_order_popup(self, table_index, message, response):
        """주문 요청 팝업"""
        popup = Toplevel(self.root)
        popup.geometry("400x200")
        popup.title(f"Table {table_index}에서의 새 주문 요청")

        # 주문 메시지 표시
        label = tk.Label(popup, text=message, font=("Arial", 12))
        label.pack(pady=10)

        # 버튼 프레임
        button_frame = tk.Frame(popup)
        button_frame.pack(pady=10)

        # 수락 버튼
        accept_button = tk.Button(
            button_frame,
            text="수락",
            command=lambda: self.handle_accept_order(table_index, message, popup, response)
        )
        accept_button.pack(side="left", padx=5)

        # 거절 버튼
        reject_button = tk.Button(
            button_frame,
            text="거절",
            command=lambda: self.handle_reject_order(table_index, popup, response)
        )
        reject_button.pack(side="right", padx=5)


    def handle_accept_order(self, table_index, message, popup, response):
        """주문 수락 처리."""
        popup.destroy()
        self.accept_order(table_index, message)
        
        # response 설정
        response.success = True
        response.message = f"Order for Table {table_index} accepted."
        self.node.get_logger().info(response.message)


    
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
        insert.insert_table_orders(table_index, order_detail)



    def handle_reject_order(self, table_index, popup, response):
        """주문 거절 처리."""
        popup.destroy()  # 팝업 닫기
        self.cancel_order(table_index, response)  # response를 전달

        # response 설정
        response.success = False
        response.message = f"Order for Table {table_index} rejected."
        self.node.get_logger().info(response.message)


    
    def cancel_order(self, table_index, response):
        """주문 취소."""
        if table_index not in self.table_labels:
            print(f"[ERROR] Invalid table index: {table_index}")
            response.success = False
            response.message = f"Invalid table index: {table_index}"
            return

        # 팝업 창 생성
        popup = Toplevel(self.root)
        popup.geometry("300x250")
        popup.title(f"테이블 {table_index} 주문 취소")

        # 안내 레이블
        label = tk.Label(popup, text="취소 사유를 선택하세요:", font=("Arial", 12))
        label.pack(pady=10)

        # 선택된 사유를 저장할 변수
        selected_reason = tk.StringVar(value="")

        # 취소 사유 리스트
        reasons = ["재료 부족", "고객 요청", "기타"]

        def handle_confirm():
            """확인 버튼 동작."""
            reason = selected_reason.get().strip()
            if not reason:
                messagebox.showerror("오류", "취소 사유를 선택하세요.")
                return

            # 팝업 창 닫기
            popup.destroy()

            # 타이머 제거
            if table_index in self.timers:
                del self.timers[table_index]

            # response 업데이트
            response.success = False
            response.message = f"Order for Table {table_index} rejected. Reason: {reason}"
            self.node.get_logger().info(response.message)

        # 취소 사유 버튼 생성
        for reason in reasons:
            tk.Radiobutton(
                popup,
                text=reason,
                variable=selected_reason,
                value=reason,
                font=("Arial", 10),
                anchor="w"
            ).pack(anchor="w", padx=10, pady=2)

        # 버튼 프레임
        button_frame = tk.Frame(popup)
        button_frame.pack(pady=20)

        # 확인 버튼
        confirm_button = tk.Button(button_frame, text="확인", command=handle_confirm, width=10)
        confirm_button.pack(side="left", padx=10)

        # 취소 버튼
        cancel_button = tk.Button(button_frame, text="취소", command=popup.destroy, width=10)
        cancel_button.pack(side="right", padx=10)



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
    
    def poll_events(self):
        """이벤트 큐를 지속적으로 확인"""
        try:
            while True:
                # 큐에서 이벤트를 가져옴
                event = self.event_queue.get_nowait()

                # 이벤트 처리 (여기서는 주문 팝업 처리)
                if event["type"] == "order_request":
                    self.show_order_popup(event["table_index"], event["message"], event["response"])

                # 이벤트 처리 완료 표시
                self.event_queue.task_done()
        except queue.Empty:
            pass
        finally:
            # 100ms마다 이벤트 큐 확인
            self.root.after(100, self.poll_events)

    def create_number_buttons(self, rows, cols):
        """숫자 버튼 생성."""
        for i in range(rows):
            for j in range(cols):
                num = i * cols + j + 1
                button = tk.Button(self.seat_frame, text=str(num), width=5, height=2,
                                   command=lambda n=num: self.prompt_transport(n))
                button.grid(row=i, column=j, padx=5, pady=5)

    def prompt_transport(self, number):
        """운반 작업 확인 팝업."""
        response = messagebox.askokcancel("운반 확인", f"{number}번에 운반하겠습니까?")
        if response:
             # 주문 큐와 currenttable에서 해당 데이터 삭제
            for order in self.orders:
                if order["table"] == number:
                    self.orders.remove(order)
                    print(f"Order for Table {number}가 주문 큐에서 삭제되었습니다.")
                    self.update_fifo_listbox()
                    break
            # 테이블 주문에서 해당 데이터 삭제
            delete = Delete()
            delete.delete_table_order(number)
            # 테이블 주문 현황 업데이트
            self.table_labels[number].config(text=f"테이블 {number}\n주문 없음")
            self.node.navigate_to_goals([number - 1])

    def show_today_sales(self):
        """오늘의 매출 팝업."""
        total_sales = 1000  # 임시 매출 데이터
        popup = Toplevel(self.root)
        popup.geometry("300x150")
        popup.title("오늘의 매출")
        label = tk.Label(popup, text=f"오늘의 매출: {total_sales} 원", font=("Arial", 14))
        label.pack(pady=20)
        close_button = tk.Button(popup, text="닫기", command=popup.destroy)
        close_button.pack(pady=10)


class KitNode(Node):
    def __init__(self, event_queue):
        super().__init__("kit_node")
        self.event_queue = event_queue  # GUI와 통신할 이벤트 큐
        self.order_service = self.create_service(OrderService, 'order_service', self.handle_order_request)
        self.is_processing_request = False

        # Action client
        self.len = 10
        self.init_pose = [-2.0, -0.5, 0.0, 1.0]
        self.goal_poses = [
            [0.8, 0.6], [0.7, -0.3], [0.9, -0.7], [0.068, 1.4], [0.0417, -0.41], 
            [-0.02, -1.47], [-1.23, 1.396], [-1.5, -0.088], [-1.16, -1.42], [-1.79, -0.292]
        ]

        self.publisher = self.create_publisher(String, 'navigation_feedback', 10)
        self.set_initial_pose_service_client = self.create_client(SetInitialPose, '/set_initial_pose')
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_initial_pose not available, waiting again...')

        self.set_initial_pose(*self.init_pose)
        self.starting_position = self.init_pose[:2]

        self.current_goal_index = 0
        self.selected_goals = []

    def set_initial_pose(self, x, y, z, w):
        req = SetInitialPose.Request()
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        req.pose.pose.covariance = [0.1] * 36
        future = self.set_initial_pose_service_client.call_async(req)
        self.get_logger().info("[INFO] Initial pose set.")
        return future.result()

    def navigate_to_goals(self, goals):
        self.selected_goals = goals
        self.current_goal_index = 0
        self.navigate_to_selected_goal()

    def navigate_to_selected_goal(self):
        if self.current_goal_index < len(self.selected_goals):
            goal_index = self.selected_goals[self.current_goal_index]
            self.navigate_to_pose_send_goal(goal_index)
        else:
            self.return_to_start()

    def navigate_to_pose_send_goal(self, goal_index):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.goal_poses[goal_index][0]
        goal_msg.pose.pose.position.y = self.goal_poses[goal_index][1]
        goal_msg.pose.pose.orientation.w = 1.0
        send_goal_future = self.navigate_to_pose_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.on_goal_reached)

    def on_goal_reached(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("[WARN] Action goal rejected.")
            return
        self.get_logger().info("[INFO] Action goal accepted.")
        action_result_future = goal_handle.get_result_async()
        action_result_future.add_done_callback(self.on_goal_finished)

    def on_goal_finished(self, future):
        action_status = future.result().status
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[INFO] Reached goal.")
            self.current_goal_index += 1
            self.navigate_to_selected_goal()
        else:
            self.get_logger().warn(f"[WARN] Navigation failed with status: {action_status}")
            self.return_to_start()

    def return_to_start(self):
        self.get_logger().info("[INFO] Returning to start...")
        return_goal_msg = NavigateToPose.Goal()
        return_goal_msg.pose.header.frame_id = "map"
        return_goal_msg.pose.pose.position.x = self.starting_position[0]
        return_goal_msg.pose.pose.position.y = self.starting_position[1]
        return_goal_msg.pose.pose.orientation.w = 1.0
        self.navigate_to_pose_action_client.send_goal_async(return_goal_msg)

    def handle_order_request(self, request, response):
        """주문 요청 처리 (동기 처리)"""
        if self.is_processing_request:
            # 이미 요청 처리 중인 경우 로그 출력
            self.get_logger().warn("A request is already being processed. Ignoring new request.")
            response.success = False
            response.message = "Server busy. Try again later."
            return response

        # 요청 처리 시작
        self.is_processing_request = True
        self.get_logger().info(f"Processing request for Table {request.table_index}")

        # 이벤트 큐에 요청 추가 (GUI 처리를 위해)
        self.event_queue.put({
            "type": "order_request",
            "table_index": request.table_index,
            "message": request.order_detail,
            "response": response,
        })

        # 주문 처리 완료
        self.process_order(request.table_index, request.order_detail, response)
        return response

    def process_order(self, table_index, order_detail, response):
        """주문 처리 및 응답 설정"""
        # 주문 처리 (여기서는 수락 예제로 처리)
        response.success = True
        response.message = f"Order for Table {table_index} accepted."
        self.get_logger().info(response.message)

        # 요청 처리 완료 상태 업데이트
        self.is_processing_request = False

    def check_response(self, table_index, response):
        """response 상태를 비동기로 확인"""
        if response.success:
            # 응답 처리 완료
            self.get_logger().info(f"Order processed for Table {table_index}. Response: {response.message}")
            self.is_processing_request = False  # 요청 처리 완료 플래그 설정
            return  # 함수 종료로 반복 중단

        # response가 아직 처리되지 않은 경우, 다시 확인
        self.create_timer(0.1, lambda: self.check_response(table_index, response))


    def accept_order_callback(self, table_index, message):
        """주문 수락 처리."""
        self.get_logger().info(f"Order accepted for Table {table_index}: {message}")

    def reject_order_callback(self, table_index):
        """주문 거절 처리."""
        self.get_logger().info(f"Order rejected for Table {table_index}")

    def transport_to_table(self, table_index):
        """테이블로 운반."""
        self.get_logger().info(f"Transporting to Table {table_index}")


def main():
    rclpy.init()

    root = tk.Tk()
    event_queue = queue.Queue()
    node = KitNode(event_queue)
    gui = KitGUI(root, node, event_queue)

    # ROS2 실행기 스레드 시작
    executor = rclpy.executors.MultiThreadedExecutor()
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor.add_node(node)
    executor_thread.start()

    # GUI 실행
    root.protocol("WM_DELETE_WINDOW", lambda: on_close(node, executor, executor_thread))
    root.mainloop()

def on_close(node, executor, executor_thread):
    executor.shutdown()
    executor_thread.join()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)




if __name__ == "__main__":
    main()
