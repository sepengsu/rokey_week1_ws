import tkinter as tk
from tkinter import ttk, messagebox
import threading
import rclpy
from rclpy.node import Node
from queue import Queue  # 큐 모듈 추가
from ros_msgs.srv import OrderService

class KioskGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.root.title("Self-Order Kiosk")
        self.root.geometry("800x600")
        self.ros_node = ros_node  # ROS2 노드 인스턴스
        self.response_queue = Queue()  # 응답 메시지를 저장할 큐

        # 장바구니와 금액 초기화
        self.cart = {}  # {'Menu Item': [price, quantity]}
        self.total_price = 0

        self.setup_sidebar()
        self.setup_menu_display()
        self.setup_cart_display()
        self.update_total_price()
        
        # 주기적으로 큐를 확인하는 함수 추가
        self.check_response_queue()

    def setup_sidebar(self):
        sidebar = tk.Frame(self.root, bg="#333", width=200)
        sidebar.pack(side=tk.LEFT, fill=tk.Y)

        # Category Buttons
        categories = ["채소 샐러드", "단백질 샐러드"]
        for category in categories:
            btn = tk.Button(sidebar, text=category, bg="#006400", fg="white", relief=tk.FLAT,
                            command=lambda c=category: self.filter_menu(c))
            btn.pack(fill=tk.X, padx=10, pady=5)

        # Send and Cancel Buttons
        send_button = tk.Button(sidebar, text="전송", bg="#4CAF50", fg="white", command=self.send_order)
        send_button.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)

        cancel_button = tk.Button(sidebar, text="취소", bg="#f44336", fg="white", command=self.clear_cart)
        cancel_button.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)

    def setup_menu_display(self):
        self.menu_items = [
            ("5가지 샐러드", 12000, "채소 샐러드"),
            ("카프레제 샐러드", 12000, "채소 샐러드"),
            ("그린 샐러드", 8000, "채소 샐러드"),
            ("과일 샐러드", 13000, "채소 샐러드"),
            ("치킨 샐러드", 11000, "단백질 샐러드"),
            ("새우 샐러드", 14000, "단백질 샐러드"),
            ("훈제연어 샐러드", 15000, "단백질 샐러드"),
            ("스테이크 샐러드", 16000, "단백질 샐러드"),
        ]

        canvas = tk.Canvas(self.root, bg="gray")
        self.scrollable_frame = ttk.Frame(canvas)
        scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=canvas.yview)
        scrollbar.pack(side=tk.RIGHT, fill="y")
        canvas.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        canvas.configure(yscrollcommand=scrollbar.set)

        scrollable_window = canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

        self.update_menu_display()

    def setup_cart_display(self):
        cart_frame = tk.Frame(self.root, bg="white", width=300)
        cart_frame.pack(side=tk.RIGHT, fill=tk.Y)
        cart_frame.pack_propagate(False)
        self.cart_frame = cart_frame

        self.total_price_label = tk.Label(self.root, text="총 주문 금액: 0원")
        self.total_price_label.pack(side=tk.BOTTOM, pady=10)

    def update_menu_display(self):
        for widget in self.scrollable_frame.winfo_children():
            widget.destroy()

        for name, price, category in self.menu_items:
            item_frame = tk.Frame(self.scrollable_frame, bd=2, relief=tk.RIDGE)
            item_frame.pack(fill=tk.X, padx=10, pady=10)

            item_name = tk.Label(item_frame, text=name)
            item_name.pack(anchor="w")
            item_price = tk.Label(item_frame, text=f"{price}원")
            item_price.pack(anchor="w")

            add_to_cart_button = tk.Button(item_frame, text="장바구니에 담기", bg="#238c00", fg="white",
                                           command=lambda name=name, price=price: self.add_to_cart(name, price))
            add_to_cart_button.pack(anchor="e", pady=5)

    def filter_menu(self, category):
        self.current_category = category
        self.update_menu_display()

    def add_to_cart(self, item_name, item_price):
        if item_name in self.cart:
            self.cart[item_name][1] += 1
        else:
            self.cart[item_name] = [item_price, 1]

        self.total_price += item_price
        self.update_total_price()
        self.update_cart_display()

    def remove_from_cart(self, item_name):
        if item_name in self.cart:
            item_price, item_quantity = self.cart[item_name]

            if item_quantity > 1:
                self.cart[item_name][1] -= 1
            else:
                del self.cart[item_name]

            self.total_price -= item_price
            self.update_total_price()
            self.update_cart_display()

    def update_total_price(self):
        self.total_price_label.config(text=f"총 주문 금액: {self.total_price}원")

    def update_cart_display(self):
        for widget in self.cart_frame.winfo_children():
            widget.destroy()

        for item_name, (item_price, item_quantity) in self.cart.items():
            item_container = tk.Frame(self.cart_frame)
            item_container.pack(fill=tk.X, pady=2)

            item_label = tk.Label(item_container, text=f"{item_name} x{item_quantity} - {item_price * item_quantity}원")
            item_label.pack(side=tk.LEFT)

            add_btn = tk.Button(item_container, text="+", command=lambda name=item_name: self.add_to_cart(name, item_price))
            add_btn.pack(side=tk.RIGHT, padx=5)

            remove_btn = tk.Button(item_container, text="-", command=lambda name=item_name: self.remove_from_cart(name))
            remove_btn.pack(side=tk.RIGHT)
            
    def check_response_queue(self):
        # 큐에 응답 메시지가 있는 경우 팝업 표시
        if not self.response_queue.empty():
            is_approved, message = self.response_queue.get()
            if is_approved:
                messagebox.showinfo("Order Status", f"수락되었습니다: {message}")
            else:
                messagebox.showerror("Order Status", f"거절되었습니다: {message}")

        # 일정 간격으로 큐를 다시 확인
        self.root.after(100, self.check_response_queue)

    def send_order(self):
        if not self.cart:
            messagebox.showwarning("Empty Cart", "장바구니가 비어 있습니다.")
            return
        self.ros_node.send_order_request(self.cart)

    def clear_cart(self):
        self.cart.clear()
        self.total_price = 0
        self.update_total_price()
        self.update_cart_display()

class OrderNode(Node):
    def __init__(self, response_queue: Queue):
        super().__init__('order_client')
        self.client = self.create_client(OrderService, 'order_service')
        self.response_queue = response_queue  # 큐 참조 저장
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_order_request(self, cart_data: dict):
        """서버로 주문 요청을 보냅니다."""
        # 테이블 번호와 주문 상세 메시지 생성
        table_index = 1  # 예제에서는 테이블 번호를 고정값으로 설정
        message = "\n".join([f"({item_name}, {quantity})" for item_name, (price, quantity) in cart_data.items()])

        # OrderService 요청 생성
        request = OrderService.Request()
        request.table_index = table_index
        request.order_detail = message

        # 비동기 서비스 호출
        self.get_logger().info(f"Sending order request: {message}")
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        """서버로부터 응답을 처리합니다."""
        try:
            response = future.result()
            # 응답을 큐에 추가
            self.response_queue.put((True, response.message))
            self.get_logger().info(f"Received response: {response.message}")
        except Exception as e:
            # 에러 응답을 큐에 추가
            self.response_queue.put((False, f"Service call failed: {e}"))
            self.get_logger().error(f"Service call failed: {e}")


def main():
    # ROS2 초기화
    rclpy.init()
    
    # 큐 생성 (ROS 노드와 GUI 간 데이터 교환용)
    response_queue = Queue()
    
    # ROS2 노드 생성
    ros_node = OrderNode(response_queue)
    
    # Tkinter GUI 초기화
    root = tk.Tk()
    kiosk_gui = KioskGUI(root, ros_node)  # KioskGUI 인스턴스 생성
    
    # ROS2 노드를 별도의 스레드에서 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,))
    ros_thread.start()
    
    # Tkinter 이벤트 루프 실행
    root.protocol("WM_DELETE_WINDOW", lambda: (rclpy.shutdown(), root.destroy()))
    root.mainloop()
    
    # ROS 스레드 종료 대기
    ros_thread.join()

if __name__ == "__main__":
    main()
