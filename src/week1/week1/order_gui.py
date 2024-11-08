import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
from queue import Queue
import threading
import rclpy
from rclpy.node import Node
from ros_msgs.srv import OrderService

class KioskGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.root.title("Self-Order Kiosk")
        self.root.geometry("1200x800")
        self.ros_node = ros_node
        self.response_queue = Queue()

        self.cart = {}
        self.total_price = 0
        self.current_category = None

        self.setup_sidebar()
        self.setup_menu_display()
        self.setup_cart_display()
        self.update_total_price()

        self.check_response_queue()

    def setup_sidebar(self):
        sidebar = tk.Frame(self.root, bg="#333", width=200)
        sidebar.pack(side=tk.LEFT, fill=tk.Y)

        categories = ["채소 샐러드", "단백질 샐러드"]
        for category in categories:
            btn = tk.Button(sidebar, text=category, bg="#006400", fg="white", relief=tk.FLAT,
                            command=lambda c=category: self.filter_menu(c))
            btn.pack(fill=tk.X, padx=10, pady=5)

        table_frame = tk.Frame(sidebar, bg="#555", padx=10, pady=10)
        table_frame.pack(fill=tk.X, padx=10, pady=10)

        tk.Label(table_frame, text="테이블 번호 입력", fg="white", bg="#555", font=("Arial", 12)).pack(pady=5)
        self.table_number_var = tk.StringVar()
        table_entry = tk.Entry(table_frame, textvariable=self.table_number_var, width=10)
        table_entry.pack(pady=5)

        set_table_button = tk.Button(table_frame, text="설정", bg="#4CAF50", fg="white",
                                     command=self.set_table_number)
        set_table_button.pack(pady=5)

        send_button = tk.Button(sidebar, text="전송", bg="#4CAF50", fg="white", command=self.send_order)
        send_button.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)

        cancel_button = tk.Button(sidebar, text="취소", bg="#f44336", fg="white", command=self.clear_cart)
        cancel_button.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)

    def setup_menu_display(self):
        self.menu_items = [
            ("오색 샐러드", 12000, "채소 샐러드", "./src/week1/images/오색샐러드.png"),
            ("카프레제 샐러드", 12000, "채소 샐러드", "./src/week1/images/카프레제샐러드.png"),
            ("그린 샐러드", 8000, "채소 샐러드", "./src/week1/images/그린샐러드.png"),
            ("과일 샐러드", 13000, "채소 샐러드", "./src/week1/images/과일샐러드.png"),
            ("치킨 샐러드", 11000, "단백질 샐러드", "./src/week1/images/치킨샐러드.png"),
            ("새우 샐러드", 14000, "단백질 샐러드", "./src/week1/images/새우샐러드.png"),
            ("훈제연어 샐러드", 15000, "단백질 샐러드", "./src/week1/images/훈제연어샐러드.png"),
            ("스테이크 샐러드", 16000, "단백질 샐러드", "./src/week1/images/스테이크샐러드.png"),
        ]

        menu_frame = tk.Frame(self.root, bg="#f0f0f0", padx=0, pady=0)
        menu_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(menu_frame, bg="#f0f0f0", height=600)
        self.scrollable_frame = ttk.Frame(canvas)
        scrollbar = ttk.Scrollbar(menu_frame, orient="vertical", command=canvas.yview)
        scrollbar.pack(side=tk.RIGHT, fill="y")
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        canvas.configure(yscrollcommand=scrollbar.set)

        scrollable_window = canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(scrollable_window, width=e.width))
        self.scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

        self.update_menu_display()

    def setup_cart_display(self):
        cart_frame = tk.Frame(self.root, bg="#f8f8f8", width=500)
        cart_frame.pack(side=tk.RIGHT, fill=tk.Y)
        cart_frame.pack_propagate(False)
        self.cart_frame = cart_frame

        cart_title = tk.Label(cart_frame, text="장바구니", font=("Arial", 16, "bold"), bg="#f8f8f8")
        cart_title.pack(pady=10)

        self.total_price_label = tk.Label(cart_frame, text="총 주문 금액: 0원", font=("Arial", 14, "bold"), bg="#f8f8f8")
        self.total_price_label.pack(side=tk.BOTTOM, pady=20)

    def update_menu_display(self):
        for widget in self.scrollable_frame.winfo_children():
            widget.destroy()

        for name, price, category, image_path in self.menu_items:
            if self.current_category is None or self.current_category == category:
                item_frame = tk.Frame(self.scrollable_frame, bd=4, relief=tk.RAISED, bg="white")
                item_frame.pack(fill=tk.X, padx=10, pady=10)

                try:
                    image = Image.open(image_path)
                    image = image.resize((120, 120), Image.LANCZOS)
                    photo = ImageTk.PhotoImage(image)
                    image_label = tk.Label(item_frame, image=photo, bg="white")
                    image_label.image = photo
                    image_label.pack(side=tk.LEFT, padx=10)
                except FileNotFoundError:
                    print(f"Image not found: {image_path}")

                item_details_frame = tk.Frame(item_frame, bg="white")
                item_details_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10)

                item_name = tk.Label(item_details_frame, text=name, font=("Arial", 12, "bold"), bg="white")
                item_name.pack(anchor="w", pady=(5, 0))
                item_price = tk.Label(item_details_frame, text=f"{price}원", font=("Arial", 12), bg="white")
                item_price.pack(anchor="w", pady=(0, 5))

                add_to_cart_button = tk.Button(item_details_frame, text="장바구니에 담기", bg="#238c00", fg="white",
                                               command=lambda name=name, price=price: self.add_to_cart(name, price),
                                               font=("Arial", 12, "bold"))
                add_to_cart_button.pack(side=tk.RIGHT, pady=10, padx=10)

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
            if widget not in [self.total_price_label]:
                widget.destroy()

        for item_name, (item_price, item_quantity) in self.cart.items():
            item_container = tk.Frame(self.cart_frame, bg="#f8f8f8")
            item_container.pack(fill=tk.X, pady=5, padx=10)

            item_label = tk.Label(item_container, text=f"{item_name} x{item_quantity} - {item_price * item_quantity}원",
                                  font=("Arial", 12), bg="#f8f8f8")
            item_label.pack(side=tk.LEFT)

            add_btn = tk.Button(item_container, text="+",
                                command=lambda name=item_name: self.add_to_cart(name, item_price), font=("Arial", 10, "bold"))
            add_btn.pack(side=tk.RIGHT, padx=5)

            remove_btn = tk.Button(item_container, text="-",
                                   command=lambda name=item_name: self.remove_from_cart(name), font=("Arial", 10, "bold"))
            remove_btn.pack(side=tk.RIGHT)

    def check_response_queue(self):
        if not self.response_queue.empty():
            is_approved, message = self.response_queue.get()
            if is_approved:
                messagebox.showinfo("Order Status", f"수락되었습니다: {message}")
            else:
                messagebox.showerror("Order Status", f"거절되었습니다: {message}")

        self.root.after(100, self.check_response_queue)

    def send_order(self):
        if not self.cart:
            messagebox.showwarning("Empty Cart", "장바구니가 비어 있습니다.")
            return

        if not hasattr(self, 'table_number') or self.table_number is None:
            messagebox.showwarning("Table Number Missing", "테이블 번호를 설정하세요.")
            return

        self.ros_node.send_order_request(self.cart, self.table_number)

    def clear_cart(self):
        self.cart.clear()
        self.total_price = 0
        self.update_total_price()
        self.update_cart_display()

    def set_table_number(self):
        table_number = self.table_number_var.get()
        if table_number.isdigit() and int(table_number) > 0:
            messagebox.showinfo("테이블 설정", f"테이블 번호가 {table_number}번으로 설정되었습니다.")
            self.table_number = int(table_number)
        else:
            messagebox.showerror("입력 오류", "유효한 테이블 번호를 입력하세요.")


class OrderNode(Node):
    def __init__(self, response_queue: Queue):
        super().__init__('order_client')
        self.client = self.create_client(OrderService, 'order_service')
        self.response_queue = response_queue
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_order_request(self, cart_data: dict, table_index: int):
        message = "\n".join([f"({item_name}, {quantity})" for item_name, (price, quantity) in cart_data.items()])

        request = OrderService.Request()
        request.table_index = table_index
        request.order_detail = message

        self.get_logger().info(f"Sending order request: Table {table_index}, Order: {message}")
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.response_queue.put((True, response.message))
            self.get_logger().info(f"Received response: {response.message}")
        except Exception as e:
            self.response_queue.put((False, f"Service call failed: {e}"))
            self.get_logger().error(f"Service call failed: {e}")


class OrderNode(Node):
    def __init__(self, response_queue: Queue):
        super().__init__('order_client')
        self.client = self.create_client(OrderService, 'order_service')
        self.response_queue = response_queue  # 큐 참조 저장
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_order_request(self, cart_data: dict, table_index: int):
        """서버로 주문 요청을 보냅니다."""
        # 주문 상세 메시지 생성
        message = "\n".join([f"({item_name}, {quantity})" for item_name, (price, quantity) in cart_data.items()])

        # OrderService 요청 생성
        request = OrderService.Request()
        request.table_index = table_index  # GUI에서 전달된 테이블 번호 사용
        request.order_detail = message

        # 비동기 서비스 호출
        self.get_logger().info(f"Sending order request: Table {table_index}, Order: {message}")
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
