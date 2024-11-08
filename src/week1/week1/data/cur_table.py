import sqlite3
import pandas as pd
class CurrentTable():
    def __init__(self):
        '''
        table은 다음과 같은 구조를 가진다. 
        (오더 id, 테이블 번호, 메뉴 아이디, 수량, 주문 시간)
        pk는 오더 id
        columns: order_id, table_id, menu_id, quantity, order_time 
        order_id와 order_time는 자동으로 생성
        '''
        self.conn = sqlite3.connect("./src/week1/current_table_orders.db")
        self.cursor = self.conn.cursor()

    def show_table_orders(self):
        ''' 테이블별 주문 정보 반환 '''
        df = pd.read_sql_query("SELECT * FROM current_table_orders", self.conn)
        table_orders = {}
        for i in range(1, 10):
            table_orders[i] = []
        for index, row in df.iterrows():
            table_id = row["table_id"]
            menu_id = row["menu_id"]
            quantity = row["quantity"]
            id_data = [table_orders[table_id][0] for table_id in table_orders] if table_orders[table_id] != [] else []
            if menu_id in id_data: # 만약에 메뉴 아이디가 이미 있으면
                table_orders[table_id][0] += quantity
            else:
                table_orders[table_id].append({menu_id: quantity})
        return table_orders if table_orders!={} else None # 테이블 주문 정보 반환 (없으면 None 반환)
    
    def del_table_order_all(self, table_id):
        self.cursor.execute("DELETE FROM current_table_orders WHERE table_id=?", (table_id,)) # 테이블 주문 정보 삭제
        self.conn.commit()
        print(f"{table_id}번 테이블 주문 정보가 삭제되었습니다.")

    def insert_table_order(self, table_id, menu_id, quantity):
        ''' 테이블 주문 정보 추가, 이미 있는 메뉴라면 수량만 추가 '''
        self.cursor.execute("INSERT INTO current_table_orders (table_id, menu_id, quantity) VALUES (?, ?, ?)", (table_id, menu_id, quantity))
        self.conn.commit()
        print(f"{table_id}번 테이블에 {quantity}개의 {menu_id} 메뉴가 추가되었습니다.")
    

    def close(self):
        self.conn.close()

