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

    def show_table_orders(self, table_index):
        ''' 
        특정 테이블 주문 정보 반환
        1. 테이블 인텍스 지정 
        2. 테이블 주문 정보 반환
        3. 테이블 주문 정보를 정리하여 반환
        '''
        query = """
                SELECT 
                    menu_id, 
                    SUM(quantity) AS total_quantity
                FROM 
                    current_table_orders
                WHERE 
                    table_id = ?
                GROUP BY 
                    menu_id
                """
        df = pd.read_sql_query(query, self.conn, params=(table_index,))
        if df.empty:
            return f"테이블 {table_index}번 \n주문 없음"
        return df # 테이블 주문 정보 반환
    
    def del_table_order_all(self, table_id):
        self.cursor.execute("DELETE FROM current_table_orders WHERE table_id=?", (table_id,)) # 테이블 주문 정보 삭제
        self.conn.commit()
        print(f"{table_id}번 테이블 주문 정보가 삭제되었습니다.")

    def insert_table_order(self, table_id, menu_id, quantity):
        ''' 테이블 주문 정보 추가'''
        self.cursor.execute("INSERT INTO current_table_orders (table_id, menu_id, quantity) VALUES (?, ?, ?)", (table_id, menu_id, quantity))
        self.conn.commit()
        print(f"{table_id}번 테이블에 {quantity}개의 {menu_id} 메뉴가 추가되었습니다.")


    def close(self):
        self.conn.close()

