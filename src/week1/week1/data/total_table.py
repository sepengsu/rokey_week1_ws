import sqlite3
import pandas as pd
from .cur_table import CurrentTable

class TotalTable:
    def __init__(self):
        '''
        (오더 id, 테이블 번호, 메뉴 아이디, 수량, 주문 시간)
        '''
        self.conn = sqlite3.connect("./src/week1/full_order_history.db")
        self.cursor = self.conn.cursor()
    
    def del_table_order_all(self, table_id):
        self.cursor.execute("DELETE FROM full_order_history WHERE table_id=?", (table_id,)) # 테이블 주문 정보 삭제
        self.conn.commit()
        print(f"{table_id}번 테이블 주문 정보가 삭제되었습니다.")

    def del_table_order(self, table_id, menu_id):
        self.cursor.execute("DELETE FROM full_order_history WHERE table_id=? AND menu_id=?", (table_id, menu_id))
        self.conn.commit()
        print(f"{table_id}번 테이블의 {menu_id} 메뉴 주문 정보가 삭제되었습니다.")

    def insert_table_order(self, table_id:int):
        '''
        1. 현재 테이블의 주문 정보 불러오기
        2. 주문 정보를 정리하여 반환 (dataframe: menu_id, quantity의 두 컬럼으로 구성 )
        3. 주문 정보를 total_table에 추가
        '''
        cur_table = CurrentTable()
        table_orders = cur_table.show_table_orders(table_id)
        cur_table.close()
        # 주문 정보를 total_table에 추가
        for i in range(len(table_orders)):
            menu_id = table_orders['menu_id'][i]
            quantity = int(table_orders['total_quantity'][i])
            self.cursor.execute("INSERT INTO full_order_history (table_id, menu_id, quantity) VALUES (?, ?, ?)", (table_id, menu_id, quantity))
            self.conn.commit()
        print(f"[FULL ORDER]{table_id}번 테이블 주문 정보 입력")

    def close(self):
        self.conn.close()
