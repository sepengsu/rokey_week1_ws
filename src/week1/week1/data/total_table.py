import sqlite3
import pandas as pd

class TotalTable:
    def __init__(self):
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

    def insert_table_order(self, table_id, menu_id, quantity):
        self.cursor.execute("INSERT INTO full_order_history (table_id, menu_id, quantity) VALUES (?, ?, ?)", (table_id, menu_id, quantity))
        self.conn.commit()
        print(f"{table_id}번 테이블에 {quantity}개의 {menu_id} 메뉴가 추가되었습니다.")
    

    def close(self):
        self.conn.close()

class Analysis:
    def __init__(self):
        self.conn = sqlite3.connect("./src/week1/full_order_history.db")
        self.cursor = self.conn.cursor()

    def analyze_orders(self):
        ''' 주문 데이터 분석 '''
        df = pd.read_sql_query("SELECT * FROM full_order_history", self.conn)
        if df.empty:
            return None
        df = df.groupby("menu_id").sum()
        df = df.sort_values(by="quantity", ascending=False)
        return df
    
    def close(self):
        self.conn.close()