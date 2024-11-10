import sqlite3
import pandas as pd
from .menu import MENU_DICT

class Statics:
    '''
    역사 db에 대해서 통계를 내는 클래스
    1. 데이터베이스 연결
    2. 오늘 날짜 or 특정 날짜의 주문 정보를 불러오기
    3. 주문 정보를 정리하여 반환
    (DB의 구조)
    (오더 id, 테이블 번호, 메뉴 아이디, 수량, 주문 시간)
    column: order_id, table_id, menu_id, quantity, order_time
    '''
    def __init__(self):
        self.conn = sqlite3.connect("./src/week1/full_order_history.db")
        self.cursor = self.conn.cursor()
    
    
    def show_table_today(self):
        ''' 
        오늘 날짜의 주문 정보 반환
        1. 오늘 날짜 지정
        2. 오늘 날짜의 주문 정보 반환
        3. 오늘 날짜의 주문 정보를 정리하여 반환
        '''
        query = """
                SELECT 
                    menu_id, 
                    SUM(quantity) AS total_quantity
                FROM 
                    full_order_history
                WHERE 
                    order_time LIKE ?
                GROUP BY 
                    menu_id
                """
        df = pd.read_sql_query(query, self.conn, params=(f"{pd.Timestamp.today().date()}%",))
        if df.empty:
            return f"주문 없음"
        return df
    
    def analyze_df(self,df):
        ''' 
        (menu_id, total_quantity)의 데이터프레임을 받아서 분석
        1. 데이터프레임을 받아서 분석
        2. 가장 많이 주문된 메뉴 순으로 정렬
        3. 총 주문량
        4. 총 매출 반환
        '''
        if df.empty:
            return None
        df = df.groupby("menu_id").sum()
        df = df.sort_values(by="total_quantity", ascending=False)
        analy_df = df.reset_index()[["menu_id", "total_quantity"]]
        # 총 주문량
        total_quantity = df["total_quantity"].sum()
        # 총 매출
        total_sales = 0
        for menu_id in df.index:
            total_sales += MENU_DICT[menu_id] * df.loc[menu_id, "total_quantity"]
        return analy_df, total_quantity, total_sales
    
    def close(self):
        self.conn.close()