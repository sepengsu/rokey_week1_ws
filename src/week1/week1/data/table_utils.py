import pandas as pd
from .cur_table import CurrentTable
from .total_table import TotalTable
import time

menu_items = [
    ("오색 샐러드", 12000, "채소 종류"),
    ("카프레제 샐러드", 12000, "채소 종류"),
    ("그린 샐러드", 8000, "채소 종류"),
    ("과일 샐러드", 13000, "채소 종류"),
    ("치킨 샐러드", 11000, "단백질 종류"),
    ("새우 샐러드", 14000, "단백질 종류"),
    ("훈제연어 샐러드", 15000, "단백질 종류"),
    ("스테이크 샐러드", 16000, "단백질 종류"),
]

class Show:
    '''
    GUI에 표시하기 위한 전처리 작업
    '''

    def show_table_orders(self):
        ''' 테이블별 주문 정보를 주방 gui에 표시하기 위한 작업 '''
        # 현재 주문 정보 불러오기
        cur_table = CurrentTable()
        try:
            self.table_orders = cur_table.show_table_orders()
            cur_table.close()
        except:
            return ["" for _ in range(9)]
        texts = []
        for i in range(1, 10):
            if self.table_orders[i] != []: # 주문이 있는 경우
                text = ""
                for order in self.table_orders[i]: # [(메뉴 아이디, 수량), ...]
                    for menu_id, quantity in order.items():
                        text += f"{menu_id}({quantity}개)\n"
                texts.append(text)
            else:
                texts.append("")
        return texts
    
    def order_order(self):
        pass
    
class Delete:
    '''
    주문을 끝내고 취소하기 위한 버튼 
    '''
    def delete_table_order(self, table_id):
        ''' 테이블 주문 정보 삭제 '''
        cur_table = CurrentTable()
        cur_table.del_table_order_all(table_id)
        cur_table.close()
        return f"{table_id}번 테이블 주문 정보가 삭제되었습니다."
    
class Insert:
    '''
    주문을 추가하기 위한 작업
    '''
    def insert_table_orders(self,table_id:int,order:str):
        ''' 주문 정보 전처리
        주문 정보: 
        (메뉴 아이디, 수량)
        (메뉴 아이디, 수량) ...

        0. 주문
        1. 주문 정보를 줄 단위로 나누기
        2. 메뉴 아이디와 수량 나누기
        3. 테이블에 
        '''
        order_list = order.split("\n")
        for i in range(len(order_list)):
            menu_id,quantity = order_list[i][1:-1].split(",")


    def insert_table_order(self, table_id, menu_id, quantity):
        ''' 테이블 주문 정보 추가 '''
        cur_table = CurrentTable()
        cur_table.insert_table_order(table_id, menu_id, quantity)
        cur_table.close()
        # 역사에도 추가
        total_table = TotalTable()
        total_table.insert_table_order(table_id, menu_id, quantity)
        total_table.close()
        return f"{table_id}번 테이블에 {quantity}개의 {menu_id} 메뉴가 추가되었습니다."