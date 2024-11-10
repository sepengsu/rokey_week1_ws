import pandas as pd
from .cur_table import CurrentTable
from .total_table import TotalTable
import time

class Show:
    '''
    GUI에 표시하기 위한 전처리 작업
    ''' 
    def show_cur_table_orders(self):
        ''' 테이블별 주문 정보를 주방 gui에 표시하기 위한 작업 '''
        # 현재 주문 정보 불러오기
        cur_table = CurrentTable()
        # table별 주문 정보를 정리하여 반환
        texts = []
        for table_id in range(1, 10):
            # 테이블 주문 정보 불러오기
            table_orders = cur_table.show_table_orders(table_id)
            if type(table_orders) == str:
                texts.append(table_orders)
            else:
                text = f"{table_id}번 테이블\n"
                for i in range(len(table_orders)):
                    text += f"{table_orders['menu_id'][i]}: {table_orders['total_quantity'][i]}개\n"
                texts.append(text)
        cur_table.close()
        return texts
    
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
    def insert_cur_table_orders(self,table_id:int,order:str):
        ''' 
        cur_table_orders에 주문 정보 추가
        주문 정보 전처리
        주문 정보: 
        (메뉴 아이디, 수량)
        (메뉴 아이디, 수량) ...

        0. 주문 정보 불러오기 
        1. 주문 정보를 줄 단위로 나누기
        2. 메뉴 아이디와 수량 나누기
        3. db에 주문 정보 추가
        '''
        order_list = order.split("\n")
        for i in range(len(order_list)):
            menu_id,quantity = order_list[i][1:-1].split(",")
            quantity = int(quantity)
            table = CurrentTable()
            table.insert_table_order(table_id, menu_id, quantity)
            table.close()

        print(f"{table_id}번 테이블에 주문이 추가되었습니다.")

    def insert_total_table_orders(self,table_id:int):
        ''' 
        total_table_orders에 주문 정보 추가
        1. 현재 주문 정보 불러오기
        2. 주문 정보를 정리하여 반환
        3. 주문 정보를 total_table에 추가
        '''
        try: 
            total_table = TotalTable()
            total_table.insert_table_order(table_id)
            total_table.close()
        except:
            return None
        return f"{table_id}번 테이블 주문 정보가 추가되었습니다."
    
    def load_cur_table_orders(self, table_id):
        ''' 테이블 주문 정보 불러오기 '''
        cur_table = CurrentTable()
        table_orders = cur_table.show_table_orders(table_id)
        cur_table.close()
        return table_orders # 테이블 주문 정보 반환