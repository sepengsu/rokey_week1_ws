import sqlite3

def create_current_table_orders_db():
    '''
    (오더 id, 테이블 번호, 메뉴 아이디, 수량, 주문 시간)을 저장하는 데이터베이스 테이블 생성
    pk는 오더 id
    '''
    conn = sqlite3.connect("./src/week1/current_table_orders.db")
    cursor = conn.cursor()
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS current_table_orders (
            order_id INTEGER PRIMARY KEY AUTOINCREMENT,
            table_id INTEGER,
            menu_id TEXT,
            quantity INTEGER,
            order_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP 
        )
    """)
    conn.commit()
    conn.close()
    print("current_table_orders.db 생성 완료")

def create_full_order_history_db():
    '''
    (오더 id, 테이블 번호, 메뉴 아이디, 수량, 주문 시간)을 저장하는 데이터베이스 테이블 생성
    pk는 오더 id
    '''
    conn = sqlite3.connect("./src/week1/full_order_history.db")
    cursor = conn.cursor()
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS full_order_history (
            order_id INTEGER PRIMARY KEY AUTOINCREMENT,
            table_id INTEGER,
            menu_id TEXT,
            quantity INTEGER,
            order_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)
    conn.commit()
    conn.close()
    print("full_order_history.db 생성 완료")

def initialize_all_databases():
    create_current_table_orders_db()
    create_full_order_history_db()
    print("모든 데이터베이스 테이블 생성 완료")

# 스크립트를 직접 실행할 때만 데이터베이스를 초기화
if __name__ == "__main__":
    initialize_all_databases()
