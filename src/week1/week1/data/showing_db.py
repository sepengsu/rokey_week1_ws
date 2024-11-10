import sqlite3
import pandas as pd

def print_db_contents(db_path, db_name):
    conn = sqlite3.connect(db_path)
    df = pd.read_sql_query("SELECT * FROM " + db_name, conn)
    print(f'{db_name} contents:')
    print(df)
    conn.close()


# Replace with your actual database paths
db1_path = './src/week1/current_table_orders.db'
db2_path = './src/week1/full_order_history.db'

print_db_contents(db1_path, 'current_table_orders')
print('-' * 50)
print_db_contents(db2_path, 'full_order_history')