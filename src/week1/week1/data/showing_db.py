import sqlite3

def print_db_contents(db_path, db_name):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = cursor.fetchall()
    
    print(f"Contents of {db_name}:")
    for table in tables:
        print(f"\nTable: {table[0]}")
        cursor.execute(f"SELECT * FROM {table[0]}")
        rows = cursor.fetchall()
        for row in rows:
            print(row)
    
    conn.close()

# Replace with your actual database paths
db1_path = './src/week1/current_table_orders.db'
db2_path = './src/week1/full_order_history.db'

print_db_contents(db1_path, 'Database 1')
print_db_contents(db2_path, 'Database 2')