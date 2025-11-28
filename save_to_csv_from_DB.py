# migrate_ranges_to_csv.py
import pymysql
import json
import pandas as pd

NUM_RANGES = 360
SRC_TABLE = "scanmocktb"
OUTPUT_CSV = "ranges_and_action.csv"

config = dict(
    host='localhost',
    user='root',
    password='000630',
    database='scanmockdb',
    charset='utf8mb4',
    cursorclass=pymysql.cursors.DictCursor,
    autocommit=False
)

def fetch_scanmock_data():
    sql = f"SELECT ranges, act FROM {SRC_TABLE};"
    with pymysql.connect(**config) as con:
        with con.cursor() as cur:
            cur.execute(sql)
            rows = cur.fetchall()
    return rows

def parse_ranges(ranges_json):
    """
    main.py에서 ranges는 JSON 문자열로 저장되므로
    이것을 float 리스트(길이 360)로 변환한다.
    """
    try:
        data = json.loads(ranges_json)
        if isinstance(data, list) and len(data) == NUM_RANGES:
            return [float(x) for x in data]
        else:
            return None
    except Exception:
        return None

def migrate_to_csv():
    rows = fetch_scanmock_data()

    converted_rows = []
    drop_count = 0

    for row in rows:
        ranges = parse_ranges(row["ranges"])
        act = row["act"]

        if ranges is None:
            drop_count += 1
            continue

        converted_rows.append(ranges + [act])

    # create DataFrame
    columns = [f"range_{i}" for i in range(NUM_RANGES)] + ["act"]
    df = pd.DataFrame(converted_rows, columns=columns)

    # save CSV
    df.to_csv(OUTPUT_CSV, index=False, encoding='utf-8-sig')

    print(f"[완료] CSV 생성됨: {OUTPUT_CSV}")
    print(f"[총 행수] {len(df)} (제외된 파싱 실패 {drop_count}건)")

if __name__ == "__main__":
    migrate_to_csv()