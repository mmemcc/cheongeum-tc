from flask import Flask, request, jsonify
from datetime import datetime
import os

app = Flask(__name__)
UPLOAD_FOLDER = './uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# 현재 파일명 저장용
CURRENT_FILENAME = ""

@app.route('/upload/init', methods=['POST'])
def init_upload():
    global CURRENT_FILENAME
    try:
        json_data = request.get_json(force=True)
        if not json_data:
            return "[SERVER] No JSON payload", 400
        print("[SERVER] INIT DATA:", json_data)
        print("[SERVER] Headers:", dict(request.headers))

        start_time = json_data.get("start_time")
        elapsed_time = json_data.get("elapsed_time")
        relay_state = json_data.get("relay_state")
        samp_num_vib = json_data.get("samp_num_vib")
    
    
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        CURRENT_FILENAME = f"{timestamp}.txt"

        filepath = os.path.join(UPLOAD_FOLDER, CURRENT_FILENAME)
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write("[HEADER]\n")
            f.write(f"START_TIME: {start_time} usec\n")
            f.write(f"ELAPSED_TIME: {elapsed_time} usec\n")
            f.write(f"RELAY_STATE: {relay_state}\n")
            f.write(f"ACCELERATION_DATA: PERIOD SAMPLE NUMBER INDEX\n")
            f.write(f"1600 HZ {samp_num_vib} X,Y,Z\n")
            f.write("[END OF HEADER]\n")
            f.write("\n")

        print(f"[SERVER] Upload initialized: {CURRENT_FILENAME}")
        return f"[SERVER] Upload started with filename: {CURRENT_FILENAME}", 200

    except Exception as e:
        print("[SERVER ERROR]", e)
        return "[SERVER] Error in init", 500


@app.route('/upload', methods=['POST'])
def upload_line():
    global CURRENT_FILENAME
    try:
        if not CURRENT_FILENAME:
            return "[SERVER] Filename not set. Call /upload/init first.", 400

        data_line = request.get_data(as_text=True)
        if not data_line:
            return "[SERVER] Empty data", 400
        filepath = os.path.join(UPLOAD_FOLDER, CURRENT_FILENAME)
        with open(filepath, 'a', encoding='utf-8') as f:
            f.write(data_line)
            return "[SERVER] Data received", 200

    except Exception as e:
        print("[SERVER ERROR]", e)
        return "[SERVER] Upload error", 500


@app.route('/upload/end', methods=['POST'])
def end_upload():
    global CURRENT_FILENAME
    try:
        if not CURRENT_FILENAME:
            return "[SERVER] No file to end.", 400

        filepath = os.path.join(UPLOAD_FOLDER, CURRENT_FILENAME)
        with open(filepath, 'a', encoding='utf-8') as f:
            f.write("\n[UPLOAD END]\n")

        print(f"[SERVER] Upload complete: {CURRENT_FILENAME}")
        CURRENT_FILENAME = ""  # 초기화
        return "[SERVER] Upload finished", 200

    except Exception as e:
        print("[SERVER ERROR]", e)
        return "[SERVER] Error in end", 500


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
