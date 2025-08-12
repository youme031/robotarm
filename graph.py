import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time

# --- 설정 ---
SERIAL_PORT = 'COM3'   # 장치 관리자에서 확인
BAUD_RATE = 115200
MAX_DATA_POINTS = 200  # 그래프에 표시할 최대 데이터 개수

# --- 시리얼 포트 연결 ---
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(1)
    ser.reset_input_buffer()
    print(f"'{SERIAL_PORT}'에 연결되었습니다. 그래프를 시작합니다...")
except serial.SerialException as e:
    print(f"오류: '{SERIAL_PORT}'를 열 수 없습니다. 포트 번호/충돌 확인")
    print(e)
    raise SystemExit

# --- 데이터 저장소 ---
angle_data = deque(maxlen=MAX_DATA_POINTS)

# --- 그래프 설정 ---
fig, ax = plt.subplots(figsize=(12, 6))
line, = ax.plot([], [], '-')        # 색 지정 X (기본값)
ax.set_title("Output Shaft Angle (deg)")
ax.set_xlabel("Time (samples)")
ax.set_ylabel("Angle (degrees)")
ax.set_ylim(0, 360)
ax.set_xlim(0, MAX_DATA_POINTS)
ax.grid(True)

def update(frame):
    try:
        # 버퍼에 쌓인 줄을 가능한 한 많이 처리 (지연 감소)
        while ser and ser.in_waiting > 0:
            line_bytes = ser.readline()
            if not line_bytes:
                break
            s = line_bytes.decode('utf-8', errors='ignore').strip()
            if not s:
                continue
            try:
                angle = float(s)   # 펌웨어가 "123.45" 형식으로 전송
                angle_data.append(angle)
            except ValueError:
                # 수신중 섞인 쓰레기 라인 무시
                pass

        line.set_data(range(len(angle_data)), angle_data)

        # 포인트가 꽉 차면 오른쪽으로 스크롤
        n = len(angle_data)
        if n < MAX_DATA_POINTS:
            ax.set_xlim(0, MAX_DATA_POINTS)
        else:
            ax.set_xlim(n - MAX_DATA_POINTS, n)

    except Exception as e:
        print(f"업데이트 중 오류: {e}")

    return line,

ani = FuncAnimation(fig, update, interval=50, blit=True)
plt.tight_layout()
try:
    plt.show()
finally:
    if ser and ser.is_open:
        ser.close()
        print(f"'{SERIAL_PORT}' 연결을 종료했습니다.")