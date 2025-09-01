import serial
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from collections import deque
import time, threading, queue, tkinter as tk
from tkinter import ttk

# -------- 설정 (Settings) --------
SERIAL_PORT = 'COM8' # 사용자 환경에 맞게 수정 (Adjust to your COM port)
BAUD_RATE   = 115200
MAX_N       = 400 # 그래프에 표시할 최대 데이터 포인트 수 (Max data points to display)

# -------- 스레드 통신용 (For thread communication) --------
data_queue = queue.Queue()
stop_thread = False

current_target = 0.0
uphill_state   = None
ser = None

def send_target(deg: float):
    """지정한 각도를 시리얼 포트로 전송합니다."""
    global current_target, ser
    try:
        if ser and ser.is_open:
            ser.write(f"T,{deg}\n".encode("utf-8"))
            current_target = deg % 360.0
    except Exception as e:
        print(f"[SEND ERROR] {e}")

def serial_reader():
    """시리얼 포트로부터 데이터를 지속적으로 읽어 큐에 추가하는 스레드 함수."""
    global stop_thread, ser
    serial_buffer = ""
    print("Serial reader thread started.")
    while not stop_thread:
        try:
            if ser and ser.in_waiting > 0:
                serial_buffer += ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
                lines = serial_buffer.split("\n")
                serial_buffer = lines[-1]
                for line in lines[:-1]:
                    line = line.strip()
                    if not line or line.startswith("ACK"):
                        continue

                    parts = line.split(",")
                    try:
                        # --- ✨ 변경: 4개의 데이터를 받도록 수정 ---
                        if len(parts) >= 4:
                            m_360 = float(parts[0])
                            e     = float(parts[1])
                            is_up = int(parts[2])
                            tgt_u = float(parts[3]) # 실제 목표 각도(Unwrapped)
                            data_queue.put((m_360, e, is_up, tgt_u))
                    except (ValueError, IndexError) as parse_error:
                        # --- ✨ 변경: 에러를 출력하여 디버깅 용이하게 ---
                        print(f"[PARSE ERROR] Line: '{line}', Error: {parse_error}")
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"[Serial Thread ERROR] {e}")
            time.sleep(0.1)
    print("Serial reader thread stopped.")

# -------- 시리얼 연결 (Serial Connection) --------
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(1)
    ser.reset_input_buffer()
    print(f"[OK] Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"[ERROR] Cannot open {SERIAL_PORT}: {e}")
    ser = None

# -------- Tk GUI --------
root = tk.Tk()
root.title("L1 Motor PID Debugging")

root.columnconfigure(0, weight=0)
root.columnconfigure(1, weight=1)
root.rowconfigure(0, weight=1)

ctrl = ttk.Frame(root, padding=8)
ctrl.grid(row=0, column=0, sticky="ns")

ttk.Label(ctrl, text="목표각도(°):").grid(row=0, column=0, padx=4, pady=4)
ent = ttk.Entry(ctrl, width=10)
ent.grid(row=0, column=1, padx=4, pady=4)
ent.insert(0, "0")
ttk.Button(ctrl, text="보내기", command=lambda: (send_target(float(ent.get())))).grid(row=0, column=2, padx=4, pady=4)

def mk_btn(text, val, c, r):
    """목표 각도 설정을 위한 단축 버튼 생성 함수"""
    ttk.Button(
        ctrl,
        text=text,
        command=lambda: (ent.delete(0, tk.END), ent.insert(0, str(val)), send_target(val))
    ).grid(row=r, column=c, padx=2, pady=2, sticky="ew")

mk_btn("0°",   0,   0, 1)
mk_btn("90°",  90,  1, 1)
mk_btn("180°", 180, 2, 1)

state_var = tk.StringVar(value="STATE: ?")
ttk.Label(ctrl, textvariable=state_var, relief="groove", padding=6)\
   .grid(row=2, column=0, columnspan=3, sticky="ew", pady=(10, 0))

# [수정] 현재 각도 표시 GUI 추가
ttk.Label(ctrl, text="현재 각도 (deg):").grid(row=3, column=0, columnspan=3, sticky="ew", pady=(10, 2))
current_angle_var = tk.StringVar(value="0.00")
ttk.Label(ctrl, textvariable=current_angle_var, font=("Courier", 14, "bold"), foreground="blue", relief="sunken", padding=4)\
   .grid(row=4, column=0, columnspan=3, sticky="ew")


# -------- 플롯 (Plot) --------
plot_frame = ttk.Frame(root, padding=(8, 8))
plot_frame.grid(row=0, column=1, sticky="nsew")
plot_frame.rowconfigure(0, weight=1)
plot_frame.columnconfigure(0, weight=1)

fig, ax = plt.subplots(figsize=(10, 6))
ax2 = ax.twinx()

l_meas, = ax.plot([], [], '-',  label='Angle (0-360, deg)')
l_tgt,  = ax.plot([], [], '--', label='Target (0-360, deg)')
l_err,  = ax2.plot([], [], ':', color='r', label='Error (deg)')

ax.set_title("L1 Motor PID Debugging")
ax.set_xlabel("Samples")
ax.set_ylabel("Angle (deg)")
ax2.set_ylabel("Error (deg)", color='r')
ax.grid(True)
ax.set_ylim(-10, 370)
ax2.set_ylim(-185, 185)

lines = [l_meas, l_tgt, l_err]
ax.legend(lines, [l.get_label() for l in lines], loc='upper right')

canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
toolbar = NavigationToolbar2Tk(canvas, plot_frame, pack_toolbar=False)
toolbar.update()
toolbar.grid(row=1, column=0, sticky="ew")

# -------- 데이터 버퍼 및 상태 (Data Buffers & State) --------
meas_buf     = deque(maxlen=MAX_N)
errb         = deque(maxlen=MAX_N)
tgt_buf      = deque(maxlen=MAX_N)
sample_count = 0

def update(frame):
    """그래프를 주기적으로 업데이트하는 함수"""
    global sample_count, uphill_state

    while not data_queue.empty():
        try:
            # --- ✨ 변경: 4개의 데이터를 큐에서 가져옴 ---
            m_360, e, is_up, tgt_u = data_queue.get_nowait()

            meas_buf.append(m_360)
            errb.append(e)
            # --- ✨ 변경: 수신된 실제 목표 각도를 0~360으로 변환하여 버퍼에 추가 ---
            tgt_buf.append(tgt_u % 360.0)

            uphill_state = (is_up != 0)

            # 현재 각도 텍스트 값 업데이트
            current_angle_var.set(f"{m_360:.2f}")

            sample_count += 1
        except queue.Empty:
            break
        except Exception as e:
            print(f"[Update Error] {e}")
            continue

    if len(meas_buf) > 0:
        xs = range(max(0, sample_count - MAX_N), sample_count)
        l_meas.set_data(xs, list(meas_buf))
        l_err.set_data(xs, list(errb))
        l_tgt.set_data(xs, list(tgt_buf))

        ax.set_xlim(xs.start, xs.stop)

        state_var.set("STATE: ?" if uphill_state is None
                      else ("STATE: UPHILL" if uphill_state else "STATE: DOWNHILL"))

    return l_meas, l_err, l_tgt

ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

def on_close():
    """창을 닫을 때 시리얼 포트를 닫고 스레드를 종료합니다."""
    global stop_thread, ser
    stop_thread = True
    time.sleep(0.1)
    try:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")
    except:
        pass
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
threading.Thread(target=serial_reader, daemon=True).start()
root.mainloop()
