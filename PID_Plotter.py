import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time

# -------- 설정 --------
SERIAL_PORT = 'COM3'     # 포트만 수정
BAUD_RATE   = 115200
MAX_N       = 400

# -------- 시리얼 --------
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    time.sleep(1)
    ser.reset_input_buffer()
    print(f"[OK] Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"[ERR] Cannot open {SERIAL_PORT}: {e}")
    raise SystemExit

# -------- 데이터 버퍼 --------
meas = deque(maxlen=MAX_N)
tgt  = deque(maxlen=MAX_N)
uout = deque(maxlen=MAX_N)
errb = deque(maxlen=MAX_N)

# -------- 플롯 --------
fig, ax = plt.subplots(figsize=(12,6))
l_meas, = ax.plot([], [], '-',  label='meas(deg)')
l_tgt ,  = ax.plot([], [], '--', label='target(deg)')
ax2 = ax.twinx()
l_u,     = ax2.plot([], [], ':', label='u (duty 0~1)')

ax.set_title("PID Tuning - Angle & Output (viewer-only)")
ax.set_xlabel("samples")
ax.set_ylabel("angle (deg)")
ax2.set_ylabel("u (duty)")
ax.set_ylim(0, 360)
ax2.set_ylim(0, 1.0)
ax.grid(True)
lns = [l_meas, l_tgt, l_u]
labs = [l.get_label() for l in lns]
ax.legend(lns, labs, loc='upper left')

txt = ax.text(0.01, 0.95, "", transform=ax.transAxes, va='top', ha='left',
              bbox=dict(boxstyle='round', alpha=0.2))

def update(_):
    # 들어온 만큼 읽기
    try:
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                break
            parts = line.split(',')
            if len(parts) < 4:  # meas,target,u,err
                continue
            try:
                m = float(parts[0]); t = float(parts[1])
                u = float(parts[2]); e = float(parts[3])
            except ValueError:
                continue
            meas.append(m); tgt.append(t); uout.append(u); errb.append(e)
    except Exception as e:
        print("[RX ERR]", e)

    n = len(meas)
    xs = range(max(0, n - MAX_N), n)
    l_meas.set_data(xs, list(meas))
    l_tgt.set_data(xs, list(tgt))
    l_u.set_data(xs, list(uout))

    if n < MAX_N:
        ax.set_xlim(0, MAX_N)
    else:
        ax.set_xlim(n - MAX_N, n)

    if n > 0:
        txt.set_text(f"last meas={meas[-1]:.2f}°  target={tgt[-1]:.2f}°  "
                     f"u={uout[-1]:.3f}  err={errb[-1]:.2f}")
    return l_meas, l_tgt, l_u, txt

ani = FuncAnimation(fig, update, interval=50, blit=True)
plt.tight_layout()
try:
    plt.show()
finally:
    if ser and ser.is_open:
        ser.close()
        print("Serial closed.")
