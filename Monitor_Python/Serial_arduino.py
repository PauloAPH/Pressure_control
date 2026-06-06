import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import time
import scipy.io
import os

ports = serial.tools.list_ports.comports()
for p in ports:
    print(p)

serialInst = serial.Serial()
serialInst.baudrate = 9600
serialInst.port = 'COM4'  # Linux: '/dev/ttyACM0'
serialInst.timeout = 5
serialInst.open()

OUTPUT_DIR   = "figuras"
SAMPLE_MS    = 20
PRESS_SCALE  = 0.1

os.makedirs(OUTPUT_DIR, exist_ok=True)


def timestamp():
    return time.strftime("%Y_%m_%d_%H%M%S")


def recebe_identificacao(n_samples):
    dc = np.zeros(n_samples, dtype=np.int32)
    p1 = np.zeros(n_samples, dtype=np.float32)
    p2 = np.zeros(n_samples, dtype=np.float32)
    p3 = np.zeros(n_samples, dtype=np.float32)
    p4 = np.zeros(n_samples, dtype=np.float32)

    print(f"  Recebendo {n_samples} amostras...")
    for i in range(n_samples):
        dc[i] = int(serialInst.readline().decode('utf-8').strip())
        p1[i] = float(serialInst.readline().decode('utf-8').strip())/100
        p2[i] = float(serialInst.readline().decode('utf-8').strip())/100
        p3[i] = float(serialInst.readline().decode('utf-8').strip())/100 
        p4[i] = float(serialInst.readline().decode('utf-8').strip())/100 

        p1[i] = ((p1[i]/5) - 0.1204)/0.003
        p2[i] = ((p2[i]/5) - 0.1773)/0.003
        p3[i] = ((p3[i]/5) - 0.1295)/0.003
        p4[i] = ((p4[i]/5) - 0.1231)/0.003

    t  = np.arange(n_samples) * (SAMPLE_MS / 1000.0)
    ts = timestamp()

    mat_path = os.path.join(OUTPUT_DIR, f"idsist_{ts}.mat")
    scipy.io.savemat(mat_path, {
        'dc': dc.astype(np.float64), 'p1': p1.astype(np.float64),
        'p2': p2.astype(np.float64), 'p3': p3.astype(np.float64),
        'p4': p4.astype(np.float64), 'time': t.astype(np.float64),
        'Ts': np.float64(SAMPLE_MS / 1000.0), 'n_samples': np.float64(n_samples),
    })
    print(f"  Dados salvos: {mat_path}")

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    axes[0].step(t, dc, where='post', color='tab:blue', linewidth=1.2)
    axes[0].set_ylabel("Duty Cycle da Bomba [%]")
    axes[0].set_title(f"Identificacao de Sistemas – Bomba  ({ts})")
    axes[0].set_ylim(0, 105)
    axes[0].grid(True)
    axes[1].plot(t, p1, label='Sensor 1')
    axes[1].plot(t, p2, label='Sensor 2')
    axes[1].plot(t, p3, label='Sensor 3')
    axes[1].plot(t, p4, label='Sensor 4')
    axes[1].set_xlabel("Tempo [s]")
    axes[1].set_ylabel("Pressao [bar]")
    axes[1].legend()
    axes[1].grid(True)
    plt.tight_layout()
    png_path = os.path.join(OUTPUT_DIR, f"idsist_{ts}.png")
    plt.savefig(png_path, dpi=150)
    plt.close(fig)
    print(f"  Grafico salvo: {png_path}")


t_test     = 0
start_time = None
x          = 0

while True:
    raw = serialInst.readline()
    if not raw:
        continue
    try:
        line = raw.decode('utf-8').rstrip('\r\n')
    except UnicodeDecodeError:
        continue

    if line == "Iniciar teste? 1 = Teste1 (controle), 2 = Teste2 (identificacao)":
        x = input(line)
        serialInst.reset_input_buffer()
        serialInst.write(x.encode())

    elif line == "Entre com a pressao":
        serialInst.reset_input_buffer()
        x = input(line + ": ")
        serialInst.write(x.encode())

    elif line == "Entre com o tempo":
        serialInst.reset_input_buffer()
        t_test = input(line + " (segundos): ")
        serialInst.reset_input_buffer()
        serialInst.write(t_test.encode())

    elif line == "Teste em andamento":
        print(line)
        start_time = time.time()

    elif line == "Imprimir resultado?":
        elapsed = time.time() - start_time if start_time else 0
        print(f"--- {elapsed:.2f} seconds ---")
        serialInst.reset_input_buffer()
        x = input(line + " (1=Sim): ")
        serialInst.reset_input_buffer()
        serialInst.write(x.encode())

    elif line == "Imprimindo":
        n      = int(t_test) * 100
        data   = [np.zeros(n) for _ in range(8)]
        t_axis = np.arange(0, int(t_test), 0.01)
        for i in range(n):
            for ch in range(8):
                data[ch][i] = int(serialInst.readline().decode('utf-8').strip())
        ts = timestamp()
        fig, ax = plt.subplots(figsize=(12, 5))
        labels = ['Sensor1','Sensor2','Sensor3','Sensor4','P_Ref1','P_Ref2','P_Ref3','P_Ref4']
        for ch in range(8):
            ax.plot(t_axis, data[ch], label=labels[ch])
        ax.set_xlim(0, int(t_test))
        ax.set_xlabel("Tempo [s]")
        ax.set_ylabel("Pressao [bar*10]")
        ax.legend()
        ax.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(OUTPUT_DIR, f"{ts}.png"), dpi=150)
        plt.close(fig)
        scipy.io.savemat(os.path.join(OUTPUT_DIR, f"{ts}.mat"), {
            'pref1': data[4], 'p1': data[0], 'p2': data[1],
            'p3':    data[2], 'p4': data[3], 'time': t_axis,
            'pref2': data[5], 'pref3': data[6], 'pref4': data[7],
        })
        print(f"Teste1 salvo: {ts}")

    elif line == "IDSIST_START":
        n_samples = int(serialInst.readline().decode('utf-8').strip())
        print(f"  Inicio identificacao – {n_samples} amostras.")
        recebe_identificacao(n_samples)

    elif line == "IDSIST_END":
        print("  Transmissao encerrada.\n")

    else:
        if line:
            print(f"[Arduino] {line}")