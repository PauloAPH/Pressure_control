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
serialInst.port = 'COM4'
serialInst.timeout = 5
serialInst.open()

OUTPUT_DIR = "figuras"
SAMPLE_MS  = 20
OFFSETS    = [0.1204, 0.1773, 0.1295, 0.1231]

os.makedirs(OUTPUT_DIR, exist_ok=True)


def timestamp():
    return time.strftime("%Y_%m_%d_%H%M%S")


def converte_pressao(raw, offset):
    v = raw * (3.3 / 256.0)
    return ((v / 5.0) - offset) / 0.003


def recebe_identificacao(n_samples, prefixo):
    dc = np.zeros(n_samples, dtype=np.int32)
    p  = [np.zeros(n_samples, dtype=np.float32) for _ in range(4)]

    print(f"  Recebendo {n_samples} amostras ({prefixo})...")
    for i in range(n_samples):
        dc[i] = int(serialInst.readline().decode('utf-8').strip())
        for ch in range(4):
            raw = int(serialInst.readline().decode('utf-8').strip())
            p[ch][i] = converte_pressao(raw, OFFSETS[ch])

    t  = np.arange(n_samples) * (SAMPLE_MS / 1000.0)
    ts = timestamp()

    mat_path = os.path.join(OUTPUT_DIR, f"{prefixo}_{ts}.mat")
    scipy.io.savemat(mat_path, {
        'dc':       dc.astype(np.float64),
        'p1':       p[0].astype(np.float64),
        'p2':       p[1].astype(np.float64),
        'p3':       p[2].astype(np.float64),
        'p4':       p[3].astype(np.float64),
        'time':     t.astype(np.float64),
        'Ts':       np.float64(SAMPLE_MS / 1000.0),
        'n_samples': np.float64(n_samples),
    })
    print(f"  Dados salvos: {mat_path}")

    titulo = "Identificacao Bomba" if prefixo == "idsist_bomba" else "Identificacao ABS"
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    axes[0].step(t, dc, where='post', color='tab:blue', linewidth=1.2)
    axes[0].set_ylabel("PWM (%)")
    axes[0].set_title(f"{titulo}  ({ts})")
    axes[0].set_ylim(0, 105)
    axes[0].grid(True)
    for ch in range(4):
        axes[1].plot(t, p[ch], label=f'Sensor {ch+1}')
    axes[1].set_xlabel("Tempo [s]")
    axes[1].set_ylabel("Pressao [bar]")
    axes[1].legend()
    axes[1].grid(True)
    plt.tight_layout()
    png_path = os.path.join(OUTPUT_DIR, f"{prefixo}_{ts}.png")
    plt.savefig(png_path, dpi=150)
    plt.close(fig)
    print(f"  Grafico salvo: {png_path}")


t_test      = 0
start_time  = None
x           = 0
modo_idsist = "idsist_bomba"

while True:
    raw = serialInst.readline()
    if not raw:
        continue
    try:
        line = raw.decode('utf-8').rstrip('\r\n')
    except UnicodeDecodeError:
        continue

    if "Iniciar teste?" in line:
        print(line)
        x = input("Escolha (1/2/3): ")
        if   x.strip() == '2': modo_idsist = "idsist_bomba"
        elif x.strip() == '3': modo_idsist = "idsist_abs"
        serialInst.reset_input_buffer()
        serialInst.write(x.encode())

    elif line == "Entre com o PWM da bomba (%)":
        serialInst.reset_input_buffer()
        x = input(line + ": ")
        serialInst.write(x.encode())

    elif line == "Entre com o PWM das valvulas ABS (%)":
        serialInst.reset_input_buffer()
        x = input(line + ": ")
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
        labels = ['Sensor1','Sensor2','Sensor3','Sensor4',
                  'P_Ref1', 'P_Ref2', 'P_Ref3', 'P_Ref4']
        for ch in range(8):
            ax.plot(t_axis, data[ch], label=labels[ch])
        ax.set_xlim(0, int(t_test))
        ax.set_xlabel("Tempo [s]")
        ax.set_ylabel("Pressao [bar]")
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
        print(f"  Inicio transmissao – {n_samples} amostras.")
        recebe_identificacao(n_samples, modo_idsist)

    elif line == "IDSIST_END":
        print("  Transmissao encerrada.\n")

    else:
        if line:
            print(f"[Arduino] {line}")