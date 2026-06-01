# subway_controller_bt.py
# Versão com suporte a USB Serial OU Bluetooth (HC-06)
# Protocolo idêntico: cada linha contém um caractere de comando (L/R/U/D/H/S)

import serial
import serial.tools.list_ports
import pyautogui
import time
import tkinter as tk
from tkinter import ttk, messagebox
import threading

BAUD_RATE_USB = 115200
BAUD_RATE_BT  = 115200   # HC-06 configurado em 115200

KEY_MAPPING = {
    "L": "left",
    "R": "right",
    "U": "up",
    "D": "down",
    "H": "space",
    "S": None
}


class KeyboardController:
    def __init__(self):
        self.current_keys = set()

    def press_key(self, key):
        if key and key not in self.current_keys:
            pyautogui.keyDown(key)
            self.current_keys.add(key)

    def release_key(self, key):
        if key and key in self.current_keys:
            pyautogui.keyUp(key)
            self.current_keys.remove(key)

    def release_all(self):
        for key in list(self.current_keys):
            pyautogui.keyUp(key)
        self.current_keys.clear()

    def press_and_release(self, key):
        if key:
            pyautogui.press(key)

    def set_state(self, command):
        if command == "L":
            self.press_key("left")
            self.release_key("right")
        elif command == "R":
            self.press_key("right")
            self.release_key("left")
        elif command in ("U", "D", "H"):
            self.press_and_release(KEY_MAPPING[command])
        elif command == "S":
            self.release_key("left")
            self.release_key("right")


class SerialReader:
    def __init__(self, port, baud, callback, status_callback):
        self.port = port
        self.baud = baud
        self.callback = callback
        self.status_callback = status_callback
        self.running = False
        self.serial = None
        self.thread = None

    def start(self):
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            self.status_callback("Conectado!", True)
            return True
        except Exception as e:
            self.status_callback(f"Erro: {e}", False)
            return False

    def stop(self):
        self.running = False
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
        if self.thread:
            self.thread.join(timeout=1)
        self.status_callback("Desconectado", False)

    def _read_loop(self):
        self.serial.reset_input_buffer()
        while self.running:
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        cmd = line[0]
                        if cmd in KEY_MAPPING:
                            self.callback(cmd)
                        else:
                            print(f"[DEBUG] ignorado: {repr(line)}")
            except Exception as e:
                print(f"[ERRO serial] {e}")
                self.running = False
                break
            time.sleep(0.005)


class SubwayControllerGUI:
    def __init__(self):
        self.keyboard = KeyboardController()
        self.serial_reader = None

        self.root = tk.Tk()
        self.root.title("Subway Surfers Controller — USB / Bluetooth")
        self.root.geometry("480x540")
        self.root.resizable(False, False)

        dark_bg = "#1e1e2e"
        dark_fg = "#cdd6f4"
        accent  = "#89b4fa"
        self.root.configure(bg=dark_bg)

        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(".", background=dark_bg, foreground=dark_fg)
        style.configure("TFrame",  background=dark_bg)
        style.configure("TLabel",  background=dark_bg, foreground=dark_fg, font=("Segoe UI", 11))
        style.configure("TButton", font=("Segoe UI", 10), padding=8)
        style.configure("Accent.TButton", font=("Segoe UI", 11, "bold"),
                        background=accent, foreground="#1e1e2e")
        style.map("Accent.TButton", background=[("active", "#89cff0")])

        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.pack(expand=True, fill="both")

        ttk.Label(main_frame, text="🎮 Subway Surfers Controller",
                  font=("Segoe UI", 16, "bold")).pack(pady=(0, 6))

        ttk.Label(main_frame,
                  text="Controle por IMU + Botões — USB Serial ou Bluetooth HC-06",
                  justify="center", font=("Segoe UI", 9)).pack(pady=(0, 16))

        # --- Seleção de modo de conexão ---
        mode_frame = ttk.Frame(main_frame)
        mode_frame.pack(pady=(0, 10))
        ttk.Label(mode_frame, text="Modo:").pack(side="left", padx=(0, 8))
        self.mode_var = tk.StringVar(value="USB")
        ttk.Radiobutton(mode_frame, text="USB Serial", variable=self.mode_var,
                        value="USB", command=self._on_mode_change).pack(side="left", padx=4)
        ttk.Radiobutton(mode_frame, text="Bluetooth (HC-06)", variable=self.mode_var,
                        value="BT",  command=self._on_mode_change).pack(side="left", padx=4)

        # --- Porta serial ---
        port_frame = ttk.Frame(main_frame)
        port_frame.pack(pady=6)
        ttk.Label(port_frame, text="Porta:").pack(side="left", padx=5)
        self.port_var = tk.StringVar()
        self.port_dropdown = ttk.Combobox(port_frame, textvariable=self.port_var,
                                          state="readonly", width=14)
        self.port_dropdown.pack(side="left", padx=5)
        ttk.Button(port_frame, text="🔄", width=3,
                   command=self.refresh_ports).pack(side="left", padx=5)

        # --- Baud rate ---
        baud_frame = ttk.Frame(main_frame)
        baud_frame.pack(pady=4)
        ttk.Label(baud_frame, text="Baud rate:").pack(side="left", padx=5)
        self.baud_var = tk.StringVar(value=str(BAUD_RATE_USB))
        baud_cb = ttk.Combobox(baud_frame, textvariable=self.baud_var,
                               values=["9600", "57600", "115200"], state="readonly", width=10)
        baud_cb.pack(side="left", padx=5)

        # --- Status ---
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(pady=10)
        self.led_canvas = tk.Canvas(status_frame, width=16, height=16,
                                    bg=dark_bg, highlightthickness=0)
        self.led_canvas.pack(side="left", padx=(0, 8))
        self.led_circle = self.led_canvas.create_oval(2, 2, 14, 14, fill="#444", outline="")
        self.status_label = ttk.Label(status_frame, text="Desconectado",
                                      font=("Segoe UI", 10))
        self.status_label.pack(side="left")

        self.last_cmd_label = ttk.Label(main_frame, text="Último comando: ---",
                                        font=("Segoe UI", 12))
        self.last_cmd_label.pack(pady=8)

        self.connect_btn = ttk.Button(main_frame, text="🔌 CONECTAR",
                                      style="Accent.TButton",
                                      command=self.toggle_connection)
        self.connect_btn.pack(pady=16)

        ttk.Label(main_frame,
                  text=("🎮 Comandos recebidos:\n"
                        "   L = ⬅️ Esquerda    R = ➡️ Direita\n"
                        "   U = 🔼 Pular        D = 🔽 Deslizar\n"
                        "   H = 🛹 Hoverboard   S = ⏸️ Neutro\n\n"
                        "Bluetooth: emparelhe o HC-06 (SUBWAY_CTRL / PIN 1234)\n"
                        "e selecione a porta COM correspondente."),
                  justify="left", font=("Segoe UI", 9)).pack(pady=8)

        self.debug_label = ttk.Label(main_frame, text="", font=("Segoe UI", 8))
        self.debug_label.pack()

        self.refresh_ports()

    # ---- helpers ----

    def _on_mode_change(self):
        mode = self.mode_var.get()
        self.baud_var.set(str(BAUD_RATE_BT if mode == "BT" else BAUD_RATE_USB))

    def update_led(self, color):
        cores = {"green": "#a6e3a1", "red": "#f38ba8", "gray": "#585b70"}
        self.led_canvas.itemconfig(self.led_circle, fill=cores.get(color, cores["gray"]))

    def update_status(self, message, is_connected):
        self.status_label.config(text=message)
        self.update_led("green" if is_connected else "red")

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_dropdown["values"] = ports
        if ports:
            self.port_var.set(ports[0])

    def on_command_received(self, command):
        print(f"[RECEBIDO] {command}")
        names = {
            "L": "⬅️ ESQUERDA", "R": "➡️ DIREITA",
            "U": "🔼 PULAR",    "D": "🔽 DESLIZAR",
            "H": "🛹 HOVERBOARD", "S": "⏸️ NEUTRO"
        }
        self.last_cmd_label.config(text=f"Último comando: {names.get(command, command)}")
        self.keyboard.set_state(command)
        self.debug_label.config(text=f"Teclas ativas: {self.keyboard.current_keys}")

    def toggle_connection(self):
        if self.serial_reader and self.serial_reader.running:
            self.serial_reader.stop()
            self.keyboard.release_all()
            self.serial_reader = None
            self.connect_btn.config(text="🔌 CONECTAR")
            self.update_status("Desconectado", False)
            self.last_cmd_label.config(text="Último comando: ---")
            self.debug_label.config(text="")
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showwarning("Aviso", "Selecione uma porta serial!")
                return
            baud = int(self.baud_var.get())
            self.serial_reader = SerialReader(port, baud,
                                              self.on_command_received,
                                              self.update_status)
            if self.serial_reader.start():
                mode = self.mode_var.get()
                self.connect_btn.config(
                    text=f"🔴 DESCONECTAR ({'BT' if mode == 'BT' else 'USB'})"
                )

    def run(self):
        self.root.mainloop()
        if self.serial_reader:
            self.serial_reader.stop()
        self.keyboard.release_all()


if __name__ == "__main__":
    app = SubwayControllerGUI()
    app.run()