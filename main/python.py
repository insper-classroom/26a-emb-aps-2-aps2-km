# subway_controller_bt.py
# Versão com suporte a USB Serial OU Bluetooth (HC-06)
# Protocolo: cada linha contém um caractere de comando (L/R/U/D/H/S/X)
# 'X' = GAME OVER -> Pico aciona motor vibratório
#
# DETECÇÃO DE GAME OVER:
#   Detecta o botão verde "PLAY" na ROI inferior da tela.
#   Esse botão só existe na tela de game over.

import serial
import serial.tools.list_ports
import time
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import cv2
import numpy as np

try:
    import mss
    USE_MSS = True
except ImportError:
    import pyautogui
    USE_MSS = False

BAUD_RATE_USB = 115200
BAUD_RATE_BT  = 115200

KEY_MAPPING = {
    "L": "left",
    "R": "right",
    "U": "up",
    "D": "down",
    "H": "space",
    "S": None,
    "X": None
}

# =====================================================================
# PARÂMETROS DE DETECÇÃO DE GAME OVER
# Calibrados a partir da tela real do Subway Surfers (1250x697).
# PLAY_ROI_REL usa proporções (0.0–1.0) — funciona em qualquer resolução.
# =====================================================================
PLAY_ROI_REL       = (0.50, 0.82, 0.70, 0.98)
LOWER_GREEN        = np.array([40, 120,  80], dtype=np.uint8)
UPPER_GREEN        = np.array([80, 255, 255], dtype=np.uint8)
MIN_GREEN_PX       = 15000
GAME_OVER_COOLDOWN = 5.0


# =====================================================================
# DETECTOR DE GAME OVER
# =====================================================================
class GameOverDetector:
    def __init__(self, on_game_over_cb, log_cb=None):
        self.on_game_over_cb = on_game_over_cb
        self.log_cb          = log_cb or (lambda msg: None)
        self.running         = False
        self._thread         = None
        self._last_detection = 0.0

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        modo = "mss" if USE_MSS else "pyautogui"
        self.log_cb(f"[GameOver] Detector iniciado (captura: {modo}).")

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=2)
        self.log_cb("[GameOver] Detector parado.")

    def _captura_frame(self):
        """Captura a tela inteira e retorna como array BGR."""
        if USE_MSS:
            with mss.mss() as sct:
                monitor = sct.monitors[1]
                shot = sct.grab(monitor)
                frame = np.array(shot, dtype=np.uint8)[:, :, :3]
                return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            import pyautogui
            screenshot = pyautogui.screenshot()
            return cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)

    def _detecta_play_verde(self, frame):
        """
        Recorta a ROI inferior direita e conta pixels verdes do botão PLAY.
        Retorna True se encontrar >= MIN_GREEN_PX pixels verdes.
        """
        h, w = frame.shape[:2]
        x1 = int(PLAY_ROI_REL[0] * w)
        y1 = int(PLAY_ROI_REL[1] * h)
        x2 = int(PLAY_ROI_REL[2] * w)
        y2 = int(PLAY_ROI_REL[3] * h)
        roi  = frame[y1:y2, x1:x2]
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        return cv2.countNonZero(mask) >= MIN_GREEN_PX

    def _loop(self):
        while self.running:
            try:
                now = time.time()
                if now - self._last_detection < GAME_OVER_COOLDOWN:
                    time.sleep(0.2)
                    continue

                frame = self._captura_frame()
                if self._detecta_play_verde(frame):
                    self._last_detection = now
                    self.log_cb("[GameOver] *** GAME OVER detectado! Enviando X ao Pico. ***")
                    self.on_game_over_cb()

            except Exception as e:
                self.log_cb(f"[GameOver] Erro: {e}")

            time.sleep(0.1)


# =====================================================================
# KEYBOARD CONTROLLER
# =====================================================================
class KeyboardController:
    def __init__(self):
        self.current_keys = set()

    def press_key(self, key):
        if key and key not in self.current_keys:
            import pyautogui
            pyautogui.keyDown(key)
            self.current_keys.add(key)

    def release_key(self, key):
        if key and key in self.current_keys:
            import pyautogui
            pyautogui.keyUp(key)
            self.current_keys.remove(key)

    def release_all(self):
        import pyautogui
        for key in list(self.current_keys):
            pyautogui.keyUp(key)
        self.current_keys.clear()

    def press_and_release(self, key):
        if key:
            import pyautogui
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
        # "X" não gera tecla


# =====================================================================
# SERIAL READER
# =====================================================================
class SerialReader:
    def __init__(self, port, baud, callback, status_callback):
        self.port            = port
        self.baud            = baud
        self.callback        = callback
        self.status_callback = status_callback
        self.running         = False
        self.serial          = None
        self.thread          = None

    def start(self):
        try:
            self.serial  = serial.Serial(self.port, self.baud, timeout=0.1)
            self.running = True
            self.thread  = threading.Thread(target=self._read_loop, daemon=True)
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

    def send(self, text: str):
        """
        Envia string para o Pico via USB Serial.
        Usado para enviar 'X\\n' quando o game over é detectado.
        """
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(text.encode("utf-8"))
                print(f"[Serial] Enviado: {repr(text)}")
            except Exception as e:
                print(f"[ERRO send] {e}")
        else:
            print("[ERRO send] Serial nao conectada!")

    def _read_loop(self):
        """Lê respostas do Pico (debug/eco) — não interfere no envio do 'X'."""
        self.serial.reset_input_buffer()
        while self.running:
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        # O primeiro caractere é o comando ecoado pelo Pico
                        cmd = line[0]
                        if cmd == 'X':
                            print(f"[DEBUG Pico] {repr(line)}")
                            continue
                        if cmd in KEY_MAPPING:
                            self.callback(cmd)
                        else:
                            print(f"[DEBUG Pico] {repr(line)}")
            except Exception as e:
                print(f"[ERRO serial] {e}")
                self.running = False
                break
            time.sleep(0.005)


# =====================================================================
# GUI PRINCIPAL
# =====================================================================
class SubwayControllerGUI:
    def __init__(self):
        self.keyboard      = KeyboardController()
        self.serial_reader = None
        self.go_detector   = None

        self.root = tk.Tk()
        self.root.title("Subway Surfers Controller — USB / Bluetooth")
        self.root.geometry("500x640")
        self.root.resizable(False, False)

        dark_bg = "#1e1e2e"
        dark_fg = "#cdd6f4"
        accent  = "#89b4fa"
        self.root.configure(bg=dark_bg)

        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(".",              background=dark_bg, foreground=dark_fg)
        style.configure("TFrame",         background=dark_bg)
        style.configure("TLabel",         background=dark_bg, foreground=dark_fg,
                        font=("Segoe UI", 11))
        style.configure("TButton",        font=("Segoe UI", 10), padding=8)
        style.configure("Accent.TButton", font=("Segoe UI", 11, "bold"),
                        background=accent, foreground="#1e1e2e")
        style.map("Accent.TButton", background=[("active", "#89cff0")])

        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.pack(expand=True, fill="both")

        ttk.Label(main_frame, text="🎮 Subway Surfers Controller",
                  font=("Segoe UI", 16, "bold")).pack(pady=(0, 4))
        ttk.Label(main_frame,
                  text="IMU + Botões | USB Serial ou Bluetooth HC-06",
                  justify="center", font=("Segoe UI", 9)).pack(pady=(0, 14))

        # --- Modo de conexão ---
        mode_frame = ttk.Frame(main_frame)
        mode_frame.pack(pady=(0, 8))
        ttk.Label(mode_frame, text="Modo:").pack(side="left", padx=(0, 8))
        self.mode_var = tk.StringVar(value="USB")
        ttk.Radiobutton(mode_frame, text="USB Serial",
                        variable=self.mode_var, value="USB",
                        command=self._on_mode_change).pack(side="left", padx=4)
        ttk.Radiobutton(mode_frame, text="Bluetooth (HC-06)",
                        variable=self.mode_var, value="BT",
                        command=self._on_mode_change).pack(side="left", padx=4)

        # --- Porta serial ---
        port_frame = ttk.Frame(main_frame)
        port_frame.pack(pady=6)
        ttk.Label(port_frame, text="Porta:").pack(side="left", padx=5)
        self.port_var      = tk.StringVar()
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
        ttk.Combobox(baud_frame, textvariable=self.baud_var,
                     values=["9600", "57600", "115200"],
                     state="readonly", width=10).pack(side="left", padx=5)

        # --- Status ---
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(pady=10)
        self.led_canvas  = tk.Canvas(status_frame, width=16, height=16,
                                     bg=dark_bg, highlightthickness=0)
        self.led_canvas.pack(side="left", padx=(0, 8))
        self.led_circle  = self.led_canvas.create_oval(2, 2, 14, 14,
                                                        fill="#444", outline="")
        self.status_label = ttk.Label(status_frame, text="Desconectado",
                                      font=("Segoe UI", 10))
        self.status_label.pack(side="left")

        self.last_cmd_label = ttk.Label(main_frame, text="Último comando: ---",
                                        font=("Segoe UI", 12))
        self.last_cmd_label.pack(pady=8)

        # --- Game Over detector ---
        go_frame = ttk.Frame(main_frame)
        go_frame.pack(pady=6)
        self.go_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(go_frame,
                        text="🎯 Detectar Game Over automaticamente",
                        variable=self.go_var,
                        command=self._toggle_go_detector).pack()
        self.go_status_label = ttk.Label(go_frame,
                                         text="Detector: inativo",
                                         font=("Segoe UI", 9))
        self.go_status_label.pack()

        # --- Botão conectar ---
        self.connect_btn = ttk.Button(main_frame, text="🔌 CONECTAR",
                                      style="Accent.TButton",
                                      command=self.toggle_connection)
        self.connect_btn.pack(pady=14)

        ttk.Label(main_frame,
                  text=("Comandos:  L=⬅️  R=➡️  U=🔼  D=🔽  H=🛹  S=⏸️  X=💀\n\n"
                        "Game Over: detecta botão PLAY verde na tela.\n"
                        "Bluetooth: emparelhe HC-06 (SUBWAY_CTRL / 1234)."),
                  justify="center", font=("Segoe UI", 9)).pack(pady=6)

        self.debug_label = ttk.Label(main_frame, text="", font=("Segoe UI", 8))
        self.debug_label.pack()

        self.refresh_ports()

    # ------------------------------------------------------------------
    def _on_mode_change(self):
        self.baud_var.set(str(BAUD_RATE_BT if self.mode_var.get() == "BT"
                              else BAUD_RATE_USB))

    def update_led(self, color):
        cores = {"green": "#a6e3a1", "red": "#f38ba8", "gray": "#585b70"}
        self.led_canvas.itemconfig(self.led_circle,
                                   fill=cores.get(color, cores["gray"]))

    def update_status(self, message, is_connected):
        self.status_label.config(text=message)
        self.update_led("green" if is_connected else "red")

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_dropdown["values"] = ports
        if ports:
            self.port_var.set(ports[0])

    def _log(self, msg):
        print(msg)
        self.root.after(0, lambda: self.debug_label.config(text=msg[-90:]))

    # ------------------------------------------------------------------
    # Game Over
    # ------------------------------------------------------------------
    def _toggle_go_detector(self):
        if self.go_var.get():
            if not (self.serial_reader and self.serial_reader.running):
                messagebox.showwarning(
                    "Aviso",
                    "Conecte ao Pico antes de ativar o detector de Game Over.")
                self.go_var.set(False)
                return
            self.go_detector = GameOverDetector(
                on_game_over_cb=self._on_game_over_detected,
                log_cb=self._log
            )
            self.go_detector.start()
            self.go_status_label.config(text="Detector: ATIVO 🟢 (botão PLAY verde)")
        else:
            if self.go_detector:
                self.go_detector.stop()
                self.go_detector = None
            self.go_status_label.config(text="Detector: inativo")

    def _on_game_over_detected(self):
        """
        Chamado pela thread do detector.
        Envia 'X\\n' ao Pico via USB Serial → task_usb_rx lê → motor vibra.
        """
        print("[GameOver] Enviando X para o Pico...")
        if self.serial_reader and self.serial_reader.running:
            self.serial_reader.send("X\n")
        else:
            print("[GameOver] ERRO: serial nao esta conectada!")
        self.root.after(0, lambda: self.last_cmd_label.config(
            text="Último comando: 💀 GAME OVER"))

    # ------------------------------------------------------------------
    def on_command_received(self, command):
        """Recebe comandos ecoados pelo Pico (debug)."""
        print(f"[Pico->PC] {command}")
        names = {
            "L": "⬅️ ESQUERDA", "R": "➡️ DIREITA",
            "U": "🔼 PULAR",    "D": "🔽 DESLIZAR",
            "H": "🛹 HOVERBOARD", "S": "⏸️ NEUTRO",
            "X": "💀 GAME OVER"
        }
        self.last_cmd_label.config(
            text=f"Último comando: {names.get(command, command)}")
        self.keyboard.set_state(command)
        self.debug_label.config(
            text=f"Teclas ativas: {self.keyboard.current_keys}")

    # ------------------------------------------------------------------
    def toggle_connection(self):
        if self.serial_reader and self.serial_reader.running:
            if self.go_detector:
                self.go_detector.stop()
                self.go_detector = None
                self.go_var.set(False)
                self.go_status_label.config(text="Detector: inativo")
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

    # ------------------------------------------------------------------
    def run(self):
        self.root.mainloop()
        if self.go_detector:
            self.go_detector.stop()
        if self.serial_reader:
            self.serial_reader.stop()
        self.keyboard.release_all()


if __name__ == "__main__":
    app = SubwayControllerGUI()
    app.run()