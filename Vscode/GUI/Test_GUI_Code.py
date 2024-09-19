import tkinter as tk
from tkinter import ttk
import serial
import threading

# Seriële verbinding met STM32 microcontroller
ser = serial.Serial('COM3', 9600, timeout=1)  # Pas aan naar de juiste COM-poort

class TestSystemGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Test Systeem GUI")
        
        # Grootte van het venster aanpassen
        self.root.geometry("800x600")  # Breedte x Hoogte in pixels
        
        self.create_widgets()

        # Start een thread om data van STM32 te lezen
        self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
        self.read_thread.start()

    def create_widgets(self):
        # L1, L2, L3 sliders voor stroomwaarden
        self.create_slider("L1 Stroom (Ampère):", 5, 500, 1, "L1", 0)
        self.create_slider("L2 Stroom (Ampère):", 5, 500, 1, "L2", 1)
        self.create_slider("L3 Stroom (Ampère):", 5, 500, 1, "L3", 2)

        # THD% sliders
        self.create_slider("THD% L1:", 5, 50, 1, "THD_L1", 3)
        self.create_slider("THD% L2:", 5, 50, 1, "THD_L2", 4)
        self.create_slider("THD% L3:", 5, 50, 1, "THD_L3", 5)

        # Knoppen voor testbediening
        ttk.Button(self.root, text="Start Test", command=self.send_settings).grid(row=6, column=0, pady=20, padx=20)
        ttk.Button(self.root, text="Stop Test", command=self.stop_test).grid(row=6, column=1, pady=20, padx=20)

        # Actieve testscenario weergave
        self.active_scenario_label = tk.Label(self.root, text="Actief Testscenario: Geen", font=("Arial", 16))
        self.active_scenario_label.grid(row=7, column=0, columnspan=2, pady=40)

    def create_slider(self, label_text, min_val, max_val, resolution, name, row):
        label = tk.Label(self.root, text=label_text, font=("Arial", 12))
        label.grid(row=row, column=0, padx=20, pady=10)
        slider = tk.Scale(self.root, from_=min_val, to=max_val, resolution=resolution, orient=tk.HORIZONTAL, length=400)
        slider.grid(row=row, column=1, padx=20, pady=10)
        setattr(self, name, slider)

    def send_settings(self):
        # Lees de waarden van alle sliders
        l1 = self.L1.get()
        l2 = self.L2.get()
        l3 = self.L3.get()
        thd_l1 = self.THD_L1.get()
        thd_l2 = self.THD_L2.get()
        thd_l3 = self.THD_L3.get()

        # Format data om te verzenden naar STM32
        data = f"L1:{l1},L2:{l2},L3:{l3},THD_L1:{thd_l1},THD_L2:{thd_l2},THD_L3:{thd_l3}\n"
        ser.write(data.encode())
        print(f"Verstuurd: {data}")

    def stop_test(self):
        ser.write(b"STOP\n")  # Verstuur stopcommando naar STM32

    def read_from_serial(self):
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                self.update_active_scenario(line)

    def update_active_scenario(self, scenario):
        # Update de GUI om het actieve scenario weer te geven
        self.active_scenario_label.config(text=f"Actief Testscenario: {scenario}")

# Start de GUI
root = tk.Tk()
app = TestSystemGUI(root)
root.mainloop()

# Sluit de seriële verbinding
ser.close()