import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import serial
import threading

# Seriële verbinding met STM32 microcontroller
ser = serial.Serial('COM6', 9600, timeout=1)  # Pas aan naar de juiste COM-poort

class TestSystemGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Test Systeem GUI")
        self.root.geometry("900x700")  # Groot venster voor alle velden

        # Notebook (tabbladen) creëren
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(pady=10, expand=True)

        # Hoofdpagina (voor stroominstellingen en samenvattingen van THD)
        self.main_frame = ttk.Frame(self.notebook, width=900, height=700)
        self.notebook.add(self.main_frame, text="Hoofdpagina")

        # THD-pagina's
        self.thd_page1 = ttk.Frame(self.notebook, width=900, height=700)
        self.thd_page2 = ttk.Frame(self.notebook, width=900, height=700)
        self.thd_page3 = ttk.Frame(self.notebook, width=900, height=700)
        
        self.notebook.add(self.thd_page1, text="THD Scenario 1")
        self.notebook.add(self.thd_page2, text="THD Scenario 2")
        self.notebook.add(self.thd_page3, text="THD Scenario 3")

        # Maak de widgets voor stroominstellingen en THD samenvattingen
        self.create_main_frame_widgets()

        # Maak de widgets voor de drie THD-pagina's
        self.create_thd_page_widgets(self.thd_page1, "THD_Scenario_1")
        self.create_thd_page_widgets(self.thd_page2, "THD_Scenario_2")
        self.create_thd_page_widgets(self.thd_page3, "THD_Scenario_3")

        # Start een thread om data van STM32 te lezen
        self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
        self.read_thread.start()

    def create_main_frame_widgets(self):
        # Stroom Test Scenario's - één veld voor L1, L2, L3 per scenario
        self.create_label(self.main_frame, "Stroom Test Scenario's", 0, 0, font_size=16, colspan=2)

        # Stroominstellingen voor drie scenario's met validatie
        self.create_validated_input(self.main_frame, "Scenario 1 Stroom (Ampère):", 1, 0, "Stroom_Scenario_1", for_current=True)
        self.create_validated_input(self.main_frame, "Scenario 2 Stroom (Ampère):", 2, 0, "Stroom_Scenario_2", for_current=True)
        self.create_validated_input(self.main_frame, "Scenario 3 Stroom (Ampère):", 3, 0, "Stroom_Scenario_3", for_current=True)

        # THD samenvattingen (deze worden geüpdatet met de ingevulde waarden uit de THD tabbladen)
        self.thd_summary_1 = self.create_label(self.main_frame, "THD Scenario 1: Geen gegevens ingevoerd", 4, 0, font_size=10, colspan=2)
        self.thd_summary_2 = self.create_label(self.main_frame, "THD Scenario 2: Geen gegevens ingevoerd", 5, 0, font_size=10, colspan=2)
        self.thd_summary_3 = self.create_label(self.main_frame, "THD Scenario 3: Geen gegevens ingevoerd", 6, 0, font_size=10, colspan=2)

        # Verzend instellingen-knop
        ttk.Button(self.main_frame, text="Verzend instellingen", command=self.send_settings).grid(row=8, column=0, pady=20, padx=20)

        # Knoppen voor testbediening
        ttk.Button(self.main_frame, text="Start Test", command=self.send_settings).grid(row=8, column=1, pady=20, padx=20)
        ttk.Button(self.main_frame, text="Stop Test", command=self.stop_test).grid(row=8, column=2, pady=20, padx=20)

        # Knop voor standaard testprocedure
        ttk.Button(self.main_frame, text="Standaard Testprocedure", command=self.run_standard_test).grid(row=9, column=0, pady=20, padx=20)

        # Real-time weergave van data van het Nucleo-board
        self.create_label(self.main_frame, "Real-time Data van Nucleo", 10, 0, font_size=16, colspan=3)
        self.data_display = tk.Text(self.main_frame, height=10, width=70)
        self.data_display.grid(row=11, column=0, columnspan=3, padx=20, pady=20)

    def create_thd_page_widgets(self, page, prefix):
        # RMS Stroom en harmonischen invoervelden, inclusief validatie voor RMS-stroom
        self.create_validated_input(page, "RMS Stroom (Ampère):", 1, 0, f"{prefix}_RMS", for_current=True)
        
        # Invoervelden voor alle oneven harmonischen tot de 13e
        for i, harmonic in enumerate([3, 5, 7, 9, 11, 13], start=2):
            self.create_validated_input(page, f"{harmonic}e Harmonische (%)", i, 0, f"{prefix}_Harm_{harmonic}")

        # Opslaan-knop om samenvatting op de hoofdpagina te updaten
        ttk.Button(page, text="Opslaan", command=lambda: self.update_thd_summary(prefix)).grid(row=8, column=0, pady=20, padx=20)

    def create_validated_input(self, parent, label_text, row, col, name, for_current=False):
        label = tk.Label(parent, text=label_text)
        label.grid(row=row, column=col, padx=20, pady=10, sticky="w")
        entry = tk.Entry(parent, width=20)
        entry.grid(row=row, column=col+1, padx=20, pady=10)
        entry.insert(0, "0")  # Vul het veld automatisch met 0
        entry.bind("<FocusOut>", lambda event, entry=entry: self.validate_input(entry, for_current))
        setattr(self, name, entry)

    def validate_input(self, entry, for_current):
        value = entry.get().replace(",", ".")  # Vervang komma door punt voor validatie
        if value == "":  # Laat lege velden als 0 behandelen
            entry.insert(0, "0")
            return True

        try:
            if for_current:
                value_float = float(value)
                decimal_places = len(value.split(".")[1]) if "." in value else 0
                if not (5 <= value_float <= 500) or decimal_places > 3:
                    raise ValueError
            else:
                value_int = int(value)
                if not (5 <= value_int <= 50):
                    raise ValueError
        except (ValueError, IndexError):
            if for_current:
                messagebox.showerror("Ongeldige stroominvoer",
                                     "Voer een geldige stroomwaarde in.\nHet bereik moet tussen 5 en 500 Ampère RMS liggen, met maximaal 3 decimalen.")
            else:
                messagebox.showerror("Ongeldige harmonische invoer",
                                     "Voer een geldig harmonisch percentage in.\nHet percentage moet tussen 5 en 50% liggen en alleen hele getallen accepteren.")
            entry.focus_set()
            return False

    def run_standard_test(self):
        # Vul de standaard testprocedure in
        self.Stroom_Scenario_1.delete(0, tk.END)
        self.Stroom_Scenario_1.insert(0, "25")
        self.Stroom_Scenario_2.delete(0, tk.END)
        self.Stroom_Scenario_2.insert(0, "6.8")
        self.Stroom_Scenario_3.delete(0, tk.END)
        self.Stroom_Scenario_3.insert(0, "9186")

        # THD Scenario 1
        self.THD_Scenario_1_RMS.delete(0, tk.END)
        self.THD_Scenario_1_RMS.insert(0, "20")
        self.THD_Scenario_1_Harm_3.delete(0, tk.END)
        self.THD_Scenario_1_Harm_3.insert(0, "15")
        self.THD_Scenario_1_Harm_5.delete(0, tk.END)
        self.THD_Scenario_1_Harm_5.insert(0, "20")
        self.THD_Scenario_1_Harm_7.delete(0, tk.END)
        self.THD_Scenario_1_Harm_7.insert(0, "10")
        self.THD_Scenario_1_Harm_9.delete(0, tk.END)
        self.THD_Scenario_1_Harm_9.insert(0, "8")
        self.THD_Scenario_1_Harm_11.delete(0, tk.END)
        self.THD_Scenario_1_Harm_11.insert(0, "6")
        self.THD_Scenario_1_Harm_13.delete(0, tk.END)
        self.THD_Scenario_1_Harm_13.insert(0, "5")

        # THD Scenario 2
        self.THD_Scenario_2_RMS.delete(0, tk.END)
        self.THD_Scenario_2_RMS.insert(0, "20")
        self.THD_Scenario_2_Harm_3.delete(0, tk.END)
        self.THD_Scenario_2_Harm_3.insert(0, "15")
        self.THD_Scenario_2_Harm_5.delete(0, tk.END)
        self.THD_Scenario_2_Harm_5.insert(0, "5")
        self.THD_Scenario_2_Harm_7.delete(0, tk.END)
        self.THD_Scenario_2_Harm_7.insert(0, "0")  # Not defined, set to 0
        self.THD_Scenario_2_Harm_9.delete(0, tk.END)
        self.THD_Scenario_2_Harm_9.insert(0, "0")  # Not defined, set to 0
        self.THD_Scenario_2_Harm_11.delete(0, tk.END)
        self.THD_Scenario_2_Harm_11.insert(0, "0")  # Not defined, set to 0
        self.THD_Scenario_2_Harm_13.delete(0, tk.END)
        self.THD_Scenario_2_Harm_13.insert(0, "0")  # Not defined, set to 0

        # THD Scenario 3
        self.THD_Scenario_3_RMS.delete(0, tk.END)
        self.THD_Scenario_3_RMS.insert(0, "20")
        self.THD_Scenario_3_Harm_3.delete(0, tk.END)
        self.THD_Scenario_3_Harm_3.insert(0, "30")
        self.THD_Scenario_3_Harm_5.delete(0, tk.END)
        self.THD_Scenario_3_Harm_5.insert(0, "5")
        self.THD_Scenario_3_Harm_7.delete(0, tk.END)
        self.THD_Scenario_3_Harm_7.insert(0, "10")
        self.THD_Scenario_3_Harm_9.delete(0, tk.END)
        self.THD_Scenario_3_Harm_9.insert(0, "0")  # Not defined, set to 0
        self.THD_Scenario_3_Harm_11.delete(0, tk.END)
        self.THD_Scenario_3_Harm_11.insert(0, "0")  # Not defined, set to 0
        self.THD_Scenario_3_Harm_13.delete(0, tk.END)
        self.THD_Scenario_3_Harm_13.insert(0, "0")  # Not defined, set to 0

        # Verstuur automatisch de instellingen
        self.send_settings()

    def update_thd_summary(self, prefix):
        try:
            rms = float(getattr(self, f"{prefix}_RMS").get().replace(",", "."))
        except ValueError:
            messagebox.showerror("Ongeldige invoer", "Voer een geldig getal in voor de RMS Stroom.")
            return

        harmonic_values = []
        for harmonic in [3, 5, 7, 9, 11, 13]:
            try:
                harmonic_value = int(getattr(self, f"{prefix}_Harm_{harmonic}").get())
                harmonic_values.append(f"{harmonic}e: {harmonic_value}%")
            except ValueError:
                messagebox.showerror("Ongeldige invoer", f"Voer een geldig percentage in voor de {harmonic}e harmonische.")
                return

        summary_text = f"Stroom L1, L2, L3: {rms} A; THD% = ({', '.join(harmonic_values)})"
        if "Scenario_1" in prefix:
            self.thd_summary_1.config(text=summary_text)
        elif "Scenario_2" in prefix:
            self.thd_summary_2.config(text=summary_text)
        elif "Scenario_3" in prefix:
            self.thd_summary_3.config(text=summary_text)

    def send_settings(self):
        try:
            stroom_scenario_1 = float(self.Stroom_Scenario_1.get().replace(",", "."))
            stroom_scenario_2 = float(self.Stroom_Scenario_2.get().replace(",", "."))
            stroom_scenario_3 = float(self.Stroom_Scenario_3.get().replace(",", "."))
        except ValueError:
            messagebox.showerror("Ongeldige invoer", "Voer een geldige stroomwaarde in voor alle scenario's.")
            return

        try:
            thd_1_rms = float(self.THD_Scenario_1_RMS.get().replace(",", "."))
            thd_1_harmonics = [int(self.THD_Scenario_1_Harm_3.get()), int(self.THD_Scenario_1_Harm_5.get()),
                               int(self.THD_Scenario_1_Harm_7.get()), int(self.THD_Scenario_1_Harm_9.get()),
                               int(self.THD_Scenario_1_Harm_11.get()), int(self.THD_Scenario_1_Harm_13.get())]

            thd_2_rms = float(self.THD_Scenario_2_RMS.get().replace(",", "."))
            thd_2_harmonics = [int(self.THD_Scenario_2_Harm_3.get()), int(self.THD_Scenario_2_Harm_5.get()),
                               int(self.THD_Scenario_2_Harm_7.get()), int(self.THD_Scenario_2_Harm_9.get()),
                               int(self.THD_Scenario_2_Harm_11.get()), int(self.THD_Scenario_2_Harm_13.get())]

            thd_3_rms = float(self.THD_Scenario_3_RMS.get().replace(",", "."))
            thd_3_harmonics = [int(self.THD_Scenario_3_Harm_3.get()), int(self.THD_Scenario_3_Harm_5.get()),
                               int(self.THD_Scenario_3_Harm_7.get()), int(self.THD_Scenario_3_Harm_9.get()),
                               int(self.THD_Scenario_3_Harm_11.get()), int(self.THD_Scenario_3_Harm_13.get())]
        except ValueError:
            messagebox.showerror("Ongeldige invoer", "Voer geldige THD-waarden in voor alle scenario's.")
            return

        data = f"STROOM_S1={stroom_scenario_1:.3f};STROOM_S2={stroom_scenario_2:.3f};STROOM_S3={stroom_scenario_3:.3f};"
        data += f"THD_S1_RMS={thd_1_rms:.3f};THD_S1_3H={thd_1_harmonics[0]};THD_S1_5H={thd_1_harmonics[1]};"
        data += f"THD_S1_7H={thd_1_harmonics[2]};THD_S1_9H={thd_1_harmonics[3]};THD_S1_11H={thd_1_harmonics[4]};THD_S1_13H={thd_1_harmonics[5]};"
        data += f"THD_S2_RMS={thd_2_rms:.3f};THD_S2_3H={thd_2_harmonics[0]};THD_S2_5H={thd_2_harmonics[1]};"
        data += f"THD_S2_7H={thd_2_harmonics[2]};THD_S2_9H={thd_2_harmonics[3]};THD_S2_11H={thd_2_harmonics[4]};THD_S2_13H={thd_2_harmonics[5]};"
        data += f"THD_S3_RMS={thd_3_rms:.3f};THD_S3_3H={thd_3_harmonics[0]};THD_S3_5H={thd_3_harmonics[1]};"
        data += f"THD_S3_7H={thd_3_harmonics[2]};THD_S3_9H={thd_3_harmonics[3]};THD_S3_11H={thd_3_harmonics[4]};THD_S3_13H={thd_3_harmonics[5]};\n"

        ser.write(data.encode())
        print(f"Verzonden gegevens: {data}")

    def stop_test(self):
        ser.write(b"STOP\n")

    def read_from_serial(self):
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                self.update_data_display(line)

    def update_data_display(self, data):
        self.data_display.insert(tk.END, data + "\n")
        self.data_display.see(tk.END)

# Start de GUI
root = tk.Tk()
app = TestSystemGUI(root)
root.mainloop()

# Sluit de seriële verbinding
ser.close()
