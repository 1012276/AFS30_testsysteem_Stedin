import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import time

class TestSystemGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Test Systeem GUI")
        self.root.geometry("900x700")

        # Seriële verbinding met het Nucleo-bord
        try:
            self.ser = serial.Serial('COM6', 9600, timeout=1)
        except serial.SerialException as e:
            messagebox.showerror("Seriële fout", f"Kon de seriële poort niet openen: {e}")
            self.ser = None
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

        # Statusbalk
        self.status_bar = tk.Label(self.root, text="Status: wachten op initialisatie testsyteem", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        # Label voor actief scenario
        self.active_scenario_label = tk.Label(self.root, text="Actief Scenario: Wachten op start", font=("Arial", 12), bg="lightgray")
        self.active_scenario_label.pack(pady=10)


        # Start een aparte thread om data van het Nucleo-bord te lezen
        if self.ser:
            self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
            self.read_thread.start()

    def create_label(self, parent, text, row, col, font_size=10, colspan=1):
        label = tk.Label(parent, text=text, font=("Arial", font_size))
        label.grid(row=row, column=col, columnspan=colspan, padx=10, pady=10, sticky="w")
        return label


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

 

    def create_thd_page_widgets(self, page, prefix):
        # RMS Stroom en harmonischen invoervelden, inclusief validatie voor RMS-stroom
        self.create_validated_input(page, "RMS Stroom (Ampère):", 1, 0, f"{prefix}_RMS", for_current=True)
        
        # Invoervelden voor alle oneven harmonischen tot de 13e
        for i, harmonic in enumerate([3, 5, 7, 9, 11, 13], start=2):
            self.create_validated_input(page, f"{harmonic}e Harmonische (%)", i, 0, f"{prefix}_Harm_{harmonic}")

        # Opslaan-knop om samenvatting op de hoofdpagina te updaten
        ttk.Button(page, text="Opslaan", command=lambda: self.update_thd_summary(prefix)).grid(row=8, column=0, pady=20, padx=20)

    def create_validated_input(self, parent, label_text, row, col, name, for_current=False):
            """
            Creëer een invoerveld dat alleen geldige waarden accepteert.
            Voor harmonische: 5-50%.
            Voor stroom: 5-500 met een stapgrootte van één duizendste, komma toegestaan.
            """
            label = tk.Label(parent, text=label_text)
            label.grid(row=row, column=col, padx=20, pady=10, sticky="w")
            entry = tk.Entry(parent, width=20)
            entry.grid(row=row, column=col+1, padx=20, pady=10)
            # Zorg ervoor dat de standaardwaarde 0 is
            entry.insert(0, "0")

            entry.bind("<FocusOut>", lambda event, entry=entry: self.validate_input(entry, for_current))
            setattr(self, name, entry)

    def validate_input(self, entry, for_current):
        """
        Controleer of de invoer correct is. 
        Voor harmonische: controleer of de waarde een geheel getal is tussen 5 en 50.
        Voor stroom: controleer of de waarde een decimaal getal is tussen 5 en 500 met maximaal drie decimalen.
        Gebruiker kan komma gebruiken in plaats van een punt.
        """
        value = entry.get().replace(",", ".")  # Vervang komma door punt voor validatie
        if value == "":
            return True  # Leeg veld is toegestaan totdat de gebruiker het invult
        try:
            if for_current:
                # Controleer voor stroom (bereik 5 - 500, maximaal drie decimalen)
                value_float = float(value)
                decimal_places = len(value.split(".")[1]) if "." in value else 0
                if value_float != 0 and (not (5 <= value_float <= 500) or decimal_places > 3):
                    raise ValueError
            else:
                # Controleer voor harmonische (alleen gehele getallen tussen 5 - 50)
                value_int = int(value)
                if value_int != 0 and not (5 <= value_int <= 50):
                    raise ValueError
        except (ValueError, IndexError):
            # Toon specifieke foutmeldingen afhankelijk van het type invoer
            if for_current:
                messagebox.showerror("Ongeldige stroominvoer",
                                     "Voer een geldige stroomwaarde in.\nHet bereik moet tussen 5 en 500 Ampère RMS liggen, met maximaal 3 decimalen.")
            else:
                messagebox.showerror("Ongeldige harmonische invoer",
                                     "Voer een geldig harmonisch percentage in.\nHet percentage moet tussen 5 en 50% liggen en alleen hele getallen accepteren.")
            entry.focus_set()  # Plaats de cursor terug in het veld
            return False

    def run_standard_test(self):
        # Vul de standaard testprocedure in
        self.Stroom_Scenario_1.delete(0, tk.END)
        self.Stroom_Scenario_1.insert(0, "25")
        self.Stroom_Scenario_2.delete(0, tk.END)
        self.Stroom_Scenario_2.insert(0, "6,8")
        self.Stroom_Scenario_3.delete(0, tk.END)
        self.Stroom_Scenario_3.insert(0, "9,186")

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

        # Update samenvatting voor THD Scenario 1
        self.update_thd_summary("THD_Scenario_1")

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

        # Update samenvatting voor THD Scenario 2
        self.update_thd_summary("THD_Scenario_2")

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

        # Update samenvatting voor THD Scenario 3
        self.update_thd_summary("THD_Scenario_3")

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
        if self.ser:
            self.ser.write(data.encode())
            print(f"Verzonden gegevens: {data}")

    def stop_test(self):
        ser.write(b"STOP\n")

 
    def update_status(self, status):
        """Update de status in de GUI met icoontjes en kleuren"""
        if status == "aan het wachten op de instellingen":
            self.status_bar.config(text="⏳ Status: Wachten op instellingen", bg="dim gray", fg="white")
        elif status == "gereed":
            self.status_bar.config(text="✔️ Status: Gereed", bg="blue", fg="white")
        elif status == "bezig met de testprocedure":
            self.status_bar.config(text="🔄 Status: Bezig met testprocedure", bg="green", fg="white")
        elif status == "gestopt":
            self.status_bar.config(text="🟥 Status: Gestopt", bg="red", fg="white")  
        elif status == "gepauzeerd":
            self.status_bar.config(text="⏸️ Status: Gepauzeerd", bg="goldenrod", fg="white")
        elif status == "voltooid":
            self.status_bar.config(text="✅ Status: Voltooid", bg="dark green", fg="white")

        messagebox.showinfo("Status Update", f"Het testsysteem is nu {status}.")
    def update_active_scenario(self, scenario):
        """Update het actieve testscenario op de GUI en markeer het visueel"""
        # Update het actieve scenario label
        self.active_scenario_label.config(text=f"Actief Scenario: {scenario}", bg="yellow", fg="black")

        # Start knipperend effect voor visuele aandacht
        self.blink_active_scenario()

        # Toon een pop-up melding voor de gebruiker
        messagebox.showinfo("Nieuw Testscenario", f"Nieuw testscenario gestart: {scenario}")

    def blink_active_scenario(self, count=0):
        ## Laat het label knipperen om de gebruiker te attenderen op een nieuw scenario.
        if count < 6:  # Laat het label 3 keer knipperen
            current_bg = self.active_scenario_label.cget("bg")
            next_bg = "red" if current_bg == "yellow" else "yellow"
            self.active_scenario_label.config(bg=next_bg)
            # Na 500ms opnieuw de kleur veranderen
            self.root.after(500, self.blink_active_scenario, count + 1)
        else:
            # Zet de kleur na het knipperen weer terug naar standaard
            self.active_scenario_label.config(bg="lightgray")
    

    def read_from_serial(self):
        #Lees de data van het Nucleo-bord en update de GUI-status
        while True:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    # Lees de data van het Nucleo-bord
                    line = self.ser.readline().decode('utf-8').strip()

                    # Controleer welke status het Nucleo-bord stuurt
                    if line == "WACHT_OP_INSTELLINGEN":
                        self.update_status("aan het wachten op de instellingen")
                    elif line == "GEREED":
                        self.update_status("gereed")
                    elif line == "BEZIG_MET_TEST":
                        self.update_status("bezig met de testprocedure")
                    elif line == "GEPAUZEERD":
                        self.update_status("gepauzeerd")
                    elif line == "GESTOPT":
                        self.update_status("gestopt")    
                    elif line == "VOLTOOID":
                        self.update_status("voltooid")
                    elif line.startswith("ACTIEF_SCENARIO"):
                        scenario_nummer = line.split("=")[-1]
                        self.update_active_scenario(f"Scenario {scenario_nummer}")

                except Exception as e:
                    print(f"Fout bij het lezen van seriële data: {e}")

    def on_closing(self):
        """Sluit de seriële verbinding wanneer de GUI wordt gesloten"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.quit()

# Start de GUI
root = tk.Tk()
app = TestSystemGUI(root)
root.protocol("WM_DELETE_WINDOW", app.on_closing)  # Zorg ervoor dat seriële poort wordt afgesloten
root.mainloop()

