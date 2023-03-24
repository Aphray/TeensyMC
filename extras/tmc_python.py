import serial
import _tkinter
import tkinter as tk
import tkinter.ttk as ttk
from serial.tools.list_ports import comports


BAUDRATE = 115200


class TeensyMCController(serial.Serial):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.baudrate = BAUDRATE
        self.connected = False

    def connect(self, port: str):
        self.port = port
        try:
            self.open()
            self.connected = True
        except serial.SerialException:
            return False
        return True

    def disconnect(self):
        if self.connected:
            self.close()
            self.connected = False


class TeensyMCGUI(tk.Tk):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.control = TeensyMCController()

        self.title('TeensyMC GUI')

        self.port_select = ttk.Combobox(self)
        self.refresh_btn = ttk.Button(self, text='Refresh Ports', command=self.populate_ports)
        self.connect_btn = ttk.Button(self, text='Connect', command=self.connect)
        self.port_select.grid(row=0, column=0)
        self.refresh_btn.grid(row=0, column=1) 
        self.connect_btn.grid(row=0, column=2)
        

        self.frame = ttk.Frame(self)
        self.frame.grid(row=1, column=0, columnspan=3)

        self.stepper_count = tk.StringVar()
        ttk.Label(self.frame, text='# of Steppers:').grid(row=0, column=0)
        ttk.Spinbox(self.frame, textvariable=self.stepper_count).grid(row=0, column=1)

        grid = ttk.Frame(self.frame)
        grid.grid(row=1, column=0, columnspan=2)

        ttk.Button(grid, text='Test').pack()

        self.enable_widgets(False)
        

    def populate_ports(self):
        ports = []
        for port in comports():
            ports.append(port.name)
        self.port_select['values'] = ports

    def connect(self):
        port = self.port_select.get()
        if (self.control.connect(port)):
            self.enable_widgets(True)
            self.connect_btn.config(text='Disconnect', command=self.disconnect)

    def disconnect(self):
        if self.control.connected:
            self.control.disconnect()
            self.enable_widgets(False)
            self.connect_btn.config(text='Connect', command=self.connect)

    def enable_widgets(self, enable: bool):
        for child in self.frame.winfo_children():
            try:
                child.configure(state='disabled' if not enable else 'enabled')
            except _tkinter.TclError:
                continue




gui = TeensyMCGUI()
gui.mainloop()