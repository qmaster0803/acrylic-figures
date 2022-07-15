import tkinter
from tkinter import ttk
import serial
import time

mousewheel_speed = 30
device_addr = "/dev/ttyUSB0"
tx_delay = 0.01


sliders_changed = False
device_state = [0] * 16
exit = False

def on_closing():
    port.close()
    exit = True

def ch_slider_scroll(event):
    global sliders_changed
    if(event.num == 4 or event.delta == 120):
        event.widget.set(event.widget.get()-mousewheel_speed)
    if(event.num == 5 or event.delta == -120):
        event.widget.set(event.widget.get()+mousewheel_speed)
    sliders_changed = True

def ch_slider_move(event):
    global sliders_changed
    sliders_changed = True

def set_mode0():
    global port
    port.write(b'\x03')
    time.sleep(tx_delay)
    port.write(b'\x00')
    time.sleep(tx_delay)
    reload_device_state()

def set_mode1():
    global port
    port.write(b'\x03')
    time.sleep(tx_delay)
    port.write(b'\x01')
    time.sleep(tx_delay)
    reload_device_state()

def set_mode2():
    global port
    port.write(b'\x03')
    time.sleep(tx_delay)
    port.write(b'\x02')
    time.sleep(tx_delay)
    reload_device_state()

def save_mode1():
    global port
    current_saved_values = get_eeprom_config(0)
    for i in range(16):
        if(device_state[i] != current_saved_values[i]):
            print("Channel", i, "changed to", device_state[i])
            set_eeprom_channel_value(i, device_state[i], 0)

def save_mode2():
    pass

def get_eeprom_config(mode):
    global port
    result = []
    port.write(b'\x04')
    time.sleep(tx_delay)
    port.write(mode.to_bytes(1, "big"))
    while(port.in_waiting != 32): pass
    values = port.read_all()
    values = list(values)
    for i in range(16):
        msb = values[i*2]
        lsb = values[i*2+1]
        ch_val = (msb<<8) | lsb
        result.append(ch_val)
    return result

def set_eeprom_channel_value(channel, value, mode):
    global port
    msb = value.to_bytes(2, "big")[0].to_bytes(1, "big")
    lsb = value.to_bytes(2, "big")[1].to_bytes(1, "big")
    port.write(b'\x02')
    time.sleep(tx_delay)
    port.write(channel.to_bytes(1, "big"))
    time.sleep(tx_delay)
    port.write(msb)
    time.sleep(tx_delay)
    port.write(lsb)
    time.sleep(tx_delay)
    port.write(mode.to_bytes(1, "big"))
    time.sleep(tx_delay)

def set_channel_value(channel, value):
    global port
    msb = value.to_bytes(2, "big")[0].to_bytes(1, "big")
    lsb = value.to_bytes(2, "big")[1].to_bytes(1, "big")
    port.write(b'\x01')
    time.sleep(tx_delay)
    port.write(channel.to_bytes(1, "big"))
    time.sleep(tx_delay)
    port.write(msb)
    time.sleep(tx_delay)
    port.write(lsb)
    time.sleep(tx_delay)

def reload_device_state():
    global port, ch_sliders
    port.write(b'\x05')
    while(port.in_waiting != 32): pass
    values = port.read_all()
    values = list(values)
    for i, ch in enumerate(ch_sliders):
        msb = values[i*2]
        lsb = values[i*2+1]
        ch_val = (msb<<8) | lsb
        device_state[i] = ch_val
        ch.set(ch_val)



port = serial.Serial(device_addr, 115200)
root = tkinter.Tk()
root.title("LED Controller")

top_frame = tkinter.Frame(root)
#load_EEPROM_label = tkinter.Label(top_frame, text="Load from controller:")
#load_EEPROM_b1    = tkinter.Button(top_frame, text="Mode 1", width=10)
#load_EEPROM_b2    = tkinter.Button(top_frame, text="Mode 2", width=10)
#load_EEPROM_label.grid(column=0, row=0)
#load_EEPROM_b1.grid(column=1, row=0)
#load_EEPROM_b2.grid(column=2, row=0)

select_work_mode_label = tkinter.Label(top_frame,  text="Select working mode:")
select_work_mode_b1    = tkinter.Button(top_frame, text="Off",    width=10, command=set_mode0)
select_work_mode_b2    = tkinter.Button(top_frame, text="Mode 1", width=10, command=set_mode1)
select_work_mode_b3    = tkinter.Button(top_frame, text="Mode 2", width=10, command=set_mode2)
select_work_mode_label.grid(column=0, row=1)
select_work_mode_b1.grid(column=1, row=1)
select_work_mode_b2.grid(column=2, row=1)
select_work_mode_b3.grid(column=3, row=1)

save_work_mode_label = tkinter.Label(top_frame,  text="Save config to:")
save_work_mode_b1    = tkinter.Button(top_frame, text="Mode 1", width=10, command=save_mode1)
save_work_mode_b2    = tkinter.Button(top_frame, text="Mode 2", width=10, command=save_mode2)
save_work_mode_label.grid(column=0, row=2)
save_work_mode_b1.grid(column=2, row=2)
save_work_mode_b2.grid(column=3, row=2)

top_frame.grid(column=0, row=0, columnspan=16)

ch_sliders = []
for i in range(16):
    slider = ttk.Scale(root, from_=0, to=4095, orient="vertical", length=400, command=ch_slider_move)
    slider.grid(column=i, row=1)
    slider.bind("<MouseWheel>", ch_slider_scroll)
    slider.bind("<Button-4>", ch_slider_scroll)
    slider.bind("<Button-5>", ch_slider_scroll)
    ch_label = tkinter.Label(root, text=str(i))
    ch_label.grid(column=i, row=2)
    ch_sliders.append(slider)

#initialize sliders with current device state
reload_device_state()

while(not exit):
    root.update()
    if(sliders_changed):
        for i, ch in enumerate(ch_sliders):
            curr = int(ch.get())
            if(device_state[i] != curr):
                set_channel_value(i, curr)
                device_state[i] = curr
        sliders_changed = False