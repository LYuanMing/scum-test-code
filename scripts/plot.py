import datetime
from math import ceil
import random
from matplotlib import pyplot as plt
import serial
import re

x = []
rx_settings = []
tx_settings = []
temperature = []
y  = []
now_index = 1
temperature_ticks = [25, 30, 35, 40, 45]
temperature_labels = ["25", "30", "35", "40", "45"]

plt.ion()

class Figure:
    setting_base = (22, 0, 0)
    def __init__(self) -> None:
        self.start_time = datetime.datetime.now().timestamp()
        self.raw_rx_settings = []
        self.raw_tx_settings = []
        self.raw_rc_2m_settings = []
        self.rx_settings = []
        self.tx_settings = []
        self.rc_2m_settings = []
        self.temperature = []
        self.avg_fo = []
        self.avg_if = []
        self.avg_count = []
        self.x = []
        # , self.avg_fo_axs, self.avg_if_axs, self.avg_count_axs
        self.fig, ((self.rx_settings_axs, self.avg_if_axs), (self.tx_settings_axs, self.avg_fo_axs), (self.rc_2m_settings_axs, self.avg_count_axs)) = plt.subplots(3, 2, figsize=(13, 7))
        self.rx_temperature_axs = self.rx_settings_axs.twinx()
        self.tx_temperature_axs = self.tx_settings_axs.twinx()
        self.rc_2m_temperature_axs = self.rc_2m_settings_axs.twinx()
        # self.fig, (self.rx_settings_axs, self.tx_settings_axs, self.RC_2M_settings_axs) = plt.subplots(3, 1, figsize=(10, 8))
        plt.subplots_adjust(left=0.08, right=0.98, top=0.95, bottom=0.10, wspace=0.25, hspace=0.4)

    @staticmethod
    def convert_setting(coarse, mid, fine):
        return (coarse - Figure.setting_base[0]) * 1024 + (mid - Figure.setting_base[1]) * 32 + (fine - Figure.setting_base[2])

    @staticmethod
    def reverse_setting(value):
        return (int(value / 1024) + Figure.setting_base[0]), (int(value % 1024 / 32) + Figure.setting_base[1]), (value % 1024 % 32 + Figure.setting_base[2])

    def append_rx_setting(self, coarse, mid, fine):
        self.raw_rx_settings.append((coarse, mid, fine))
        self.rx_settings.append(self.convert_setting(coarse, mid, fine))
    
    def append_tx_setting(self, coarse, mid, fine):
        self.raw_tx_settings.append((coarse, mid, fine))
        self.tx_settings.append(self.convert_setting(coarse, mid, fine))

    def append_rc_2m_setting(self, coarse, mid, fine):
        self.raw_rc_2m_settings.append((coarse, mid, fine))
        self.rc_2m_settings.append(self.convert_setting(coarse, mid, fine))

    def append_temperature(self, temperature):
        self.temperature.append(temperature)
    
    def append_average_frequency_offset(self, avg_fo):
        self.avg_fo.append(avg_fo)

    def append_average_intermediate_frequency_count(self, avg_if):
        self.avg_if.append(avg_if)
    
    def append_average_2M_RC_count(self, avg_count):
        self.avg_count.append(avg_count)

    def increase_x(self):
        if len(self.x) != 0:
            self.x.append(self.x[-1] + 1)
        else:
            self.x.append(1)

    def record_timestamp(self):
        self.x.append(datetime.datetime.now().timestamp() - self.start_time)

    def draw_setting(self, axs, settings):
        axs.clear()
        scatter_plot = axs.scatter(self.x, settings)

        # 自动调整y轴刻度
        max_value = max(settings) if len(settings) != 0 else 32
        min_value = min(settings) if len(settings) != 0 else 0
        step = ceil((max_value - min_value) / 5)
        ticks = []
        ticklabels = []
        for i in range(0, 6):
            value = min_value + i * step
            coarse, mid, fine = Figure.reverse_setting(value)
            ticks.append(value)
            ticklabels.append(f'{coarse}.{mid}.{fine}')
        axs.set_yticks(ticks)
        axs.set_yticklabels(ticklabels)
        axs.set_ylabel("frequency settings")
        axs.set_xlabel("time (seconds)")

        return scatter_plot
    
    def draw_temperature(self, axs):
        axs.set_yticks(temperature_ticks)
        axs.set_yticklabels(temperature_labels)
        axs.set_ylabel("temperature")
        line_plot = axs.plot(self.x, self.temperature, color='red')
        return line_plot[0]

    def draw_rx_setting(self):
        scatter_plot = self.draw_setting(self.rx_settings_axs, self.rx_settings)
        
        line_plot = self.draw_temperature(self.rx_temperature_axs)

        self.rx_settings_axs.legend([scatter_plot, line_plot], ["LC OSC frequency setting (RX)", "temperature(°C)"], loc='upper left')

    def draw_tx_setting(self):
        scatter_plot = self.draw_setting(self.tx_settings_axs, self.tx_settings)
        
        line_plot = self.draw_temperature(self.tx_temperature_axs)
        
        self.tx_settings_axs.legend([scatter_plot, line_plot], ["LC OSC frequency setting (TX)", "temperature(°C)"], loc='upper left')

    def draw_rc_2m_setting(self):
        scatter_plot = self.draw_setting(self.rc_2m_settings_axs, self.rc_2m_settings)
        
        line_plot = self.draw_temperature(self.rc_2m_temperature_axs)
        
        self.rc_2m_settings_axs.legend([scatter_plot, line_plot], ["2M RC OSC frequency setting", "temperature(°C)"], loc='upper left')

    def draw_scatter(self, axs, data, ylabel):
        axs.clear()
        axs.scatter(self.x, data)
        axs.set_ylabel(ylabel)
        axs.set_xlabel("time (seconds)")

    def draw_avg_if(self):
        self.draw_scatter(self.avg_if_axs, self.avg_if, "intermediate frequency count")

    def draw_avg_fo(self):
        self.draw_scatter(self.avg_fo_axs, self.avg_fo, "frequency offset")

    def draw_avg_count(self):
        self.draw_scatter(self.avg_count_axs, self.avg_count, "2M RC frequency count")

    def update(self):
        self.draw_rx_setting()
        self.draw_tx_setting()
        self.draw_rc_2m_setting()
        self.draw_avg_if()
        self.draw_avg_fo()
        self.draw_avg_count()
        plt.ioff()

# fig = plt.figure()

# def append_data_to_figure(index, setting):
#     x.append(index)
#     y.append(convert_setting(setting[0], setting[1], setting[2]))
#     print(x, y)
#     fig.clf()
#     ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
#     ax.set_yticks([convert_setting(22, 16, 0), convert_setting(23, 0, 0), convert_setting(23, 16, 0), convert_setting(24, 0, 0), convert_setting(24, 16, 0)])
#     ax.set_yticklabels(['22.16.0', '23.0.0', '23.16.0', '24.0.0', '24.16.0'])
#     ax.scatter(x, y)

#     plt.pause(1)
#     plt.ioff()

# port_name = input("Please input {COM*}:")
port_name = "COM9"
port = serial.Serial(port_name, 19200)
f = None
first_setup = True
while True:
    data = port.readline()
    print("Received: ", data)
    try:
        data = data.decode('utf-8')
    except Exception:
        continue
    raw_data_array = data.split("|")
    if len(raw_data_array) == 4:
        if first_setup:
            f = Figure()
            first_setup = False

        tx_pattern = re.compile(r'TX setting (\d*) (\d*) (\d*) \(avg_fo=(-?\d*)\)')
        tx_coarse, tx_mid, tx_fine, avg_fo = tx_pattern.search(raw_data_array[0]).groups()

        rx_pattern = re.compile(r'RX setting (\d*) (\d*) (\d*) \(avg_if=(-?\d*)\)')
        rx_coarse, rx_mid, rx_fine, avg_if = rx_pattern.search(raw_data_array[1]).groups()

        _2M_pattern = re.compile(r'2M setting (\d*) (\d*) (\d*) \(avg_count_2M=(-?\d*)\)')
        _2M_coarse, _2M_mid, _2M_fine, avg_count = _2M_pattern.search(raw_data_array[2]).groups()

        temperature_pattern = re.compile(r'temp=(-?\d*)')
        temperature,  = temperature_pattern.search(raw_data_array[3]).groups()

        f.append_tx_setting(int(tx_coarse), int(tx_mid), int(tx_fine))
        f.append_average_frequency_offset(int(avg_fo))
        f.append_rx_setting(int(rx_coarse), int(rx_mid), int(rx_fine))
        f.append_average_intermediate_frequency_count(int(avg_if))
        f.append_rc_2m_setting(int(_2M_coarse), int(_2M_mid), int(_2M_fine))
        f.append_average_2M_RC_count(int(avg_count))
        f.append_temperature(float(temperature))
        # f.increase_x()
        f.record_timestamp()
        f.update()
        plt.pause(0.01)

    with open(f"output.txt", "a") as out:
        out.write("Received: " + str(data) + '\n')


# f = Figure()

# for i in range(300):
#     f.append_rx_setting(23, random.randint(0, 31), random.randint(0, 31))
#     f.append_tx_setting(23, random.randint(0, 31), random.randint(0, 31))
#     f.append_rc_2m_setting(23, random.randint(0, 31), random.randint(0, 31))
#     f.append_average_intermediate_frequency_count(500 + random.randint(-30, 30))
#     f.append_average_2M_RC_count(60000 + random.randint(-60, 60))
#     f.append_average_frequency_offset(0 + random.randint(-10, 10))
#     f.append_temperature(28 + random.randint(0, 20))
#     f.increase_x()
#     f.update()
#     plt.pause(0.1)
