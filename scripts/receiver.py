import datetime
from math import ceil
import random
from matplotlib import pyplot as plt
import serial
import re

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
    # with open(f"output.txt", "a") as out:
    #     out.write("Received: " + str(data) + '\n')


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
