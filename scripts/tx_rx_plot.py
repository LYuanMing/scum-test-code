
import matplotlib.pyplot as plt

packet_len = [5, 10, 15, 20]

transmitter_pdr = [46.97, 24.42, 24.16, 24.60]
receiver_pdr = [94.48, 95.68, 96.03, 95.1]

transmitter_pdr_error_bar = [5.129544056, 4.202404314, 5.921575002, 4.532709404]
receiver_pdr_error_bar = [1.997372011, 1.476277054, 1.374037772, 1.534123643]

# y = transmitter_pdr
# y_error = transmitter_pdr_error_bar

y = receiver_pdr
y_error = receiver_pdr_error_bar


plt.bar(packet_len, y, yerr=y_error, capsize=5)
plt.xlabel('Packet Length')
plt.ylabel('PDR')
# plt.title('Transmitter\'s PDR')
plt.title("Receiver's PDR")
plt.show()