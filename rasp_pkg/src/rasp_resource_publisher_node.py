#!/usr/bin/python3

import rospy
# Import python library to get the cpu/memory usage
import psutil
#import necessary msgs
from rasp_pkg.msg import raspData
from std_msgs.msg import Float32

class RaspResourcePublisher:
    def __init__(self, id):
        self.clover_id = id
        self.raspData_pub = rospy.Publisher(f"/clover_{self.clover_id}/cpu_usage", raspData, queue_size=10)
        self.rate = rospy.Rate(1) # 1 Hz

    def publish_data(self):
        while not rospy.is_shutdown():
            try:
                # Receiving data from rasp
                # CPU
                cpu_usage_percent = psutil.cpu_percent(interval=0.5)
                cpu_freq = psutil.cpu_freq()

                # Memory
                virtual_memory = psutil.virtual_memory()
                virtual_memory_percent = virtual_memory.percent

                # Sensors data  -> Receive data from the raspberry pi available sensors
                sensors_temperature_total = psutil.sensors_temperatures()

                sensors_temperature_current = psutil.sensors_temperatures()['cpu_thermal'][0]           

                # Network data:

                net_data = psutil.net_if_addrs()['wlan0'][0]
                net_io_counters = psutil.net_io_counters()
                net_status = psutil.net_if_stats()['wlan0'][0]

                # net_v4 = psutil.net_if_adrrs()['wlp2s0'][0]
                # adrrs_v4 = net_v4.address # endereço da conexão ipv4
                # netmask_v4 = net_v4.netmask # mascara de rede

                # net_io_counters = psutil.net_io_counters()
                # bytes_sent = net_io_counters.bytes_sent
                # bytes_received = net_io_counters.bytes_recv
                # pkgs_sent = net_io_counters.packets_sent
                # pkgs_received = net_io_counters.packets_recv
                # erros_sending = net_io_counters.errin
                # erros_receiving = net_io_counters.errout

                # net_status = psutil.net_if_stats()[]
                # net_isUp = net_status.isup
                # net_speed = net_status.speed # MB
                # net_type = net_status.duplex  # NIC_DUPLEX_FULL, NIC_DUPLEX_HALF(só ida ou só volta) or NIC_DUPLEX_UNKNOWN.
                # net_flag = net_status.flags # up, running, broadcasting ...

                # Process information
                # Get the 3 process that are responsible for the most cpu usage:
                process_list = []
                for process in psutil.process_iter():
                    process_info = process.as_dict(['cpu_percent', 'name'])
                    if process_info['cpu_percent'] != 0:
                        process_list.append(process_info)
                process_list = sorted(process_list, key = lambda p: p['cpu_percent'], reverse = True)[:3]

                # Defining the message to be sent
                rasp_data = raspData()

                rasp_data.cpu_usage_percent = cpu_usage_percent
                rasp_data.cpu_freq_current = cpu_freq.current
                rasp_data.cpu_freq_min = cpu_freq.min
                rasp_data.cpu_freq_max = cpu_freq.max
                rasp_data.virtualMemory_percent = virtual_memory_percent
                rasp_data.cpu_temperature = sensors_temperature_current.current
                rasp_data.net_data_adress = net_data.address
                rasp_data.bytes_sent = self.convertBytesToGigaB(net_io_counters.bytes_sent) 
                rasp_data.bytes_recv = self.convertBytesToGigaB(net_io_counters.bytes_recv)
                rasp_data.packets_sent = net_io_counters.packets_sent
                rasp_data.packets_recv = net_io_counters.packets_recv
                #rasp_data.net_data_max = net_status.mtu               


                for process in process_list:
                    rasp_data.process_usage_list.append(process['cpu_percent'])
                    rasp_data.process_name_list.append(process['name'])

                # Publishes the msg
                self.raspData_pub.publish(rasp_data)

                self.rate.sleep()

            except rospy.ServiceException as e:
                print(f"Publish rasp data from clover{self.clover_id} failed %s", e)


    # def convertDataToDict(self, data):
    #     data_dict = data._asdict()
    #     data_1024 = {}
    #     for key, value in data_dict.items():
    #         data_1024[key] = value
    #     return data_1024             

    # def convertDataToDictGB(self, data):
    #     data_dict = data._asdict()
    #     data_1024 = {}
    #     for key, value in data_dict.items():
    #         converted_value = self.convertBytesToGigaB(value)
    #         data_1024[key] = converted_value
    #     return data_1024         


    def convertBytesToGigaB(self,data):
        return data/1024/1024/1024

if __name__ == '__main__':
    rospy.init_node('rasp_resource_publisher')
    print("Publishing rasp data")
    ID = rospy.get_param('clover_id')
    drone1 = RaspResourcePublisher(id=ID)

    try:
        drone1.publish_data()

    except rospy.ROSInterruptException:
        pass




