#!/usr/bin/python3

import rospy
# Import python library to get the cpu/memory usage
import psutil
#import necessary msgs
from rasp_pkg.msg import raspData
from rasp_pkg.msg import processInfo
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
                # cpu_freq_dict =  self.convertDataToDict(cpu_freq)

                # Memory
                virtual_memory = psutil.virtual_memory()
                # virtual_memory_dict = self.convertDataToDict(virtual_memory)
                virtual_memory_percent = virtual_memory.percent
                # swap_memory = psutil.swap_memory()
                # swap_memory_dict = self.convertDataToDict(swap_memory)

                # Sensors data -> Testar dados na rasp antes de definir como passar na msg
                sensors_battery = psutil.sensors_battery()
                sensors_temperature_total = psutil.sensors_temperatures()

                # sensors_temperature = psutil.sensors_temperatures()['coretemp'][0]                
                # sensors_temperature = self.convertDataToDict(sensors_temperature)

                # Disk data
                # disk_usage = psutil.disk_usage('/')
                # disk_usage = self.convertDataToDict(disk_usage)

                # Test the collected data
                # print(f"virtual_memory{virtual_memory_gB} \n\n swap memory{swap_memory_gB} \n\n cpu_freq{cpu_freq} \n\n cpu_usage: {cpu_usage_percent}%")
                print(f"\n\n Virtual memory: {virtual_memory}")
                print(f"\n\n Uso da cpu: {cpu_usage_percent}, Freq: {cpu_freq}")
                print(f"\n\n temperatura todos sensores: {sensors_temperature_total}")
                print(f"\n\n bateria sensores: {sensors_battery}")
                # print(f"\n\n Uso do disco: {disk_usage}")

                
                # Network data:

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
                # Da pra pegar dados individualizados de uso de cpu, memória etc que cada processo está consumindo
                # Get the 3 process that are responsible for the most cpu usage:
                process_list = []
                for process in psutil.process_iter():
                    process_info = process.as_dict(['cpu_percent', 'name'])
                    if process_info['cpu_percent'] != 0:
                        process_list.append(process_info)
                process_list = sorted(process_list, key = lambda p: p['cpu_percent'], reverse = True)[:3]
                # print(f"\n\n Processos que mais demandam cpu: {process_list} \n\n ")

                # print(f"{cpu_usage_percent},  {cpu_freq_dict}, {virtual_memory_percent},{virtual_memory_dict}, {process_list}")



                # Defining the message to be sent
                rasp_data = raspData()

                rasp_data.cpu_usage_percent = cpu_usage_percent
                rasp_data.cpu_freq_current = cpu_freq.current
                rasp_data.cpu_freq_min = cpu_freq.min
                rasp_data.cpu_freq_max = cpu_freq.max
                rasp_data.virtualMemory_percent = virtual_memory_percent


                for process in process_list:
                    rasp_data.process_usage_list.append(process['cpu_percent'])
                    rasp_data.process_name_list.append(process['name'])


                    

                # cpu_msg = Float32()
                # cpu_msg.data = cpu_usage_percent

                # mem_msg = Float32()
                # mem_msg.data = virtual_mem_percent

                # self.cpu_pub.publish(cpu_msg)
                # self.mem_per_pub.publish(mem_msg)
                self.raspData_pub.publish(rasp_data)
                self.rate.sleep()
            
            except rospy.ServiceException as e:
                print(f"Publish rasp data from clover{self.clover_id} failed %s", e)


    def convertDataToDict(self, data):
        data_dict = data._asdict()
        data_1024 = {}
        for key, value in data_dict.items():
            data_1024[key] = value
        return data_1024             

    def convertDataToDictGB(self, data):
        data_dict = data._asdict()
        data_1024 = {}
        for key, value in data_dict.items():
            converted_value = self.convertBytesToGigaB(value)
            data_1024[key] = converted_value
        return data_1024         


    def convertBytesToGigaB(self,data):
        return data/1024/1024/1024

if __name__ == '__main__':
    rospy.init_node('rasp_resource_publisher')
    print("Publishing rasp data")
    drone1 = RaspResourcePublisher(id='1')

    try:
        drone1.publish_data()

    except rospy.ROSInterruptException:
        pass


