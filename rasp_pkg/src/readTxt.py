import os


def read_config_file(file_path):
    config = {}
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            if line.strip():
                key, value = line.strip().split(':')
                # print('Key', key)
                # print('Value', value)
                config[key.strip()] = value.strip()
    return config

def main():
    directory = os.getcwd() + "/swarm_in_blocks/rasp_pkg/src"
    file_name = "config.txt"
    file_path = os.path.join(directory, file_name)
    config = read_config_file(file_path)

    master_server_ip = config.get('Master Server IP Address')
    port = config.get('Port')
    clover_id = config.get('cloverID')

    print('Master Server IP Address:', master_server_ip)
    print('Port:', port)
    print('cloverID:', clover_id)

if __name__ == '__main__':
    main()