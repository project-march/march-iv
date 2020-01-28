#!/usr/bin/python3
import socket
import esppy
import datetime
import atexit


def receive_messages(input_ip, port, pub_joint_angles, window_joint_angles, esp):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((input_ip, port))

    # compute_window =esp.get_window("March_test/March_cq/computeWindow")
    # copy_window = esp.get_window("March_test/March_cq/copyWindow")
    i = 0
    while True:
        i = i+1
        data, addr = sock.recvfrom(1024)
        datachannels = data.split()
        timestr = datetime.datetime.fromtimestamp(float(datachannels[1])).strftime("%Y-%m-%d %H:%M:%S.%f")
        type = datachannels[0].decode("UTF-8")
        if type=="joint_angles":
            values = [float(val) for val in datachannels[2:]]
            csv = '1,' + timestr + ',[' + ";".join([str(value) for value in values]) + "]\n"
            pub_joint_angles.send(csv)
            print("added joint_angle")
        # if type=="temperature":
        #     value = float(datachannels[2])
        #     csv = '1,' + timestr + ',' + str(value)+'\n'
        #     pub_temperature.send(csv)


def exit_function(pub1):
    print('Goodbye')
    pub1.close()



if __name__ == '__main__':
    url = "http://localhost:9900"
    esp = esppy.ESP(url)
    print(esp)
    window_joint_angles  = esp.get_window("test_project/contquery/sourceWindowJoint")
    # window_temperature  = esp.get_window("March_test/March_cq/sourceWindowTemperature")

    pub_joint_angles = window_joint_angles.create_publisher(blocksize=1, dateformat="%Y-%m-%d %H:%M:%S")
    # pub_temperature = window_temperature.create_publisher(blocksize=1, dateformat="%Y-%m-%d %H:%M:%S")
    atexit.register(exit_function, pub_joint_angles)
    receive_messages("127.0.0.1", 32000, pub_joint_angles, window_joint_angles, esp)
