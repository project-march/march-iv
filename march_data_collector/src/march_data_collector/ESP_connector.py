#!/usr/bin/python3
import socket
import esppy

def receive_messages(input_ip, port):

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((input_ip, port))
    while True:
        data, addr = sock.recvfrom(1024)
        datachannels = data.split()
        print("data = \n")
        print(data)
        print("\n datachannels \n")
        print(datachannels)

        pub.send(???)

if __name__ == '__main__':
    url = "http://localhost:9901"
    esp = esppy.ESP(url)
    window = esp.get_window("March_test/March_cq/March")

    pub = esppy.Publisher(window)


    receive_messages("127.0.0.1", 32000)

