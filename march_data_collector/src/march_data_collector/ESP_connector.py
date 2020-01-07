#!/usr/bin/python3
import socket
import esppy
import datetime


# def receive_messages(input_ip, port, pub, window):
#
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock.bind((input_ip, port))
#     while True:
#         data, addr = sock.recvfrom(1024)
#         datachannels = data.split()
#         msg = "hello world"
#         pub.send(msg)
#         print(window.get_events())
#
# def on_event(event):
#     print(event.columns)


if __name__ == '__main__':
    url = "http://localhost:9900"
    esp = esppy.ESP(url)
    print(esp)
    window = esp.get_window("March_test/March_cq/March")
    print(window)


    pub = window.create_publisher(blocksize=1, rate=100, dateformat="%Y-%m-%d %H:%M:%s.%f")
    pub.send('1,"a", 1\n')
    print(window.get_events())
    # pub.send(str(time.time())+",a,2,234,234")
    # pub.send(str(time.time())+",a,3")
    # sub = esppy.windows.Subscriber(window, format="csv", on_event=on_event)
    # sub.start()



    # receive_messages("127.0.0.1", 32000, pub, window)
    #/publishers/March_test/March_cq/March/?blocksize=1&dateformat=%Y%m%dT%H:%M:%S.%f&format=csv&opcode=insert&pause=0&rate=0

