#!/usr/bin/python3
import socket
import esppy
import datetime


def receive_messages(input_ip, port, pub, window):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((input_ip, port))
    i = 0
    while i<100:
        data, addr = sock.recvfrom(1024)
        datachannels = data.split()
        values = [float(val) for val in datachannels[1:]]
        timestr = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        csv = '1,' + timestr + ',"' + datachannels[0].decode("UTF-8")+'",' + ",".join([str(value) for value in values]) + "\n"
        pub.send(csv)
        print(window.get_events())
        i = i +1



if __name__ == '__main__':
    url = "http://localhost:9900"
    esp = esppy.ESP(url)
    print(esp)
    window = esp.get_window("March_test/March_cq/March")
    print(window)


    #
    pub = window.create_publisher(blocksize=1, dateformat="%Y-%m-%d %H:%M:%S")
    # pub.send('1,"a", 1\n')
    # print(window.get_events())
    # pub.send(str(time.time())+",a,2,234,234")
    # pub.send(str(time.time())+",a,3")
    # sub = esppy.windows.Subscriber(window, format="csv", on_event=on_event)
    # sub.start()



    receive_messages("127.0.0.1", 32000, pub, window)
    #/publishers/March_test/March_cq/March/?blocksize=1&dateformat=%Y%m%dT%H:%M:%S.%f&format=csv&opcode=insert&pause=0&rate=0

    pub.close()