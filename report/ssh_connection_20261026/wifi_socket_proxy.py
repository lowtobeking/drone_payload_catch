"""SSH transport explicitly through Wi-Fi; leaves all adapter settings untouched."""
import socket
import sys
import threading
sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.setsockopt(socket.IPPROTO_IP,31,socket.htonl(23))
sock.settimeout(8)
sock.bind(("192.168.1.4",0))
sock.connect(("192.168.1.5",22))
sock.settimeout(None)
def upload():
    try:
        while True:
            data=sys.stdin.buffer.read1(65536)
            if not data:
                sock.shutdown(socket.SHUT_WR)
                return
            sock.sendall(data)
    except OSError:
        return
thread=threading.Thread(target=upload,daemon=True)
thread.start()
try:
    while True:
        data=sock.recv(65536)
        if not data:
            break
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()
finally:
    sock.close()
