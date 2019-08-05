import socket
import threading


class UdpListener(threading.Thread):
    def __init__(self, host=None, port=None, timeout=0.1):
        super(UdpListener, self).__init__()

        # get ip address
        if host is None:
            host = socket.gethostbyname(socket.gethostname())
        else:
            host = socket.gethostbyname(host)

        # create socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(timeout)
        self.sock.bind((host, port))

        # thread management
        self.alive = threading.Event()
        self.alive.set()

        self.handlers = {}

    def subscribe(self, name, handler):
        self.handlers[name] = handler

    def join(self, timeout=1):
        self.alive.clear()
        threading.Thread.join(self, timeout)
        self.sock.close()

    def run(self):
        while self.alive.isSet():
            try:
                data = self.sock.recv(1024)
                for name in self.handlers:
                    self.handlers[name](data)
            except socket.timeout:
                pass
