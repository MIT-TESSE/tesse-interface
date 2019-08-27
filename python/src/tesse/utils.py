import socket
import threading
import defusedxml.ElementTree as ET


class UdpListener(threading.Thread):
    __epsilon_timing__ = 0.0025 # 1/(2*200), the game runs at ~200 Hz but sometimes the timing is a little off.
                                # We use this epsilon to approximate that 1/rate time has elapsed in the game.

    def __init__(self, host=None, port=None, timeout=0.1, rate=None):
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

        # Desired rate that messages are processed
        self.rate = rate

        self.handlers = {}

    def subscribe(self, name, handler):
        self.handlers[name] = handler

    def join(self, timeout=1):
        self.alive.clear()
        threading.Thread.join(self, timeout)
        self.sock.close()

    def run(self):
        last_game_time = -1e6  # initialize
        while self.alive.isSet():
            try:
                data = self.sock.recv(1024)
            except socket.timeout as error:
                print("UdpListener error: ", error)
                continue

            # Process the message if the game has elapsed *at least* 1/self.rate
            game_time = float(ET.fromstring(data.decode('utf-8')).find('time').text)
            if self.rate is None or game_time - last_game_time >= 1.0/self.rate - self.__epsilon_timing__:
                for name in self.handlers:
                    self.handlers[name](data)
                last_game_time = game_time
