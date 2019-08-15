import socket
import threading
import time

class UdpListener(threading.Thread):
    def __init__(self, host='<broadcast>', port=None, timeout=0.1, rate=None):
        """ Initializer for UdpListener object.

            Derived from a threading.Thread object, UdpListener will spin off
            a new thread and listen for data coming in on a specified port.
            It supports handing that data with an arbitrary number and type
            of custom callback functions.

            Args:
                host: A string representing the hostname which will be used
                    to listen for data. Default is '<broadcast>', which will
                    iterate over all network interfaces for send.
                port: An integer representing the port number to listen to.
                    This must be set by the user and has no valid default.
                timeout: A float representing the timeout period for the socket.
                    Default is 0.1 seconds.
                rate: A float representing the desired rate at which to call
                    all custom callbacks on the incoming data. Default is None,
                    which will send data to the callbacks as fast as possible.
        """
        super(UdpListener, self).__init__()
        if rate>=200 or rate is None:
            self.rate = None
        else:
            self.rate = rate

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
        """ Adds custom callback function to dictionary of subscribers.

            Args:
                name: A string representing the unique name of the callback.
                handler: A function or class (with __call__ implemented)
                    to use as a callback when data is received.
        """
        self.handlers[name] = handler

    def join(self, timeout=1):
        """ Safely close thread and unbind socket. """
        self.alive.clear()
        threading.Thread.join(self, timeout)
        self.sock.close()

    def run(self):
        """ Main loop for receiving data and calling callbacks. """
        start = time.time()
        while self.alive.isSet():
            try:
                data = self.sock.recv(1024)
            except socket.timeout as error:
                print "UdpListener error: ", error
                continue

            for name in self.handlers:
                self.handlers[name](data)

            if self.rate is not None:
                while (1.0/self.rate) - (time.time()-start) > 0:
                    try:
                        _ = self.sock.recv(1024)
                    except socket.timeout:
                        print "UdpListener error: inner timeout"
                        continue
                start = time.time()
