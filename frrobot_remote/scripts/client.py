import socket
import os
import time
import sys,signal

def client():
    s = socket.socket()
    s.connect(('192.168.124.10', 8484))
    
    while True:
        try:
            s.send(b'Hello from client')
            time.sleep(0.5)
            
        except KeyboardInterrupt:
            print('Server shutting down...')
            s.close()

def signal_handler(sig, frame):
    print('Exiting gracefully...')
    sys.exit(0)
    
    
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)  # 捕获 Ctrl+C 信号
    client()  # 在子进程中运行客户端

