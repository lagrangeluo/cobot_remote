import socket
import signal
import sys
import json

def signal_handler(sig, frame):
    print('Exiting gracefully...')
    sys.exit(0)

def start_server():
    # 创建一个TCP/IP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定到地址和端口
    server_address = ('localhost', 8484)
    server_socket.bind(server_address)

    # 开始监听连接
    server_socket.listen(1)
    print(f'Server listening on {server_address}')

    while True:
        try:
            # 等待连接
            connection, client_address = server_socket.accept()
            print(f'Connection from {client_address}')

            # 接收数据
            while True:
                data = connection.recv(1024)
                if data:
                    message = data.decode()
                    print(f'Received: {message}')
                    # 如果接收到 "exit"，则关闭连接
                    if message.strip().lower() == "exit":
                        break
                else:
                    break
            
            # 关闭连接
            print(f'Closing connection from {client_address}')
            connection.close()
        except KeyboardInterrupt:
            print('Server shutting down...')
            break
        except Exception as e:
            print(f'Error: {e}')
            break

    server_socket.close()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)  # 捕获 Ctrl+C 信号
    start_server()
