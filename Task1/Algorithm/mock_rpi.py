import socket
import pickle
import time

HOST = "127.0.0.1"  # Localhost (simulating RPi)
PORT = 4060  # Same as in your `constants.RPI_PORT`

def mock_rpi():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"Mock RPi Server running on {HOST}:{PORT}...")

    conn, addr = server.accept()
    print(f"Connected by {addr}")

    try:
        while True:
            # Simulate sending obstacle data to Algo
            obstacle_data = [[10, 10, 'N'], [80, 80, 'N', 1]]  # Simulated data
            data = pickle.dumps(obstacle_data)
            conn.sendall(data)
            print("Sent mock obstacle data to Algo:", obstacle_data)

            # Wait for a response
            response = conn.recv(1024)
            if not response:
                break
            print("Received from Algo:", response.decode())

            time.sleep(5)  # Simulate delay between messages
    except KeyboardInterrupt:
        print("Mock RPi Server shutting down...")
    finally:
        conn.close()
        server.close()

if __name__ == "__main__":
    mock_rpi()
