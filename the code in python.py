"""with the arduino program loaded, let's use this one to perform bidirectional communication,
make sure to close the arduino ide program or any program that is using the serial comic method with the arduino"""

import serial
import serial.tools.list_ports
import threading
import time

# Class for secure sharing of the 'state' variable
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()  # Mutex for synchronization between threads
        self.state = None

# Function to send data to the serial port
def send_serial_data(ser, data):
    ser.write(data.encode())  # Encode and send data to the serial port

# Function to read data from the serial port
def read_serial_data(ser):
    return ser.readline().decode().strip()  # Read and decode data from the serial port

# Function executed in a thread to receive data from the Arduino
def receivedata_from_arduino(shared_state, ser):
    try:
        while True:
            data = read_serial_data(ser)
            if data:
                print(f"Data received from Arduino: {data}")
    except serial.SerialException as e:
        print(f"Error with the serial port: {e}")
    except KeyboardInterrupt:
        pass

# Function to find the device port with the desired baud rate
def find_device_port(target_baudrate, timeout=1):
    while True:
        available_ports = list(serial.tools.list_ports.comports())

        for port_info in available_ports:
            try:
                port = serial.Serial(port_info.device, baudrate=target_baudrate, timeout=timeout)
                port.close()
                return port_info.device  # Return the found device port
            except (OSError, serial.SerialException):
                pass

        print("Device not found. Waiting 1 second and trying again...")
        time.sleep(1)

# Main function executed in a thread
def main_loop(device_port, target_baudrate):
    try:
        ser = serial.Serial(device_port, target_baudrate)
        print(f"Connection established with {device_port}.")

        shared_state = SharedState()

        # Thread to receive data from Arduino
        receive_thread = threading.Thread(target=receive_data_from_arduino, args=(shared_state, ser))
        receive_thread.daemon = True  # Thread will be terminated when the main program ends
        receive_thread.start()

        while True:
            with shared_state.lock:
                shared_state.state = input("Enter the state to send to Arduino (or 'exit' to quit): ")
            
            if shared_state.state.lower() == 'exit':
                break

            send_serial_data(ser, shared_state.state)
            print(f"Data sent to Arduino: {shared_state.state}")

    except serial.SerialException as e:
        print(f"Error with the serial port: {e}")
    except FileNotFoundError as e:
        print(f"File not found: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Connection with the serial port closed.")
        print("Closed.")

# Main function to configure and execute the program
def main():
    target_baudrate = input("Specify the microcontroller operating frequency:\n\nExamples:\n1 - Arduino (9600 baud)\n2 - ESP (115200 baud): ")

    if target_baudrate == '1':
        target_baudrate = 9600
    elif target_baudrate == '2':
        target_baudrate = 115200
    else:
        print("Unrecognized frequency. Using 9600 baud by default.")
        target_baudrate = 9600

    device_port = find_device_port(target_baudrate)

    if device_port:
        print(f"Device found on port: {device_port}")
        main_loop(device_port, target_baudrate)
    else:
        print("Device not found.")

# Check if the script is being executed as the main program
if __name__ == "__main__":
    main()
