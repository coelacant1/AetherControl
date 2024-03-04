import serial
import time
import os

# Function to initialize the serial connection
def init_serial_connection(port='COM9', baud_rate=38400):
    try:
        ser = serial.Serial(port, baud_rate, timeout=1, bytesize=8, parity='N', stopbits=1)
        if ser.isOpen():
            print(f"Connected to {port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect to {port} at {baud_rate} baud. Error: {e}")
        return None
    
# Function to send a command to the CNC machine and wait for an "ok" response
def send_command(ser, command):
    if ser:
        print(f"Sending command: {command}")
        ser.write(f"{command}\n".encode())
        
        # Wait for the "ok" response
        response = ""
        while "ok" not in response:
            if ser.in_waiting > 0:
                response += ser.read(ser.in_waiting).decode()
                time.sleep(0.1)  # Small delay to allow more data to arrive
            
        print("Received response: ok")
    else:
        print("Serial connection not established.")

# Function to send a command to the CNC machine and wait for an "ok" response
def send_command(ser, command):
    if ser:
        print(f"Sending command: {command}")
        ser.write(f"{command}\n".encode())
        
        # Wait for the "ok" response
        response = ""
        while "ok" not in response:
            if ser.in_waiting > 0:
                response += ser.read(ser.in_waiting).decode()
                time.sleep(0.01)  # Small delay to allow more data to arrive

        response = response.replace("\n", "")
        response = response.replace("\r", "")
            
        print("Received response: ", response)
    else:
        print("Serial connection not established.")

# Function to close the serial connection
def close_serial_connection(ser):
    if ser:
        ser.close()
        print("Serial connection closed.")

# New function to read G-code commands from a file
def load_gcode_commands(filename):
    with open(filename, 'r') as file:
        commands = file.read().splitlines()
    return commands

# Modified main function to control the CNC machine using commands from a file
def main():
    # Path to the G-code commands file
    filename = os.path.join(os.path.dirname(__file__), 'gcode_commands.txt')
    
    # Initialize serial connection
    ser = init_serial_connection()

    # Load G-code commands from the file
    commands = load_gcode_commands(filename)

    # Send commands to the CNC machine
    for cmd in commands:
        if cmd != "":
            send_command(ser, cmd)

    # Close the serial connection
    close_serial_connection(ser)

if __name__ == "__main__":
    while True:
        main()
