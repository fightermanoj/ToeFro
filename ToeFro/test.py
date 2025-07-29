import socket
import csv
import os

# --- UDP Configuration ---
# The IP address to listen on. '' means listen on all available interfaces.
LISTEN_IP = '' # Listen on all available interfaces
LISTEN_PORT = 2390 # The same port the ESP32 is sending data to

# --- CSV Configuration ---
CSV_FILENAME_PREFIX = "udp_test_"
CSV_FILENAME_SUFFIX = ".csv"
CSV_HEADER = [
    "Timestamp_ms", # Added Timestamp_ms to the header
    "Euler_X", "Euler_Y", "Euler_Z",
    "Gyro_X", "Gyro_Y", "Gyro_Z",
    "LinearAccel_X", "LinearAccel_Y", "LinearAccel_Z",
    "Magnetometer_X", "Magnetometer_Y", "Magnetometer_Z",
    "RawAccel_X", "RawAccel_Y", "RawAccel_Z",
    "Gravity_X", "Gravity_Y", "Gravity_Z",
    "Quat_W", "Quat_X", "Quat_Y", "Quat_Z",
    "Temperature",
    "Cal_System", "Cal_Gyro", "Cal_Accel", "Cal_Mag"
]

def find_unique_filename(prefix, suffix):
    """
    Finds the next available unique filename (e.g., udp_test_1.csv, udp_test_2.csv).
    """
    index = 1
    while True:
        filename = f"{prefix}{index}{suffix}"
        if not os.path.exists(filename):
            return filename
        index += 1

def parse_bno_data(data_string):
    """
    Parses the tab-separated BNO055 data string into a dictionary.
    """
    parsed_data = {}
    # Split by tab to get major sections
    sections = data_string.split('\t')

    for section in sections:
        # Each section can contain multiple key:value pairs separated by spaces
        parts = section.strip().split(' ')
        for part in parts:
            if ':' in part:
                key, value = part.split(':', 1)
                try:
                    # Try to convert to float, otherwise keep as string
                    parsed_data[key.strip()] = float(value.strip())
                except ValueError:
                    # For calibration values, they are integers
                    try:
                        parsed_data[key.strip()] = int(value.strip())
                    except ValueError:
                        parsed_data[key.strip()] = value.strip() # Fallback to string

    return parsed_data

def get_row_from_parsed_data(parsed_data):
    """
    Extracts values from the parsed dictionary in the order of CSV_HEADER.
    """
    row = []
    # Define the keys in the exact order they appear in the CSV_HEADER
    # Added "time" to the ordered_keys
    ordered_keys = [
        "time", # Added this key to match the ESP32 output
        "ex", "ey", "ez",
        "avgx", "avgy", "avgz",
        "lax", "lay", "laz",
        "mx", "my", "mz",
        "rax", "ray", "raz",
        "agx", "agy", "agz",
        "qw", "qx", "qy", "qz",
        "temp",
        "calsys", "calgyro", "calaccel", "calmag"
    ]
    for key in ordered_keys:
        row.append(parsed_data.get(key, 'N/A')) # Use .get() to handle missing keys gracefully
    return row

def start_udp_listener_and_logger():
    """
    Starts a UDP listener to receive data from the ESP32, parses it,
    and logs it to a uniquely named CSV file.
    """
    print(f"Starting UDP listener on port {LISTEN_PORT}...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    csv_file = None
    csv_writer = None

    try:
        sock.bind((LISTEN_IP, LISTEN_PORT))
        print("UDP listener started. Waiting for data...")

        # Find a unique filename for the CSV
        output_filename = find_unique_filename(CSV_FILENAME_PREFIX, CSV_FILENAME_SUFFIX)
        print(f"Logging data to: {output_filename}")

        # Open the CSV file for writing
        csv_file = open(output_filename, 'w', newline='')
        csv_writer = csv.writer(csv_file)

        # Write the header row
        csv_writer.writerow(CSV_HEADER)

        while True:
            data, addr = sock.recvfrom(1024)
            decoded_data = data.decode('utf-8')

            print(f"Received from {addr}: {decoded_data}")

            # Parse the data string
            parsed_sensor_data = parse_bno_data(decoded_data)

            # Get the row in the correct order for CSV
            csv_row = get_row_from_parsed_data(parsed_sensor_data)

            # Write the data row to the CSV file
            csv_writer.writerow(csv_row)
            csv_file.flush() # Ensure data is written to disk immediately

    except socket.error as e:
        print(f"Socket error: {e}")
    except KeyboardInterrupt:
        print("\nUDP listener and logger stopped by user.")
    finally:
        # Close the socket
        sock.close()
        print("Socket closed.")
        # Close the CSV file if it was opened
        if csv_file:
            csv_file.close()
            print(f"CSV file '{output_filename}' closed.")

if __name__ == "__main__":
    start_udp_listener_and_logger()
