import serial
import time
import sys
import serial.tools.list_ports

def find_pico_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # MacOS Pico usually shows up as usbmodem
        if "usbmodem" in p.device or "Pico" in p.description:
            return p.device
    return None

def download_csv(port):
    print(f"Connecting to {port}...")
    try:
        # Increase timeout just in case
        ser = serial.Serial(port, 9600, timeout=2)
        time.sleep(2) # Wait for DTR reset
        
        # Flush input before sending command
        ser.reset_input_buffer()
        
        # Retry loop for sending 'd' command
        print("Sending Dump Command (will retry)...")
        
        # Initialize variables
        csv_content = []
        raw_buffer = ""
        capturing = False
        start_time = time.time()
        command_sent_time = time.time()
        
        while True:
            # Send 'd' every 2 seconds if we haven't started capturing
            if time.time() - command_sent_time > 2.0 and not capturing:
                print("...sending 'd' again...")
                ser.write(b'd')
                ser.flush()
                command_sent_time = time.time()

            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                raw_buffer += chunk
                
                # Process complete lines from buffer
                while '\n' in raw_buffer:
                    line, raw_buffer = raw_buffer.split('\n', 1)
                    line = line.strip()
                    
                    if not line: continue
                    
                    # print(f"Received: {line}") # Optional: Uncomment to see verified lines
                    
                    if "START CSV DUMP" in line:
                        capturing = True
                        print("Capture Started!")
                        continue
                    
                    if "END CSV DUMP" in line:
                        print("Capture Complete!")
                        # Break out of both loops
                        ser.close()
                        
                        if csv_content:
                            save_csv(csv_content)
                        else:
                            print("No CSV lines captured.")
                        return
                    
                    if capturing:
                        csv_content.append(line)
            
            time.sleep(0.01)

    except Exception as e:
        print(f"Error: {e}")

def save_csv(content):
    filename = "scan_log.csv"
    with open(filename, "w") as f:
        f.write("Freq_MHz,Timestamp_ms,RSSI_dBm,Data_Hex\n")
        count = 0
        for row in content:
            if "," in row and "Freq" not in row:
                f.write(row + "\n")
                count += 1
    print(f"\nSUCCESS: Saved {count} rows to {filename}")


if __name__ == "__main__":
    port = find_pico_port()
    if not port:
        # Fallback to asking user or checking args
        if len(sys.argv) > 1:
            port = sys.argv[1]
        else:
            print("Could not auto-detect Pico. Please specify port as argument.")
            print("Example: python download_logs.py /dev/cu.usbmodem1234")
            sys.exit(1)
    
    download_csv(port)
