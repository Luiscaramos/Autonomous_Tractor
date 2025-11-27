import serial
import csv
import time

PORT = "COM5"      # Change to your serial port
BAUD = 115200
CSV_FILENAME = "motor_prbs_log.csv"

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
        print(f"Connected to {PORT}")
        time.sleep(2)
    except Exception as e:
        print("Error:", e)
        return

    csv_file = open(CSV_FILENAME, mode="w", newline="")
    writer = csv.writer(csv_file)

    # CSV header
    writer.writerow(["mcu_time_ms", "position_M1", "position_M2"])

    print("Logging started… CTRL+C to stop.")

    try:
        while True:
            if ser.in_waiting:
                line = ser.readline().decode(errors="ignore").strip()

                # Expect:  "1234 15.3 17.8"
                parts = line.split()
                if len(parts) != 3:
                    continue

                try:
                    t_mcu  = int(parts[0])
                    pos1   = float(parts[1])
                    pos2   = float(parts[2])
                except ValueError:
                    continue

                writer.writerow([t_mcu, pos1, pos2])
                print(t_mcu, pos1, pos2)

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        ser.close()
        csv_file.close()
        print(f"Saved to {CSV_FILENAME}")


if __name__ == "__main__":
    main()