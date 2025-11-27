import serial
import csv
import time

PORT = "COM4"      # Change to your serial port
BAUD = 57600
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
    writer.writerow(["mcu_time_ms", "posM1", "posM2", "CH1_DC", "CH2_DC"])

    print("Logging started… CTRL+C to stop.")

    try:
        while True:

            if ser.in_waiting:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                # -------- STOP SIGNAL FROM STM32 --------
                if line == "S":
                    print("Stop signal received from STM32. Ending capture.")
                    break

                parts = line.split()
                # Expecting 5 fields: time pos1 pos2 CH1 CH2
                if len(parts) != 5:
                    continue

                try:
                    time_ms = int(parts[0])
                    posM1   = int(parts[1])
                    posM2   = int(parts[2])
                    ch1     = int(parts[3])
                    ch2     = int(parts[4])

                except ValueError:
                    continue

                writer.writerow([time_ms, posM1, posM2, ch1, ch2])
                #print(time_ms, posM1, posM2, ch1, ch2)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        ser.close()
        csv_file.close()
        print(f"Saved to {CSV_FILENAME}")


if __name__ == "__main__":
    main()
