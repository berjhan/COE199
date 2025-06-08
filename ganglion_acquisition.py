import threading
import asyncio
import time
import multiprocessing
import pandas as pd
from datetime import datetime, timedelta
from brainflow.board_shim import BoardShim, BrainFlowInputParams, BoardIds
from bleak import BleakClient, BleakScanner
import psutil
import os


# =========================
# CONFIGURATION
# =========================
GANGLION_MAC = "ca:ae:fd:20:3e:1a"  # replace with your Ganglion MAC address
NRF_NAME = "berj_nrf"               # advertised BLE name
NRF_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"  # NUS service
NRF_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"     # RX char UUID
voltage_values = []

# =========================
# Ganglion Reader Thread
# =========================
def ganglion_reader():
    print("[Ganglion] Starting...")
    BoardShim.enable_dev_board_logger()
    params = BrainFlowInputParams()
    params.mac_address = GANGLION_MAC
    board = BoardShim(BoardIds.GANGLION_NATIVE_BOARD, params)

    board.prepare_session()
    board.start_stream()
    stream_start_time = datetime.now()
    time.sleep(100)  # Stream duration
    data = board.get_board_data()
    board.stop_stream()
    board.release_session()

    # Transpose data and compute timestamps
    df = pd.DataFrame(data).T
    sample_rate = BoardShim.get_sampling_rate(BoardIds.GANGLION_NATIVE_BOARD)
    timestamps = [stream_start_time + timedelta(seconds=i / sample_rate) for i in range(len(df))]
    df.insert(0, "Timestamp", [ts.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] for ts in timestamps])

    df.to_csv("ganglion_emg_100s_6.csv", index=False)
    print("[Ganglion] Data saved.")

# =========================
# nRF Reader Coroutine
# =========================
async def nrf_reader():
    p = psutil.Process(os.getpid())
    p.nice(psutil.HIGH_PRIORITY_CLASS)  # Or REALTIME_PRIORITY_CLASS
    p.cpu_affinity([0])  # Use CPU 0 only
    print("[nRF] Scanning for device...")
    device = await BleakScanner.find_device_by_filter(lambda d, _: NRF_NAME in d.name if d.name else False)
    if not device:
        print("[nRF] Device not found.")
        return

    async with BleakClient(device) as client:
        print(f"[nRF] Connected to {device.address}")
        data_log = []

        def handle_rx(_, data):
            try:
                line = data.decode(errors='ignore').strip()
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                data_log.append((timestamp, line))
                # Decode BLE byte data
                line = data.decode(errors='ignore').strip()

                # Get current timestamp with milliseconds
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                # Try to convert the string to a float voltage value
                # try:
                #     voltage = float(line)
                #     voltage_values.append(voltage)
                #     data_log.append((timestamp, voltage))
                # except ValueError:
                #     print(f"[nRF] ⚠️ Received non-numeric line: {line}")
                    # return
            except Exception as e:
                print(f"Error handling BLE data: {e}")
                # Debug print (optional)
                # print(f"[nRF] {timestamp} | {voltage:.9f}")

            # except Exception as e:
            #     print(f"Error handling BLE data: {e}")

        # Start receiving notifications on the given characteristic
        await client.start_notify(NRF_CHAR_UUID, handle_rx)

        # Collect data for 30 seconds (adjust as needed)
        await asyncio.sleep(10)

        await client.stop_notify(NRF_CHAR_UUID)

        # Save collected data to CSV
        df = pd.DataFrame(data_log, columns=["Timestamp", "Data"])
        df.to_csv("python_nrf_channels_10s.csv", index=False)
        print("[nRF] Data saved to nrf_data.csv")

# =========================
# Thread Launcher
# =========================
def start_nrf_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(nrf_reader())

# =========================
# Main
# =========================
if __name__ == "__main__":

    print("[Main] Finished collecting data from both devices.")

    g_thread = threading.Thread(target=ganglion_reader)
    n_thread = threading.Thread(target=start_nrf_thread)

    g_thread.start()
    g_thread.join()
    # time.sleep(2)  # delays for 2 seconds
    # n_thread.join()


# # main.py
# from multiprocessing import Process
# from ganglion_reader import run_ganglion_reader
# from nrf_reader import run_nrf_reader

# if __name__ == "__main__":
#     g_proc = Process(target=run_ganglion_reader)
#     n_proc = Process(target=run_nrf_reader)

#     g_proc.start()
#     n_proc.start()

#     g_proc.join()
#     n_proc.join()

#     print("[Main] Finished collecting data from both devices.")
