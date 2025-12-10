import queue
import threading
import serial
import os
import serial.tools.list_ports
import time

from collections import deque
from glove2robot.utils.bowie import BowieGlove

def find_bowie_port():
    """
    Find all connected ports and return the bowie port device

    Returns:
        string: com port for the bowie glove or None if it is not found.
    """

    ports = serial.tools.list_ports.comports()

    for port in ports:
        if port.product == "BowieGlove":
            return port.device
    return None

def init_bowie_stream():
    """
    Connects to bowie glove via serial port

    Returns:
        bowie object for reading in raw data
    """
    port = find_bowie_port()

    if port is None:
        print(f"[ERROR] Bowie port not found! Is it plugged in?")
        exit(0)
    else:
        print(f"Bowie glove found on {port}!")
        bowie = BowieGloveStream(port)
    return bowie

class BowieGloveStream:

    def __init__(self, port):
        self.bowie = BowieGlove(port)

        self.recording = False
        self.raw_data = queue.Queue()

        # short buffer of organized data
        self.bowie_mag_deque = [deque(maxlen=10) for _ in range(40)]
        self.bowie_quat_deque = [deque(maxlen=10) for _ in range(20)]
        self.bowie_lock = threading.Lock()



    def read_data(self, stop_event):
        """
        Read in all the raw bowie data and keep it in the self.raw_data queue.
        Handles SerialException and attempts to reconnect if the device disconnects.
        """
        while not stop_event.is_set():
            try:
                data = self.bowie.read()
                if data:
                    self.raw_data.put((time.monotonic_ns(), data))
            except serial.SerialException as e:
                print(f"[ERROR] SerialException: {e}. Attempting to reconnect...")
                self._reconnect_bowie()
            except Exception as e:
                print(f"[ERROR] Unexpected error in read_data: {e}")
                break
                # continue

    def _reconnect_bowie(self):
        """
        Attempt to reconnect the BowieGlove device.
        """
        try:
            self.bowie.close()
            time.sleep(1)  # Wait briefly before reconnecting
            self.bowie.connect()
            print("[INFO] Reconnected to BowieGlove.")
        except Exception as e:
            print(f"[ERROR] Failed to reconnect to BowieGlove: {e}")



    # def process_data(self, stop_event, main_window):
    def process_data(self, stop_event):
        """
        Iterates through any data available in self.raw_data, decodes it, and saves decoded data to corresponding data queue
        """
        while not stop_event.is_set():
            if self.raw_data.empty() is False:
                ts, data =  self.raw_data.get()
                if len(data) < 37:
                    print(f"[ERROR] Data too short, skipping")
                    continue

                decoded_data = self.bowie.decode(data)
                for item in decoded_data:
                    item = item.to_dict()
                    # print(item)
                    if self.recording: # just for some efficiency
                        self.append_bowie_data((ts, item))

                    # visual block
                    # try:
                    #     if "mag" in item:
                    #         sensor_id = item["sensorId"]
                    #         if (
                    #             sensor_id == main_window.selected_sensor_id
                    #         ):  # Check if the current sensor matches the selected one
                    #             x = item["mag"]["x"]
                    #             y = item["mag"]["y"]
                    #             z = item["mag"]["z"]
                    #             # main_window.update_plot(
                    #             #     x_value=x, y_value=y, z_value=z, timestamp=ts
                    #             # )

                    # except Exception as e:
                        continue
            else:
                ts = time.perf_counter()
                while time.perf_counter() - ts < 0.005:
                        pass
        return None

    def append_bowie_data(self, data):
        """
        Given decoded dictionary (data), append data to the correct deque (mag or quat) corresponding to the sensorId

        Args:
            data: a tuple of (timestamp, dictionary of decoded bowie data)
        """

        (ts, data_dict) = data
        if "sensorId" in data_dict:
            if "mag" in data_dict:
                idx = data_dict["sensorId"]
                with self.bowie_lock:
                    self.bowie_mag_deque[idx - 1].append(data)
            if "quat" in data_dict:
                idx = data_dict["sensorId"]
                with self.bowie_lock:
                    self.bowie_quat_deque[idx - 1].append(data)
