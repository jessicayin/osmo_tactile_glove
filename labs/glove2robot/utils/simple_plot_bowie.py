from collections import deque
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import time
import time

import betterproto
import utils.bowiepb as bpb
import numpy as np
import serial
from cobs import cobs
from pynput import keyboard
import plotly.io as pio

# Use the "browser" renderer for live updates
pio.renderers.default = "browser"

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\x00")
        if i >= 0:
            r = self.buf[: i + 1]
            self.buf = self.buf[i + 1 :]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\x00")
            if i >= 0:
                r = self.buf + data[: i + 1]
                self.buf[0:] = data[i + 1 :]
                return r
            else:
                self.buf.extend(data)


class BowieGlove:
    def __init__(self, port):
        self._dev = None
        self.port = port
        self.tlc = 0
        self.drop = 0
        self.bad_pkt = 0
        self.connect()
        self.reader = ReadLine(self._dev)

    def connect(self):
        self._dev = serial.Serial(self.port)
        self._dev.reset_output_buffer()
        self._dev.reset_input_buffer()
        if not self._dev.is_open:
            raise IOError(f"Unable to open BowieGlove at {self.port}!")

    def read(self):
        data = self.reader.readline()
        return data

    def decode(self, data):
        cobs_data = data.split(b"\x00")
        frames = []
        for c_data in cobs_data:
            bowie_frame = bpb.Data()
            try:
                pb_data = cobs.decode(c_data)
                bowie_frame = bowie_frame.parse(pb_data)
                frames.append(bowie_frame)
                self.tlc += 1
            except cobs.DecodeError:
                self._dev.reset_input_buffer()
                self.drop += 1
                print(f">> COBS decode error on data: {cobs_data}")
                # TODO this is not handling [META EVENT] messages well
            except Exception as e:
                self.drop += 1
                pass
        return frames

    def close(self):
        if self.is_open:
            self._dev.close()
            print(f"Closed the COM port: {self.port}")
        else:
            print(f"COM port {self.port} is already closed.")
        return None

    @property
    def is_open(self):
        return self._dev.is_open

def initialize_plot():
    """
    Initializes the Plotly figure and traces for real-time plotting.
    Returns the figure and data queues for x, y, z differences.
    """
    max_points = 100  # Maximum number of points to display on the plot
    data_queue_x = deque(maxlen=max_points)  # Store the most recent x differences
    data_queue_y = deque(maxlen=max_points)  # Store the most recent y differences
    data_queue_z = deque(maxlen=max_points)  # Store the most recent z differences

    # Create subplots for x, y, z differences
    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, subplot_titles=[
        "Magnetometer X Difference", "Magnetometer Y Difference", "Magnetometer Z Difference"
    ])

    # Add traces for x, y, z differences
    trace_x = go.Scatter(y=[], mode="lines", name="Magnetometer X Difference", line=dict(color="red"))
    trace_y = go.Scatter(y=[], mode="lines", name="Magnetometer Y Difference", line=dict(color="green"))
    trace_z = go.Scatter(y=[], mode="lines", name="Magnetometer Z Difference", line=dict(color="blue"))

    fig.add_trace(trace_x, row=1, col=1)
    fig.add_trace(trace_y, row=2, col=1)
    fig.add_trace(trace_z, row=3, col=1)

    # Set layout
    fig.update_layout(
        height=800,
        title="Real-Time Magnetometer Differences",
        xaxis_title="Time",
        yaxis_title="Value",
        showlegend=True
    )

    return fig, data_queue_x, data_queue_y, data_queue_z


def update_plot(fig, data_queue_x, data_queue_y, data_queue_z):
    """
    Updates the Plotly figure with new data from the queues.
    Dynamically adjusts the x and y axes.
    """
    if len(data_queue_x) > 0:
        # Update traces with x and y data
        fig.data[0].x = list(range(len(data_queue_x)))  # X-axis for X difference trace
        fig.data[0].y = list(data_queue_x)  # Y-axis for X difference trace

        fig.data[1].x = list(range(len(data_queue_y)))  # X-axis for Y difference trace
        fig.data[1].y = list(data_queue_y)  # Y-axis for Y difference trace

        fig.data[2].x = list(range(len(data_queue_z)))  # X-axis for Z difference trace
        fig.data[2].y = list(data_queue_z)  # Y-axis for Z difference trace

        # Dynamically update the y-axis ranges
        fig.update_yaxes(range=[min(data_queue_x) - 10, max(data_queue_x) + 10], row=1, col=1)
        fig.update_yaxes(range=[min(data_queue_y) - 10, max(data_queue_y) + 10], row=2, col=1)
        fig.update_yaxes(range=[min(data_queue_z) - 10, max(data_queue_z) + 10], row=3, col=1)

        # Dynamically update the x-axis range
        fig.update_xaxes(range=[0, len(data_queue_x)], row=1, col=1)
        fig.update_xaxes(range=[0, len(data_queue_y)], row=2, col=1)
        fig.update_xaxes(range=[0, len(data_queue_z)], row=3, col=1)
        # fig.write_html("temp_plot.html", auto_open=False)
        # fig.show(renderer="browser")
        # fig.update_traces()
        pio.show(rendere="browser")


def main() -> None:
    # Initialize BowieGlove
    bowie = BowieGlove("/dev/cu.usbmodem1234567891")
    bowie._dev.reset_input_buffer()

    # Initialize the plot
    fig, data_queue_x, data_queue_y, data_queue_z = initialize_plot()

    # Variables to store the most recent magnetometer values for each sensor
    mag_35 = {"x": None, "y": None, "z": None}
    mag_15 = {"x": None, "y": None, "z": None}

    # Start live plotting
    fig.show(renderer="browser")  # Open the plot in the browser

    while bowie.is_open:
        try:
            # Read and decode data
            raw_data = bowie.read()
            decoded_data = bowie.decode(raw_data)

            # Process data
            for item in decoded_data:
                item_dict = item.to_dict()
                if "mag" in item_dict:
                    sensor_id = item_dict.get("sensorId")
                    mag_data = item_dict["mag"]
                    print(mag_data)
                    print(sensor_id)

                    # Update the most recent values for each sensor
                    if sensor_id == 35:
                        mag_35 = mag_data
                    elif sensor_id == 15:
                        mag_15 = mag_data

                    # Compute the difference if both sensors have data
                    if all(mag_35.values()) and all(mag_15.values()):
                        diff_x = mag_35["x"] - mag_15["x"]
                        diff_y = mag_35["y"] - mag_15["y"]
                        diff_z = mag_35["z"] - mag_15["z"]

                    else: #only sensor15 has data :(
                        diff_x = mag_15["x"]
                        diff_y = mag_15["y"]
                        diff_z = mag_15["z"]

                    # Append differences to queues
                    data_queue_x.append(diff_x)
                    data_queue_y.append(diff_y)
                    data_queue_z.append(diff_z)
                    print(f"Queue X: {list(data_queue_x)}")
                    print(f"Queue Y: {list(data_queue_y)}")
                    print(f"Queue Z: {list(data_queue_z)}")

                    # Update the plot
                    update_plot(fig, data_queue_x, data_queue_y, data_queue_z)

                    # Pause to simulate real-time updates
                    time.sleep(0.01)

        except KeyError:
            print(f"[KEY_ERROR]")
            pass
        except Exception as e:
            print(f"[ERROR]: {e}")
            pass


if __name__ == "__main__":
    main()
