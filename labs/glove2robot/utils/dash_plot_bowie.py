from collections import deque
from dash import Dash, dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import threading
import time
import betterproto
import utils.bowiepb as bpb
import numpy as np
import serial
from cobs import cobs

# Global variables for data queues
max_points = 100
data_queue_x = deque(maxlen=max_points)
data_queue_y = deque(maxlen=max_points)
data_queue_z = deque(maxlen=max_points)

# Variables to store the most recent magnetometer values for each sensor
mag_35 = {"x": None, "y": None, "z": None}
mag_15 = {"x": None, "y": None, "z": None}

# Initialize Dash app
app = Dash(__name__)

# Layout for the Dash app
app.layout = html.Div([
    html.H1("Delta Magnetic Flux"),
    dcc.Graph(id="live-plot"),
    dcc.Interval(
        id="interval-component",
        interval=100,  # Update every 100ms
        n_intervals=0
    )
])

# Callback to update the plot
@app.callback(
    Output("live-plot", "figure"),
    [Input("interval-component", "n_intervals")]
)
def update_plot(n_intervals):
    # Create the figure with three subplots
    # fig = make_subplots(
    #     rows=3, cols=1,
    #     shared_xaxes=True,
    #     subplot_titles=["Magnetometer X Difference", "Magnetometer Y Difference", "Magnetometer Z Difference"]
    # )
    fig = make_subplots(
        rows=3, cols=1,
        shared_xaxes=True,
        subplot_titles=["Magnetic Flux X", "Magnetic Flux Y", "Magnetic Flux Z"]
    )

    # Add traces for x, y, z differences
    fig.add_trace(go.Scatter(
        x=list(range(len(data_queue_x))),
        y=list(data_queue_x),
        mode="lines",
        name="Magnetometer",
        line=dict(color="red")
    ), row=1, col=1)

    fig.add_trace(go.Scatter(
        x=list(range(len(data_queue_y))),
        y=list(data_queue_y),
        mode="lines",
        name="Magnetometer Y Difference",
        line=dict(color="green")
    ), row=2, col=1)

    fig.add_trace(go.Scatter(
        x=list(range(len(data_queue_z))),
        y=list(data_queue_z),
        mode="lines",
        name="Magnetometer Z Difference",
        line=dict(color="blue")
    ), row=3, col=1)

    # Update layout
    fig.update_layout(
        height=800,
        xaxis_title="Time",
        yaxis_title="Value",
        showlegend=False  # Disable legend since each subplot has its own title
    )

    # Update y-axis titles for each subplot
    # fig.update_yaxes(title_text="X Difference", row=1, col=1)
    # fig.update_yaxes(title_text="Y Difference", row=2, col=1)
    # fig.update_yaxes(title_text="Z Difference", row=3, col=1)
        # Set fixed y-axis ranges for each subplot
    fig.update_yaxes(title_text="X Difference", range=[-500, 1000], row=1, col=1)
    fig.update_yaxes(title_text="Y Difference", range=[-500, 1000], row=2, col=1)
    fig.update_yaxes(title_text="Z Difference", range=[-500, 1000], row=3, col=1)

    return fig


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

# Function to read and process data from BowieGlove
def read_bowie_data():
    global mag_35, mag_15
    bowie = BowieGlove("/dev/cu.usbmodem1234567891")
    bowie._dev.reset_input_buffer()

    while bowie.is_open:
        try:
            # Read and decode data
            raw_data = bowie.read()
            decoded_data = bowie.decode(raw_data)

            # Process data
            for item in decoded_data:
                item_dict = item.to_dict()
                print(item_dict)
                if "mag" in item_dict:
                    sensor_id = item_dict.get("sensorId")
                    mag_data = item_dict["mag"]
                    # print(sensor_id, mag_data)
                    # print(mag_data)

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

                    else: #only mag15 working rn :(
                        diff_x = mag_15["x"]
                        diff_y = mag_15["y"]
                        diff_z = mag_15["z"]

                    # Append differences to queues
                    data_queue_x.append(diff_x)
                    data_queue_y.append(diff_y)
                    data_queue_z.append(diff_z)

        except Exception as e:
            print(f"[ERROR]: {e}")
            pass


# Main function to start the Dash app and data processing
def main():
    # Start the data reading thread
    data_thread = threading.Thread(target=read_bowie_data, daemon=True)
    data_thread.start()

    # Run the Dash app
    app.run(debug=True, use_reloader=False)


if __name__ == "__main__":
    main()
