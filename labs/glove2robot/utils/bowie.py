import time

import betterproto
import utils.bowiepb as bpb
import numpy as np
import serial
from cobs import cobs
from pynput import keyboard


# class TimeDelta:
#     def __init__(self, name: str, window: int = 10) -> None:
#         self.name = name
#         self.prev = 0
#         self.cnt = 0
#         self.td = {}
#         self.window = window
#         self.hz = []
#         self.delta = []

#     def step(self) -> None:
#         self.cnt += 1

#     def result(self) -> None:
#         if self.cnt % self.window == 0:
#             self.td = {
#                 "key": self.name,
#                 "ms": np.average(self.delta),
#                 "hz": np.average(self.hz),
#             }
#             print(self.td)
#             self.hz = []
#             self.delta = []
#             self.cnt = 0
#         self.step()

#     def __call__(self, ts: int) -> None:
#         delta = ts - self.prev
#         self.delta.append(delta)
#         if self.prev == ts:
#             return
#         self.prev = ts
#         self.hz.append(1 / delta * 1e3)
#         self.result()


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


def main() -> None:

    # bowie = BowieGlove("/dev/tty.usbmodem1234567891")
    bowie=BowieGlove("/dev/ttyACM0")
    # td_bowie = TimeDelta("bowie_data")
    bowie._dev.reset_input_buffer()
#    import ipdb; ipdb.set_trace()
    while bowie.is_open:

        try:
            data = bowie.read()
            data = bowie.decode(data)
            for item in data:
                # toggle off if you want to see error output more clearly
                print(item.to_dict())
        except KeyError:
            print(f"[KEY_ERROR]")
            pass
        except Exception as e:
            print(f"[ERROR]: {e}")
            pass



if __name__ == "__main__":
    main()

import time

# import betterproto
import glove2robot.utils.bowiepb as bpb
import numpy as np
import serial
from cobs import cobs
from pynput import keyboard


# class TimeDelta:
#     def __init__(self, name: str, window: int = 10) -> None:
#         self.name = name
#         self.prev = 0
#         self.cnt = 0
#         self.td = {}
#         self.window = window
#         self.hz = []
#         self.delta = []

#     def step(self) -> None:
#         self.cnt += 1

#     def result(self) -> None:
#         if self.cnt % self.window == 0:
#             self.td = {
#                 "key": self.name,
#                 "ms": np.average(self.delta),
#                 "hz": np.average(self.hz),
#             }
#             print(self.td)
#             self.hz = []
#             self.delta = []
#             self.cnt = 0
#         self.step()

#     def __call__(self, ts: int) -> None:
#         delta = ts - self.prev
#         self.delta.append(delta)
#         if self.prev == ts:
#             return
#         self.prev = ts
#         self.hz.append(1 / delta * 1e3)
#         self.result()


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
                # self._dev.reset_input_buffer()  #comment it out if using in separate thread from reading data
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




def main() -> None:

    bowie = BowieGlove("/dev/tty.usbmodem1234567891")
    # bowie = BowieGlove("/dev/ttyACM0")
    # td_bowie = TimeDelta("bowie_data")
    bowie._dev.reset_input_buffer()

    while bowie.is_open:

        try:
            data = bowie.read()
            data = bowie.decode(data)
            for item in data:
                # toggle off if you want to see error output more clearly
                print(item.to_dict())
        except KeyError:
            print(f"[KEY_ERROR]")
            pass
        except Exception as e:
            print(f"[ERROR]: {e}")
            pass


if __name__ == "__main__":
    main()
