import time
from math import pi, sin
import numpy as np
import inspire_modbus_lib as hand
import pickle
import os
from datetime import datetime


class InspireDataCollection():
    def __init__(self, ip_address, port, test_duration, rate_hz, handedness, data_folder, grasp_duration=None):
        # connect to Inspire
        self.client = hand.open_modbus(ip_address, port)
        print("Initializing hand")
        # set default force and speed parameters
        # hand.write6(self.client, 'speedSet', [1000, 1000, 1000, 1000, 1000, 1000])
        hand.write6(self.client, 'speedSet', [100, 100, 100, 100, 100, 100])
        hand.write6(self.client, 'forceSet', [500, 500, 500, 500, 500, 500])
        #start with open hand
        hand.write6(self.client, 'angleSet', [1000, 1000, 1000, 1000, 1000, 1000])
        time.sleep(1)
        print("Done initializing hand")
        self.test_duration = test_duration
        self.rate_hz = rate_hz
        self.handedness = handedness
        self.data_folder = data_folder
        self.time_write = []
        self.position_write = []
        self.time_read = []
        self.position_read = []
        self.current_read = []
        self.touch_read = []
        if grasp_duration:
            self.grasp_duration = grasp_duration

    def interval_grasp(self):
        test_start_time = time.perf_counter()
        interval = 5 #seconds between each grasp
        while (time.perf_counter() - test_start_time) < self.test_duration:
            success = self.run_grip_test()
            if success:
                current_time = time.perf_counter()
                next_call_time = current_time + interval
                while time.perf_counter() < next_call_time:
                    print("Time until next grasp: ", next_call_time -time.perf_counter())
                    time.sleep(0.1)
            else:
                print("Error")
                break
        self.client.close()

    def send_and_receive(self, positions):
        self.position_write.append(positions.copy())
        hand.write6(self.client, 'angleSet', positions)
        #current_time = time.time()
        self.time_write.append(time.time())
        #while time.time() - current_time < (1/self.rate_hz):
        #    pass
        time.sleep(1/self.rate_hz)
        self.time_read.append(time.time())
        self.position_read.append(hand.read6(self.client, 'angleAct'))
        self.touch_read.append(hand.read_all_tactile(self.client))
        return True

    def run_grip_test(self):
        grasp_open = [1000, 1000, 1000, 1000, 1000, 1000]
        grasp_close = [0, 0, 0, 0, -1, -1] #need to figure out thumb position later
        start_time = time.perf_counter()
        try:
            current_datetime = datetime.fromtimestamp(time.perf_counter())
            self.folder_name = os.path.join(self.data_folder, current_datetime.strftime('%Y-%m-%d_%H-%M-%S') + "_"+ str(self.rate_hz)+"hz_" + str(self.reply_mode) + "reply_" + str(self.test_duration) + "s_" + self.handedness + str(self.grasp_duration) + "grasp_s")
            print("creating data folder {folder}".format(folder=self.folder_name))
            os.mkdir(self.folder_name)
        except:
            print("Make sure parent folders {folder} exist. Otherwise, this data folder already exists. Exiting now to prevent overwriting.".format(folder=self.data_folder))
            return False
        try:
            print("Moving to init position")
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < 1: #1 second to get to open grasp
                self.send_and_receive(grasp_open)
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < self.grasp_duration:
                print("Time: ", time.perf_counter() - start_time)
                self.send_and_receive(grasp_close)

            start_time = time.perf_counter()
            print("Open grasp: ", time.perf_counter() - start_time)
            while time.perf_counter() - start_time < 1: #1 second to get to open grasp
                self.send_and_receive(grasp_open)
        finally:
            print("Open grasp: ", time.perf_counter() - start_time)
            while time.perf_counter() - start_time < 1: #1 second to get to open grasp
                self.send_and_receive(grasp_open)
            self.save_data()
        return True

    def run_sine_wave(self):
        start_time = time.time()
        time_write = []
        position_write = []
        time_read = []
        position_read = []
        current_read = []
        touch_read = []

        try:
            current_datetime = datetime.fromtimestamp(time.time())
            self.folder_name = os.path.join(self.data_folder, current_datetime.strftime('%Y-%m-%d_%H-%M-%S') + "_"+ str(self.rate_hz)+"hz_" + str(self.test_duration) + "s_" + self.handedness + "_sine")
            print("creating data folder {folder}".format(folder=self.folder_name))
            os.mkdir(self.folder_name)

        except:
            print("Make sure parent folders {folder} exist. Otherwise, this data folder already exists. Exiting now to prevent overwriting.".format(folder=self.data_folder))
            return

        pos = [1000, 1000, 1000, 1000, -1, -1]
        while time.time() - start_time < self.test_duration:
           # print("Time:", (time.time() - start_time)/60)
            try:
                # for i in range(0, 4): #fingers only for now
                    # time_constant = 2.0*pi
                    # phase_offset = i * (2 * pi) / 12
                    # clock_signal = time.perf_counter() * time_constant #one ground truth, everything else offset from here
                    # amplitude = 500
                    # joint_offset = 500 #so that joint angles stay within joint limits (not negative numbers with pure sine wave)
                    # pos[i] = (amplitude * sin(clock_signal + phase_offset) + joint_offset) * (180./pi)
                t = time.time() - start_time
                q0_angle = int(500 + 200 * np.sin(2.0 * t))
                q1_angle = int(500 + 200 * np.sin(2.0 * t + np.pi/2))
                q2_angle = int(500 + 200 * np.sin(2.0 * t + np.pi))
                q3_angle = int(500 + 200 * np.sin(2.0 * t + 3 * np.pi/2))
                pos = [q0_angle, q1_angle, q2_angle, q3_angle, -1, -1]
                self.send_and_receive(pos)
            except KeyboardInterrupt:
                print("Keyboard interrupt")
                self.save_data()
                self.client.close()
                return
            except Exception as e:
                print(f"An error occurred: {e}")
                self.save_data()
                self.client.close()
                return
        # self.save_data()
        self.client.close()
        return

    def save_data(self):
        #save data to pkl files
        with open(self.folder_name+"/t_write.pkl", "wb") as file:
            pickle.dump(self.time_write, file)
        with open(self.folder_name + "/t_read.pkl", "wb") as file:
            pickle.dump(self.time_read, file)
        with open(self.folder_name+"/pos_write.pkl", "wb") as file:
            pickle.dump(self.position_write, file)
        with open(self.folder_name+ "/pos_read.pkl", "wb") as file:
            pickle.dump(self.position_read, file)
        with open(self.folder_name + "/touch_read.pkl", "wb") as file:
            pickle.dump(self.touch_read, file)
        print("saved to: ", self.folder_name)



def main():
    test_duration_minutes = 5
    test_duration = test_duration_minutes*60
    rate_hz = 50
    reply_mode = 0
    handedness= "right"
    data_folder = "./bowie_data/" #these parent folders need to exist before data collection starts
    grasp_duration_minutes = 0.2
    grasp_duration = grasp_duration_minutes * 60

    ip_address = "192.168.11.210"
    port = 6000
    inspire = InspireDataCollection(ip_address, port, test_duration, rate_hz, handedness, data_folder, grasp_duration)
    inspire.run_sine_wave()


if __name__ == "__main__":
    main()
