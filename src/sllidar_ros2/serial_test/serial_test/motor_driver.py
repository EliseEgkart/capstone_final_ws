import numpy as np
import serial
from serial_test.helper import Helper

class MotorDriver:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB_RS485', 115200, timeout=0.5)
        self.RMID = 183
        self.TMID = 172
        self.driverID = 1

        self.encoder_gain = 250  # encoder change value over 360 degree
        self.rpm1, self.rpm2 = 0, 0
        self.current1, self.current2 = 0., 0.
        self.status1, self.status2 = 0, 0
        self.pos1, self.pos2 = 0, 0
    
    def version_check(self):
        pid = 1
        byChkSend = np.array((self.RMID + self.TMID + 1 + 4 + 1 + pid) % 256, dtype=np.uint8)
        chk = ~byChkSend + 1
        data = np.array([self.RMID, self.TMID, 1, 4, 1, pid, chk], dtype=np.uint8)
        print(data)
        self.ser.write(data.tobytes())

        readbytes = self.ser.read(size=7)
        received_data = np.frombuffer(readbytes, dtype=np.uint8)
        print(received_data)

    def send_torque_cmd(self, t1, t2):
        pid = 209
        datanum = 7
        t1_ = np.array(Helper.int16_to_uint8arr(np.array(t1, dtype=np.int16)), dtype=np.uint8)
        t2_ = np.array(Helper.int16_to_uint8arr(np.array(-t2, dtype=np.int16)), dtype=np.uint8)
        data = np.array(1, dtype=np.uint8)
        data = np.append(data, t1_)
        data = np.append(data, np.array(1, dtype=np.uint8))
        data = np.append(data, t2_)
        data = np.append(data, np.array(2, dtype=np.uint8))

        send_data = np.array([self.RMID, self.TMID, self.driverID, pid, datanum], dtype=np.uint8)
        send_data = np.append(send_data, data)
        byChkSend = np.array(np.sum(send_data, dtype=np.uint8))
        chk = np.array(~byChkSend + 1, dtype=np.uint8)
        send_data = np.append(send_data, chk)

        self.ser.write(send_data.tobytes())

    def recv_motor_state(self):
        readbytes = self.ser.read(size=24)
        data = np.frombuffer(readbytes, dtype=np.uint8)

        if len(data) == 0:
            print("[recv_motor_state] Error: Receive data timeout")
            return
        if len(data) != 24:
            print("[recv_motor_state] Error: Invalid data length")
            return

        exitflag = False
        exitflag |= (data[0] != self.TMID)
        exitflag |= (data[1] != self.RMID)
        exitflag |= (data[2] != self.driverID)
        exitflag |= (data[3] != 210)
        exitflag |= (data[4] != 18)
        exitflag |= (np.sum(data, dtype=np.uint8) != 0)

        if exitflag:
            print("[recv_motor_state] Error: Invalid frame received")
            return
        else:
            self.rpm1 = -Helper.uint8arr_to_int16(data[5], data[6])
            self.current1 = Helper.uint8arr_to_int16(data[7], data[8])
            self.status1 = data[9]
            self.pos1 = -Helper.uint8arr_to_int32(data[10], data[11], data[12], data[13])

            self.rpm2 = Helper.uint8arr_to_int16(data[14], data[15])
            self.current2 = Helper.uint8arr_to_int16(data[16], data[17])
            self.status2 = data[18]
            self.pos2 = Helper.uint8arr_to_int32(data[19], data[20], data[21], data[22])

    def recv_watch_delay(self): # PID_COM_WATCH_DELAY
        pid = 185
        byChkSend = np.array((self.RMID + self.TMID + 1 + 4 + 1 + pid) % 256, dtype=np.uint8)
        chk = ~byChkSend + 1
        data = np.array([self.RMID, self.TMID, 1, 4, 1, pid, chk], dtype=np.uint8)
        print(data)
        self.ser.write(data.tobytes())

        readbytes = self.ser.read(size=8)
        received_data = np.frombuffer(readbytes, dtype=np.uint8)
        print(received_data)
    
    def recv_stop_status(self): # PID_STOP_STATUS
        pid = 24
        new_stop_status = 1     # Default = 1
        
        byChkSend = np.array((self.RMID + self.TMID + 1 + 4 + 1 + pid) % 256, dtype=np.uint8)
        chk = ~byChkSend + 1
        data = np.array([self.RMID, self.TMID, 1, 4, 1, pid, chk], dtype=np.uint8)
        self.ser.write(data.tobytes())

        readbytes = self.ser.read(size=7)
        received_data = np.frombuffer(readbytes, dtype=np.uint8)
        if len(received_data) == 7:
            current_stop_status = received_data[5]
        else:
            current_stop_status = new_stop_status
        
        if new_stop_status != current_stop_status:
            byChkSend = np.array((self.RMID + self.TMID + 1 + pid + 1 + new_stop_status) % 256, dtype=np.uint8)
            chk = ~byChkSend + 1
            data = np.array([self.RMID, self.TMID, 1, pid, 1, new_stop_status, chk], dtype=np.uint8)
            self.ser.write(data.tobytes())
            print("stop status changed from " + str(current_stop_status) + " to " + str(new_stop_status) + ".")
        else:
            print(received_data)

    def recv_read_this(self):
        pid = 135
        data_size = 2
        byChkSend = np.array((self.RMID + self.TMID + 1 + 4 + 1 + pid) % 256, dtype=np.uint8)
        chk = ~byChkSend + 1
        data = np.array([self.RMID, self.TMID, 1, 4, 1, pid, chk], dtype=np.uint8)
        print(data)
        self.ser.write(data.tobytes())

        readbytes = self.ser.read(size=data_size+6)
        received_data = np.frombuffer(readbytes, dtype=np.uint8)
        print(received_data)
    
    def write_BAUD(self):
        pid = 135
        new_BAUD = 4        # Default = 2

        data_size = 2
        byChkSend = np.array((self.RMID + self.TMID + 1 + pid + data_size + 0xAA + new_BAUD) % 256, dtype=np.uint8)
        chk = ~byChkSend + 1
        data = np.array([self.RMID, self.TMID, 1, pid, data_size, 0xAA, new_BAUD, chk], dtype=np.uint8)
        
        print("BAUDRATE UPDATED!")
        self.ser.write(data.tobytes())

    def send_position_cmd(self, p1, p2, mv1, mv2):
        pid = 206
        datanum = 15
        p1_ = np.array(Helper.int32_to_uint8arr(np.array(-p1, dtype=np.int32)), dtype=np.uint8)
        mv1_ = np.array(Helper.int16_to_uint8arr(np.array(mv1, dtype=np.int16)), dtype=np.uint8)
        p2_ = np.array(Helper.int32_to_uint8arr(np.array(p2, dtype=np.int32)), dtype=np.uint8)
        mv2_ = np.array(Helper.int16_to_uint8arr(np.array(mv2, dtype=np.int16)), dtype=np.uint8)
        data = np.array(1, dtype=np.uint8)
        data = np.append(data, p1_)
        data = np.append(data, mv1_)
        data = np.append(data, np.array(1, dtype=np.uint8))
        data = np.append(data, p2_)
        data = np.append(data, mv2_)
        data = np.append(data, np.array(2, dtype=np.uint8))

        send_data = np.array([self.RMID, self.TMID, self.driverID, pid, datanum], dtype=np.uint8)
        send_data = np.append(send_data, data)
        byChkSend = np.array(np.sum(send_data, dtype=np.uint8))
        chk = np.array(~byChkSend + 1, dtype=np.uint8)
        send_data = np.append(send_data, chk)

        self.ser.write(send_data.tobytes())

    def send_vel_cmd(self, v1, v2):
        pid = 207
        datanum = 7
        v1_ = np.array(Helper.int16_to_uint8arr(np.array(-v1, dtype=np.int16)), dtype=np.uint8)
        v2_ = np.array(Helper.int16_to_uint8arr(np.array(v2, dtype=np.int16)), dtype=np.uint8)
        data = np.array(1, dtype=np.uint8)
        data = np.append(data, v1_)
        data = np.append(data, np.array(1, dtype=np.uint8))
        data = np.append(data, v2_)
        data = np.append(data, np.array(2, dtype=np.uint8))

        send_data = np.array([self.RMID, self.TMID, self.driverID, pid, datanum], dtype=np.uint8)
        send_data = np.append(send_data, data)
        byChkSend = np.array(np.sum(send_data, dtype=np.uint8))
        chk = np.array(~byChkSend + 1, dtype=np.uint8)
        send_data = np.append(send_data, chk)

        self.ser.write(send_data.tobytes())
