#! /usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import _thread
import tkinter as tk
#import serial.tools.list_ports
from serial.tools import list_ports
from tkinter import ttk
from tkinter import scrolledtext
import time
import struct
import binascii

from math import pi as PI, degrees, radians, sin, cos
import os
import sys,  traceback
from serial.serialutil import SerialException
from serial import Serial
import serial
import matplotlib.pyplot as plt
from drawnow import *
import atexit
import time


SerialPort = serial.Serial()
GUI = tk.Tk()  # 父容器
GUI.title("Serial Tool")  # 父容器标题
GUI.geometry("500x500")  # 父容器大小

Information = tk.LabelFrame(GUI, text="操作信息", padx=10, pady=10)  # 水平，垂直方向上的边距均为10
Information.place(x=20, y=20)
Information_Window = scrolledtext.ScrolledText(Information, width=20, height=5, padx=10, pady=10, wrap=tk.WORD)
Information_Window.grid()

Send = tk.LabelFrame(GUI, text="发送指令", padx=10, pady=5)  # 水平，垂直方向上的边距均为 10
Send.place(x=240, y=20)

DataSend = tk.StringVar()  # 定义DataSend为保存文本框内容的字符串

EntrySend = tk.StringVar()
Send_Window = ttk.Entry(Send, textvariable=EntrySend, width=23)
Send_Window.grid()

def WriteData():
    global DataSend
    DataSend = EntrySend.get()
    Information_Window.insert("end", '发送指令为：' + str(DataSend) + '\n')
    Information_Window.see("end")
    SerialPort.write(bytes(DataSend, encoding='utf8'))

tk.Button(Send, text="发送", command=WriteData).grid(pady=5, sticky=tk.E)

Receive = tk.LabelFrame(GUI, text="接收区", padx=10, pady=10)  # 水平，垂直方向上的边距均为 10
Receive.place(x=240, y=124)
Receive_Window = scrolledtext.ScrolledText(Receive, width=18, height=9, padx=8, pady=10, wrap=tk.WORD)
Receive_Window.grid()


option = tk.LabelFrame(GUI, text="选项", padx=10, pady=10)  # 水平，垂直方向上的边距均为10
option.place(x=20, y=150, width=203)  # 定位坐标
# ************创建下拉列表**************
ttk.Label(option, text="串口号:").grid(column=0, row=0)  # 添加串口号标签
ttk.Label(option, text="波特率:").grid(column=0, row=1)  # 添加波特率标签

Port = tk.StringVar()  # 端口号字符串
Port_list = ttk.Combobox(option, width=12, textvariable=Port, state='readonly')
ListPorts = list(serial.tools.list_ports.comports())
Port_list['values'] = [i[0] for i in ListPorts]
#Port_list.current(0)
Port_list.grid(column=1, row=0)  # 设置其在界面中出现的位置  column代表列   row 代表行

BaudRate = tk.StringVar()  # 波特率字符串
BaudRate_list = ttk.Combobox(option, width=12, textvariable=BaudRate, state='readonly')
BaudRate_list['values'] = (1200, 2400, 4800, 9600, 14400, 19200, 38400, 43000, 57600, 76800, 115200)
BaudRate_list.current(3)
BaudRate_list.grid(column=1, row=1)  # 设置其在界面中出现的位置  column代表列   row 代表行


option_t = tk.LabelFrame(GUI, text="sensor_info", padx=10, pady=10)  # 水平，垂直方向上的边距均为10
option_t.place(x=20, y=350, width=203)  # 定位坐标
# ************创建下拉列表**************
ttk.Label(option_t, text="sensor1:").grid(column=0, row=0)  # 添加串口号标签
ttk.Label(option_t, text="sensor2:").grid(column=0, row=1)  # 添加波特率标签

Port_t = tk.IntVar()
Port_list_t = tk.Message(option_t, width=60, textvariable=Port_t, justify='right')
Port_list_t.grid(column=1, row=0)  # 设置其在界面中出现的位置  column代表列   row 代表行

BaudRate_t = tk.IntVar()
BaudRate_list_t = tk.Message(option_t, width=60, textvariable=BaudRate_t, justify='right')
BaudRate_list_t.grid(column=1, row=1)  # 设置其在界面中出现的位置  column代表列   row 代表行


switch = tk.LabelFrame(GUI, text="", padx=10, pady=10)  # 水平，垂直方向上的边距均为 10
switch.place(x=20, y=250, width=203)  # 定位坐标
###########################################################################################
class Arduino:
    ''' Configuration Parameters
    '''
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.5):

        self.PID_RATE = 1  # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.

        self.WAITING_FF = 0
        self.WAITING_AA = 1
        self.RECEIVE_LEN = 2
        self.RECEIVE_PACKAGE = 3
        self.RECEIVE_CHECK = 4
        self.HEADER0 = 0xff
        self.HEADER1 = 0xaa
        self.REC_CMD = 5
        self.count = 0
        self.data1 = 0
        self.data2 = 0
        self.error_flag = 0
        self.SUCCESS = 0
        self.FAIL = -1

        self.receive_state_ = self.WAITING_FF
        self.receive_check_sum_ = 0
        self.payload_command = ''
        self.payload_ack = ''
        self.payload_args = bytes('','ascii')
        self.payload_len = 0
        self.byte_count_ = 0
        self.receive_message_length_ = 0

        # Keep things thread safe
        self.mutex = _thread.allocate_lock()

        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS

        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS

    def connect(self):
        try:
            print("Connecting to Arduino on port", self.port, "...")
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout,
                               writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            state_, val = self.get_baud()
            if val != self.baudrate:
                time.sleep(1)
                state_, val = self.get_baud()
                if val != self.baudrate:
                    raise SerialException
            print("Connected at", self.baudrate)
            print("Arduino is ready.")

        except SerialException:
            print("Serial Exception:")
            print(sys.exc_info())
            print("Traceback follows:")
            traceback.print_exc(file=sys.stdout)
            print("Cannot connect to Arduino!")
            os._exit(1)

    def open(self):
        self.port.open()

    def close(self):
        self.port.close()

    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd)

    def receiveFiniteStates(self, rx_data):
        print("rec_finite :", rx_data)
        if self.receive_state_ == self.WAITING_FF:
            #print("wait_0xff")
            # print str(binascii.b2a_hex(rx_data))
            if rx_data == b'\xff':
                self.receive_state_ = self.WAITING_AA
                self.receive_check_sum_ = 0
                self.receive_message_length_ = 0
                self.byte_count_ = 0
                self.payload_ack = ''
                self.payload_args =bytes('','ascii')
                self.payload_len = 0
                self.count = 0

        elif self.receive_state_ == self.WAITING_AA:
            if rx_data == b'\xaa':
                self.receive_state_ = self.REC_CMD
            else:
                self.receive_state_ = self.WAITING_FF
        elif self.receive_state_ == self.REC_CMD:
            self.count += 1
            if self.count == 1:
                self.data1, = struct.unpack("B", rx_data)
            elif self.count == 2:
                self.data2, = struct.unpack("B", rx_data)
                self.receive_state_ = self.RECEIVE_LEN
                if self.error_flag == 0 and self.data1 != 0:
                    self.error_flag = 1
                if self.data2 != 0 and self.error_flag == 0:
                    self.error_flag = 1
        elif self.receive_state_ == self.RECEIVE_LEN:
            self.receive_message_length_, = struct.unpack("B", rx_data)
            print("len = ", self.receive_message_length_)
            self.receive_state_ = self.RECEIVE_PACKAGE
        elif self.receive_state_ == self.RECEIVE_PACKAGE:
            if self.byte_count_ == 0:
                self.payload_ack = rx_data
            else:
                #print("before payload_args ++")
                try:
                    self.payload_args +=bytes(rx_data)
                except:
                    print("error !!! ")
                #print("after payload_args ++")
            self.byte_count_ += 1
            # print "byte:"+str(byte_count_) +","+ "rece_len:"+str(receive_message_length_)

            if self.byte_count_ >= self.receive_message_length_:
                print("-------------------------")
                self.receive_state_ = self.RECEIVE_CHECK
        elif self.receive_state_ == self.RECEIVE_CHECK:
            #if(rx_data == (unsigned char)receive_check_sum_):
            if 1:
                self.receive_state_ = self.WAITING_FF
                # print str(binascii.b2a_hex(value))
                # left, right, = struct.unpack('hh', value)
                # print "left:"+str(left)+", right:"+str(right)
                return 1
            else:
                self.receive_state_ = self.WAITING_FF
        else:
            print("return 1")
            self.receive_state_ = self.WAITING_FF;
        return 0

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        c = self.port.read(1)
        #print(str(binascii.b2a_hex(c)))
        while self.receiveFiniteStates(c) != 1:
            try:
                c = self.port.read(1)
                #print(str(binascii.b2a_hex(c)))
            except:
                print("Exception Read error " )
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return 0
        return 1

    def recv_ack(self):
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def execute(self, cmd):
        self.mutex.acquire()
        try:
            self.port.flushInput()
        except:
            pass
        ntries = 1
        attempts = 0
        try:
            print("1----------------------------------------------------")
            self.port.write(cmd)
            res = self.recv(self.timeout)
            print("2----------------------------------------------------")
            while attempts < ntries and res != 1:
                try:
                    self.port.flushInput()
                    self.port.write(cmd)
                    res = self.recv(self.timeout)
                except:
                    print("Exception executing command1: " + str(binascii.b2a_hex(cmd)))
                attempts += 1
        except:
            self.mutex.release()
            print("Exception executing command2: " + str(binascii.b2a_hex(cmd)))
            return 0

        self.mutex.release()
        return 1

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        cmd_str = struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x00) + struct.pack("B", 0x01)
        print("start exec cmd __get baud")
        if (self.execute(cmd_str)) == 1 and self.payload_ack == b'\x00':
            #self.payload_args = bytes(self.payload_args, 'utf-8')
            print("self.payload_args = ", self.payload_args)

            val, = struct.unpack('I', self.payload_args)
            return self.SUCCESS, val
        else:
            return self.FAIL, 0

    def get_encoder_counts(self):
        cmd_str = struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x02) + struct.pack("B", 0x03)
        if (self.execute(cmd_str)) == 1 and self.payload_ack == b'\x00':
            # left_enc,right_enc, = struct.unpack('hh', self.payload_args)
            left_enc, right_enc, = struct.unpack('HH', self.payload_args)
            return self.SUCCESS, left_enc, right_enc
        else:
            return self.FAIL, 0, 0

    def reset_encoders(self):
        cmd_str = struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x03) + struct.pack("B", 0x04)
        if (self.execute(cmd_str)) == 1 and self.payload_ack == b'\x00':
            return self.SUCCESS
        else:
            return self.FAIL

    def get_check_sum(self, list):
        list_len = len(list)
        cs = 0
        for i in range(list_len):
            # print i, list[i]
            cs += list[i]
        cs = cs % 255
        return cs

    def drive(self, left, right):
        data1 = struct.pack("h", left)
        d1, d2 = struct.unpack("BB", data1)
        #	print "left:", left, "right:", right
        data2 = struct.pack("h", right)
        c1, c2 = struct.unpack("BB", data2)

        self.check_list = [0x05, 0x04, d1, d2, c1, c2]
        self.check_num = self.get_check_sum(self.check_list)
        cmd_str = struct.pack("4B", self.HEADER0, self.HEADER1, 0x05, 0x04) + struct.pack("hh", left,right) + struct.pack("B",
         self.check_num)
        if (self.execute(cmd_str)) == 1 and self.payload_ack == b'\x00':
            return self.SUCCESS
        else:
            return self.FAIL

    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)

""" Class to receive Twist commands and publish Odometry data """

###########################################################################################


def Close_Serial():
    SerialPort.close()

def Open_Serial():
	global values
	haha=0
    # if not SerialPort.isOpen():
    #     SerialPort.port = Port_list.get()
    #     SerialPort.baudrate = BaudRate_list.get()
    #     SerialPort.timeout = 0.1
    #     #SerialPort.open()

    #     #if SerialPort.isOpen():
    #     t = threading.Thread(target=Poll)
    #     t.setDaemon(True)
    #     t.start()
    # else:
    #     SerialPort.close()
	while(1):
	    valueRead = serialArduino.readline()
	    print(valueRead)
	    valueInInt = float(valueRead)
	    values.append(valueInInt)
	    haha+=1
	    if(haha==300):
	    	haha=0
	    	drawnow(plotValues)
	    	values=[]
	    
def Poll():
    #SerialPort.open()
    myArduino = Arduino(SerialPort.port, 115200, 0.5)
    myArduino.connect()

    while True:
        #Receive_Window.insert("end", str(SerialPort.readline()) + '\n')
        #Receive_Window.see("end")
        global Port_t
        Port_t.set(myArduino.baudrate)
        if(Port_t.get() > 100000):
            Port_t.set(Port_t.get()+1)
        time.sleep(1)  # 休眠0.1秒
tk.Button(switch, text="开始采集", command=Open_Serial).pack(side="left", padx=13)
tk.Button(switch, text="停止采集", command=Close_Serial).pack(side="right", padx=13)

def plotValues():
    plt.title('Serial Advalue from stm32')
    plt.grid(True)
    plt.ylabel('temperature')
    plt.plot(values,'rx-', label='AD')
    plt.legend(loc='upper right')

values = []

serialArduino = serial.Serial('COM3', 115200)
if __name__ == '__main__':
	GUI.mainloop()
