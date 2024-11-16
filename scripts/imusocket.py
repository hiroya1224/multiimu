#!/usr/bin/env python3
# ref https://qiita.com/t_katsumura/items/a83431671a41d9b6358f

import socket
import numpy as np
import base64

class BaseClient:
    def __init__(self, timeout:int=10, buffer:int=1024):
        self.__socket = None
        self.__address = None
        self.__timeout = timeout
        self.__buffer = buffer
        self.dt = 0.

    def connect(self, address, family:int, typ:int, proto:int):
        self.__address = address
        self.__socket = socket.socket(family, typ, proto)
        self.__socket.settimeout(self.__timeout)
        self.__socket.connect(self.__address)

    def disconnect(self):
        try:
            self.__socket.shutdown(socket.SHUT_RDWR)
            self.__socket.close()
        except:
            pass

    def send(self, message) -> None:
        self.__socket.send(message.encode('utf-8'))

    def received(self):
        try:
            result = self.__socket.recv(self.__buffer).decode('utf-8')
        except OSError as e:
            print(e)
            return ""
        
        return result


class ImuDataSocketClient(BaseClient):
    ## ref: https://www.mouser.com/datasheet/2/783/BST_BMX055_DS000-1509552.pdf
    ACC_SCALE = {0: 2.  / 32768. * 9.80665,
                 1: 4.  / 32768. * 9.80665,
                 2: 8.  / 32768. * 9.80665,
                 3: 16. / 32768. * 9.80665}
    GYR_SCALE = {3: 2000 / 32768. * 0.01745329251,
                 2: 1000 / 32768. * 0.01745329251,
                 1: 500  / 32768. * 0.01745329251,
                 0: 250  / 32768. * 0.01745329251,}
    
    def __init__(self, host:str="0.0.0.0", port:int=5000) -> None:
        self.server = (host,port)
        self.t0 = None
        super().__init__(timeout=60, buffer=65536)
        super().connect(self.server, socket.AF_INET, socket.SOCK_STREAM, 0)
    

    @staticmethod
    def parse_sensor_data_unit(msb_str, lsb_str, lsb_shift):
        msb = int(msb_str, 16)
        lsb = int(lsb_str, 16)
        msb_shift = 8 - lsb_shift
        raw = (msb << msb_shift) + (lsb >> lsb_shift)

        ## sign
        bit_length = msb_shift + 8
        if (raw >> (bit_length - 1)):
            raw = raw - (1 << bit_length)

        return raw


    def parse_single_data(self, raw_data: str):
        result = [0,0,0,0,0,0]
        lsb_shifts = [0,0,0,0,0,0]
        for i in range(6):
            result[i] = self.parse_sensor_data_unit(raw_data[4*i : 4*i+2],
                                                    raw_data[4*i+2 : 4*i+4],
                                                    lsb_shifts[i])

        acc_scale = self.ACC_SCALE[int(raw_data[31], 16) & 0x03]
        gyr_scale = self.GYR_SCALE[int(raw_data[31], 16) >> 2]
        mac_address = raw_data[24:30]

        acc = np.array(result[0:3]) * acc_scale
        gyr = np.array(result[3:6]) * gyr_scale
        # mag = np.array(result[6:9])

        return {"acc": acc, "gyr": gyr, "mac": mac_address}
    

    def parse_received_message_base64(self, message_recv:str):
        dataset = []
        # print(message_recv)
        for m in message_recv.split("|")[:-1]:
            try:
                _timestamp, _data = m[:11], m[11:]
                timestamp = int(base64.b64decode((_timestamp + "=").encode()).hex(), 16)
                # data = base64.b64decode((_data + "".join(["=" for _ in range(
                #     (4 - (len(_data) % 4)) % 4
                # )])).encode()).hex()

                ## assertion check
                assert len(_data) == 22, "len(_data) == {} != 22".format(len(_data))

                data = base64.b64decode((_data + "==").encode()).hex()
                if self.t0 is None:
                    self.t0 = timestamp
                if timestamp < self.t0:
                    raise ValueError
                
                dataset.append([timestamp, self.parse_single_data(data)])

            except (ValueError, KeyError, AssertionError) as e:
                print(e)
                continue
        return dataset
    

    def received_data(self):
        message_recv = self.received()
        dataset = self.parse_received_message_base64(message_recv)
        return dataset
    
    def received_data_as_dict(self):
        message_recv = self.received()
        dataset = self.parse_received_message_base64(message_recv)
        return [{
                    "timestamp": d[0] * 1e-9,
                    "data": {
                        "acc": d[1]["acc"].tolist(),
                        "gyr": d[1]["gyr"].tolist(),
                        "mac": d[1]["mac"],
                    }
                } for d in dataset]
    
    def send_registered_signal(self):
        self.send("1")


class DataContainer:
    def __init__(self, length=100):
        # self.axes = {"acc": self.fig.add_subplot(3,1,1),
        #              "gyr": self.fig.add_subplot(3,1,2),
        #              "mag": self.fig.add_subplot(3,1,3)}
        
        self.length = length
        self.t0 = None
        self.tlist = [0. for _ in range(self.length)]
        # self.dlist = {"acc": [np.zeros(3) for _ in range(length)],
        #               "gyr": [np.zeros(3) for _ in range(length)],
        #               "mag": [np.zeros(3) for _ in range(length)]}
        self.dlist = {"acc": [np.zeros(3) for _ in range(length)],
                      "gyr": [np.zeros(3) for _ in range(length)]}
        
    def update(self, datalist):
        for data in datalist:
            _t, d = data
            t = _t / 10**9
            if self.t0 is None:
                self.t0 = t
            self.tlist.append(t - self.t0)
            self.tlist.pop(0)
            for k in ["acc", "gyr"]:
                self.dlist[k].append(d[k])
                self.dlist[k].pop(0)
