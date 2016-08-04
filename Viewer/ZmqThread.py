from PySide import QtCore
import json
import zmq
import logging
import numpy as np

class ZmqThread(QtCore.QThread):
    data_ready = QtCore.Signal(float, float, object)
    fov_ready = QtCore.Signal(int, object, object) # corrisponde a intero, id e lettera T o A
    #aggiunto
    data_benefit = QtCore.Signal(float, object) # corrisp al valore di benefit e alla B
    data_potential = QtCore.Signal(float, object)
    data_coveredsquare = QtCore.Signal(float, object)
    data_payoff = QtCore.Signal(float)

    def __init__(self, sim_file):
        #print 'nome del file cambiato : ', sim_file
        """
            Constructor of the ProdifconThread.
            At this stage the principal args from the configuration file are parsed
            :param sim_file:
        """
        super(ZmqThread, self).__init__()
        self.agentData = {}    
        self.sim_file = sim_file
        self.area_data = []
        with open(self.sim_file) as s_f:
            data = json.load(s_f)            
            self.area_data = [data["Area"]["height"], data["Area"]["length"]] # [y_lim, x_lim] of the area
            sensors = data["Agents"]["sensors"]
            for i, val in enumerate(sensors):
                self.agentData[i] = [sensors[i][0], sensors[i][3]] # [BigRadius, FovAngle_Rad]

    def initStage(self):
        return self.area_data

    def initBenefit(self):
        return self.data_benefit

    def initPotential(self):
        return self.data_potential

    def initCoveredSquare(self):
        return self.data_coveredsquare
    
    def run(self):
        context = zmq.Context(1)
        sock = context.socket(zmq.PAIR)
        sock.bind("tcp://*:5555")
        print("ZMQ connection successfull")
        while True:
            message = sock.recv()
            if len(message) == 0:
                continue

            message_vec = message.split(',')
            if message_vec[0] == 'A':
                id = int(message_vec[1])
                x_pos = float(message_vec[2])
                y_pos = float(message_vec[3])
                heading = float(message_vec[4])
                self.data_ready.emit(x_pos, y_pos, "A_{}".format(id))
                # Calculate FOV
                fov_s = heading - self.agentData[id][1] / 2
                fov_e = heading + self.agentData[id][1] / 2
                t = np.linspace(fov_s, fov_e, 50)

                x = np.append([0], self.agentData[id][0] * np.sin(t))
                y = np.append([0], self.agentData[id][0] * np.cos(t))
                x = np.append(x, [0])
                y = np.append(y, [0])
                x = x + x_pos
                y = y + y_pos
                self.fov_ready.emit(id, x, y)
                
            if message_vec[0] == 'T':
                id = int(message_vec[1])
                x_pos = float(message_vec[2])
                y_pos = float(message_vec[3])
                self.data_ready.emit(x_pos, y_pos, "T_{}".format(id))

            if message_vec[0] == 'B':
                benefit = float(message_vec[1])
                #print benefit
                self.data_benefit.emit(benefit, "B")

            if message_vec[0] == 'P':
                potential = float(message_vec[1])
                #print potential
                self.data_potential.emit(potential, "P")
            
            if message_vec[0] == 'C':
                coveredsquare = float(message_vec[1])
                #print coveredsquare
                self.data_coveredsquare.emit(coveredsquare, "C")

            #if message_vec[0] == 'M':
                #payoff = float(message_vec[1])
                #self.data_payoff.emit(payoff, "M")
    
    def stop(self):
        return