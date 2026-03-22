from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QDockWidget, QFrame, QSplitter, QHBoxLayout
from pyqtgraph import PlotWidget
from PySide6.QtCore import Qt, QTimer   
import pyqtgraph as pg

import numpy as np
from npy_append_array import NpyAppendArray

count = 0

class EcoDrive(QWidget):
    def __init__(self, aebfStream, elecTele_bufferSize=5000):
        super().__init__()
        self.setWindowTitle("EcoDrive V0")
        layout = QVBoxLayout()

        vsplitter_left = QSplitter(Qt.Vertical)
        vsplitter_right = QSplitter(Qt.Vertical)
        hsplitter = QSplitter(Qt.Horizontal)
        text = QLabel("EcoDrive V0 - Graphs and Controls")
        vsplitter_left.addWidget(text)    

        self.graph_elecTele = PlotWidget()
        vsplitter_right.addWidget(self.graph_elecTele)
    
        # Bottom widget: controls
        controls_frame = QFrame()
        controls_layout = QVBoxLayout(controls_frame)
        controls_layout.addWidget(QLabel("Controls"))
        self.toggle_elecTele_button = QPushButton("Start elecTele Log")
        self.toggle_elecTele_button.clicked.connect(self.toggle_elecTeleFileLog)
        controls_layout.addWidget(self.toggle_elecTele_button)



        vsplitter_right.addWidget(controls_frame)  # bottom half

        hsplitter.addWidget(vsplitter_left)  # left half
        hsplitter.addWidget(vsplitter_right)
        layout.addWidget(hsplitter)  # right half

        aebfStream.register_service(0xC1, self.feed_electricalTele_xC1)
        self.setLayout(layout)

        self.ELECTELE_CHANNELS = 5
        self.ELECTELE_SAMPLES_PER_FRAME = 24
        
        self.elecTele_bufferHead = 0
        self.elecTele_fileLogNew = 0
        self.elecTele_fileLog = False
        self.elecTele_saveFile = None
        self.elecTele_samplenum = np.zeros((elecTele_bufferSize,), dtype=np.uint32)  # To store frame indices
        self.elecTele_data = np.zeros((elecTele_bufferSize, self.ELECTELE_CHANNELS), dtype=np.float32)  # 12 samples, 5 values each
        self.elecTele_plotChannels = []
        colors = ['blue', 'red', 'green', 'yellow', 'white']
        for i in range(self.ELECTELE_CHANNELS):
            self.elecTele_plotChannels.append(self.graph_elecTele.plot(self.elecTele_data[:, i], pen=pg.mkPen(colors[i], width=3)))
        # Timer for graph updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)  # update every 50 ms

        self.phase = 0

    def connected(self):
        print("EcoDrive connected")
        return True

    def update(self):
        elec_plot = np.roll(self.elecTele_data, -self.elecTele_bufferHead, axis=0)  # Roll to align head at index 0
        samplenum = np.roll(self.elecTele_samplenum, -self.elecTele_bufferHead)  # Align sample numbers as well
        for i in range(elec_plot.shape[1]):
            self.elecTele_plotChannels[i].setData(elec_plot[:400,  i])
        if(self.elecTele_fileLog):
            # Write new data to file
            if self.elecTele_fileLogNew:
                combined = np.column_stack((samplenum[self.elecTele_bufferHead - self.elecTele_fileLogNew:self.elecTele_bufferHead], elec_plot[self.elecTele_bufferHead - self.elecTele_fileLogNew:self.elecTele_bufferHead, :]))
                self.elecTele_saveFile.append(combined)
                self.elecTele_fileLogNew = 0

    def toggle_elecTeleFileLog(self):
        if self.elecTele_fileLog:
            self.elecTele_saveFile.close()
            self.elecTele_fileLog = False
            self.toggle_elecTele_button.setText("Start elecTele Log")
        else:
            self.elecTele_saveFile = NpyAppendArray("elecTele_log.npy", delete_if_exists=True)
            self.elecTele_fileLog = True
            self.toggle_elecTele_button.setText("Stop elecTele Log")

    def feed_electricalTele_xC1(self, deviceID, serviceID, payload):
        global count
        frame_index = int.from_bytes(payload[:4], 'little', signed=False)

        data_array = np.frombuffer(payload[4:], dtype=np.int16)
        frame = data_array.reshape((self.ELECTELE_SAMPLES_PER_FRAME,
                                self.ELECTELE_CHANNELS))

        N = self.ELECTELE_SAMPLES_PER_FRAME
        size = self.elecTele_data.shape[0]
        head = self.elecTele_bufferHead

        end = head + N
        self.elecTele_fileLogNew += N

        if end <= size:
            # no wrap
            self.elecTele_data[head:end, :] = frame
            self.elecTele_samplenum[head:end] = np.arange(frame_index,
                                                          frame_index + N)
        else:
            # wrap
            first = size - head
            second = N - first

            self.elecTele_data[head:size, :] = frame[:first]
            self.elecTele_data[0:second, :] = frame[first:]

            self.elecTele_samplenum[head:size] = np.arange(frame_index,
                                                           frame_index + first)
            self.elecTele_samplenum[0:second] = np.arange(frame_index + first,
                                                          frame_index + N)

        self.elecTele_bufferHead = (head + N) % size


    def feed_mechanicalTele_xC2(self, deviceID, serviceID, payload):
        print("EcoDrive graph_mechTele")

    