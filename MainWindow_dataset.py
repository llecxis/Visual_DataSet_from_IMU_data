import sys
import numpy as np
import time
import socket
import traceback
# import network
import os
# from scipy import interpolate
from scipy.linalg import logm, expm

from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

import Worker
import Draw
import QtrCalc as QC
from CP_Rotation import CP_Rotation as R
from utils import trap_exc_during_debug
import CalibIMUsInQuat as CalibIMUs
import Video_maker


class MainWindow(QMainWindow):

    NUM_THREADS = 5 # number of phones
    sig_abort = pyqtSignal()
    sig_abort_workers = pyqtSignal()
    sig_abort_cams = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.title = "Система сбора, обработки и визуализации деятельности работников предприятия"
        self.top = 200
        self.left = 500
        self.width = 1000
        self.height = 800
        self.WorkerAdress = list()
        self.num_workers = 0
        self.sens_data = {} #dictionary of dicts
        self.slider_end = 1
        self.state = -1
        self.n_time = False
        self.t_time = False
        self.directory = None
        self.count = 0

        self.rigth_list = [ 5559+2*i for i in range(self.NUM_THREADS)]

        mtxBnoAtVtk = np.array([[0,0,1], [1,0,0], [0,-1,0]])
        
        N_segmentAtBody = np.array([[1,0,0], [0,1,0], [0,0,1]])

        self.rotBnoAtVtk = R.from_matrix(mtxBnoAtVtk) #rotation from BNO FoR to VTK FoR

        self.N_segmentAtBody = R.from_matrix(N_segmentAtBody)
        
        self.setWindowIcon(QtGui.QIcon("icon.png"))
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        
        
        self.createSensorsDock()  

        self.createToolBar()  

        # self.createExplorerDock()     
        
        self.createLogDockWidget()
        
        self.createCentralWidget()

        self.configureClicks()

        self.createMenu()

        self.log_text.append("Initialization Ok")


        self.ci = CalibIMUs.CalibIMUs()
        self.iCalibStage = 0
        

        self.show()
    

    def clearLog(self):
        command = self.log_text.toPlainText()
        # self.log_text.clear()
        self.log_text.setText(str(eval(command)))

    def configureClicks(self):
        self.clear_log.clicked.connect(self.clearLog)
        self.play.clicked.connect(self.vtkCall)
        self.stop.clicked.connect(self.vtkEndCall)
        self.pause.clicked.connect(self.calibrate)


        # self.pause.clicked.connect(self.vtkEnCall)

        self.tuning_n.clicked.connect(self.n_qtr_shift)
        self.tuning_z.clicked.connect(self.z_qtr_shift)
        self.tuning_t.clicked.connect(self.t_qtr_shift)

        # прописывать обработку всех кликов по кнопкам        

    def n_qtr_shift(self):

        if (self.state == 0):                        
            qtr=[1.0,0.0,0.0,0.0]
            rot = {}
            self.log_text.append('N calibration, t = ' + str(self.ti) )
            # print(self.qtrs)
            for port in self.qtrs['ports']:
                rot[port] = self.rotBnoAtVtk*R.from_quat(self.qtrs[port])
                self.log_text.append( str(port) + str(rot[port].as_quat()) )

            if ('5563' in self.qtrs) == 0:
                rot['5563'] = self.rotBnoAtVtk*R.from_quat(qtr)
                self.qtrs['5563'] = qtr
                # print(self.qtrs['5563'])

            self.iCalibStage = 1
            self.ci.setN_calibImusAtGlob(rot['5563'], rot[self.arm_l])

            for port in self.qtrs['ports']:
                if (port != '5563'):
                    self.ci.addSegmentImu(self.N_segmentAtBody, rot[port]) #FOR from visual anatomical
            self.log_text.append('N calibration: done')

        if (self.state == 1):
            rot = {}
            self.log_text.append('N calibration: ' + str(self.ti))
            for port in self.vm.qtrs:
                rot[port] = self.rotBnoAtVtk*R.from_quat(self.vm.frame[port][self.ti])
                self.log_text.append( str(port) + str(rot[port].as_quat()) )

            self.iCalibStage = 1
            self.ci.setN_calibImusAtGlob(rot[self.body], rot[self.arm_l])

            for port in self.vm.qtrs:
                if (port != self.body):                 
                    self.ci.addSegmentImu(self.N_segmentAtBody, rot[port]) #FOR from visual anatomical
            self.log_text.append('N calibration: done')

    def t_qtr_shift(self):

        rot = {}
        self.log_text.append('T calibration: ' + str(self.ti))
        if (self.state == 0):
            for port in self.qtrs['ports']:
                rot[port] = self.rotBnoAtVtk*R.from_quat(self.qtrs[port])
                self.log_text.append( str(port) + str(rot[port].as_quat()))

            self.iCalibStage = 2
            self.ci.doCalibration(rot[self.arm_l], 1, 0)
            self.flag_norm = 1
            self.log_text.append('T calibration: done')
            self.log_text.append('Push \"Play\" button')

        if (self.state == 1):
            for port in self.vm.qtrs:
                rot[port] = self.rotBnoAtVtk*R.from_quat(self.vm.frame[port][self.ti])
                self.log_text.append( str(port) + str(rot[port].as_quat()))

            self.iCalibStage = 2
            self.ci.doCalibration(rot[self.arm_l], 1, 0)
            self.flag_norm = 1
            self.log_text.append('T calibration: done')
            self.log_text.append('Push \"Play\" button')
        

    def calibrate(self):
        if self.state == 1:
            n_time = 500
            t_time = 1500

            if not(self.t_time):
                self.t_time = t_time

            if not(self.n_time):
                self.n_time = n_time
            
            self.sld.setValue(self.n_time)
            self.n_qtr_shift()
            self.sld.setValue(self.t_time)
            self.t_qtr_shift()
            self.sld.setValue(0)

        
    def qtr_to_mtx(self, qtr):
        end = np.identity(3)
        temp = vtk.vtkQuaterniond(qtr)
        temp.ToMatrix3x3(end)
        return end

    def mtx_to_qtr(self,mtx):        
        end = vtk.vtkQuaterniond(np.array([1.,0.,0.,0.]))
        # print(mtx.as_matrix())
        end.FromMatrix3x3(mtx)
        return end

    def z_qtr_shift(self):

        return 0

    def vtkCall(self):
        
        self.play.setDisabled(True)
        # self.play.setText('Pause')
        # self.stop.setEnabled(True)
        self.timer.start(self.timeStep)

    def vtkEndCall(self):
        # self.stop.setDisabled(True)
        self.play.setEnabled(True)
        self.timer.stop()
        self.sld.setValue(0)

        self.create_dataset()

    # def create_dataset(self):
    #     local_path = os.getcwd()
    #     filename = os.path.join(local_path,'dataset/' + 'points_13_19_38_0' + '.skeleton')
    #     # print(self.scene.pos_set[0].values())
    #     try:
    #         with open(filename, 'w') as the_file:
    #             the_file.write('# no of observerd skeletons in current frame\n# num sceletons\n# tracking id of the skeleton, clipedEdges, handLeftConfidence, handLeftState, handRightConfidence, handRightState, isResticted, leanX, leanY, trackingState\n# num jointCount\n# x, y, z, depthX, depthY, colorX, colorY, orientationW, orientationX, orientationY, orientationZ, trackingState\n110011\n')
    #             for key in self.scene.pos_set[0].keys():
    #                 the_file.write('1\n' + str(int(key)) + ' skelet ID 1 1 1 1 1 0 -0.163 -0.44 1\n19\n')
    #                 i=0
    #                 for dic in self.scene.pos_set:                        
    #                     if (i != 7) and (i != 8) and (i != 13) and (i != 14) and (i != 15)  and (i != 16) and (i != 23)  and (i != 24) and (i != 27) and (dic != {}):
    #                         # print(dic[key])
    #                         the_file.write('\t'.join('{:<10f}'.format(el) for el in list(dic[key][0:3]))) 
    #                         the_file.write('\t'.join('{:<4}'.format(int(el)) for el in list(dic[key][4:-4])))
    #                         the_file.write('\t'.join('{:<10f}'.format(el) for el in list(dic[key][-4:])))
    #                         the_file.write('\t2\n')
                    
    #                     i+=1
    #                 # the_file.write('\n')
    #     except:
    #         print("Mistake",key,dic,i) 

    def create_dataset(self):
        pass
    #     local_path = os.getcwd()
    #     # print(self.directory[-21:])
    #     filename = os.path.join(local_path,'dataset/' + self.directory[-21:] + '.csv')

    #     # elems = list(self.vm.frame.keys())

    #     try:
    #         with open(filename, 'w') as the_file:
    #             the_file.write('time,' + ','.join(el for el in self.vm.frame.keys()) +'\n')
    #             i=0
    #             for t in self.vm.time:
    #                 the_file.write('{:<2f}'.format(t) + ',' )                    
    #                 for elem in  self.vm.frame.keys():
    #                     # print(i)
                        
    #                     # temp = R.from_quat(self.vm.frame[elem][i])
    #                     # temp_0 = R.from_quat(self.vm.frame['5563'][i])
    #                     # log_temp = logm(temp)
                        
    #                     the_file.write(','.join('{:<8f}'.format(el) for el in self.vm.frame[elem][i]))
    #                     # the_file.write('\t'.join('{:<4}'.format(int(el)) for el in list(dic[key][4:-4])))
    #                     # the_file.write('\t'.join('{:<10f}'.format(el) for el in list(dic[key][-4:])))
    #                     if elem!= '5571':
    #                         the_file.write(',')
    #                 i+=1
    #                 the_file.write('\n')
    #     except:
    #         print("Mistake") 
    #     print('File written')

    def KeyPress(self,obj, event):
        import re
        key = obj.GetKeySym() #works fine

        global k
        if ( re.match(r'\d', str(key) ) ):
            k = int(key) + 0
            self.log_text.append(self.obj_list[k] + '  ' + str(k))
        if (key == "Left") and (self.state == 1):
            self.sld.setValue(self.sld.value()-10)
        if (key == "Right") and (self.state == 1):
            self.sld.setValue(self.sld.value()+10)
        if (key == "Up"):
            self.scene.init_pos_actor (k,np.array([0.001,0.000,0.000]))
            self.iren.Render()
        if (key == "Down"):
            self.scene.init_pos_actor (k,np.array([-0.001,0.000,0.000]))
            self.iren.Render()
        if (key == "q"):
            self.scene.init_pos_actor (k,np.array([0.000,0.001,0.000]))
            self.iren.Render()
        if (key == "a"):
            self.scene.init_pos_actor (k,np.array([0.000,-0.001,0.000]))
            self.iren.Render()
        if (key == "e"):
            self.scene.init_pos_actor (k,np.array([0.000,0.000,0.001]))
            self.iren.Render()
        if (key == "d"):
            self.scene.init_pos_actor (k,np.array([0.000,0.000,-0.001]))
            self.iren.Render()
        if (key == "b"):
            print(key)
            self.z_qtr_shift()        
        if (key == "Escape"):
            self.abort_cam()
            self.abort_workers()
        return 0
        
    def createCentralWidget(self):

        self.frame = QFrame()
        self.vl = QGridLayout()
        
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
        
        self.visualWidget = QWidget(self)
        self.play = QPushButton("play", self.visualWidget)
        self.stop = QPushButton("stop", self.visualWidget)
        

        self.tuning_n = QPushButton("N - pose", self.visualWidget)
        self.tuning_t = QPushButton("T - pose", self.visualWidget)
        
        self.tuning_z = QPushButton("Recording", self.visualWidget)
        self.pause = QPushButton("Calibrate", self.visualWidget)

        self.vl.addWidget(self.vtkWidget,1,1)
       
        buttons_layout = QVBoxLayout()
        buttons_layout.addStretch(1)
        buttons_layout.addWidget(self.pause)
        buttons_layout.addWidget(self.play)
        buttons_layout.addWidget(self.stop)       
        buttons_layout.addStretch(1)
        buttons_layout.addWidget(self.tuning_n)
        buttons_layout.addWidget(self.tuning_t)
        buttons_layout.addStretch(1)        
        buttons_layout.addWidget(self.tuning_z)        
        buttons_layout.addStretch(1)        

        #Create vtk obj
        self.scene = Draw.vtpDrawScene()
        directory = 'geometry/'
        obj = [directory + el for el in os.listdir(directory)]
        rigth_list = ['geometry/hat_jaw.vtp', 'geometry/hat_skull.vtp', 'geometry/hat_spine.vtp', 'geometry/humerus.vtp', 'geometry/humerus_l.vtp', 'geometry/radius_lv.vtp', 'geometry/radius_rv.vtp', 'geometry/scapula.vtp', 'geometry/scapula_l.vtp', 'geometry/thorax.vtp', 'geometry/ulna_lv.vtp', 'geometry/ulna_rv.vtp','geometry/sacrumv.vtp','geometry/pelvis_rv.vtp','geometry/pelvis_lv.vtp','geometry/femur_r.vtp','geometry/femur_l.vtp','geometry/fibula_l.vtp','geometry/fibula_r.vtp', 'geometry/tibia_r.vtp', 'geometry/tibia_l.vtp','geometry/talus_r.vtp','geometry/talus_l.vtp','geometry/foot_r.vtp','geometry/foot_l.vtp','geometry/bofoot_r.vtp','geometry/bofoot_l.vtp']
        # print(len(rigth_list))
        if list(set(obj) ^ set(rigth_list)) != []:
            temp = list(set(obj) ^ set(rigth_list))
            for el in temp:
                rigth_list.append(el)
        self.obj_list = rigth_list
        self.ren = self.scene.initScene_qt(self.obj_list)
        self.initial_qtr_norm()
        

        #Settings
        self.ren.SetBackground(0.2, 0.2, 0.2)
        self.timeStep = 20 #ms
        self.total_t = 0
        
        # #check for phones
        # self.start_threads()

        self.renWin = self.vtkWidget.GetRenderWindow()
        self.renWin.AddRenderer(self.ren)  #02.04.2020

        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.iren.AddObserver("KeyPressEvent", self.KeyPress)
        
        # Slider
        self.sld = QSlider(Qt.Horizontal, self)
        self.sld.setFocusPolicy(Qt.NoFocus)
        self.sld.setGeometry(30, 40, 100, 30)
        self.sld.setValue(0)
        self.sld.valueChanged[int].connect(self.changeValue)        
        self.vl.addWidget(self.sld,2,1,2,1)
           

        self.label = QLabel("time")
        self.vl.addWidget(self.label,2,2)   

        self.vl.addLayout(buttons_layout,1,2)
        self.visualWidget.setLayout(self.vl)

        self.frame.setLayout(self.vl)
        self.setCentralWidget(self.frame)
        self.iren.Initialize()

        # self.label = QLabel(self)
        # self.label.setPixmap(QPixmap('mute.png'))
        # self.label.setGeometry(160, 40, 80, 30)

        # Create Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.timerCallback)
        
    def changeValue(self, value):
        self.ti = value
        self.label.setText(str(round(self.vm.time[self.ti],4)))
        # print(value, self.vm.time[self.ti])  

    def timerCallback(self): #сюда преобразование координат qweqrty
        
        try:
            if (self.state == 0 ) and (self.flag_norm == 1):
                for el in range(len(self.scene.modelActor)):
                    
                    temp_shift = np.array([0.,0.,0.])
                    self.motion_flag = np.array([0,[1.,0.,0.,0.]])
                    temp_qtr =  self.ci.calcBodyAtGlob(self.rotBnoAtVtk*R.from_quat(self.qtrs['5563'])) #* self.rotBox0AtIMU0                смена x и y- self.rotBnoAtVtk_norm                            калибровка self.rotBox0AtIMU0 * 
                    
                    if (el == 4):
                        temp_qtr_2 = self.ci.calcSegmentAtBody(0,self.rotBnoAtVtk*R.from_quat(self.qtrs['5571']),temp_qtr)
                        self.motion_flag =  [1, temp_qtr_2.as_quat()]

                    if (el == 5) or (el == 10):
                        temp_qtr_2 = self.ci.calcSegmentAtBody(0,self.qtrs[5569],temp_qtr)
                        temp_shift = self.shift_calculus(4,el,self.ci.calcSegmentAtBody(0,self.qtrs[5571],temp_qtr).as_matrix())
                        self.motion_flag =  [1, temp_qtr_2.as_quat()]

                    if (el == 3):                   
                        temp_qtr_2 = self.ci.calcSegmentAtBody(0,self.qtrs[5561],temp_qtr)
                        self.motion_flag =  [1, self.RqtrTovtkqtr(temp_qtr_2.as_quat())]

                    if (el == 6) or (el == 11):
                        temp_qtr_2 = self.ci.calcSegmentAtBody(0,self.qtrs[5559],temp_qtr)
                        temp_shift = self.shift_calculus(4,el,self.ci.calcSegmentAtBody(0,self.qtrs[5561],temp_qtr).as_matrix())
                        self.motion_flag =  [1, self.RqtrTovtkqtr(temp_qtr_2.as_quat())]
                    
                    self.scene.SetRefQuatOrientation(np.array(temp_qtr.as_quat()), temp_shift, el, self.motion_flag )
                    # self.scene.SetRefQuatOrientation_arrowshow(np.array(temp_qtr.as_quat()), temp_shift, el, self.motion_flag, time.time() )

                self.iren.Render() 

            if  (self.state == 1) and (self.flag_norm == 1):
                
                if self.ti == self.slider_end:
                    # self.timer.stop()
                    self.sld.setValue(0)
                    self.play.setEnabled(True)
                    # self.play.setText('Play')

                if self.play.isEnabled():                    
                    self.sld.setValue(self.sld.value() + 5)

                self.qtr63 = self.rotBnoAtVtk*R.from_quat(self.vm.frame[self.body][self.ti])
                self.qtr61 = self.rotBnoAtVtk*R.from_quat(self.vm.frame[self.arm_r][self.ti]) # humerus_r
                self.qtr59 = self.rotBnoAtVtk*R.from_quat(self.vm.frame[self.forearm_r][self.ti]) # ulna_r + radius_r
                self.qtr71 = self.rotBnoAtVtk*R.from_quat(self.vm.frame[self.arm_l][self.ti]) # humerus_l
                self.qtr69 = self.rotBnoAtVtk*R.from_quat(self.vm.frame[self.forearm_l][self.ti]) # ulna_l + radius_l
                               

                for el in range(len(self.scene.modelActor)):
                    # print([2, 1])
                    temp_shift = np.array([0.,0.,0.])
                    temp_qtr_2 = None
                    self.motion_flag = np.array([0,[1.,0.,0.,0.]])   
                    temp_qtr =  self.ci.calcBodyAtGlob(self.qtr63) 
                    
                    if (el == 4):              
                        temp_qtr_2 = self.ci.calcSegmentAtBody(0,self.qtr71,temp_qtr)
                        self.motion_flag =  [1, np.array(temp_qtr_2.as_quat())]

                    if (el == 5) or (el == 10):
                        temp_qtr_2 = self.ci.calcSegmentAtBody(0,self.qtr69,temp_qtr)
                        temp_shift = self.shift_calculus(4,el,np.array(self.ci.calcSegmentAtBody(0,self.qtr71,temp_qtr).as_matrix()))
                        self.motion_flag =  [1, np.array(temp_qtr_2.as_quat())]

                    if (el == 3) :
                        temp_qtr_2 = self.ci.calcSegmentAtBody(0,self.qtr61,temp_qtr)
                        self.motion_flag =  [1, np.array(temp_qtr_2.as_quat())]

                    if (el == 6) or (el == 11):
                        temp_qtr_2 = self.ci.calcSegmentAtBody(0,self.qtr59,temp_qtr)

                        temp_shift = self.shift_calculus(3,el,np.array(self.ci.calcSegmentAtBody(0,self.qtr61,temp_qtr).as_matrix()))
                        self.motion_flag =  [1, np.array(temp_qtr_2.as_quat())]
                    
                    self.scene.SetRefQuatOrientation(np.array(temp_qtr.as_quat()), temp_shift, el, self.motion_flag)                    

                    temp_qtr_data = temp_qtr
                    if temp_qtr_2 != None:
                        temp_qtr_data = temp_qtr*temp_qtr_2

                    self.scene.SetRefQuatOrientation_arrowshow(np.array(temp_qtr.as_quat()), temp_shift, el, self.motion_flag, self.ti,temp_qtr_data.as_quat() )

                self.iren.Render()
            else:
                pass
        except:
            print("No worker", self.ti,el) 
    
    def shift_calculus(self,i_from_actor,i_to_actor, matrix):
        return  np.array(self.scene.modelActor[i_from_actor].GetPosition()) - np.array(self.scene.modelActor[i_to_actor].GetPosition()) + np.array(matrix).dot(np.array(self.scene.initial_pos_actors[i_to_actor]) - np.array(self.scene.initial_pos_actors[i_from_actor]))
    
    # def RqtrTovtkqtr(self, qtr):
    #     qtr = np.array([qtr[3],qtr[0],qtr[1],qtr[2]])
    #     return qtr

    def initial_qtr_norm(self):
        self.n_pos_temp_qtr = np.array([0.95, 0., 0.25, 0.])
        self.t_pos_temp_qtr = np.array([0.5, 0.5, 0.5, 0.5])
        self.z_pos_temp_qtr = np.array([0.684329, 0., 0., 0.713657])
        self.X_qtr = np.array([1, 0., 0., 0.])
        self.Y_qtr = np.array([1, 0., 0., 0.])
        self.Z_qtr = np.array([1, 0., 0., 0.])
        self.ZYX_qtr = np.array([1, 0., 0., 0.])
        self.N_arm_imu = np.array([1, 0., 0., 0.])
        self.check = np.array([1.,0.,0.,0.])
        self.eps = 0.01
        self.flag_norm = 0
        self.flag_video = 0
        self.video_show = 0
        self.motion_flag = list()
    
    def start_threads(self):

        self.__workers_done = 0
        self.__threads = []
        self.__threadsstatus = []
        self.stat = {}
        self.qtrs = {}
        self.qtrs_pure = {}
        self.shifts = {}
        self.qtrs['ports'] = []
        # self.a = [0,1,2,3]
        port = 5563
        for idx in range(self.NUM_THREADS):
            worker = Worker.Worker(idx, port)
            thread = QThread()
            thread.setObjectName('thread_' + str(idx))
            self.__threads.append((thread, worker))  # need to store worker too otherwise will be gc'd
            self.__threadsstatus.append(self.sensor_im_1) #нужно продумать как именно добавлять статусы многопоточности в виджите
            self.stat[str(port)] = -1
            # self.qtrs.append([1.,0.,0.,0.])
            # self.shifts.append([0.,0.,0.])
            worker.moveToThread(thread)

            worker.sig_shifts.connect(self.on_worker_shifts)
            worker.sig_qtr.connect(self.on_worker_qtr)
            worker.sig_status.connect(self.on_worker_status)
            worker.sig_msg.connect(self.log_text.append)

            # control worker:
            self.sig_abort_workers.connect(worker.abort)

            # get read to start worker:
            # self.sig_start.connect(worker.work)  # needed due to PyCharm debugger bug (!); comment out next line
            thread.started.connect(worker.work) #(self.port)
            # print(port)
            port = port + 2
            thread.start()  # this will emit 'started' and start thread event loop
            
    @pyqtSlot(str, list)
    def on_worker_shifts(self, worker_id: str, data: list):
        # self.log_text.append('Worker #{}: {}'.format(worker_id, data))
        # self.progress.append('{}: {}'.format(worker_id, data))
        # print(data)
        try: 
            self.shifts[worker_id] = data
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            traceback.print_exc()
    
    @pyqtSlot(str, list)
    def on_worker_qtr(self, worker_id: str, data: list):
        try: 
            self.qtrs_pure[worker_id] = data
            # print(data)
            self.qtrs[worker_id] = np.array(data) #cicle(self,data) data # self.rotBnoAtVtk.inv()

        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            traceback.print_exc()

    @pyqtSlot(str, int)
    def on_worker_status(self, worker_id: str, flag: int):
        if (self.stat[worker_id] == flag):
            pass
        elif (flag == 1):
            self.stat[worker_id] = flag
            self.sensor_im_1.setText('Active')
            self.log_text.append('Sensor ' + worker_id + ' is Active')
            self.qtrs['ports'].append(worker_id)
        else:
            self.stat[worker_id] = flag
            self.sensor_im_1.setText('Waiting')
            self.log_text.append('Connect sensor # ' + worker_id + '. Waiting...')
        self.__workers_done += 1

    @pyqtSlot()
    def abort_workers(self):
        self.log_text.append('Asking each sensor to abort')
        self.sig_abort_workers.emit()
        for thread, worker in self.__threads:  # note nice unpacking by Python, avoids indexing
            thread.quit()  # this will quit **as soon as thread event loop unblocks**            
            thread.wait()  # <- so you need to wait for it to *actually* quit
    
    @pyqtSlot()   
    def abort(self):
        self.sig_abort.emit()
        # self.log.append('Asking each worker to abort')
        for thread, task in self.__threads:  # note nice unpacking by Python, avoids indexing
            thread.quit()  # this will quit **as soon as thread event loop unblocks**
            thread.wait()  # <- so you need to wait for it to *actually* quit

        # even though threads have exited, there may still be messages on the main thread's
        # queue (messages that threads emitted before the abort):
        # self.log.append('All threads exited')

    def exit(self):
        App.quit()

    def restoreWindows(self):
        self.explorer.close()
        self.sensors.close()
        self.log.close()
        # сохранить текст из лога и записать его обратно
        self.createExplorerDock()
        self.createSensorsDock()
        self.createLogDockWidget()

    def menuActionConnect(self, name, func):
        act = QAction(name, self)
        # можно добавить иконки QIcon..
        act.triggered.connect(func)
        return act

    def createMenu(self):
        menubar = self.menuBar()
        file = menubar.addMenu("Файл")
        # file.addAction("Создать новый эксперимент")
        # file.addAction("Загрузить существующий эксперимент")
        file.addAction(self.menuActionConnect("&Выйти из приложения", exit))

        window = menubar.addMenu("Окна")
        
        window.addAction(self.menuActionConnect("Восстановить окна по умолчанию", self.restoreWindows))

    def createSensorsDock(self):
        self.sensors = QDockWidget("Состояние датчиков", self)
        self.sensors.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.TopDockWidgetArea)

        sensorsWidget = QWidget(self)

        self.sensor_im_1 = QLabel()
        self.sensor_im_1.setText("No data")
        # self.sensor_im_2 = QLabel()
        # self.sensor_im_2.setText("Image_2")
        # self.sensor_im_3 = QLabel()
        # self.sensor_im_3.setText("Image_3")

        # sensorsHBox = QHBoxLayout()
        # sensorsHBox.addWidget(self.sensor_im_1)
        # sensorsHBox.addWidget(self.sensor_im_2)
        # sensorsHBox.addWidget(self.sensor_im_3)

        flo = QFormLayout()
        e1 = QLineEdit()
        e2 = QLineEdit()

        e3 = QLineEdit()
        e4 = QLineEdit()
        e5 = QLineEdit()
        e6 = QLineEdit()
        e7 = QLineEdit()

        e3.setText('5563')
        e4.setText('5561')
        e5.setText('5567')
        e6.setText('5569')
        e7.setText('5565')

        self.textchanged_3('5563')
        self.textchanged_4('5569')
        self.textchanged_5('5565')
        self.textchanged_6('5561')
        self.textchanged_7('5567')


        
        e1.textChanged.connect(self.textchanged_1)
        e2.textChanged.connect(self.textchanged_2)

        e3.textChanged.connect(self.textchanged_3)
        e4.textChanged.connect(self.textchanged_4)
        e5.textChanged.connect(self.textchanged_5)
        e6.textChanged.connect(self.textchanged_6)
        e7.textChanged.connect(self.textchanged_7)

        flo.addRow("N calibration", e1)
        flo.addRow("T calibration", e2)

        flo.addRow("Body", e3)
        flo.addRow("Arm_L", e4)
        flo.addRow("Forearm_L", e5)
        flo.addRow("T Arm_R", e6)
        flo.addRow("Forearm_R", e7)

        flo.addRow("Phones",self.sensor_im_1)

        # namesHBox = QHBoxLayout()
        # namesHBox.addWidget(e4)
        # namesHBox.addWidget(QLabel("phone_2", sensorsWidget))
        # namesHBox.addWidget(QLabel("phone_3", sensorsWidget))

        # vBox = QVBoxLayout()
        # vBox.Layout(flo)

        sensorsWidget.setLayout(flo)
        
        self.sensors.setWidget(sensorsWidget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.sensors)

    def textchanged_1(self,text):
        self.n_time = int(text)
    
    def textchanged_2(self,text):
        self.t_time = int(text)
    
    def textchanged_3(self,text):
        self.body = text
    
    def textchanged_4(self,text):
        self.arm_l = text

    def textchanged_5(self,text):
        self.forearm_l = text

    def textchanged_6(self,text):
        self.arm_r = text

    def textchanged_7(self,text):
        self.forearm_r = text

    def setDirectoryToExplorer(self, directory):
        model = QFileSystemModel()
        model.setRootPath(directory)
        model.setNameFilterDisables(False)
        
        self.tree = QTreeView()
        self.tree.setModel(model)
        self.tree.setRootIndex(model.index(directory))
        self.tree.hideColumn(1)
        self.tree.hideColumn(2)
        self.tree.hideColumn(3)

    def createExplorerDock(self):
        self.explorer = QDockWidget("Файлы эксперимента", self)
        # self.explorer.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)

        sensorsWidget = QWidget(self)
        sensorsWidget.setMinimumWidth(self.width - (self.width / 1.1))

        self.setDirectoryToExplorer(QDir.currentPath())
        vBox = QVBoxLayout()
        vBox.addWidget(self.tree)

        sensorsWidget.setLayout(vBox)
        
        self.explorer.setWidget(sensorsWidget)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.explorer)


    def createToolBar(self):
        self.tools = QToolBar("Инструменты", self)
        self.tools.addSeparator()
        self.tools.setMovable(False)
        self.tools.setAllowedAreas(Qt.TopToolBarArea)
        self.tools.addAction("Записать движение", self.change_state_0)
        self.tools.addAction("Загрузить движение", self.change_state_1)
        self.addToolBar(Qt.TopToolBarArea, self.tools)

    def change_state_0(self):
        self.state = 0
        self.start_threads()
        self.log_text.append('State: Recording')
        self.log_text.append('Waiting for sensors')
        
    def change_state_1(self):
        self.state = 1
        self.log_text.append('State: Video player')

        if self.directory == None:
            self.directory = str(QFileDialog.getExistingDirectory(self, "Select Directory"))
            self.log_text.append('Curren directory: ' + self.directory )
            # print(self.directory)

        if self.state == 1:
            self.vm = Video_maker.video_player(self.directory)   
            self.slider_end = len(self.vm.time) - 1
            self.sld.setMinimum(0)
            self.sld.setMaximum(self.slider_end)
            self.sld.setTickPosition(QSlider.TicksBelow)
            self.sld.setTickInterval(1)
            self.ts = self.vm.new_times[0][0]
            self.log_text.append('Downloaded: ' + str(self.vm.obj))
        
        self.log_text.append('You need to move the slider and push \"Calibrate\" button.')

    def createLogDockWidget(self):
        self.log = QDockWidget("Лог", self)
        # self.log.setFeatures(QDockWidget.NoDockWidgetFeatures)
        # self.log.setAllowedAreas(Qt.NoDockWidgetArea)
        self.log.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.TopDockWidgetArea)

        logWidget = QWidget(self)
        logWidget.setMinimumWidth(self.width / 1.1)
        
        self.log_text = QTextEdit(logWidget)
        self.clear_log = QPushButton("Очистить лог", logWidget)
        logHBox = QHBoxLayout()
        logHBox.addStretch(1)
        logHBox.addWidget(self.clear_log)

        logVBox = QVBoxLayout()
        logVBox.addWidget(self.log_text)
        logVBox.addLayout(logHBox)
        logWidget.setLayout(logVBox)

        self.log.setWidget(logWidget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log)

        self.ip = '0'

        self.log_timer = QTimer()
        self.log_timer.timeout.connect(self.IP_Callback)
        self.log_timer.start(1000)
    
    def IP_Callback(self):
        host_name = socket.gethostname()
        host_ip = socket.gethostbyname(host_name)     
        if (self.ip != host_ip):
            self.ip = host_ip
            self.log_text.append('To connect IP : ' + str(host_ip) )
            self.log_text.append('Start from port: 5563')
            self.log_text.append('Choose the state')

global App
if __name__ == "__main__":
    # install exception hook: without this, uncaught exception would cause application to exit
    sys.excepthook = trap_exc_during_debug    
    App = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(App.exec())
