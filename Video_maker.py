import time
import os
import math
from datetime import datetime
import numpy as np
# from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import butter, lfilter, freqz
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
# import sympy as sym


sens_data = {}

def get_files(directory):
    files = os.listdir(directory)
    for i in range(0,len(files)):
        files[i] = directory + files[i]
    return files

def delts(temp):
    # temp = [i[0] for i in temp]
    n = temp.index(max(temp))
    return [temp[n] - i for i in temp], temp[n]

class video_player():

    def __init__(self, filename=None, ports=['5569', '5571', '5563', '5561', '5559']):

        if filename== None:
            local_path = os.getcwd()
            filename = os.path.join(local_path, '/example_data_0')
        self.obj = os.listdir(filename)
        self.qtrs = []
        self.sens_data = {} #dictionary of dicts
        # print(self.obj)
        # self.fortest = obj
        # print(self.fortest)
        old_times = []
        self.new_times = []
        for names in self.obj:    
            with open(filename + '/' + names, 'r') as the_file:        
                keys = the_file.readline().split(',')
                temp_lines = the_file.readlines()
                self.sens_data[names[-8:-4]] = {}  # dictionary of ports
                self.sens_data[names[-8:-4] + '_acc'] = {}  # dictionary of port_acc
                self.qtrs.append(names[-8:-4])
                # print(temp_lines)
                for el in temp_lines:
                    el = el[:-2].split(',')
                    # print(el)
                    self.sens_data[names[-8:-4]][float(el[1])] = [float(eli) for eli in el[4:8]]
                    self.sens_data[names[-8:-4] + '_acc'][float(el[1])] = np.array([float(eli) for eli in el[8:11]]) + np.array([float(eli) for eli in el[11:14]])
                    # print(self.sens_data[names[-8:-4] + '_acc'])
                
                # print( next(iter(sens_data[names[-8:-4]])) )


        old_times = [list(self.sens_data[i].keys()) for i in self.qtrs]

        list_delts, max_el = delts([el[0] for el in old_times])
        # print(list_delts)

        for i in range(len(old_times)):
            self.new_times.append([x + list_delts[i] - max_el  for x in old_times[i]])

        # print(list(sens_data['5561'].keys())[0:10])

        for el,i in zip(self.qtrs,range(len(self.qtrs))):
            for eli in old_times[i]:
                self.sens_data[el][eli + list_delts[i] - max_el] = self.sens_data[el].pop(eli)
                self.sens_data[el + '_acc'][eli + list_delts[i] - max_el] = self.sens_data[el + '_acc'].pop(eli)

        #################################################
        last_times = [i[-1] for i in self.new_times] #Check the times of ending from each sensor
        indx = last_times.index(min(last_times)) # to get index of the massive that has earliest time of the end
        delta_time = self.new_times[indx][-1]- self.new_times[indx][0] #getting the shortest lenth of the signal
        self.time = np.linspace(0.0, delta_time, num = (int(delta_time)+1)*100)
        # self.time = np.linspace(0.0, delta_time, num = 9001)
        ######################################################

        self.frame = {}
        
        for port,i in zip(self.qtrs,range(len(self.qtrs))):
            slerp = Slerp(self.new_times[i], R.from_quat([self.sens_data[port][el] for el in self.new_times[i]])) #make interpolation
            
            acc_int_x = interp1d(self.new_times[i], [self.sens_data[port + '_acc'][el][0] for el in self.new_times[i]])
            acc_int_y = interp1d(self.new_times[i], [self.sens_data[port + '_acc'][el][1] for el in self.new_times[i]])
            acc_int_z = interp1d(self.new_times[i], [self.sens_data[port + '_acc'][el][2] for el in self.new_times[i]])

            acc_x = acc_int_x(self.time)
            acc_y = acc_int_y(self.time)
            acc_z = acc_int_z(self.time)

            # print([self.sens_data[port + '_acc'][el][0] for el in self.new_times[i]])
            # f2 = interp1d(t, x, kind='cubic')
            # tnew = np.linspace(t[0],t[-1],num=25000,endpoint=True)

            self.frame[port] = slerp(self.time).as_quat()
            # self.frame[port + '_acc'] = [[acc_x[i], acc_y[i], acc_z[i]] for i in range(len(acc_x))]
            # print(self.frame[port + '_acc'])
        
        # print(len(self.frame['5563']))
        # print(len(self.frame['5571']))
        # print(len(self.frame['5569']))
        # print(len(self.time))



if __name__ == "__main__":
    vp = video_player('C:/Users/User/GoogleДиск/Аспирантура/MIMU_project/example_13_19_38')
    local_path = os.getcwd()

    filename = os.path.join(local_path,'IMU_qtrs_data.csv')
    print(vp.qtrs)
    with open(filename, 'w') as the_file: 
        the_file.write('time,\t\tqtrs_radius_r + qtrs_ulna_r,\t\t\t\t\tqtrs_humerus_r,\t\t\t\t\t\t\t\t\tqtrs_body,\t\t\t\t\t\t\t\t\t\tqtrs_shin_l,\t\t\t\t\t\t\t\t\tqtrs_hip_l,\t\t\t\t\t\t\t\t\t\tqtrs_ulna_l + qtrs_radius_l,\t\t\t\t\tqtrs_humerus_l\n')
        for t in range(len(vp.time)):
            the_file.write('{:<8f}'.format(vp.time[t]) + ',\t')
            for port in vp.qtrs:
                the_file.write(',\t'.join('{:<8f}'.format(el) for el in list(vp.frame[port][t])) )
                the_file.write(',\t')
            the_file.write('\n')


        
        

    # local_path = os.getcwd()
    # for name in vp.qtrs:
    #     i = 472
    #     filename = os.path.join(local_path,'xsense_data/',str(name) + '.txt')
    #     with open(filename, 'a') as the_file:
    #         for el,ac in zip(vp.frame[name],vp.frame[name + '_acc']):
    #             the_file.write('%5d' % i)
    #             the_file.write('\t\t\t\t\t\t\t\t\t\t\t\t\t\t')
    #             for eli in el:
    #                 eli = np.transpose(eli)
    #                 the_file.write('\t '.join('%.6f' % round(elem,6) for elem in eli))
    #                 the_file.write('\t ')

    #             the_file.write('\t '.join('%.6f' % round(element,6) for element in ac))
    #             the_file.write('\n')
    #             i+=1

    ################################################################# xsense
    # local_path = os.getcwd()
    # filename = os.path.join(local_path,'xsense_data/Exo_quats_new.sto')
    # with open(filename, 'a') as the_file:
    #     for i in range(len(vp.time)):
    #         the_file.write(str(vp.time[i]))
    #         the_file.write('\t')
    #         the_file.write(','.join('%.16f' % elem for elem in vp.frame['5563'][i]))
    #         the_file.write('\t')
    #         the_file.write(','.join('%.16f' % elem for elem in vp.frame['5559'][i]))
    #         the_file.write('\t')
    #         the_file.write(','.join('%.16f' % elem for elem in vp.frame['5561'][i]))
    #         the_file.write('\n ')  
    ################################################################# xsense

    # print(interp_rots1.as_quat()[1:10])
    # print(key_rots1.as_quat()[1:10])


        # last_index = [len(i) for i in self.new_times]
        # end_index = last_index.index(min(last_index))
        # self.length = len(self.new_times[end_index])
        # self.temp_time = range(self.length)

        #making farames from massiv

        # frame = {'time' : self.temp_time[0], '5559' : self.sens_data['5559'][self.new_times[0][0]], '5561' : self.sens_data['5561'][self.new_times[1][0]], '5563' : self.sens_data['5563'][self.new_times[2][0]]}
        # print(len(self.temp_time))
        # i = 2
        # frame = {'time' : self.temp_time[i], '5559' : sens_data['5559'][self.new_times[0][i]], '5561' : sens_data['5561'][self.new_times[1][i]], '5563' : self.sens_data['5563'][self.new_times[2][i]]}

# temp = list(sens_data['5563'].values())
# # w = np.array([i[0] for i in temp])
# x = np.array([i[1] for i in temp])
# y = np.array([i[2] for i in temp])
# z = np.array([i[3] for i in temp])

# plt.plot(new_times[2],x , '--', new_times[2],y, '-', new_times[2], z, '-.') #'-', '--', '-.', ':', '',
# plt.legend(['linear'], loc='best')
# plt.show()



def interpolate_rotation(q_0,q_1,q_2,q_3,t):

    t_int = int(t)
    t_fract = t - t_int

    i_1 = innerQuadPoint(q_0, q_1, q_2)
    i_2 = innerQuadPoint(q_1, q_2, q_3)

    q_i = Slerp(Slerp(q_1, q_2, t_fract), Slerp(i_1, i_2, t_fract), 2*t_fract*(1-t_fract))

    return q_i

def qtr_inv(qtr_a):

    a1 = qtr_a[0]
    a2 = qtr_a[1]
    a3 = qtr_a[2]
    a4 = qtr_a[3]

    mod = a1*a1 + a2*a2 + a3*a3 + a4*a4

    qtr_inf = np.array([a1/mod, - a2/mod,- a3/mod, - a4/mod])

    return qtr_inf

def qtr_multiplication(qtr_a, qtr_b): #Multiplication quaternions = A*B

    qtr_a = np.array(qtr_a)
    qtr_b = np.array(qtr_b)
    mod_qtr = qtr_a[0]*qtr_a[0] + qtr_a[1]*qtr_a[1] + qtr_a[2]*qtr_a[2] + qtr_a[3]*qtr_a[3] # mod for multiplication of quaternions
    
    a1 = qtr_a[0]*qtr_b[0] - qtr_a[1]*qtr_b[1] - qtr_a[2]*qtr_b[2] - qtr_a[3]*qtr_b[3]
    a2 = qtr_a[1]*qtr_b[0] + qtr_a[0]*qtr_b[1] + qtr_a[3]*qtr_b[2] - qtr_a[2]*qtr_b[3]
    a3 = qtr_a[2]*qtr_b[0] - qtr_a[3]*qtr_b[1] + qtr_a[0]*qtr_b[2] + qtr_a[1]*qtr_b[3]
    a4 = qtr_a[3]*qtr_b[0] + qtr_a[2]*qtr_b[1] - qtr_a[1]*qtr_b[2] + qtr_a[0]*qtr_b[3]
    
    qtr_mult = np.array([a1/mod_qtr,a2/mod_qtr,a3/mod_qtr,a4/mod_qtr])
    # print(qtr_multiplication)
    return qtr_mult

def angl_vector_from_qtr(qtr_a):
    mod_Vqtr = math.sqrt(qtr_a[1]*qtr_a[1] + qtr_a[2]*qtr_a[2] + qtr_a[3]*qtr_a[3])
    mod_qtr = math.sqrt(qtr_a[0]*qtr_a[0] + qtr_a[1]*qtr_a[1] + qtr_a[2]*qtr_a[2] + qtr_a[3]*qtr_a[3])

    qw = qtr_a[0]
    qi = qtr_a[1]/mod_qtr
    qj = qtr_a[2]/mod_qtr
    qk = qtr_a[3]/mod_qtr

    theta = math.atan2(mod_Vqtr,qw)
    return theta, np.array([qi,qj,qk])

def log(qtr_a):
    mod_Vqtr = math.sqrt(qtr_a[1]*qtr_a[1] + qtr_a[2]*qtr_a[2] + qtr_a[3]*qtr_a[3])
    mod_qtr = math.sqrt(qtr_a[0]*qtr_a[0] + qtr_a[1]*qtr_a[1] + qtr_a[2]*qtr_a[2] + qtr_a[3]*qtr_a[3])

    qi = qtr_a[1]/mod_Vqtr
    qj = qtr_a[2]/mod_Vqtr
    qk = qtr_a[3]/mod_Vqtr

    qtr_a = (1/mod_Vqtr * math.acos(qtr_a[0]/mod_qtr))* np.array([0,qi,qj,qk]) #usually it is written that w = 0 if log(quqternion).normalized

    return qtr_a / math.sqrt(qtr_a[0]*qtr_a[0] + qtr_a[1]*qtr_a[1] + qtr_a[2]*qtr_a[2] + qtr_a[3]*qtr_a[3])

def exp(qtr_a):
    
    mod_Vqtr = math.sqrt(qtr_a[1]*qtr_a[1] + qtr_a[2]*qtr_a[2] + qtr_a[3]*qtr_a[3])
    qi = qtr_a[1]/mod_Vqtr
    qj = qtr_a[2]/mod_Vqtr
    qk = qtr_a[3]/mod_Vqtr
    qw = math.exp(qtr_a[0])

    qi = qw * (math.cos(mod_Vqtr) + (qi/mod_Vqtr) * math.sin(mod_Vqtr) )
    qj = qw * (math.cos(mod_Vqtr) + (qj/mod_Vqtr) * math.sin(mod_Vqtr) )
    qk = qw * (math.cos(mod_Vqtr) + (qk/mod_Vqtr) * math.sin(mod_Vqtr) )
    return np.array([0,qi,qj,qk])

def innerQuadPoint(q0,q1,q2):

    vTemp = qtr_inv(q1)

    vTemp1 = qtr_multiplication(vTemp, q0)
    vTemp1 = log(vTemp1)

    vTemp2 = qtr_multiplication(vTemp, q2)
    vTemp2 = log(vTemp2)
    vTemp = (vTemp1 + vTemp2) * (-0.25)
    qResult = qtr_multiplication(q1, exp(vTemp))

    return qResult
	



# t_1 = np.array(list(sens_data['5563'].keys()))
# t_2 = np.array(list(sens_data['5561'].keys()))
# t_3 = np.array(list(sens_data['5559'].keys()))

# 
# print(temp)



# f = interp1d(t, x)
# f2 = interp1d(t, x, kind='cubic')
# tnew = np.linspace(t[0],t[-1],num=25000,endpoint=True)



# def butter_lowpass(cutoff, fs, order=5):
#     nyq = 0.5 * fs
#     normal_cutoff = cutoff / nyq
#     b, a = butter(order, normal_cutoff, btype='low', analog=False)
#     return b, a

# def butter_lowpass_filter(data, cutoff, fs, order=5):
#     b, a = butter_lowpass(cutoff, fs, order=order)
#     y = lfilter(b, a, data)
#     return y

# # Filter requirements.
# order = 10
# fs = 100      # sample rate, Hz
# cutoff = 2.1  # desired cutoff frequency of the filter, Hz

# # Get the filter coefficients so we can check its frequency response.
# b, a = butter_lowpass(cutoff, fs, order)

# # Plot the frequency response.
# w, h = freqz(b, a, worN=8000)
# plt.subplot(2, 1, 1)
# plt.plot(0.5*fs*w/np.pi, np.abs(h), 'b')
# plt.plot(cutoff, 0.5*np.sqrt(2), 'ko')
# plt.axvline(cutoff, color='k')
# plt.xlim(0, 0.5*fs)
# plt.title("Lowpass Filter Frequency Response")
# plt.xlabel('Frequency [Hz]')
# plt.grid()


# data = x
# # Filter the data, and plot both the original and filtered signals.
# cl = butter_lowpass_filter(data, cutoff, fs, order)

# plt.subplot(2, 1, 2)
# plt.plot(t, data, 'b-', label='data')
# plt.plot(t, cl, 'g-', linewidth=2, label='filtered data')
# plt.xlabel('Time [sec]')
# plt.grid()
# plt.legend()

# plt.subplots_adjust(hspace=0.35)
# plt.show()


 


# print(sens_data['5563'])
# print(sens_data['5561'])
