import numpy as np
from CP_Rotation import CP_Rotation as Rot


class SegmentIMU:
    
    def __init__(self, N_segmentAtBody, N_imuAtGlob):
        self.N_segmentAtBody = N_segmentAtBody
        self.N_imuAtGlob = N_imuAtGlob
        self.segmentAtImu = None
        

class CalibIMUs:
    
    def __init__(self):
        self.__iCalibStage = 0
        self.__N_refImuAtGlob = None  # measured at stage 1
        self.__N_armImuAtGlob = None  # measured at stage 1
        self.__bodyAtRefImu = None  # calculated at stage 2
        self.__segImus = []

    def __del__(self):
        pass

    def setN_calibImusAtGlob(self, _N_refImuAtGlob, _N_armImuAtGlob):
        # TODO: check that we are at stage 0
        # N_refImuAtGlob - measured ref IMU rotation in N-pose
        # N_armImuAtGlob - measured arm IMU rotation in N-pose
        self.__N_refImuAtGlob = _N_refImuAtGlob
        self.__N_armImuAtGlob = _N_armImuAtGlob
        self.__iCalibStage = 1

    # N_segmentAtBody - anatomical rotation of a segment in N-pose rel to body
    # N_imuAtGlob - measured segment IMU rotation in N-pose
    # Arm used in the calibration should also be added as a segment
    def addSegmentImu(self, N_segmentAtBody, N_imuAtGlob):
        # TODO: check that we are at stage 0
        
        imu = SegmentIMU(N_segmentAtBody, N_imuAtGlob)
        self.__segImus.append(imu)
        
    # T_armImuAtGlob - measured arm IMU rotation in T-pose
    # upAxisIdx - up axis index in body FOR ( e.g. 1 for Y pointing up)
    # forwAxisIdx - forward axis index in body FOR ( e.g. 2 for Z pointing forward)
    # doInvForwAxis - use in left hand calibration
    def doCalibration(self, T_armImuAtGlob, upAxisIdx, forwAxisIdx, doInvForwAxis=False):

        # TODO: check that we are at stage 1
        # TODO: add arg to use left arm in calibration doInvForwAxis
        # construct up vector
        upVec = np.array([0, 0, 0])
        upVec[upAxisIdx] = 1
        # calc arm rot in T-pose using the offset rot from stage 1
        # this will be a rotation relative to N_armImuAtGlob
        # note that the order of multiplication is the opposite 
        T_armAtN_arm = self.__N_armImuAtGlob.inv() * T_armImuAtGlob
        
        # TODO: check that the rotation angle phi is close to 90 degrees:
        # the real part of the quaternion w should be close to 0.707
        # w = cos(phi/2) - where phi is the rot angle
        # 
        # the imaginary part of the quaternion is the axis of rotation
        # in our case it is the forward-pointing vector of the body FOR
        rotQuat = T_armAtN_arm.as_quat()
        forwVecAtN_arm = np.array(rotQuat[1:])
        # print("forwVecAtN_arm", forwVecAtN_arm)
        # now transform into the glob coord
        mtx = self.__N_armImuAtGlob.as_matrix()
        forwVec = mtx.dot(forwVecAtN_arm)
        if doInvForwAxis:
            forwVec = -forwVec
        # print("forwVec", forwVec)
        
        # take the part of the forward vector which is orthogonal to the 
        # up-pointing vector by subtracting its projecton on the up vector
        proj = np.dot(forwVec, upVec)
        # print("proj", proj)
        forwVecOrth = forwVec - (upVec * proj)
        # print("forwVecOrth", forwVecOrth)
        # normalize 
        forwVecNorm = forwVecOrth/np.sqrt(np.dot(forwVecOrth, forwVecOrth))
        # print("forwVecNorm", forwVecNorm)
        
        # calc third axis vector as perpendicular to the up and forward vecs
        #sideVec = -np.cross(upVec, forwVecNorm)
        sideVec = np.cross(forwVecNorm, upVec)
#        print("sideVec", sideVec)
        # combine as rows, it's more convenient
        mtxBodyAtGlobRows = np.array([sideVec, 
                                     sideVec,
                                     sideVec])
        # put up and forw vectors at corresponding places
        mtxBodyAtGlobRows[upAxisIdx, :] = upVec
        mtxBodyAtGlobRows[forwAxisIdx, :] = forwVecNorm
        # turn axes vectors into columns
        mtxBodyAtGlob = mtxBodyAtGlobRows.transpose() 
        # we have the body FOR now
        bodyAtGlob = Rot.from_matrix(mtxBodyAtGlob)
        print("mtxBodyAtGlob",  mtxBodyAtGlob)
        print("det",  np.linalg.det(mtxBodyAtGlob))
        print("bodyAtGlob.as_quat()", bodyAtGlob.as_quat())

        # calc body rot rel to the ImuRef
        self.__bodyAtRefImu = self.__N_refImuAtGlob.inv() * bodyAtGlob
        
        # calc segment rot rel to segment IMU 
        for seg in self.__segImus:
            bodyAtSegImu = seg.N_imuAtGlob.inv() * bodyAtGlob
            seg.segmentAtImu = bodyAtSegImu * seg.N_segmentAtBody
        
        self.__iCalibStage = 2

    def calcBodyAtGlob(self, refImuAtGlob):
        # TODO: check that we are at stage 2
        # Body@Glob = Body@refImu * refImu@Glob
        return refImuAtGlob * self.__bodyAtRefImu

    def calcSegmentAtGlob(self, iSeg, segImuAtGlob):
        # TODO: check that we are at stage 2
        # Seg@Glob = Seg@SegImu * SegImu@Glob
        segAtImu = self.__segImus[iSeg].segmentAtImu
        segAtGlob = segImuAtGlob * segAtImu
        return segAtGlob
        
    def calcSegmentAtBody(self, iSeg, segImuAtGlob, refImuAtGlob):
        # TODO: check that we are at stage 2
        # Seg@Body = SegAtGlob * Body@Glob.inv()
        segAtGlob = self.calcSegmentAtGlob(iSeg, segImuAtGlob)
        bodyAtGlob = self.calcBodyAtGlob(refImuAtGlob)
        segAtBody = bodyAtGlob.inv() * segAtGlob
        return segAtBody

        
if __name__ == "__main__":
    
    calib = CalibIMUs()

    N_refImuAtGlob = Rot.from_quat([1, 0, 0, 0])
    N_armImuAtGlob = Rot.from_quat([1, 0, 0, 0])
    # 90 deg rotation around horiz x-z axis at 45 degr
    # quat = [0.7071068, 0.5, 0, 0.5]

    # axis [0.8, 0.0, 0.6 ] 60 degrees
    quat = [0.8660254, 0.4, 0.0, 0.3]

    # axis [0.5, 0.1, 0.5 ] 70 degrees
    # quat = [0.819152, 0.4015838, 0.0803168, 0.4015838]

    T_armImuAtGlob = Rot.from_quat(quat)
    
    ax = quat[1:]
    print("Axis norm ", ax/np.sqrt(np.dot(ax, ax)))
    upAxisIdx = 1  # y axis in body coord
    forwAxisIdx = 2  # z axis in body coord
    
    calib.setN_calibImusAtGlob(N_refImuAtGlob, N_refImuAtGlob)
    for i in range(0, 4):
        calib.addSegmentImu(Rot.random(), Rot.random())
    
    calib.doCalibration(T_armImuAtGlob, upAxisIdx, forwAxisIdx)
    
    print(calib.calcBodyAtGlob(Rot.random()).as_quat())
    
    # calib.setN_calibImusAtGlob(Rot.random(), Rot.random())
    # for i in range(0, 4):
    #     calib.addSegmentImu(Rot.random(), Rot.random())
    
    # calib.doCalibration(Rot.random(), 1,0)
    
    # print(calib.calcBodyAtGlob(Rot.random()).as_quat() )
    # for iStep in range (0, 1000):
    #     for i in range(0, 4):
    #         print(calib.calcSegmentAtGlob(1, Rot.random()).as_quat() )
    #         print(calib.calcSegmentAtBody(1, Rot.random(), Rot.random()).as_quat() )
    
    # input("Press Enter to exit")
    
#    del calib
