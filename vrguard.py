import sys
import time
import openvr as vr
import numpy as np

# TODO Push this to __main__
if not vr.isRuntimeInstalled():
    print("Runtime not installed")
    exit()

class VRGuard():
    # Public members
    app = None
    bounds = None
    lastBoundUpdate = 0
    minHeight = 0.6

    # ########### Is OpenVR properly running (a.k.a a HMD is found initialized)?
    @classmethod
    def isVRRunning(cls):
        cls.startContext()
        if(not cls.app): return False
        return cls.app.isTrackedDeviceConnected(vr.k_unTrackedDeviceIndex_Hmd)
    
    # ########### Initialize openvr context
    @classmethod
    def startContext(self):
        if(self.app): return
        try:
            # Try to init openvr
            self.app = vr.init(vr.VRApplication_Utility)
            return
        except:
            print("Could not initialize openvr subsystem")
        # We failed. Try to invalidate any possible open instance via shutdown (should never be necessary, but we do it anyway)
        try:
            self.app = None
            vr.shutdown()
        except:
            pass

    # ########### Extracts the position of a pose/HmdMatrix34_t
    @staticmethod
    def getPos(mat):
        pos = np.array((0.0,0.0,0.0))
        pos[0] = mat[0][3]
        pos[1] = mat[1][3]
        pos[2] = mat[2][3]
        return pos
    
    # ########### Updates the chaperone bounds
    def updateBounds(self):
        self.bounds = None # Invalidate any earlier bounds
        try:
            chaperone = vr.VRChaperone()
            _,area = chaperone.getPlayAreaRect()
            if(area.vCorners[0] != 0 and area.vCorners[1] != 0 and area.vCorners[2] != 0):
                self.bounds = area
            return True
        except:
            return False

    # ########### Returns the distance of a given position to the shaper bounds
    def getDistanceToBounds(self, pos):
        if(not self.bounds): return None
        v0 = self.bounds.vCorners[0]
        v1 = self.bounds.vCorners[1]
        v2 = self.bounds.vCorners[2]
        v3 = self.bounds.vCorners[3]
        x = (v0[0],v1[0],v2[0],v3[0]) # x axis extrema
        z = (v0[2],v1[2],v2[2],v3[2]) # z axis extrema
        xMax = max(x)
        xMin = min(x)
        zMax = max(z)
        zMin = min(z)

        # Out of bounds
        if(pos[0] > xMax): return 0
        if(pos[0] < xMin): return 0
        if(pos[2] > zMax): return 0
        if(pos[2] < zMin): return 0

        

        # Now calculate actual minimal distance
        delta = min(abs(np.array((xMax - pos[0], pos[0] - xMin, zMax - pos[2], pos[2] - zMin))))
        return delta

    # ########### Runs one single "check distance to bounds" tick
    def tick(self):
        if(not self.app): return -1

        # Update chaperbounds regularily (Hard-Coded to every 10 seconds)
        t = time.time()
        if(t - self.lastBoundUpdate > 10):
            self.updateBounds()
            self.lastBoundUpdate = t

        # Cycle through all VR devices, calculate minimal distance to chaperbounds and return it
        dist = []
        for dev in range(vr.k_unMaxTrackedDeviceCount):
            if not self.app.isTrackedDeviceConnected(dev): continue
            c = self.app.getTrackedDeviceClass(dev)
            if(c == vr.TrackedDeviceClass_Controller):
                _,_,pose = self.app.getControllerStateWithPose(vr.TrackingUniverseStanding,dev)
                pos = self.getPos(pose.mDeviceToAbsoluteTracking)
                if(pos[1] > self.minHeight): # Is the controller currently located higher than min height (e.g. not laying on the floor?)
                    dist.append(self.getDistanceToBounds(pos))
        if(len(dist) > 0):
            dist = min(np.array(dist,dtype=np.float))
            return dist
        return -1
    
    # ########### cstor
    def __init__(self):
        self.startContext() # Start OpenVR context
        self.updateBounds() # Update shaperbounds

def main():
    g = VRGuard()
    while True:
        if(g.isVRRunning()):
            # Executed, when VR is running
            dist = g.tick()
            if dist >= 0:
                print(dist)
        else:
            # VR either not running or initializing an OpenVR context failed -> Get new VR context and wait 10 seconds
            g.startContext()
            time.sleep(10)
        time.sleep(0.1)
    vr.shutdown()

if __name__ == '__main__':
    main()