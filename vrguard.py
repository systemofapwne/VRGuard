import time
import openvr as vr
from pygame import mixer
import numpy as np
import math

g_Near = "assets/271096__ianstargem__industrial-factory-fans-loop.mp3"
g_Over = "assets/371177__samsterbirdies__alert.mp3"

class VRGuard():
    # Public members
    m_App = None
    m_Bounds = None
    m_BoundsLastUpdate = 0
    m_minHeight = 0.6
    m_minDist = 0.4

    # ########### Is OpenVR properly running (a.k.a a HMD is found initialized)?
    @classmethod
    def isVRRunning(cls):
        cls.startContext()
        if(not cls.m_App): return False
        return cls.m_App.isTrackedDeviceConnected(vr.k_unTrackedDeviceIndex_Hmd)
    
    # ########### Initialize openvr context
    @classmethod
    def startContext(self):
        if(self.m_App): return
        try:
            # Try to init openvr
            self.m_App = vr.init(vr.VRApplication_Utility)
            return
        except:
            print("Could not initialize openvr subsystem")
        # We failed. Try to invalidate any possible open instance via shutdown (should never be necessary, but we do it anyway)
        try:
            self.m_App = None
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
    
    # ########### Updates the chaperone m_Bounds
    def updateBounds(self):
        self.m_Bounds = None # Invalidate any earlier m_Bounds
        try:
            chaperone = vr.VRChaperone()
            _,area = chaperone.getPlayAreaRect()
            if(area.vCorners[0] != 0 and area.vCorners[1] != 0 and area.vCorners[2] != 0):
                self.m_Bounds = area
            return True
        except:
            return False

    # ########### Returns the distance of a given position to the shaper m_Bounds
    def getDistanceToBounds(self, pos):
        if(not self.m_Bounds): return None
        v0 = self.m_Bounds.vCorners[0]
        v1 = self.m_Bounds.vCorners[1]
        v2 = self.m_Bounds.vCorners[2]
        v3 = self.m_Bounds.vCorners[3]
        x = (v0[0],v1[0],v2[0],v3[0]) # x axis extrema
        z = (v0[2],v1[2],v2[2],v3[2]) # z axis extrema
        xMax = max(x)
        xMin = min(x)
        zMax = max(z)
        zMin = min(z)

        # Out of m_Bounds
        if(pos[0] > xMax): return 0
        if(pos[0] < xMin): return 0
        if(pos[2] > zMax): return 0
        if(pos[2] < zMin): return 0

        # Now calculate actual minimal distance
        delta = min(abs(np.array((xMax - pos[0], pos[0] - xMin, zMax - pos[2], pos[2] - zMin))))
        return delta

    # ########### Runs one single "check distance to m_Bounds" tick
    def tick(self):
        if(not self.m_App): return -1

        # Update chaperbounds regularily (Hard-Coded to every 10 seconds)
        t = time.time()
        if(t - self.m_BoundsLastUpdate > 10):
            self.updateBounds()
            self.m_BoundsLastUpdate = t

        # Cycle through all VR devices, calculate minimal distance to chaperbounds and return it
        dist = []
        for dev in range(vr.k_unMaxTrackedDeviceCount):
            if not self.m_App.isTrackedDeviceConnected(dev): continue
            c = self.m_App.getTrackedDeviceClass(dev)
            if(c == vr.TrackedDeviceClass_Controller):
                _,_,pose = self.m_App.getControllerStateWithPose(vr.TrackingUniverseStanding,dev)
                pos = self.getPos(pose.mDeviceToAbsoluteTracking)
                if(pos[1] > self.m_minHeight): # Is the controller currently located higher than min height (e.g. not laying on the floor?)
                    dist.append(self.getDistanceToBounds(pos))
        if(len(dist) > 0):
            dist = min(np.array(dist,dtype=np.float))
            return dist
        return -1
    
    # ########### Starts the VRGuard
    def start(self):
        self.m_Running = True

        # Small buffer (512) reduces latency. Not sure, if this proposed fix of "pre_init, init, quit and init" will get rid of the ramining (annoying) lag
        mixer.pre_init(44100, -16, 2, 512)
        mixer.init()
        mixer.quit()
        mixer.init(44100, -16, 2, 512)

        isPlaying = 0 # 0: No, 1: Near, 2: Over
        while self.m_Running:
            if(self.isVRRunning()):
                # Executed, when VR is running
                dist = self.tick()
                if dist >= 0:
                    if(dist == 0):
                        # Change audio-file and play it, if not already playing
                        if(isPlaying != 2):
                            isPlaying = 2
                            mixer.music.stop()
                            mixer.music.load(g_Over)
                            mixer.music.play(-1)
                    elif(dist < self.m_minDist):
                        amp = 1 - math.exp(-3*(self.m_minDist - dist)/self.m_minDist) # Make amplitude grow exponentially from 1/e^2 to 1
                        mixer.music.set_volume(amp)
                        # Change audio-file and play it, if not already playing
                        if(isPlaying != 1):
                            isPlaying = 1
                            mixer.music.stop()
                            mixer.music.load(g_Near)
                            mixer.music.play(-1)
                    elif(isPlaying != 0):
                        isPlaying = 0
                        mixer.music.stop()
                elif(isPlaying != 0):
                    isPlaying = 0
                    mixer.music.stop()
            else:
                # VR either not running or initializing an OpenVR context failed -> Get new VR context and wait 10 seconds
                self.startContext()
                time.sleep(10)
            time.sleep(0.01)

        mixer.music.stop()
        vr.shutdown()
    
    # ########### cstor
    def __init__(self):
        self.startContext() # Start OpenVR context
        self.updateBounds() # Update shaperbounds

# ########### Main
def main():
    guard = VRGuard()
    guard.start()

if __name__ == '__main__':
    if not vr.isRuntimeInstalled():
        print("Runtime not installed")
        exit()
    main()