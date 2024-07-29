import openvr
import time

def main():
    vr_system = openvr.init(openvr.VRApplication_Scene)
    
    poses = (openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount)()
    
    while True:
        openvr.VRCompositor().waitGetPoses(poses, len(poses), None, 0)
        
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if poses[i].bPoseIsValid:
                device_class = vr_system.getTrackedDeviceClass(i)
                if device_class == openvr.TrackedDeviceClass_Controller:
                    print(f"Controller {i} is being tracked")
                    pose = poses[i].mDeviceToAbsoluteTracking
                    print_pose(pose)
                elif device_class == openvr.TrackedDeviceClass_GenericTracker:
                    print(f"Tracker {i} is being tracked")
                    pose = poses[i].mDeviceToAbsoluteTracking
                    print_pose(pose)
        
        time.sleep(1)

def print_pose(pose):
    for row in range(3):
        for col in range(4):
            print(pose[row][col], end=' ')
        print()
    print()

if __name__ == "__main__":
    main()
