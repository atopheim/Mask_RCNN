#!/usr/bin/env python3
"""
Software towards autonomous docking
Author: Torbjørn Opheim
"""

import cv2 as cv
import gi
import numpy as np
import time
from pymavlink import mavutil

gi.require_version('Gst', '1.0')
from gi.repository import Gst

mavutil.set_dialect("ardupilotmega")

autopilot = mavutil.mavlink_connection('udpin:192.168.2.1:14550')

msg = None

# wait for autopilot connection
while msg is None:
        msg = autopilot.recv_msg()

print("Program has started with connection",msg)

class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary
        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """
        Transform byte array into np array
        Args:
            sample (TYPE): Description
        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """
        Get Frame
        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """
        Check if frame is available
        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """
        Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK

#Camera parameters obtained from camera calibration of PiCAM 1.0.1
fx = 2561
fy = 2545.54
cx = 988
cy = 557
pi_mtx = np.array([[ fx,0,cx ],[ 0,fy,cy ],[ 0,0,1 ]],dtype=np.float32)
pi_dist = np.array([ 6.47233420e-02, 1.13627373e+00, 2.73447271e-03, 7.81975172e-03, -5.43129287e+00], dtype=np.float32 )

# Declare variables of tracking object
    # Chessboard
window_size = 2.1 # cm
dim_x = 9 # corners in x direction
dim_y = 6 # corners in y direction

print("Starting program","\n",".","\n",".","\n")
# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Prepare object points in tracking objects coordinate system.
objp = np.zeros((dim_x*dim_y,3), np.float32)
objp[:,:2] = np.mgrid[0:dim_x*window_size:window_size,0:dim_y*window_size:window_size].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    # Color Scheme - BGR
    img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5) #X
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5) #Y
    img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5) #Z
    return img

def set_rc_channel_pwm(id, pwm = 1500):
        if id < 1:
                print("Channel doesn't exist")
                return
        if id < 9:
                rc_channel_values = [65535 for _ in range(8)]
                rc_channel_values[id - 1] = pwm
                autopilot.mav.rc_channels_override_send(
                autopilot.target_system,                # target_system
                autopilot.target_component,             # target_component
                *rc_channel_values)

def regulate(tvecs, rvecs, setvec, Kp = 5):
    error_x = setvec[0]-tvecs[0]
    error_y = setvec[1]-tvecs[1]
    error_z = setvec[2]-tvecs[2]
    error_roll = 0 # neglecting roll and pitch behavior because it's not relevant in this test.
    error_pitch = 0
    error_yaw = setvec[5] -rvecs[2]

    if error_x < -2: # deadband adjusting
        dead_adj_x = -25
    elif error_x > 2:
        dead_adj_x = 25
    else:
        pass
    
    if error_y < -2:
        dead_adj_y = -25
    elif error_y > 2:
        dead_adj_y = 25
    else:
        pass
    
    if error_z < -2:
        dead_adj_z = -25
    elif error_z > 2:
        dead_adj_z = 25
    else:
        pass

    if error_yaw < -2:
        dead_adj_yaw = -25
    elif error_yaw > 2:
        dead_adj_yaw = 25
    else:
        pass


    u_x = int(-error_x[0]*Kp+dead_adj_x)
    print("pwm_x",1500+u_x)
    u_y = int(error_y[0]*Kp+dead_adj_y)
    print("pwm_y",1500+u_y)
    u_z = int(-error_z[0]*Kp+dead_adj_z)
    print("pwm_z",1500+u_z)
    u_yaw = int(error_yaw[0]*Kp+dead_adj_yaw)
    print("pwm_yaw",1500+u_yaw)
    autopilot.wait_heartbeat()
    duration = 1 # define and assign
    while(duration<=5000):
        set_rc_channel_pwm(6, 1500+u_x) 
        set_rc_channel_pwm(1, 1500+u_y)
        set_rc_channel_pwm(5, 1500+u_z)
        set_rc_channel_pwm(4, 1500+u_yaw)
        duration += 1
    
setvec = [0,0,40,0,0,0]

# Arming ROV
# The values of these heartbeat fields is not really important here
# I just used the same numbers that QGC uses
# It is standard practice for any system communicating via mavlink emit the HEARTBEAT message at 1Hz! Your autopilot may not behave the way you want otherwise!
autopilot.mav.heartbeat_send(
6, # type
8, # autopilot
192, # base_mode
0, # custom_mode
4, # system_status
3  # mavlink_version
)

autopilot.mav.command_long_send(
1, # autopilot system id
1, # autopilot component id
400, # command id, ARM/DISARM
0, # confirmation
1, # arm!
0,0,0,0,0,0 # unused parameters for this command
)

# Wait a heartbeat before sending commands
autopilot.wait_heartbeat()

if __name__ == '__main__':
    cycle = 1
    # Create the video object
    # Add port= if is necessary to use a different one
    #video = Video()
    video = Video(port=4777)
    while True:
        print("Cycle =", cycle)
        # Wait for the next frame
        if not video.frame_available():
            continue

        img = video.frame()
        #cv.imshow('frame', img)
        #if cv.waitKey(1) & 0xFF == ord('q'):
        #    break   
        gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, (dim_x,dim_y),None)
        cv.destroyAllWindows()
        if ret == True:
            corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            # Find the rotation and translation vectors.
            ret,rvecs, tvecs = cv.solvePnP(objp, corners2, pi_mtx, pi_dist) 
            print("RotationVec", rvecs, "TranslationVec", tvecs)
            axis = np.float32([[4+2,2,0], [4,2+2,0], [4,2,-2]]).reshape(-1,3)
            imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, pi_mtx, pi_dist)
            #img = draw(img,corners2, imgpts)
            #format
            #cv.namedWindow('img',cv.WINDOW_NORMAL)
            #cv.resizeWindow('img', 800,600)
            #cv.imshow('img',img)
            
            #if cv.waitKey(1000) & 0xFF == ord('q'):
            #    break
            regulate(tvecs,rvecs,setvec)
            #fname = "frame" + str(cycle)
            #cv.imwrite(str(fname+".png"),img)
            cycle += 1
        else:
            print("Lost track")
cv.destroyAllWindows()

autopilot.mav.command_long_send(
1, # autopilot system id
1, # autopilot component id
400, # command id, ARM/DISARM
0, # confirmation
0, # disarm!
0,0,0,0,0,0 # unused parameters for this command
)
