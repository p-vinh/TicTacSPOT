# implement pickup class
import cv2 as cv
from cv2 import mat
import numpy as np
from bosdyn.client import ImageClient
from bosdyn.client.estop import EstopClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.api import geometry_pb2, robot_command_pb2

# make sure holding toy

    # if not, pickup




def detectFiducial():
    markerDict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_16h5)
        
    param_marker = aruco.DetectorParameters()
    cap = cv.VideoCapture(0)
    
    #################################
    # This is the camera matrix, assembled from calibration.
    camera_matrix = np.array( [[general_settings["camera_focal_length_x"], 0, general_settings["camera_center_x"]], [0, general_settings["camera_focal_length_y"], general_settings["camera_center_x"]], [0, 0, 1]], dtype = "double" )
    # Coefficients are [k1, k2, p1, p2, k3] as per OpenCV documentation
    distortion_coefficients = np.array( [general_settings["camera_k1"], general_settings["camera_k2"], general_settings["camera_p1"], general_settings["camera_p2"], general_settings["camera_k3"]] )
    #################################
#access SPOT front Camera 
# find the center -  
(center_px_x, center_px_y) = find_center_px(dogtoy.image_properties.coordinates)

# once find center, move arm forward
# move to pose along a 2d plane
static synchro_se2_trajectory_point_command(center_px_x, center_px_y, # goal_heading, frame_name, params=None, body_height=0.0, locomotion_hint=1, build_on_command=None)


def detectFiducial():
    markerDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_16h5)
    param_marker = cv.aruco.DetectorParameters()
    cap = cv.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Detect markers
        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(frame, markerDict, parameters=param_marker)
        
        if ids is not None:
            # Assuming only one marker is detected
            marker_id = ids[0][0]
            if marker_id == YOUR_FIDUCIAL_ID:
                # Calculate center
                center_px_x = int((corners[0][0][0][0] + corners[0][0][2][0]) / 2)
                center_px_y = int((corners[0][0][0][1] + corners[0][0][2][1]) / 2)
                
                # Move arm to center
                moveArmToCenter(center_px_x, center_px_y)
                
                # Draw detected marker and center on frame
                cv.aruco.drawDetectedMarkers(frame, corners)
                cv.circle(frame, (center_px_x, center_px_y), 5, (0, 0, 255), -1)
        
        cv.imshow('frame', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

def moveArmToCenter(center_px_x, center_px_y):
    # Connect to SPOT
    # Assuming you have created clients for robot command
    robot_command_client = RobotCommandClient()
    estop_client = EstopClient()
    image_client = ImageClient()
    
    # Check robot estop
    estop_client.get_status().stop_level == estop_client.get_status().STOP_LEVEL_NONE
    
    # Get current robot state
    robot_state = robot_command_client.robot_state()
    
    # Assuming you have defined a function to convert pixel coordinates to robot coordinates
    robot_x, robot_y = pixelToRobotCoordinates(center_px_x, center_px_y, robot_state.kinematic_state.transforms_snapshot)

    # Create a SE2 trajectory point command
    command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=robot_x,
        goal_y=robot_y,
        goal_heading=0,  # Assuming heading is not relevant for arm movement
        frame_name='odom',
        params=None,
        body_height=0.0,
        locomotion_hint=1
    )

    # Send the command
    robot_command_client.robot_command(lease=None, command=command)

def pixelToRobotCoordinates(center_px_x, center_px_y, transforms_snapshot):

    # Assuming you have a function to transform pixel coordinates to robot coordinates using transforms_snapshot
    # This involves transforming from the camera frame to the base frame
    # You might need to use calibration data to perform this transformation
    # You can use the camera intrinsic parameters and distortion coefficients to undistort the pixel coordinates
    # and then transform them using the camera extrinsic parameters
    # Return the robot coordinates
    return robot_x, robot_y

# Pose estimation

    def camMatrix, distCoeffs;
        if(estimatePose) {
            # You can read camera parameters from tutorial_camera_params.yml
            bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
            if(!readOk) {
                cerr << "Invalid camera file" << endl;
                return 0;
            }
        }

    # set coordinate system
    def objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    vector<int> ids;
    vector<vector<Point2f> > corners, rejected;
    # detect markers and estimate pose
    detector.detectMarkers(image, corners, ids, rejected);
    size_t nMarkers = corners.size();
    vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);
    if(estimatePose && !ids.empty()) {
        # Calculate pose for each marker
        for (size_t i = 0; i < nMarkers; i++) {
            solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
        }
    }
    

# Call the function to start detection and arm movement
detectFiducial()




















































