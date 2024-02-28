# implement pickup class

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