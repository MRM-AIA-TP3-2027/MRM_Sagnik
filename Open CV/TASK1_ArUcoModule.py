"""Use of ArUco : It uses the rover cam (In practical conditions) to detect surroundings. Uses the predefined nXn matrices to detect markers conatining unique marker id's which are actually the surroudings and detects its path or obstacles. It also helps in detecting the rovers approximate position with respect to that specific marker."""



import cv2      #OpenCV library for video capture and image processing
import numpy as np         #Numpy library for array operations
import cv2.aruco as aruco       #Aruco module for marker detection

def initialize_camera():
    
    #Invoking the webcame to start capturing feed and 0 for selecting the default webcam of the system - Laptop

    cap = cv2.VideoCapture(0)           
    if not cap.isOpened():
        print("Error. Webcam not detected.")
        exit()

    print("Webcam started successfully.")
    return cap


def load_dictionaries():

    dictionaries = {
        
        #So basically we are loading pre defined markers in the aruco library. The n X n is the size which is defined by no of binary grid and 50 defines the total no of unique markers in that specific sized dictionary
        
        "4x4": aruco.getPredefinedDictionary(aruco.DICT_4X4_50),     
        "5x5": aruco.getPredefinedDictionary(aruco.DICT_5X5_50),
        "6x6": aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    }

    return dictionaries


def create_detector_parameters():

    #Creating a default set of parameter/rules for marker detectionusing DetectorParameters() present in the Aruco module. Also we can add custom parameters to it for better detection
    
    parameters = aruco.DetectorParameters_create()
    return parameters


def get_camera_calibration():

    #Camera control parameters using np array. Here in the matrix the first row first number 800 defines the focal length of the webcam along x axis and 320 defines the optical centre along x axis, simmilarly for y and z axis. All of these values are in pixels. The third row is standard values along z axis. all the values are standard assuming no lens distortion.
    
    camera_matrix = np.array([          
        [800, 0, 320],
        [0, 800, 240],
        [0, 0, 1]
    ], dtype=np.float32)

    #Creating a numpy zero array of 5 rows and 1 coloumn for calculating distortions. 0 means no distortion and 1 means maximum distortion.

    distortion_coeffs = np.zeros((5, 1))

    return camera_matrix, distortion_coeffs

#This function captures colored feed using the webcam and then converts if to greyscale for better detection of markers.

def detect_markers(frame, gray, dictionaries, parameters, camera_matrix, dist_coeffs):

    #Marker size in meters (5 cm actual size)
    
    marker_size = 0.05      

    for name, dictionary in dictionaries.items():

        #Collecting 3 things the corner pixel data of the marker, the detection id of the marker and the rejected shapes that looks like a marker but aren't
        
        corners, ids, rejected = aruco.detectMarkers(
            gray,
            dictionary,
            parameters=parameters
        )

        #If statement checking if any marker is present in the frame.
        
        if ids is not None:

            #The drawDetectedMarkers() function draws a rectangular outline around the detected marker if it is present in the frame and also displays the Marker ID previously collected.
            
            aruco.drawDetectedMarkers(frame, corners, ids)

            #Detects the actual position of the marker in 3D space by diplaying the corners, actual marker dimension, webcam parameters and distortion co-effifcients. It returns the rotational vector(marker orientation) and translational vector(marker position) of the marker.
            
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                marker_size,
                camera_matrix,
                dist_coeffs
            )

            #Loops through all the detected markers using rotational and translationl vectors of marker[i].
            
            for i in range(len(ids)):

                rvec = rvecs[i]
                tvec = tvecs[i]
                print("Marker ID:", ids[i][0])
                print("Rotation Vector rvec:", rvec)
                print("Translation Vector tvec:", tvec)
                print("-------------")

                #Draws the 3D axis on the marker using the rotational and translational vectors. Takes the frame, camera parameters and distortion co-efficients as input. The axis length is assumed to be 0.03 m (3 cm).
                
                cv2.drawFrameAxes(
                    frame,
                    camera_matrix,
                    dist_coeffs,
                    rvec,
                    tvec,
                    0.03
                )

                #Extracts the z co-ordinate of the marker which represents the distance of the marker from the camera in meters.

                distance = tvec[0][2]

                #Shows the marker ID and distance infoof the mrker on the frame 

                text = f"ID: {ids[i][0]} | Dist: {distance:.2f} m"

                #Gets exact x co-ordinates and y co-ordinates of the marker.
                
                x = int(corners[i][0][0][0])
                y = int(corners[i][0][0][1]) - 10
                

                #Putting the text info on top of the frame in BGR format with font size 0.6 and thickness 2.
                
                cv2.putText(
                    frame,
                    text,
                    (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )


#Main function to actually run the program.

def main():

    print("Starting ArUco Pose Estimation")

    #Initializing the webcam
    
    cap = initialize_camera()       
    
    #Loading the predefined marker dictionaries

    dictionaries = load_dictionaries()      
    
    #Creating default parameters for marker detection

    parameters = create_detector_parameters()       
    
    #Getting the camera calibration parameters (intrinsic matrix and distortion coefficients)

    camera_matrix, dist_coeffs = get_camera_calibration()

    #Creates an infinite loop for OpenCV to continuosly capture frames untill pressing a specific key(In this case it is 'Q') to exit
    
    while True:

        #Captures the frame. Successful capturing is stored in frame and ret stores the boolean value of success or faliure in capturing the frame. 
        
        ret, frame = cap.read()

        #Error message is ret value is False, which breks the whole loop using the break statement and exits the program.
        
        if not ret:
            print("Failed to capture frame")
            break

        #Converts the frame into grey scale for procesing.
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #Calls the detection function taking colored frame, grayscale frame, Aruco dictionaries, Detection parameters, Camera matrix and distortion co-effificients as input
        
        detect_markers(
            frame,
            gray,
            dictionaries,
            parameters,
            camera_matrix,
            dist_coeffs
        )

        #Displays the processed video frame with marker detection pose estimation, x-y frame, rectangular outline and marker info. The window name is "ArUco Pose Estimation TP 3".
        
        cv2.imshow("ArUco Pose Estimation TP 3", frame)

        #Waits for 1 ms interval for a key press.
        
        key = cv2.waitKey(1) & 0xFF

        #If key pressed is 'Q' or 'q' and then it breaks the loop and exits the program.
        
        if key == ord('q'):
            print("Exiting program")
            break

    #Closing all windows after execution
    
    cap.release()
    cv2.destroyAllWindows()         



if __name__ == "__main__":
    main()
