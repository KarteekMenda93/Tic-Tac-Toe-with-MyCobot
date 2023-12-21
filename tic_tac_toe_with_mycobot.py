#!/usr/bin/env python3
import cv2
import numpy as np
import time
import os
import sys
import serial
import serial.tools.list_ports

from pymycobot.mycobot import MyCobot

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"


class Object_detect():

    def __init__(self, camera_x=160, camera_y=15):
        self.frame_count = 2
        super(Object_detect, self).__init__()
        self.mc = None

        self.plist = [
            str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()

        ]

        self.pickup_place_cords_angles = [
            [0.61, 45.87, -92.37, -41.3, 2.02, 9.58],  # init the point
            [18.8, -7.91, -54.49, -23.02, -0.79, -14.76],  # point to grab
        ]

        # parameters to calculate camera clipping parameters 计算相机裁剪参数的参数
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord 设置真实坐标的缓存
        self.cache_x = self.cache_y = 0
        # set color HSV
        self.HSV = {
            "green": [np.array([35, 43, 35]), np.array([90, 255, 255])],#cobot color
            "yellow": [np.array([11, 85, 70]), np.array([59, 255, 245])],#my colour
        }

        # use to calculate coord between cube and mycobot280
 
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the mycobot280
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the mycobot280
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        self.ratio = 0

        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params. 获取 ArUco 标记参数
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    

    def transform_frame(self, frame):
        # enlarge the image by 1.5 times
        fx = 1.5
        fy = 1.5
        frame = cv2.resize(frame, (0, 0), fx=fx, fy=fy,
                           interpolation=cv2.INTER_CUBIC)
        if self.x1 != self.x2:
            # the cutting ratio here is adjusted according to the actual situation
            frame = frame[int(self.y2*0.78):int(self.y1*1.1),
                          int(self.x1*0.86):int(self.x2*1.08)]
        return frame
      # get points of two aruco 获得两个 aruco 的点位
      
     def run(self):
        self.mc = MyCobot(self.plist[0], 115200)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 20)
        time.sleep(2.5)

    def get_calculate_params(self, img):
        # Convert the image to a gray image 将图像转换为灰度图像
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect ArUco marker.
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        """
        Two Arucos must be present in the picture and in the same order.
        There are two Arucos in the Corners, and each aruco contains the pixels of its four corners.
        Determine the center of the aruco by the four corners of the aruco.
        """
        if len(corners) > 0:
            if ids is not None:
                if len(corners) <= 1 or ids[0] == 1:
                    return None
                x1 = x2 = y1 = y2 = 0
                point_11, point_21, point_31, point_41 = corners[0][0]
                x1, y1 = int((point_11[0] + point_21[0] + point_31[0] + point_41[0]) / 4.0), int(
                    (point_11[1] + point_21[1] + point_31[1] + point_41[1]) / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int((point_1[0] + point_2[0] + point_3[0] + point_4[0]) / 4.0), int(
                    (point_1[1] + point_2[1] + point_3[1] + point_4[1]) / 4.0)

                return x1, x2, y1, y2
        return None
    
    # draw aruco
    def draw_rectangle(self, img, x, y):
        # draw rectangle on img 在 img 上绘制矩形
        cv2.rectangle(
            img,
            (x - 20, y - 20),
            (x + 20, y + 20),
            (0, 255, 0),
            thickness=2,
            lineType=cv2.FONT_HERSHEY_COMPLEX,
        )
        # add text on rectangle
        cv2.putText(img, "({},{})".format(x, y), (x, y),
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (243, 0, 0), 2,)

    # set parameters to calculate the coords between cube and mycobot280
    # 设置参数以计算立方体和 mycobot 之间的坐标
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 220.0/ratio

    # set camera clipping parameters 设置相机裁剪参数
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)

    def pump_on(self):
        # 让2号位工作
        self.mc.set_basic_output(2, 0)
        # 让5号位工作
        self.mc.set_basic_output(5, 0)

    # 停止吸泵 m5
    def pump_off(self):
        # 让2号位停止工作
        self.mc.set_basic_output(2, 1)
        # 让5号位停止工作
        self.mc.set_basic_output(5, 1)

#my function for tic tac toe
def block_centers(row, col):
    centers_of_square = {            
        (0, 0): [213.5, 57.5], (0, 1): [221.3, 7.9], (0, 2): [212.7, -49.3], (1, 0): [158.4, 50], (1, 1): [167.2, 3], (1, 2): [172.4, -51.2], (2, 0): [114.1, 52.9], 
        (2, 1): [121.1, 4.1], (2, 2): [113.9, -56.0]
    }
    if (row, col) in centers_of_square:
        return centers_of_square[(row, col)]
    else:
        return None

def detect_color(frame, lower_color, upper_color, color_name, min_rect_size):
    # Convert the frame from BGR to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Threshold the image to get only the specified color
    color_mask = cv2.inRange(hsv, lower_color, upper_color)

    # Find contours in the binary image
    contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    color_centers = []

    # Check if there are any contours before processing
    if contours:
        # Draw rectangles around the color patches
        for contour in contours:
            # Get the bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)

            # Check if the rectangle is bigger than the minimum size
            if w > min_rect_size and h > min_rect_size:
                # Calculate the center of the rectangle
                center_x = x + w // 2
                center_y = y + h // 2

                # Draw rectangle
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Append the center coordinates to the list
                color_centers.append([center_x, center_y])

    return frame, color_centers



columns = 3
rows = 3

def board_initialization():
        return [[' ' for _ in range(columns)] for _ in range(rows)]

#preparing the grid like structure for the tic tac toe
'''
takes a frame (presumably an image) as input and draws a tic-tac-toe board on it. The board is divided into a grid with horizontal and vertical lines.
    It extracts the height, width, and number of color channels from the input frame.
    It calculates the width and height of each cell in the grid based on the number of columns (columns) and rows (rows).
    It defines ranges for the x and y coordinates to help with drawing the grid lines.
    It uses OpenCV (cv2) to draw horizontal lines at each cell height and vertical lines at each cell width. The color of the lines is set to black (0, 0, 0), and the line thickness is set to 2 pixels.
    The function returns the ranges for x and y coordinates, which might be useful for placing game pieces or detecting player pickup_place_cordss.
    '''
def draw_board(frame):
    height, width, _ = frame.shape
    width_cell = width // columns
    height_cell = height // rows
    x_range = [0, width_cell, width_cell * 2, width_cell * 3]
    y_range = [0, height_cell, height_cell * 2, height_cell * 3]

    # Draw horizontal lines
    for i in range(1, rows):
        cv2.line(frame, (0, i * height_cell), (width, i * height_cell), (0,0,0), 3)

    # Draw vertical lines
    for j in range(1, columns):
        cv2.line(frame, (j * width_cell, 0), (j * width_cell, height), (0,0,0), )

    return x_range, y_range

'''
The function get_grid_indices takes a pair of coordinates (x, y) and two lists (list_x and list_y) that represent the ranges of x and y coordinates, respectively. It returns the grid indices corresponding to the cell in which the given coordinates fall.
    It finds the index of the first element in list_x that is greater than or equal to the given x coordinate. If no such element is found, it sets value_x to the last index (len(list_x) - 1).
    It does the same for the list_y, finding the index of the first element greater than or equal to the given y coordinate. If no such element is found, it sets value_y to the last index (len(list_y) 
    It returns a tuple (value_x - 1, value_y - 1). The subtraction by 1 is likely intended to convert from 1-based indexing to 0-based indexing.'''
def get_grid_indices(x, y, list_x, list_y):
    value_x = next((i for i, val in enumerate(list_x) if val >= x), len(list_x) - 1)
    value_y = next((i for i, val in enumerate(list_y) if val >= y), len(list_y) - 1)

    return value_x-1, value_y-1

def print_board(board):
    for row in board:
        print(" ||".join(row))
        print("---------")

#for the pickup_place_cordsment
def pickup_place_cords(detect, position_X, position_Y):
    detect.mc.send_angles(detect.pickup_place_cords_angles[1], 20)
    time.sleep(2)
    detect.mc.send_coords([145.1, -121.1, 167.2, -173.9, -2.67, -139.2], 40, 1)
    time.sleep(2)
    detect.mc.send_coords([135.3, -140.9, 119.1, -171.63, -1.68, -142.4], 40, 1)
    time.sleep(2)
    detect.pump_on()
    time.sleep(2)
    detect.mc.send_coords([145.1, -121.1, 167.2, -173.9, -2.67, -139.2], 40, 1)
    time.sleep(2)
    x,y = block_centers(position_X, position_Y)
    detect.mc.send_coords([x,y, 103+46, 179.87, -3.78, -62.75], 40, 1)
    time.sleep(2)
    detect.mc.send_coords([x,y, 103, 179.87, -3.78, -62.75], 40, 1)
    time.sleep(2)
    detect.pump_off()
    time.sleep(2)
    detect.mc.send_coords([x,y, 103+46, 179.87, -3.78, -62.75], 40, 1)
    time.sleep(2)
    detect.mc.send_angles(detect.pickup_place_cords_angles[0], 25)
    time.sleep(2)


def play_tic_tac_toe_with_camera(detect, cap):
    board = board_initialization()
    player_turn = True  # True for player X, False for player O
    while True:
        ret, frame = cap.read()
        #print("before transform frame")

        frame = detect.transform_frame(frame)
        #print("after transform frame")

        frame = cv2.flip(frame,-1)
        x_range, y_range = draw_board(frame)
        if not ret:
            print("Error captured frame")
            break

        # Draw Tic Tac Toe board on the frame
        #draw_board(frame)

        # Draw X or O based on player's turn
        if player_turn:
            cv2.putText(frame, "Karteek's turn", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 3, cv2.LINE_AA)
        else:
            cv2.putText(frame, "MyCobot turn", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 3, cv2.LINE_AA)

        # Detect keyboard input to make a pickup_place_cords
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # Press 'Esc' to exit
            break

        if key == ord(' ') and player_turn:  # Press 'Space' to make a pickup_place_cords for Player X
            

            image = frame  # Replace with the actual image path

            cv2.waitKey(4)
            result_image_yellow, yellow_centers = detect_color(image, lower_yellow, upper_yellow, "Yellow", min_rect_size)

           
            
            for center in yellow_centers:
                x, y = center
                value_x, value_y = get_grid_indices(x, y, x_range, y_range)
                #blue_color_list.append((value_x, value_y))
                if board[value_y][value_x] == ' ':
                    board[value_y][value_x] = 'X'

                    #draw_pickup_place_cords(image, value_y, value_x, 'O')
                
                    #print(x_range, y_range)
                    #print("green :", green_centers)
                    #print("blue ", blue_centers)
                    print_board(board)
                    print("---------------------------------")

                    cv2.imshow("Project_Tic_Tac_Toe", image)
                    #cv2.waitKey(5)
                    #time.sleep(3)
                    player_turn = not player_turn
                else:
                    print("Position opted for")
 
                


        # AI makes a pickup_place_cords for Player O
        if not player_turn:
            best_pickup_place_cords = find_best_pickup_place_cords(board)
            if best_pickup_place_cords:
                row, col = best_pickup_place_cords
                board[row][col] = 'O'
                player_turn = not player_turn
            print_board(board)
            print("******************************")
            #print(row, col)
            pickup_place_cords(detect, row, col)


        # Display the resulting frame
        cv2.imshow("Tic Tac Toe", frame)
        # Check for game over conditions
        if cross_check_winner(board, 'X'):
            print("Player wins!")
            time.sleep(3)
            break
        elif cross_check_winner(board, 'O'):
            print("Computer wins!")
            time.sleep(3)
            break
        elif is_board_full(board):
            print("Match draw!") 
            time.sleep(3)
            break

    cap.release()
    cv2.destroyAllWindows()

# Function to get available pickup_place_cordss on the board
def get_available_pickup_place_cordss(board):
    if not board or not all(isinstance(row, str) and len(row) == len(board[0]) for row in board):
        raise ValueError("Invalid board format")

    return [(i, j) for i in range(len(board)) for j in range(len(board[0])) if board[i][j] == ' ']

# Function to check if the board is full (a draw)
def is_board_full(board, rows, columns, empty_symbol=' '):
    return all(board[i][j] != empty_symbol for i in range(rows) for j in range(columns))

'''
AI algorithm for playing Tic-Tac-Toe. It uses the minimax algorithm to evaluate and choose the best pickup_place_cords for the 'O' player. 
    Initialization: best_val is set to negative infinity, and best_pickup_place_cords is set to None. These variables will be used to track the best pickup_place_cords and its associated minimax value.
    Iteration through available pickup_place_cordss: It iterates over the available pickup_place_cordss obtained from the get_available_pickup_place_cordss function. The loop variable pickup_place_cords likely represents a tuple (row, col) indicating a possible pickup_place_cords on the Tic-Tac-Toe board.
    Simulation of pickup_place_cords: For each pickup_place_cords, it temporarily places an 'O' at the specified position on the board.
    Minimax evaluation: It then calls the minimax function with the updated board, depth=0, and is_maximizing=False. The minimax function is expected to return the minimax value for the given state of the board.
    Undoing the pickup_place_cords: After evaluating the pickup_place_cords, it resets the board at the specified position to an empty space (' ') to undo the simulated pickup_place_cords.
    Updating the best pickup_place_cords: If the evaluated pickup_place_cords's minimax value (pickup_place_cords_val) is greater than the current best_val, it updates best_pickup_place_cords and best_val with the current pickup_place_cords and its value.
    Return the best pickup_place_cords: After iterating through all available pickup_place_cordss, it returns the pickup_place_cords that resulted in the highest minimax value.
'''
def find_best_pickup_place_cords(board):
    best_val = float('-inf')
    best_pickup_place_cords = None

    for pickup_place_cords in get_available_pickup_place_cordss(board):
        row, col = pickup_place_cords
        board[row][col] = 'O'
        pickup_place_cords_val = minimax(board, 0, False)
        board[row][col] = ' '

        if pickup_place_cords_val > best_val:
            best_pickup_place_cords = pickup_place_cords
            best_val = pickup_place_cords_val

    return best_pickup_place_cords

'''
function is a recursive implementation of the minimax algorithm for a Tic-Tac-Toe game. The function evaluates the game state and returns a score based on whether the 'O' player or the 'X' player is maximizing their score.
    Base Cases:
        If the 'O' player has won, the function returns 1.
        If the 'X' player has won, the function returns -1.
        If the board is full and there's no winner, the function returns 0.
    Recursive Cases:
        If it's the turn for the maximizing player (currently 'O'), the function iterates through available pickup_place_cordss, simulates each pickup_place_cords, and calls itself recursively with the updated board and the next player's turn ('X'). It then updates max_eval with the maximum value obtained from these recursive calls.
        If it's the turn for the minimizing player (currently 'X'), the function does the same but minimizes the values by updating min_eval with the minimum value obtained from recursive calls.
    Undoing pickup_place_cordss:
        After each pickup_place_cords simulation, the board is reverted to its original state by resetting the cell to an empty space.
    Return Values:
        For the maximizing player ('O'), the function returns the maximum value (max_eval) obtained from recursive calls.
        For the minimizing player ('X'), the function returns the minimum value (min_eval) obtained from recursive calls.
This function works in tandem with the find_best_pickup_place_cords function you provided earlier to determine the best pickup_place_cords for the 'O' player in the Tic-Tac-Toe game. The minimax function evaluates different pickup_place_cords possibilities by recursively exploring the game tree and assigning scores to each possible outcome.'''

def minimax(board, depth, is_maximizing):
    if cross_check_winner(board, 'O'):
        return 1
    elif cross_check_winner(board, 'X'):
        return -1
    elif is_board_full(board):
        return 0

    if is_maximizing:
        max_eval = float('-inf')
        for pickup_place_cords in get_available_pickup_place_cordss(board):
            row, col = pickup_place_cords
            board[row][col] = 'O'
            eval = minimax(board, depth + 1, False)
            board[row][col] = ' '
            max_eval = max(max_eval, eval)
        return max_eval
    else:
        min_eval = float('inf')
        for pickup_place_cords in get_available_pickup_place_cordss(board):
            row, col = pickup_place_cords
            board[row][col] = 'X'
            eval = minimax(board, depth + 1, True)
            board[row][col] = ' '
            min_eval = min(min_eval, eval)
        return min_eval

def cross_check_winner(board, player):
        # Check rows and columns
        for i in range(rows):
            if all(board[i][j] == player for j in range(columns)) or all(board[j][i] == player for j in range(rows)):
                return True

        # Check diagonals
        if all(board[i][i] == player for i in range(rows)) or all(
                board[i][rows - 1 - i] == player for i in range(rows)):
            return True

        return False

def keyboard_pickup_place_cords():
    print("Enter row and column (separated by space):")
    pickup_place_cords = input().split()
    row, col = map(int, pickup_place_cords)
    return row-1, col-1

if __name__ == "__main__":
    import platform
    # open the camera
    if platform.system() == "Windows":
        cap_num = 1
        cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)

        if not cap.isOpened():
            cap.open(1)
    elif platform.system() == "Linux":
        cap_num = -1
        cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
        if not cap.isOpened():
            cap.open()

    # init a class of Object_detect
    detect = Object_detect()
    # init mycobot280
    detect.run()
    detect.pump_off()


    _init_ = 20
    init_num = 0
    nparams = 0
    arukoOK = 0
    # Define the range of green color in HSV
    lower_green = np.array([35, 43, 35])
    upper_green = np.array([90, 255, 255])

    lower_yellow= np.array([11, 85, 70])
    upper_yellow= np.array([59, 255, 245])
    # Set the minimum rectangle size
    min_rect_size = 50


    while arukoOK == 0:
       # read camera
        #frame = cv2.imread('aruko.png')
        # deal img
        _, frame = cap.read()
        frame = detect.transform_frame(frame)

        cv2.imshow("Tic Tac Toe", frame)
        cv2.waitKey(5)        
        if _init_ > 0:
            _init_ -= 1
            continue

        # calculate the parameters of camera clipping 计算相机裁剪的参数
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                #cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_rectangle(frame, x1, y1)
                detect.draw_rectangle(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                init_num += 1
                continue
        elif init_num == 20:
            detect.set_cut_params(
                (detect.sum_x1)/20.0-20,
                (detect.sum_y1)/20.0,
                (detect.sum_x2)/20.0,
                (detect.sum_y2)/20.0-20,
            )
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        # calculate params of the coords between cube and mycobot280 计算立方体和 mycobot 之间坐标的参数
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                #cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_rectangle(frame, x1, y1)
                detect.draw_rectangle(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                nparams += 1
                continue
        elif nparams == 10:
            nparams += 1
            # calculate and set params of calculating real coord between cube and mycobot280
            detect.set_params(
                (detect.sum_x1+detect.sum_x2)/20.0,
                (detect.sum_y1+detect.sum_y2)/20.0,
                abs(detect.sum_x1-detect.sum_x2)/10.0 +
                abs(detect.sum_y1-detect.sum_y2)/10.0
            )
            print("Start")
            arukoOK = 1
            continue

    play_tic_tac_toe_with_camera(detect, cap)
