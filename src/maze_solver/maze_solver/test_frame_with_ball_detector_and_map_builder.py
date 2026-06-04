import cv2
from ultralytics import YOLO

from map_builder import MapBuilder
from ball_detector import BallDetector

def main():

    model_path='/home/egor/camera_on_top_ws/src/maze_solver/config/ball_detector.pt'
    yolo = YOLO(model_path)

    image_path = 'camera_frame.png'
    frame = cv2.imread(image_path)

    if frame is None:
        print(f'Failed to read image: {image_path}')
        return

    map_builder = MapBuilder()
    ball_detector = BallDetector()

    ball_result = ball_detector.detect_red_ball(frame)

    print(ball_result[0],ball_result[1])

    map_result = map_builder.build_map(frame)
    cv2.imshow('occupancy',map_result * 255)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    robot_initial = map_builder.detect_robot_initial(frame)
    print(robot_initial)

    exit_goal = map_builder.detect_exit()
    print(exit_goal)
    

    print('Saved:')
    #print('/tmp/map_result.png')
    results = yolo(frame,verbose =False)


if __name__ == '__main__':
    main()