

import os
import argparse
import sys
import time

import RPi.GPIO as GPIO

import cv2
import numpy as np
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision


_MARGIN = 10  # pixels
_ROW_SIZE = 10  # pixels
_FONT_SIZE = 1
_FONT_THICKNESS = 1
_TEXT_COLOR = (0, 0, 255)  # red
panel = None 

#=================================================================================================
#-------------------------------------------------------------------------------------------------
#=================================================================================================
# Set the GPIO mode
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

S_L = 37 
S_R = 11 

# Define the pins
PIN_ENA = 32
PIN_IN1 = 16   # Motor1 L298N Pin in1 
PIN_IN2 = 18   # Motor1 L298N Pin in2
PIN_ENB = 33
PIN_IN3 = 38   # Motor2 L298N Pin in3
PIN_IN4 = 40   # Motor2 L298N Pin in4

# Set up the GPIO pins
GPIO.setup(PIN_ENA, GPIO.OUT)
GPIO.setup(PIN_IN1, GPIO.OUT)
GPIO.setup(PIN_IN2, GPIO.OUT)
GPIO.setup(PIN_ENB, GPIO.OUT)
GPIO.setup(PIN_IN3, GPIO.OUT)
GPIO.setup(PIN_IN4, GPIO.OUT)

pwm_a = GPIO.PWM(PIN_ENA,100)
pwm_b = GPIO.PWM(PIN_ENB,100)

GPIO.setup(S_L,GPIO.IN)
GPIO.setup(S_R,GPIO.IN)
    
def set_motor_speeds(left_speed, right_speed):    
#     pwm_freq_a = abs(left_speed)   
#     pwm_freq_b = abs(right_speed)   
#     pwm_a = GPIO.PWM(PIN_ENA, pwm_freq_a)
#     pwm_b = GPIO.PWM(PIN_ENB, pwm_freq_b)
    
    if left_speed >= 0:
        GPIO.output(PIN_IN1, GPIO.HIGH)
        GPIO.output(PIN_IN2, GPIO.LOW)
    else:
        GPIO.output(PIN_IN1, GPIO.LOW)
        GPIO.output(PIN_IN2, GPIO.HIGH)

    if right_speed < 0:
        GPIO.output(PIN_IN3, GPIO.HIGH)
        GPIO.output(PIN_IN4, GPIO.LOW)
    else:
        GPIO.output(PIN_IN3, GPIO.LOW)
        GPIO.output(PIN_IN4, GPIO.HIGH)

    pwm_a.start(abs(left_speed))
    pwm_b.start(abs(right_speed))
#=================================================================================================
#-------------------------------------------------------------------------------------------------
#=================================================================================================






#=================================================================================================
#-------------------------------------------------------------------------------------------------
#=================================================================================================

def visualize(
    image: np.ndarray,
    detection_result: processor.DetectionResult,) -> np.ndarray:
  global panel
  for detection in detection_result.detections:
    # Draw bounding_box
    bbox = detection.bounding_box
    start_point = bbox.origin_x, bbox.origin_y
    end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
    cv2.rectangle(image, start_point, end_point, _TEXT_COLOR, 3)

    # Draw label and score
    category = detection.categories[0]
    category_name = category.category_name
    panel = category_name
    
    
    probability = round(category.score, 2)
    result_text = category_name + ' (' + str(probability) + ')'
    text_location = (_MARGIN + bbox.origin_x,
                     _MARGIN + _ROW_SIZE + bbox.origin_y)
    cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)
    
  return image


def run(model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool) -> None:
  """Capture a single image from the camera and run object detection on
  Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
    num_threads: The number of CPU threads to run the model.
    enable_edgetpu: True/False whether the model is an EdgeTPU model.
  """
  
  
  
  # Initialize the object detection model
  base_options = core.BaseOptions(
      file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
  detection_options = processor.DetectionOptions(
      max_results=3, score_threshold=0.3)
  options = vision.ObjectDetectorOptions(
      base_options=base_options, detection_options=detection_options)
  detector = vision.ObjectDetector.create_from_options(options)

  # Capture a single image from the camera
  cap = cv2.VideoCapture(camera_id)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
  
  success, image = cap.read()
  
  if not success:
    sys.exit(
        'ERROR: Unable to read from webcam. Please verify your webcam settings.'
    )

  # Convert the image from BGR to RGB as required by the TFLite model.
  rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

  # Create a TensorImage object from the RGB image.
  input_tensor = vision.TensorImage.create_from_array(rgb_image)
  
  # Run object detection estimation using the model.
  detection_result = detector.detect(input_tensor)

  # Draw keypoints and edges on input image
  image = visualize(image, detection_result)
  
  # Display the image with bounding boxes
  if "DISPLAY" in os.environ:
        cv2.imshow('object_detector', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
#=================================================================================================
#-------------------------------------------------------------------------------------------------
#=================================================================================================

def main():
  
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      default='efficientdet_lite0.tflite')
  parser.add_argument(
      '--cameraId', help='Id of camera.', required=False, type=int, default=0)
  parser.add_argument(
      '--frameWidth',
      help='Width of frame to capture from camera.',
      required=False,
      type=int,
      default=640)
  parser.add_argument(
      '--frameHeight',
      help='Height of frame to capture from camera.',
      required=False,
      type=int,
      default=480)
  parser.add_argument(
      '--numThreads',
      help='Number of CPU threads to run the model.',
      required=False,
      type=int,
      default=4)
  parser.add_argument(
      '--enableEdgeTPU',
      help='Whether to run the model on EdgeTPU.',
      action='store_true',
      required=False,
      default=False)
  args = parser.parse_args()

  run(args.model, int(args.cameraId), args.frameWidth, args.frameHeight,
      int(args.numThreads), bool(args.enableEdgeTPU))
      

if __name__ == '__main__':
    try :
        while(True):
            main()
            if GPIO.input(S_L) == 1 :
                set_motor_speeds(10,100)
            if GPIO.input(S_R) == 1 :
                set_motor_speeds(100,10)
            if GPIO.input(S_L) == 0 or GPIO.input(S_R) == 0 :
                set_motor_speeds(100,100)
            if panel == "green":
                set_motor_speeds(90,90)
            if panel == "red":
                set_motor_speeds(20,20)
            if panel == "panel001":
                set_motor_speeds(50,50)
            if panel == "panel010":
                set_motor_speeds(20,20)
            if panel == "panel011":
                set_motor_speeds(60,60)
            if panel == "panel100":
                 set_motor_speeds(50,50)
            if panel == "panel101":
                set_motor_speeds(90,90)
                time.sleep(4)
                set_motor_speeds(100,100)
            if panel == "panel110":
                set_motor_speeds(10,10)
                
    except KeyboardInterrupt:
        GPIO.cleanup()   # Clean up GPIO on program exit
