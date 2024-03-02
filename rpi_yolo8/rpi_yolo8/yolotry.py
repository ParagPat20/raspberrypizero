from ultralytics import YOLO
import cv2

# Load a COCO-pretrained YOLOv8n model
model = YOLO('yolov8n.pt')

# Define the video capture source
cap = cv2.VideoCapture(0)  # Change 0 to your camera index if needed

# Function to process each frame and display results
def process_frame(frame):
  # Run inference on the frame
  results = model(frame)

  # Draw bounding boxes and labels on the frame
  results.render()

  # Display the frame with detections
  cv2.imshow('Video with Detections', frame)

  # Exit on pressing 'q' key
  if cv2.waitKey(1) & 0xFF == ord('q'):
    return False

  return True

# Main loop to capture video and process frames
while True:
  # Capture frame-by-frame
  ret, frame = cap.read()

  # Check if frame is captured successfully
  if not ret:
    print("Error capturing frame")
    break

  # Process the frame and check for exit
  if not process_frame(frame):
    break

# Release the capture and close all windows
cap.release()
cv2.destroyAllWindows()
