import cv2 as cv
import numpy as np
from collections import deque
from ultralytics import YOLO

# Global variable to store a history of angles for smoothing
angle_history = deque(maxlen=10)  # Keeps the last 10 calculated angles

def find_correct_four_points(contour):
    """
    Finds the four critical points of a contour for a green boundary:
    - Bottom-left, Bottom-right, Top-left, Top-right
    """
    # Sort points based on y-coordinates (top to bottom)
    sorted_by_y = sorted(contour, key=lambda point: point[0][1])
    
    # Top points (smallest y-values)
    top_points = sorted_by_y[:2]
    # Bottom points (largest y-values)
    bottom_points = sorted_by_y[-2:]
    
    # Sort top points by x-coordinates (left to right)
    top_left, top_right = sorted(top_points, key=lambda point: point[0][0])
    # Sort bottom points by x-coordinates (left to right)
    bottom_left, bottom_right = sorted(bottom_points, key=lambda point: point[0][0])
    
    return bottom_left[0], bottom_right[0], top_left[0], top_right[0]

def calculate_angle(yellow_line, white_line):
    """
    Calculate the angle between the yellow and white lines.
    """
    x1, y1 = yellow_line
    x2, y2 = white_line

    # Direction vectors
    vector_yellow = np.array([0, y1 - y2])
    vector_white = np.array([x2 - x1, y2 - y1])

    # Handle edge cases
    if np.linalg.norm(vector_white) == 0:
        return 0  # No valid white line

    # Calculate angle
    angle_rad = np.arctan2(np.cross(vector_yellow, vector_white), np.dot(vector_yellow, vector_white))
    angle_deg = np.degrees(angle_rad)

    # Adjust angle for left/right side
    if angle_deg > 0:
        angle_deg = 180 - angle_deg
    elif angle_deg < 0:
        angle_deg = -180 - angle_deg

    return angle_deg

def smooth_angle(new_angle):
    """
    Smooth the angle by averaging with previous angles, with a check for large angle changes.
    """
    if len(angle_history) > 0 and abs(new_angle - angle_history[-1]) > 7:
        # If the difference between the new angle and the last stored angle exceeds 7 degrees,
        # use the average of the previous 10 angles
        smoothed_angle = np.mean(angle_history)
    else:
        # Otherwise, store the new angle directly
        smoothed_angle = new_angle

    angle_history.append(smoothed_angle)
    return smoothed_angle

def process_video_and_display(video_path, horizontal_line_position):
    # Load the video
    cap = cv.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Unable to load video at {video_path}. Please check the path.")
        return

    # Initialize YOLO model
    model = YOLO(r"C:\Users\Jejar\Desktop\EV Project\best.pt")

    # Set the desired width and height for the output window
    output_width = 1280  # Adjust width as needed
    output_height = 480  # Adjust height as needed

    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of video or error reading frames.")
            break

        frame_height, frame_width = frame.shape[:2]

        # Perform YOLO inference
        results = model(frame, agnostic_nms=True)

        # Extract detections
        detections = results[0].boxes.xyxy  # Bounding box coordinates
        masks = results[0].masks  # Segmentation masks, if available

        # Create a black background for the green line video
        black_frame = np.zeros_like(frame)

        # Middle bottom point of the display
        middle_bottom_point = (frame_width // 2, frame_height - 1)

        # Draw the configurable middle horizontal line at the user-defined position
        middle_horizontal_line_start = (0, horizontal_line_position)
        middle_horizontal_line_end = (frame_width - 1, horizontal_line_position)

        # Draw the middle horizontal line
        cv.line(black_frame, middle_horizontal_line_start, middle_horizontal_line_end, (255, 255, 255), 2)

        # Modified Yellow Line: Now connects from the bottom to the configurable horizontal line
        cv.line(black_frame, middle_bottom_point, (middle_bottom_point[0], horizontal_line_position), (0, 255, 255), 2)

        # Process YOLO detections
        red_marked_frame = frame.copy()  # Frame for red-marked visualization
        if masks:
            for mask in masks.data:  # Loop through each detected mask
                mask = mask.cpu().numpy().astype(np.uint8)  # Convert to numpy format

                # Resize the mask to match the frame size
                mask_resized = cv.resize(mask, (frame.shape[1], frame.shape[0]), interpolation=cv.INTER_NEAREST)

                # Find contours for the green boundary
                contours, _ = cv.findContours(mask_resized, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    # Draw the green boundary on the black frame
                    cv.drawContours(black_frame, [contour], -1, (0, 255, 0), 2)

                    # Collect intersection points with the horizontal white line
                    intersection_points = []
                    for point in contour:
                        point_x, point_y = point[0]
                        if abs(point_y - horizontal_line_position) < 3:  # Check if near the horizontal line
                            intersection_points.append((point_x, point_y))

                    if len(intersection_points) >= 2:  # Ensure at least two intersections are found
                        # Sort intersection points from left to right
                        intersection_points.sort(key=lambda p: p[0])
                        left_intersection = intersection_points[0]
                        right_intersection = intersection_points[-1]

                        # Calculate the mean intersection point
                        mean_x = (left_intersection[0] + right_intersection[0]) // 2
                        mean_y = horizontal_line_position
                        mean_point = (mean_x, mean_y)

                        # Draw the mean point with a red circle
                        cv.circle(black_frame, mean_point, 6, (0, 0, 255), -1)

                        # Draw a white vertical line connecting the mean point to the bottom center
                        cv.line(black_frame, mean_point, middle_bottom_point, (255, 255, 255), 2)

                        # Calculate and smooth the angle between the yellow and white lines
                        angle = calculate_angle(middle_bottom_point, mean_point)
                        smoothed_angle = smooth_angle(angle)

                        # Display the smoothed angle on the video
                        cv.putText(black_frame, f"Angle: {smoothed_angle:.2f}°", (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                # Add red overlay to the original frame for red-marked video
                red_overlay = np.zeros_like(frame)
                red_overlay[mask_resized > 0] = (0, 0, 255)  # Red for detected areas
                red_marked_frame = cv.addWeighted(red_marked_frame, 1, red_overlay, 0.5, 0)

        # Concatenate the two frames horizontally
        combined_frame = cv.hconcat([red_marked_frame, black_frame])

        # Resize the combined frame to fit the display
        resized_frame = cv.resize(combined_frame, (output_width, output_height))

        # Display the combined video
        cv.imshow("Red Marked and Black Background Video", resized_frame)

        # Exit video playback on pressing 'q'
        if cv.waitKey(1) & 0xFF == ord('q'):
            print("Playback interrupted by user.")
            break

    cap.release()
    cv.destroyAllWindows()

def main():
    video_path = r"C:\Users\Jejar\Desktop\EV Project\Video\Video3.mp4"  # Replace with the actual path to your video
    print("Starting real-time visualization of green boundary and red marked video...")

    # You can set this value to any number between 0 and L (frame height).
    horizontal_line_position = 360 

    process_video_and_display(video_path, horizontal_line_position)
    print("Process complete.")

if __name__ == "__main__":
    main()