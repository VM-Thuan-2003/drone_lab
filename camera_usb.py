import cv2
import time
import os

class CameraUsb:
    def __init__(self, resolution=(640, 480), video_filename="output.mp4"):
        # Initialize the video capture object with a USB camera (usually at index 0)
        
        # self.port = self.get_minimal_video_port()
        
        self.port = "/dev/video0"
        
        if self.port is not None:
            self.cap = cv2.VideoCapture(self.port)
        else:
            self.cap = cv2.VideoCapture(0)

        # Set the resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

        self.isRunCamera = False

        self.resolution = resolution

        # Initialize size to the full resolution
        self.original_size = resolution
        self.size = list(self.original_size)
        self.current_zoom_factor = 1.0  # Initial zoom factor

        # Initialize VideoWriter to save video
        self.video_filename = video_filename
        self.frame_width = resolution[0]
        self.frame_height = resolution[1]
        self.fps = 10  # Frames per second for the video
        self.video_writer = cv2.VideoWriter(
            self.video_filename,
            cv2.VideoWriter_fourcc(*'mp4v'),
            self.fps,
            (self.frame_width, self.frame_height)
        )

    def get_minimal_video_port(self):
        # List all video devices in the /dev directory
        dev_files = os.listdir('/dev')
        
        # Extract and sort video devices by their numeric part
        video_devices = sorted(
            [f"/dev/{file}" for file in dev_files if file.startswith('video') and file[5:].isdigit()],
            key=lambda x: int(x[10:])  # Extract the numeric part and sort by it
        )
        
        if not video_devices:
            print("No video devices found.")
            return None

        minimal_device = None
        
        print("Checking connected video devices:")
        for device in video_devices:
            # Try to open the video device with OpenCV
            cap = cv2.VideoCapture(device)
            if cap.isOpened():
                # print(f"Camera connected and accessible at {device}")
                cap.release()  # Release the camera after checking
                if minimal_device is None:
                    minimal_device = device  # Set the first accessible device as minimal
            else:
                # print(f"Failed to access camera at {device}")
                if minimal_device is None:
                    minimal_device = device  # Still set this as the minimal device if nothing else is accessible
        
        if minimal_device:
            return minimal_device
        else:
            print("No accessible video devices found.")
            return None
    
    def read_camera(self):
        # Capture an image from the camera
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to capture image from the camera")

        # Optionally apply zoom before returning the frame
        zoomed_frame = self.apply_zoom(frame)
        return zoomed_frame

    def apply_zoom(self, frame):
        # Calculate the center of the frame
        center_x, center_y = self.frame_width // 2, self.frame_height // 2

        # Calculate the size of the zoomed area
        width_zoomed = int(self.frame_width / self.current_zoom_factor)
        height_zoomed = int(self.frame_height / self.current_zoom_factor)

        # Calculate the top-left corner of the zoomed area
        top_left_x = max(center_x - width_zoomed // 2, 0)
        top_left_y = max(center_y - height_zoomed // 2, 0)

        # Extract the zoomed area from the frame
        zoomed_frame = frame[top_left_y:top_left_y + height_zoomed, top_left_x:top_left_x + width_zoomed]

        # Resize the zoomed area back to the original frame size
        zoomed_frame = cv2.resize(zoomed_frame, (self.frame_width, self.frame_height))
        return zoomed_frame

    def set_zoom(self, zoom_factor):
        if zoom_factor <= 0:
            raise ValueError("Zoom factor must be a positive number.")

        # Update the current zoom factor
        self.current_zoom_factor = zoom_factor

    def set_manual_focus(self, focus_value):
        """Set the camera to manual focus mode with a specified focus value."""
        # Ensure focus_value is within the range 0-255 (this range may vary depending on the camera)
        focus_value = max(0, min(focus_value, 255))
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
        self.cap.set(cv2.CAP_PROP_FOCUS, focus_value)  # Set manual focus

    def set_continuous_autofocus(self):
        """Enable autofocus mode."""
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)  # Enable autofocus

    def release(self):
        """Release resources: stop the camera and release the video writer."""
        self.cap.release()
        self.video_writer.release()

if __name__ == "__main__":
    # Create an instance of the Camera class
    camera = CameraUsb(resolution=(1280, 720), video_filename="output_usb_camera.mp4")

    # Example focus control
    camera.set_continuous_autofocus()  # Enable autofocus
    time.sleep(2)  # Allow time for autofocus to adjust
    # camera.set_manual_focus(100)  # Set manual focus to a specific value

    # Example zoom factors
    camera.set_zoom(2.0)  # Zoom in to 2x
    time.sleep(2)  # Allow time to observe the zoom change
    camera.set_zoom(1.0)  # Zoom out to 1x

    while True:
        # Capture an image from the camera
        image = camera.read_camera()

        # Write the frame to the video file
        camera.video_writer.write(image)

        # Display the image using OpenCV
        cv2.imshow("Captured Image", image)

        # Check if the 'q' key is pressed to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up and close all OpenCV windows
    camera.release()
    cv2.destroyAllWindows()
