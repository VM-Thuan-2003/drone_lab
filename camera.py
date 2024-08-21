from picamera2 import Picamera2
import numpy as np
import cv2  # type: ignore # OpenCV for image conversion
import time

class Camera:
    def __init__(self, resolution=(640, 480), video_filename="output.mp4"):
        # Initialize Picamera2
        self.picam2 = Picamera2()

        self.isRunCamera = False

        self.resolution = resolution

        # Create a preview configuration with the specified resolution
        self.camera_config = self.picam2.create_preview_configuration(main={"size": resolution, 'format': 'BGR888'})

        # Configure the camera with the created configuration
        self.picam2.configure(self.camera_config)

        # Start the camera
        self.picam2.start()

        # Initialize size to the full resolution
        # self.original_size = list(self.resolution)
        self.original_size = self.picam2.capture_metadata()['ScalerCrop'][2:]
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

    def read_camera(self):
        # Capture an image from the camera as a raw array
        raw_image = self.picam2.capture_array()

        # Convert the raw image to RGB888 format
        rgb_image = self.convert_to_rgb888(raw_image)

        return rgb_image

    def convert_to_rgb888(self, raw_image):
        # Check number of channels
        channels = raw_image.shape[2] if len(raw_image.shape) == 3 else 1

        if channels == 1:
            # Assuming grayscale image
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_GRAY2RGB)
        elif channels == 3:
            # Assuming BGR or RGB image
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB)
        elif channels == 4:
            # Assuming RGBA image
            rgb_image = cv2.cvtColor(raw_image, cv2.COLOR_RGBA2RGB)
        else:
            raise ValueError("Unsupported number of channels in raw image.")

        return rgb_image

    def set_manual_focus(self, focus_value):
        """Set the camera to manual focus mode with a specified lens position in meters."""
        # Ensure focus_value is in meters
        if not isinstance(focus_value, (int, float)):
            raise ValueError("focus_value must be a numeric type representing distance in meters.")
        self.picam2.set_controls({"AfMode": 0, "LensPosition": focus_value})

    def set_single_autofocus(self):
        """Set the camera to single autofocus mode."""
        # First, ensure the camera is in a mode that supports autofocus
        self.picam2.set_controls({"AfMode": 1})  # Set to single autofocus mode
        # Then, trigger autofocus
        self.picam2.set_controls({"AfTrigger": 0})

    def set_continuous_autofocus(self):
        """Set the camera to continuous autofocus mode."""
        # First, ensure the camera is in a mode that supports autofocus
        self.picam2.set_controls({"AfMode": 2})  # Set to continuous autofocus mode
        # Trigger continuous autofocus (no additional trigger might be needed)
        self.picam2.set_controls({"AfTrigger": 0})

    def set_zoom(self, zoom_factor):
        if zoom_factor <= 0:
            raise ValueError("Zoom factor must be a positive number.")

        # Calculate the zoom factor adjustment
        if zoom_factor == self.current_zoom_factor:
            return  # No change if the zoom factor is the same

        # Adjust size based on the new zoom factor
        if zoom_factor > self.current_zoom_factor:
            # Zooming in
            for _ in range(int(zoom_factor - self.current_zoom_factor)):
                self.size = [int(s * 0.95) for s in self.size]
        else:
            # Zooming out
            for _ in range(int(self.current_zoom_factor - zoom_factor)):
                self.size = [int(s / 0.95) for s in self.size]

        # Calculate the offset to center the zoomed region
        offset = [(r - s) // 2 for r, s in zip(self.original_size, self.size)]

        # Print debugging information
        print(f"size: {self.size} - offset: {offset} - full region: {offset + self.size}")

        # Set the zoom by configuring the ScalerCrop
        self.picam2.set_controls({"ScalerCrop": offset + self.size})

        # Update the current zoom factor
        self.current_zoom_factor = zoom_factor

    def release(self):
        """Release resources: stop the camera and release the video writer."""
        self.picam2.stop()
        self.video_writer.release()

if __name__ == "__main__":
    # Create an instance of the Camera class
    camera = Camera(resolution=(1280, 720), video_filename="output_7_8_2024_3.mp4")

    # Example zoom factors
    camera.set_zoom(5.0)  # Zoom in to 5x
    time.sleep(2)  # Allow time to observe the zoom change
    camera.set_zoom(2.0)  # Zoom out to 2x

    # Other operations...
    camera.set_continuous_autofocus()

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

    print("Camera released and OpenCV windows closed.")