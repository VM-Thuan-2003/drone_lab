import cv2
import os

def get_minimal_video_port():
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
            print(f"Camera connected and accessible at {device}")
            cap.release()  # Release the camera after checking
            if minimal_device is None:
                minimal_device = device  # Set the first accessible device as minimal
        else:
            print(f"Failed to access camera at {device}")
            if minimal_device is None:
                minimal_device = device  # Still set this as the minimal device if nothing else is accessible
    
    if minimal_device:
        return minimal_device
    else:
        print("No accessible video devices found.")
        return None

if __name__ == "__main__":
    minimal_port = get_minimal_video_port()
    if minimal_port:
        print(f"The minimal video device is: {minimal_port}")
