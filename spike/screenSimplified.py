import subprocess
import time
import os
import pty

def main():
    if not os.path.exists("/dev/ttyACM0"):
        return

    master_fd, slave_fd = pty.openpty()
    
    subprocess.Popen(
        ["screen", "/dev/ttyACM0", "115200"],
        stdin=slave_fd,
        stdout=slave_fd,
        stderr=slave_fd,
    )

    os.close(slave_fd)

    # Reduced initial wait time.
    # This delay is for screen to initialize. If the script fails, try increasing this value.
    time.sleep(1.0)
    os.write(master_fd, b'\x03') # Ctrl+C
    # Reduced delay between commands.
    # These are to give screen time to process each keystroke.
    time.sleep(0.2)
    os.write(master_fd, b'\x01') # Ctrl+A
    time.sleep(0.2)
    os.write(master_fd, b'k')
    time.sleep(0.2)
    os.write(master_fd, b'y')
    time.sleep(0.2)
    
    os.close(master_fd)

if __name__ == "__main__":
    main()
