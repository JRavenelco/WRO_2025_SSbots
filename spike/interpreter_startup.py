import subprocess
import time
import os
import pty

def main():
    # Check if the device exists
    if not os.path.exists("/dev/ttyACM0"):
        print("Error: Device /dev/ttyACM0 not found. Please ensure the device is connected.")
        return

    screen_command_list = ["screen", "/dev/ttyACM0", "115200"]
    print(f"Preparing to execute: {' '.join(screen_command_list)}")
    print("And then send Ctrl+A, k, y to the screen session.")

    # master_fd will be the file descriptor for the parent's side of the pty
    # slave_fd will be the file descriptor for the child's side of the pty
    master_fd, slave_fd = pty.openpty()
    
    process = None # Initialize process to None for robust cleanup

    try:
        # Start the screen process. Its stdin, stdout, and stderr will be connected to the slave end of the pty.
        process = subprocess.Popen(
            screen_command_list,
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            # close_fds=True is default for Popen when not using pipes for stdio,
            # but explicit can be good. Here, slave_fd is passed, so it's handled.
        )

        # The parent process no longer needs the slave file descriptor;
        # the child process (screen) has it.
        os.close(slave_fd)
        slave_fd = -1 # Mark as closed to avoid trying to close it again in finally

        print("Screen process started. Waiting a few moments for it to initialize...")
        # These delays are important. Adjust them if screen responds slower or faster on your system.
        time.sleep(2.5)

        # Send Ctrl+C (ASCII 0x03)
        print("Sending Ctrl+C (\\x03)")
        bytes_written = os.write(master_fd, b'\x03') # Corrected: Use single backslash for hex in byte literal
        if bytes_written != 1:
            print(f"Warning: os.write for Ctrl+C wrote {bytes_written} bytes, expected 1")
        time.sleep(0.75) # Wait for screen to process the Ctrl+C

        # Send Ctrl+A (ASCII 0x01) - screen's command character
        print("Sending Ctrl+A (\\x01)")
        bytes_written = os.write(master_fd, b'\x01') # Corrected: Use single backslash for hex in byte literal
        if bytes_written != 1:
            print(f"Warning: os.write for Ctrl+A wrote {bytes_written} bytes, expected 1")
        time.sleep(0.75) # Wait for screen to process the escape character

        # Send 'k' (for the 'kill' command in screen)
        print("Sending 'k'")
        bytes_written = os.write(master_fd, b'k')
        if bytes_written != 1:
            print(f"Warning: os.write for 'k' wrote {bytes_written} bytes, expected 1")
        time.sleep(0.75) # Wait for screen to process 'k' (it might show a confirmation prompt)

        # Send 'y' (to confirm the kill action)
        print("Sending 'y'")
        bytes_written = os.write(master_fd, b'y')
        if bytes_written != 1:
            print(f"Warning: os.write for 'y' wrote {bytes_written} bytes, expected 1")
        time.sleep(0.75) # Wait for screen to process 'y' and execute the kill

        print("Keystrokes sent. The screen window should be terminated.")
        
        # Now, wait for the screen process to exit.
        # The 'k' 'y' sequence should kill the current screen window.
        # If it's the only window, screen itself should exit.
        try:
            return_code = process.wait(timeout=10) # Wait up to 10 seconds
            print(f"Screen process exited with code: {return_code}")
        except subprocess.TimeoutExpired:
            print("Screen process did not exit within the timeout period.")
            print("This might mean it's still running (e.g., detached) or the kill command didn't fully terminate it as expected.")
            print("Attempting to terminate the process forcefully...")
            process.terminate() # Send SIGTERM
            try:
                process.wait(timeout=5) # Wait for termination
                print("Process terminated successfully after SIGTERM.")
            except subprocess.TimeoutExpired:
                print("Process did not terminate after SIGTERM. Sending SIGKILL.")
                process.kill() # Send SIGKILL
                process.wait() # Wait for kill (should be immediate)
                print("Process killed with SIGKILL.")
        except Exception as e:
             print(f"An error occurred while waiting for/terminating the screen process: {e}")

    except FileNotFoundError:
        print("Error: The 'screen' command was not found. Please ensure it is installed and in your system's PATH.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # Clean up file descriptors
        if 'master_fd' in locals() and master_fd != -1 :
             try:
                os.close(master_fd)
             except OSError as e:
                print(f"Error closing master_fd: {e}")
        
        if 'slave_fd' in locals() and slave_fd != -1: # Should have been closed or set to -1
             try:
                os.close(slave_fd)
             except OSError as e:
                print(f"Error closing slave_fd (should already be closed): {e}")
        
        # Ensure the child process is dealt with if it's still running due to an early exception
        if process and process.poll() is None:
            print("Cleaning up lingering screen process...")
            try:
                process.kill()
                process.wait()
                print("Lingering process killed.")
            except Exception as e:
                print(f"Error killing lingering process: {e}")


if __name__ == "__main__":
    main()
