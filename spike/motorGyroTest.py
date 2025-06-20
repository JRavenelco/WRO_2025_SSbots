\
import subprocess
import time
import os
# No more pty needed directly in this file for interpreter prep
import serial
import sys
import re

# --- Spike Communication Config ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- Robot Config ---
TRACTION_MOTOR_PORT = 'B'  # Port for forward movement
STEERING_MOTOR_PORT = 'F'  # Port for steering
FORWARD_SPEED = 400        # Speed for traction motor (0-1000, adjust as needed)

# --- Gyro & Control Config ---
# IMPORTANT: The 'motion_sensor' module and its functions (reset_yaw_angle, get_yaw_angle)
# are assumed based on common LEGO Spike Prime Python usage.
# The provided spike_API.txt does NOT explicitly list this module.
# If these commands fail, you'll need to find the correct API for your Spike's gyro.

TARGET_HEADING = 0.0         # Target yaw angle (0 after reset)
KP_STEERING = 15.0           # Proportional gain for steering (tune this)
STEERING_PULSE_DURATION = 0.05 # Duration for a steering motor pulse in seconds (tune this)
MAX_STEER_POWER = 350        # Max power for steering motor (-1000 to 1000, tune this)
MIN_STEER_POWER_THRESHOLD = 60 # Min absolute power to apply to steering to overcome friction (tune)
ERROR_THRESHOLD = 1.5        # Degrees of error before trying to correct (dead zone, tune this)

# --- interpreter_startup.py logic integration ---
def run_interpreter_startup_script():
    """
    Runs the interpreter_startup.py script as a separate process to prepare
    the Spike Hub.
    """
    script_path = "/home/jetson/WRO_2025_SSbots/spike/interpreter_startup.py"
    if not os.path.exists(script_path):
        print(f"Error: interpreter_startup.py not found at {script_path}")
        return False

    print(f"Running {script_path} as a separate process...")
    try:
        # Execute interpreter_startup.py using python3
        # Timeout is generous, original script has internal waits.
        completed_process = subprocess.run(
            ["python3", script_path], 
            capture_output=True, 
            text=True, 
            timeout=30,
            check=False # Do not raise exception on non-zero exit, we check returncode manually
        )
        
        print("--- interpreter_startup.py STDOUT ---")
        print(completed_process.stdout)
        print("--- interpreter_startup.py STDERR ---")
        if completed_process.stderr:
            print(completed_process.stderr)
        print("--- End of interpreter_startup.py output ---")

        if completed_process.returncode == 0:
            print(f"{script_path} completed successfully.")
            return True
        else:
            print(f"{script_path} failed with return code {completed_process.returncode}.")
            return False
            
    except subprocess.TimeoutExpired:
        print(f"{script_path} timed out.")
        return False
    except Exception as e:
        print(f"An error occurred while running {script_path}: {e}")
        return False

def send_and_parse_command(fd, command_to_send, timeout_per_read=1.5, overall_timeout_seconds=5.0, verbose=True):
    """
    Sends a command to the Spike, then reads and returns the raw output string
    until a REPL prompt '>>> ' is detected or overall timeout is reached.
    It attempts to clear previous buffer before sending.
    """
    if verbose:
        print(f"Sending to Spike (expecting REPL prompt): {command_to_send}")

    # Clear any existing input in the buffer (with a small timeout for read)
    original_fd_timeout = fd.timeout
    fd.timeout = 0.05 # Very short timeout for clearing buffer
    # Read in chunks to avoid blocking on readline if no newline is present in junk
    junk_buffer = b""
    while fd.in_waiting > 0:
        chunk = fd.read(min(fd.in_waiting, 256))
        if not chunk:
            break
        junk_buffer += chunk
        if len(junk_buffer) > 1024: # Safety break for clearing
            if verbose: print("  Cleared a large amount of junk, stopping clear.")
            break
            
    if verbose and len(junk_buffer) > 0:
        display_junk = junk_buffer.decode('utf-8', errors='ignore')
        display_junk = display_junk.replace('\\r\\n', '[CRLF]').replace('\\r', '[CR]').replace('\\n', '[LF]')
        print(f"  Cleared {len(junk_buffer)} bytes from input buffer: '{display_junk[:200]}...'")
    
    fd.timeout = timeout_per_read # Set timeout for subsequent readline operations

    fd.write((command_to_send + '\r\n').encode('utf-8'))
    
    full_response_str = ""
    start_time = time.time()
    
    while True:
        if time.time() - start_time > overall_timeout_seconds:
            if verbose:
                # Perform replacements outside the f-string
                processed_response_for_log = full_response_str.replace('[CR]', '\\\\r').replace('[LF]', '\\\\n')
                print(f"  Overall timeout ({overall_timeout_seconds}s) reached while waiting for '>>> '. Partial response: '{processed_response_for_log}'")
            break
        
        try:
            line_bytes = fd.readline() # This will use fd.timeout (timeout_per_read)
            if not line_bytes: # Timeout occurred for this readline
                # Check if prompt is already in the accumulated response, even if readline timed out
                if ">>> " in full_response_str:
                    break
                continue # Continue waiting if overall timeout not exceeded

            line_str = line_bytes.decode('utf-8', errors='ignore')
            full_response_str += line_str
            
            if ">>> " in line_str: # Check if the current line contains the prompt
                if verbose:
                    print("  REPL prompt '>>> ' detected.")
                break
        except serial.SerialException as e:
            if verbose:
                print(f"  Serial exception during readline: {e}")
            break # Exit on serial error
            
    fd.timeout = original_fd_timeout # Restore original fd timeout

    if verbose:
        processed_display = full_response_str.replace('\\r\\n', '[CRLF]').replace('\\r', '[CR]').replace('\\n', '[LF]')
        print(f"  Raw data received for '{command_to_send}': '{processed_display}'")
    
    return full_response_str

def main_robot_control():
    """Main function to control the robot."""
    if not run_interpreter_startup_script():
        print("Failed to prepare Spike interpreter using external script. Exiting.")
        return

    print("Waiting 3 seconds for serial port to be released after interpreter_startup.py execution...")
    time.sleep(3.0)

    print(f"Attempting to connect to Spike on {SERIAL_PORT} at {BAUD_RATE} bps.")
    fd = None
    try:
        fd = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) 
        print("Successfully connected to Spike via serial port.")

        print("Attempting to ensure Spike REPL is active (sending Ctrl+C, then CRLF)...")
        fd.write(b'\x03') # Send Ctrl+C to interrupt any running script/state
        time.sleep(0.3)    # Allow time for Ctrl+C to be processed
        fd.write(b'\r\n') # Send a CRLF to ensure a prompt
        time.sleep(0.3)    # Allow time for prompt to appear

        # Log any data that might have come back from the wake-up sequence.
        # send_and_parse_command will do its own buffer clearing anyway.
        initial_junk_after_wakeup = b""
        if fd.in_waiting > 0:
            initial_junk_after_wakeup = fd.read(fd.in_waiting)
        
        if initial_junk_after_wakeup:
            # Perform replacements outside the f-string
            decoded_junk = initial_junk_after_wakeup.decode('utf-8', errors='ignore')
            processed_junk_for_log = decoded_junk.replace('\r\n', '[CRLF]').replace('\r', '[CR]').replace('\n', '[LF]')
            print(f"  Data in buffer after wake-up sequence: {processed_junk_for_log}")
        else:
            print("  No initial data in buffer after wake-up sequence.")

    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        return

    try:
        print("Sending initial import commands using send_and_parse_command...")

        initial_commands = [
            "import motor",
            "from hub import port",
            "import time as spike_time" # Renaming to avoid conflict with Jetson's time
        ]

        for cmd in initial_commands:
            response = send_and_parse_command(fd, cmd, verbose=True)
            # A simple check for "Error" or "Traceback" in the response.
            # Note: The response includes the echo and REPL prompt.
            if "Traceback" in response or "Error" in response: # Crude error check
                print(f"ERROR detected during initial command '{cmd}':")
                print(response.replace('\\r\\n', '\\n').replace('\\r', '\\n')) # Cleaned for display
                print("Stopping.")
                return
            print(f"Initial command '{cmd}' potentially successful.")
        
        print("Initial Spike imports potentially successful.")

        # Diagnostic: Test basic print functionality from Spike
        print("\\nPerforming revised diagnostic print test with Spike...")
        diag_marker = "DIAG_RAW_TEST_OK"
        # Command to run on Spike. The \r\n INSIDE the print is for Spike's print function.
        diag_cmd_on_spike = f"print('{diag_marker}')" # Let Spike's print add its own newline 
        
        raw_response_str = send_and_parse_command(fd, diag_cmd_on_spike, verbose=True)
        
        # Expected echo: print('DIAG_RAW_TEST_OK\r\n')\r  (approximately)
        # Expected output from print: DIAG_RAW_TEST_OK\r\n
        
        expected_echo = diag_cmd_on_spike + '\r\n' # Expect CRLF based on repr() of raw_response_str
        print(f"  DEBUG: expected_echo: {repr(expected_echo)}")
        print(f"  DEBUG: raw_response_str: {repr(raw_response_str)}")
        echo_found_idx = raw_response_str.find(expected_echo)
        print(f"  DEBUG: echo_found_idx: {echo_found_idx}")
        print(f"  DEBUG: raw_response_str.startswith(expected_echo): {raw_response_str.startswith(expected_echo)}")
        
        diagnostic_passed = False
        if echo_found_idx != -1:
            print(f"  Diagnostic: Echo part '{diag_cmd_on_spike}' found.")
            # The actual output should appear AFTER the echo.
            search_after_echo = raw_response_str[echo_found_idx + len(expected_echo):]
            
            expected_printed_output = diag_marker + '\r\n' # Expect CRLF based on repr() of raw_response_str
            
            # Check if the expected output is at the beginning of the string after the echo
            if search_after_echo.startswith(expected_printed_output):
                print("Diagnostic print test PASSED. Marker found as direct output after echo.")
                diagnostic_passed = True
            else:
                print("WARNING: Diagnostic print test FAILED or output is not as expected.")
                # Perform replacements outside the f-string
                processed_expected_echo = expected_echo.replace('[CR]', '\\\\r').replace('[LF]', '\\\\n')
                processed_expected_output = expected_printed_output.replace('[CR]', '\\\\r').replace('[LF]', '\\\\n')
                print(f"  Expected echo: '{processed_expected_echo}'")
                print(f"  Expected output after echo (starts with): '{processed_expected_output}'")
                # Display a snippet of what was actually found after echo for debugging
                display_after_echo = search_after_echo.replace('\\r\\n', '[CRLF]').replace('\\r', '[CR]').replace('\\n', '[LF]')
                print(f"  Actual string after echo starts with: '{display_after_echo[:100]}...' (first 100 chars processed)")
        else:
            print("WARNING: Diagnostic print test FAILED. Echo of the command not found.")
            # Perform replacements outside the f-string
            processed_expected_echo = expected_echo.replace('[CR]', '\\\\r').replace('[LF]', '\\\\n')
            print(f"  Expected echo: '{processed_expected_echo}'")
        
        if not diagnostic_passed:
            display_full_raw = raw_response_str.replace('\\r\\n', '[CRLF]').replace('\\r', '[CR]').replace('\\n', '[LF]')
            print(f"  Full raw response from send_and_parse_command: '{display_full_raw}'")
            print("  Stopping further execution as diagnostic print test failed.")
            return

        # If diagnostic passed, proceed with motion sensor.
        print("\\nAttempting to use MotionSensor API...")
        
        # Command 1: Import MotionSensor
        cmd1 = "from hub import motion_sensor"
        response1 = send_and_parse_command(fd, cmd1, verbose=True)
        if "Traceback" in response1 or "Error" in response1:
            print(f"ERROR during '{cmd1}':")
            print(response1.replace('\\r\\n', '\\n').replace('\\r', '\\n'))
            print("Please check Spike API. Stopping.")
            return
        print(f"'{cmd1}' sent successfully.")

        # Diagnóstico: mostrar métodos disponibles en motor
        cmd_diag_motor = "print(dir(motor))"
        response_diag_motor = send_and_parse_command(fd, cmd_diag_motor, verbose=True)
        print("\n[DIAGNÓSTICO] dir(motor):")
        print(response_diag_motor.replace('\r\n', '\n').replace('\r', '\n'))
        # Puedes revisar aquí la lista de métodos disponibles y luego ajustar los comandos de movimiento.

        # --- Lazo de control proporcional para mantener el heading ---
        import time as host_time
        N_ITER = 10  # Número de iteraciones de prueba (ajusta o hazlo infinito)
        print("\n[CONTROL] Iniciando lazo de control de heading...")
        for i in range(N_ITER):
            # Leer yaw actual desde el Spike
            cmd_get_yaw = "print(motion_sensor.tilt_angles()[2])"
            response_get_yaw = send_and_parse_command(fd, cmd_get_yaw, verbose=False)
            # Parsear el valor de yaw
            lines = response_get_yaw.split("\n")
            yaw_actual = None
            for line in lines:
                try:
                    yaw_actual = float(line.strip())
                    break
                except ValueError:
                    continue
            if yaw_actual is None:
                print(f"[Iteración {i+1}] No se pudo leer yaw. Respuesta cruda: {response_get_yaw}")
                continue
            print(f"[Iteración {i+1}] Yaw actual: {yaw_actual}")
            # Calcular error
            error = TARGET_HEADING - yaw_actual
            # Zona muerta
            if abs(error) < ERROR_THRESHOLD:
                steer_power = 0
            else:
                steer_power = KP_STEERING * error
                # Saturar
                steer_power = max(-MAX_STEER_POWER, min(MAX_STEER_POWER, steer_power))
                # Umbral mínimo para vencer fricción
                if 0 < abs(steer_power) < MIN_STEER_POWER_THRESHOLD:
                    steer_power = MIN_STEER_POWER_THRESHOLD * (1 if steer_power > 0 else -1)
            print(f"[Iteración {i+1}] Error: {error:.2f} | Potencia steering: {steer_power}")
            # Enviar comandos a motores
            # Motor de tracción (constante)
            cmd_drive = f"motor.run_at_speed(port.{TRACTION_MOTOR_PORT}, {FORWARD_SPEED})"
            send_and_parse_command(fd, cmd_drive, verbose=False)
            # Motor de dirección (proporcional)
            cmd_steer = f"motor.run_at_speed(port.{STEERING_MOTOR_PORT}, {int(steer_power)})"
            send_and_parse_command(fd, cmd_steer, verbose=False)
            host_time.sleep(STEERING_PULSE_DURATION)
            # Parar motor de dirección para evitar sobrecalentamiento
            cmd_steer_stop = f"motor.stop(port.{STEERING_MOTOR_PORT})"
            send_and_parse_command(fd, cmd_steer_stop, verbose=False)
        # Parar motores al final
        send_and_parse_command(fd, f"motor.stop(port.{TRACTION_MOTOR_PORT})", verbose=False)
        send_and_parse_command(fd, f"motor.stop(port.{STEERING_MOTOR_PORT})", verbose=False)
        print("[CONTROL] Lazo de heading finalizado. Motores detenidos.")
        return

        if echo_idx_cmd4 != -1:
            after_echo_cmd4 = response4_raw[echo_idx_cmd4 + len(expected_echo_cmd4):]
            # Regex to find an integer or float number, possibly surrounded by whitespace, followed by \r\n
            match = re.match(r"^\s*(-?\d+(\.\d+)?)\s*\\r\\n", after_echo_cmd4) 
            if match:
                yaw_angle_str = match.group(1)
                print(f"Successfully parsed yaw angle string: '{yaw_angle_str}'")
                try:
                    yaw_angle_value = float(yaw_angle_str) # Use float for gyro
                    print(f"Current Yaw Angle: {yaw_angle_value} degrees.")
                except ValueError:
                    print(f"Error: Could not convert yaw angle string '{yaw_angle_str}' to float.")
            else:
                print("Error: Could not parse yaw angle from response after echo.")
                display_after_echo_cmd4 = after_echo_cmd4.replace('\\r\\n', '[CRLF]').replace('\\r', '[CR]').replace('\\n', '[LF]')
                print(f"  Response snippet after echo for '{cmd4}': '{display_after_echo_cmd4[:100]}...'")
        else:
            print(f"Error: Echo not found for '{cmd4}'.")
            display_response4_raw = response4_raw.replace('\\r\\n', '[CRLF]').replace('\\r', '[CR]').replace('\\n', '[LF]')
            print(f"  Full response for '{cmd4}': '{display_response4_raw}'")

        if yaw_angle_value is not None:
            print(f"Successfully retrieved yaw angle: {yaw_angle_value}")
        else:
            print("Failed to retrieve or parse yaw angle. Robot control logic cannot proceed.")
            return

        # Commenting out the rest of the logic until print/data retrieval is confirmed
        print("\\nMain control loop and further sensor interaction is currently BYPASSED for this test run.")

    except KeyboardInterrupt:
        print("\\nStopping robot (Ctrl+C pressed by user).")
    except Exception as e:
        print(f"An unexpected error occurred in main_robot_control: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if fd and fd.is_open:
            print("Attempting to stop motors on Spike and close serial connection.")
            try:
                # Send stop commands if necessary, ensure they are sent raw without send_spike_command
                fd.write(f"motor.stop(port.{TRACTION_MOTOR_PORT})\r\n".encode('utf-8'))
                time.sleep(0.1)
                fd.write(f"motor.stop(port.{STEERING_MOTOR_PORT})\r\n".encode('utf-8'))
                time.sleep(0.1)
                fd.write(b'\\x03') # Send Ctrl+C to Spike
                time.sleep(0.2)
            except Exception as e_stop:
                print(f"Error sending stop commands to Spike: {e_stop}")
            fd.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    main_robot_control()
