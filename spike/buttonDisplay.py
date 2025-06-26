import lgpio
import time

led    = 17  # GPIO pin for the LED
button = 27  # GPIO pin for the button

def setup_gpio():
    """Initialize GPIO and claim all necessary pins"""
    try:
        h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(h, button)  # Input pin for button
        lgpio.gpio_claim_output(h, led)    # Output pin for LED
        lgpio.gpio_write(h, led, 0)         # Initialize LED to off
        # Let sensors settle
        print("Initializing sensors...")
        time.sleep(1)
        return h, True
    except Exception as e:
        print(f"GPIO initialization failed: {e}")
        return None, False

def cleanup_gpio(h):
    """Release all GPIO resources"""
    try:
        lgpio.gpio_free(h, button) # Free button pin
        lgpio.gpio_free(h, led)    # Free LED pin
        lgpio.gpiochip_close(h)
        print("GPIO resources cleaned up")
    except:
        print("Error during GPIO cleanup")
        pass

h, success = setup_gpio()
if success:
    try:
        while(lgpio.gpio_read(h, button) == 1):
            lgpio.gpio_write(h, led, 1)  # Turn on LED if button is not pressed
            print("Button is not pressed")
            time.sleep(0.1)  # Polling delay to avoid busy waiting
        lgpio.gpio_write(h, led, 0)  # Turn off LED when button is pressed
        print(lgpio.gpio_read(h, button))
        print("Button is pressed, exiting test")
    finally:
        cleanup_gpio(h)  # Cleanup GPIO resources after testing
else:
    print("Failed to initialize GPIO")
