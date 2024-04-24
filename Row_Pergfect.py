import time
import numpy as np
from mpu6050 import mpu6050
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt

def setup_gpio():
    GPIO.setmode(GPIO.BCM)  # Use Broadcom pin-numbering scheme
    GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button to GPIO17
    GPIO.setup(27, GPIO.OUT)
    GPIO.setup(22, GPIO.OUT)
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(23, GPIO.OUT)

def activate_led_27(duration=0.2):
    """Activates the LED connected to GPIO pin 27 for a specified duration."""
    GPIO.output(27, GPIO.HIGH)  # Turn on LED
    time.sleep(duration)  # Keep LED on for the specified duration
    GPIO.output(27, GPIO.LOW)  # Turn off LED

def activate_led_22(duration=0.2):
    """Activates the LED connected to GPIO pin 27 for a specified duration."""
    GPIO.output(22, GPIO.HIGH)  # Turn on LED
    time.sleep(duration)  # Keep LED on for the specified duration
    GPIO.output(22, GPIO.LOW)  # Turn off LED

def activate_led_18(duration=0.2):
    """Activates the LED connected to GPIO pin 27 for a specified duration."""
    GPIO.output(18, GPIO.HIGH)  # Turn on LED
    time.sleep(duration)  # Keep LED on for the specified duration
    GPIO.output(18, GPIO.LOW)  # Turn off LED

def activate_led_23(duration=0.2):
    """Activates the LED connected to GPIO pin 27 for a specified duration."""
    GPIO.output(23, GPIO.HIGH)  # Turn on LED
    time.sleep(duration)  # Keep LED on for the specified duration
    GPIO.output(23, GPIO.LOW)  # Turn off LED

def wait_for_start():
    print("Press the button to start...")
    while GPIO.input(17):  # Wait for the button to be pressed
        pass
    print("Starting...")
    time.sleep(0.5)  # Debounce delay

def button_pressed():
    time.sleep(0.1)  # Small delay to debounce button
    return not GPIO.input(17)  # Return True if button is pressed

def rotate_data(x, y, z, angle_deg=-45):
    angle_rad = np.radians(angle_deg)
    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)
    R = np.array([
        [cos_angle, sin_angle, 0],
        [-sin_angle, cos_angle, 0],
        [0, 0, 1]
    ])
    original_vector = np.array([x, y, z])
    rotated_vector = R @ original_vector
    return rotated_vector

def get_smoothed_accel_data(sensor, alpha=0.5):
    raw_data = sensor.get_accel_data()
    smooth_data = {}
    for axis in ['x', 'y', 'z']:
        if axis not in prev_accel:
            prev_accel[axis] = raw_data[axis]
        smooth_data[axis] = prev_accel[axis] = alpha * prev_accel[axis] + (1 - alpha) * raw_data[axis]
    return smooth_data
    
def low_pass_filter(data, alpha=0.5):
    filtered_data = []
    filtered_value = data[0]  # Start with the first item
    for point in data:
        filtered_value = filtered_value * alpha + point * (1 - alpha)
        filtered_data.append(filtered_value)
    return filtered_data
    
def moving_average(values, window_size=3):
    if len(values) > window_size:
        values.pop(0)
    return sum(values) / len(values)

def calc_stroke_rate(interval, rates_list):
    print(f"Measured interval: {interval} seconds")
    if interval <= 0 or interval > 5:
        print("Invalid stroke interval")
        return None

    # Calculate strokes per minute from interval in seconds
    stroke_rate = 25 / interval
    if stroke_rate <= 20 or stroke_rate > 40:
        print("Invalid stroke interval")
        return None
    print(f"Calculated Stroke Rate: {stroke_rate} strokes/min")

    # Append new stroke rate to list
    
    rates_list.append(stroke_rate)

    # Calculate moving average if applicable
    if len(rates_list) > 1:
        average_rate = moving_average(rates_list, 5)
        print(f"Average Stroke Rate: {average_rate} strokes/min")
        return average_rate
    else:
        return stroke_rate

def find_local_extrema_dynamic(accel_data_list, delta):
    
    last_max, last_min = None, None
    length = len(accel_data_list)
    
    if length < 3:
        return last_max, last_min

    for i in range(1, length - 1):
        prev = accel_data_list[i - 1][1]
        curr = accel_data_list[i][1]
        next = accel_data_list[i + 1][1]

        # Check if current point is a local maximum
        if prev < curr > next and (last_max is None or curr > last_max[1] + delta):
            last_max = (accel_data_list[i][0], curr)
        
        # Check if current point is a local minimum
        elif prev > curr < next and (last_min is None or curr < last_min[1] - delta):
            last_min = (accel_data_list[i][0], curr)

    return last_max, last_min
    
def calculate_average_y(accel_data_list):

    sum_y = sum(data[1] for data in accel_data_list)
    count = len(accel_data_list)
    return sum_y / count if count != 0 else 0
    
def check_y_accel_thresholds(current_y, avg_y, flag_state):
    threshold_high = avg_y + 1.5 * avg_y
    threshold_low = avg_y - 1.5 * avg_y
    exceeds_high = current_y > threshold_high
    exceeds_low = current_y < threshold_low
    
    # Update flags based on current thresholds and previous state
    if not flag_state['exceeded']:
        flag_state['exceeded'] = exceeds_high or exceeds_low
    else:
        # Reset the flag if current Y acceleration is back near average
        if not exceeds_high and not exceeds_low:
            flag_state['exceeded'] = False

    return exceeds_high, exceeds_low, flag_state


def trigger_gpio_pins(exceeds_high, exceeds_low):
    if exceeds_high:
        GPIO.output(27, GPIO.HIGH)
        GPIO.output(22, GPIO.LOW)
        time.sleep(1)  # Keep GPIO 27 powered for 1 second
        GPIO.output(27, GPIO.LOW)  # Turn off GPIO 27
    elif exceeds_low:
        GPIO.output(22, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
        time.sleep(1)  # Keep GPIO 22 powered for 1 second
        GPIO.output(22, GPIO.LOW)  # Turn off GPIO 22
    else:
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.LOW)

def check_stability_over_time(accel_data, time_step, duration, tolerance):

    num_samples = int(duration / time_step)
    if len(accel_data) < num_samples:
        return False  # Not enough data to analyze

    recent_data = accel_data[-num_samples:]  # Get the last 'num_samples' data points
    mean_data = sum(recent_data) / num_samples
    return all(abs(x - mean_data) <= tolerance for x in recent_data)

def compute_90th_percentile(values):
    if len(values) == 0:
        return None
    return np.percentile(values, 90)

def check_x_accel_threshold(current_x, x_values):
    # Append the current x value to the list
    x_values.append(current_x)

    # Compute the 90th percentile of x values
    percentile_90 = compute_90th_percentile(x_values)

    # Determine if the current x value is 2 units higher than the 90th percentile
    if percentile_90 is not None and current_x > percentile_90 + 2:
        return True, percentile_90
    else:
        return False, percentile_90
        
def check_stability(accel_data, tolerance=0.3):
    if len(accel_data) < 6:  # Need at least 6 data points (the current + 5 previous)
        return False  # Not enough data to determine stability
    
    current_value = accel_data[-1]
    previous_values = accel_data[-6:-1]  # Get the last 5 values before the current one
    
    # Check if all previous values are within the tolerance of the current value
    for value in previous_values:
        if not (current_value - tolerance <= value <= current_value + tolerance):
            return False  # If any value is outside the tolerance, return False
    
    return True  # All values are within the tolerance

    
def plot_data(time_step, accel_data_list):
    t = np.arange(len(accel_data_list)) * time_step
    accel_data_array = np.array(accel_data_list)
    plt.figure(figsize=(10, 4))
    plt.plot(t, accel_data_array[:, 0], label='X')
    plt.plot(t, accel_data_array[:, 1], label='Y')
    plt.plot(t, accel_data_array[:, 2], label='Z')
    plt.title('Acceleration over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()
    plt.show()

def main():
    setup_gpio()
    sensor = mpu6050(0x68)
    sensor2 = mpu6050(0x69)
    time_step = 0.05
    global prev_accel
    prev_accel = {}
    accel_data_list = []
    accel_data_list_x = []
    stroke_rates = []
    total_y = 0  # Sum of Y accelerations
    count_y = 0  # Number of Y readings
    flag_state = {'exceeded': False}  # State to control when to raise alerts
    x_accel_history = []  # List to store history of x accelerations

    wait_for_start()

    try:
        while True:
            if button_pressed():
                print("Button pressed again, stopping...")
                break

            current_time = time.time()
            avg_accel_data = get_smoothed_accel_data(sensor)
            accel_data_list.append([avg_accel_data['x'], avg_accel_data['y'], avg_accel_data['z']])
            current_x = avg_accel_data['x']
            accel_data_list.append([current_x, avg_accel_data['y'], avg_accel_data['z']])
            accel_data_list_x.append(current_x)
            current_y = avg_accel_data['y']
            total_y += current_y
            count_y += 1
            current_avg_y = total_y / count_y  # Calculate running average of Y
            
            # Check x-acceleration threshold
            x_exceeds, x_percentile_90 = check_x_accel_threshold(current_x, x_accel_history)
            if x_exceeds:
                activate_led_18()
                print(f"X acceleration {current_x:.2f} exceeds the 90th percentile ({x_percentile_90:.2f}) by at least 2 units.")
            
            exceeds_high, exceeds_low, flag_state = check_y_accel_thresholds(current_y, current_avg_y, flag_state)
            if exceeds_high and flag_state['exceeded']:
                activate_led_27()
                print("Y acceleration exceeds 0.5 times above average.")
            if exceeds_low and flag_state['exceeded']:
                activate_led_22()
                print("Y acceleration exceeds 0.5 times below average.")
                
            if check_stability_over_time(accel_data_list_x, time_step, duration=0.2, tolerance=0.05):
                print("X acceleration has been stable for the last 0.2 seconds.")
                activate_led_23()

            time.sleep(max(0, time_step - (time.time() - current_time)))

    finally:
        print("Stopped data collection.")
        plot_data(time_step, accel_data_list)
        GPIO.cleanup()


if __name__ == "__main__":
    main()


