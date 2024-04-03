import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lti, step, find_peaks

def load_and_prepare_data(filepath):
    """
    Load and prepare step response data from a CSV file.

    Parameters:
    - filepath: Path to the CSV file containing the test data.

    Returns:
    - time: Array of time values.
    - step_response: Array of step response values.
    - input_signal: The input signal value used for the step response.
    """
    # Assuming column names match your data structure; adjust if necessary
    column_names = ['Micros', 'ForceX', 'ForceY', 'ForceZ', 'TorqueX', 'TorqueY', 'TorqueZ', 'PWM']
    data = pd.read_csv(filepath, names=column_names)
    
    # Convert microseconds to seconds for elapsed time
    data['ElapsedTime'] = (data['Micros'] - data['Micros'].iloc[0]) / 1e6
    
    # Extract time and step response data
    time = data['ElapsedTime'].values
    step_response = data['ForceX'].values  # Assuming ForceX represents the step response
    
    # Extract the input signal value (PWM)
    input_signal = data['PWM'].iloc[0]  # Assuming the PWM value is constant throughout the step response
    
    return time, step_response, input_signal

def identify_step_segments(data, pwm_threshold=127, time_buffer=1):
    """
    Identify segments of step responses based on changes in the PWM value.

    Parameters:
    - data: DataFrame containing the step response data.
    - pwm_threshold: Threshold value for detecting the rising edge of the PWM signal (default: 127).
    - time_buffer: Time buffer (in seconds) to include before and after each step response (default: 1).

    Returns:
    - A list of dictionaries, where each dictionary contains 'start', 'end', and 'pwm' keys
      defining each step response segment's properties.
    """
    segments = []
    current_segment = {'start': None, 'end': None, 'pwm': None}
    previous_pwm = 0

    for i, row in data.iterrows():
        current_pwm = row['PWM']

        if previous_pwm < pwm_threshold and current_pwm >= pwm_threshold:
            # Rising edge detected (step response starts)
            if current_segment['start'] is not None:
                # Finalize the previous segment
                current_segment['end'] = i - 1
                segments.append(current_segment.copy())

            # Start a new segment
            current_segment = {'start': i, 'end': None, 'pwm': current_pwm}

        elif previous_pwm > pwm_threshold and current_pwm <= pwm_threshold:
            # Falling edge detected (step response ends)
            if current_segment['start'] is not None:
                current_segment['end'] = i
                segments.append(current_segment.copy())

        previous_pwm = current_pwm

    # Ensure the last segment is captured
    if current_segment['start'] is not None:
        current_segment['end'] = len(data) - 1
        segments.append(current_segment.copy())

    # Adjust the start and end times of each segment based on the time buffer
    for segment in segments:
        start_time = data.loc[segment['start'], 'ElapsedTime']
        end_time = data.loc[segment['end'], 'ElapsedTime']
        segment['start'] = data.index[data['ElapsedTime'] >= start_time - time_buffer].min()
        segment['end'] = data.index[data['ElapsedTime'] <= end_time + time_buffer].max()

    return segments

def plot_step_responses(data, step_segments):
    plt.figure()
    for segment in step_segments:
        segment_data = data.iloc[segment['start']:segment['end']+1]
        plt.plot(segment_data['ElapsedTime'], segment_data['ForceX'])

    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Step Response Segments')
    plt.grid(True)
    plt.show()

def average_step_responses(data, step_segments):
    averaged_response = np.zeros_like(data['ForceX'].values)
    num_segments = len(step_segments)

    for segment in step_segments:
        segment_data = data.iloc[segment['start']:segment['end']+1]
        averaged_response[:len(segment_data)] += segment_data['ForceX'].values

    averaged_response /= num_segments
    return averaged_response

def identify_system(time, step_response, input_signal):
    """
    Identify the system's transfer function and calculate performance metrics based on the step response data.

    Parameters:
    - time: Array of time values.
    - step_response: Array of step response values.
    - input_signal: The input signal value used for the step response.

    Returns:
    - sys: An LTI system object representing the identified system.
    - metrics: A dictionary containing the calculated performance metrics.
    """
    # Find the peak response and its index
    peak_idx = np.argmax(step_response)
    peak_force = step_response[peak_idx]
    
    # Calculate the overshoot percentage
    steady_state = step_response[-1]
    overshoot_pct = (peak_force - steady_state) / steady_state * 100
    
    # Calculate the damping ratio and natural frequency
    damping_ratio = -np.log(overshoot_pct / 100) / np.sqrt(np.pi**2 + (np.log(overshoot_pct / 100))**2)
    natural_freq = 2 * np.pi / (time[peak_idx] * np.sqrt(1 - damping_ratio**2))
    
    # Create the transfer function
    num = [natural_freq**2]
    den = [1, 2*damping_ratio*natural_freq, natural_freq**2]
    sys = lti(num, den)
    
    # Calculate rise time (10% to 90% of steady-state value)
    rise_start_idx = np.argmax(step_response >= 0.1 * steady_state)
    rise_end_idx = np.argmax(step_response >= 0.9 * steady_state)
    rise_time = time[rise_end_idx] - time[rise_start_idx]
    
    # Calculate settling time (within 2% of steady-state value)
    settling_threshold = 0.02 * steady_state
    settling_idx = np.argmax(np.abs(step_response - steady_state) <= settling_threshold)
    settling_time = time[settling_idx]
    
    # Calculate output error (steady-state error)
    output_error = steady_state - input_signal
    
    # Store the metrics in a dictionary
    metrics = {
        'peak_force': peak_force,
        'max_continuous_force': steady_state,
        'rise_time': rise_time,
        'settling_time': settling_time,
        'output_error': output_error
    }
    
    return sys, metrics

def characterize_system(sys, time_range, actual_response, actual_time):
    """
    Characterize the system by simulating its step response and plotting the results
    alongside the actual averaged step response data.

    Parameters:
    - sys: An LTI system object representing the identified system.
    - time_range: Array of time values for simulation.
    - actual_response: Array of actual averaged step response values.
    - actual_time: Array of time values corresponding to the actual response.
    """
    # Simulate the step response
    _, simulated_response = step(sys, T=time_range)
    
    # Plot the simulated and actual step responses
    plt.figure()
    plt.plot(time_range, simulated_response, label='Simulated Response')
    plt.plot(actual_time, actual_response, label='Actual Response')
    plt.xlabel('Time')
    plt.ylabel('Response')
    plt.title('Step Response Comparison')
    plt.legend()
    plt.grid(True)
    plt.show()

# Load and prepare the step response data from the CSV file
filepath = 'step_response_data.csv'  # Replace with the path to your CSV file
data = load_and_prepare_data(filepath)

# Identify the multiple step response signals
step_segments = identify_step_segments(data)

# Plot the step response segments for verification
plot_step_responses(data, step_segments)

# Average the step responses
averaged_response = average_step_responses(data, step_segments)

# Identify the system and calculate performance metrics
sys, metrics = identify_system(data['ElapsedTime'].values, averaged_response, data['PWM'].iloc[0])

# Print the calculated metrics
print("Performance Metrics:")
print(f"Peak Force: {metrics['peak_force']}")
print(f"Max Continuous Force: {metrics['max_continuous_force']}")
print(f"Rise Time: {metrics['rise_time']}")
print(f"Settling Time: {metrics['settling_time']}")
print(f"Output Error: {metrics['output_error']}")

# Determine the time range based on the averaged response
start_time = data['ElapsedTime'].iloc[0]
end_time = data['ElapsedTime'].iloc[-1]
num_points = 1000
time_range = np.linspace(start_time, end_time, num_points)

# Characterize the system and plot the comparison
characterize_system(sys, time_range, averaged_response, data['ElapsedTime'].values)