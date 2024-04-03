import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.signal import find_peaks

# 1. Loading and Preparing Data
def load_and_prepare_data(filepath):
    """
    Load and prepare ramp test data from a CSV file.

    Parameters:
    - filepath: Path to the CSV file containing the test data.

    Returns:
    - A pandas DataFrame with the loaded data, including calculated elapsed time
      and identified ramp direction based on PWM/ESC changes.
    """
    # Assuming column names match your data structure; adjust if necessary
    column_names = ['Micros', 'ForceX', 'ForceY', 'ForceZ', 'TorqueX', 'TorqueY', 'TorqueZ', 'Actuation']
    data = pd.read_csv(filepath, names=column_names)
    data['ElapsedTime'] = (data['Micros'] - data['Micros'].iloc[0]) / 1e6  # Convert microseconds to seconds

    # Initialize ramp direction as 'hold' by default
    data['RampDirection'] = 'hold'

    # Keep track of the last actuation value that represented a change
    last_actuation_value = data.loc[0, 'Actuation']
    for i in range(1, len(data)):
        current_actuation_value = data.loc[i, 'Actuation']
        if current_actuation_value != last_actuation_value:  # There's a change in actuation
            if current_actuation_value > last_actuation_value:
                data.loc[i, 'RampDirection'] = 'up'
            else:
                data.loc[i, 'RampDirection'] = 'down'
            last_actuation_value = current_actuation_value  # Update the last actuation value

    return data

# 2. Identifying Segments
def identify_segments(data):
    """
    Identify segments of continuous ramp-up or ramp-down based on changes in the actuation value (PWM/ESC).

    Parameters:
    - data: DataFrame containing the ramp data.

    Returns:
    - A list of dictionaries, where each dictionary contains 'start', 'end',
      and 'direction' keys defining each segment's properties.
    """
    segments = []
    current_segment = {'start': None, 'end': None, 'direction': None}
    for i, row in data.iterrows():
        if current_segment['start'] is None:
            # Initialize the first segment
            current_segment['start'] = i
            current_segment['direction'] = row['RampDirection']
        elif row['RampDirection'] != current_segment['direction']:
            # When direction changes, finalize the current segment and start a new one
            current_segment['end'] = i - 1
            segments.append(current_segment.copy())
            current_segment = {'start': i, 'end': None, 'direction': row['RampDirection']}
        elif i == len(data) - 1:
            # Ensure the last segment is captured
            current_segment['end'] = i
            segments.append(current_segment.copy())

    return segments

# 3. plot_segments(data, segments_info)
def plot_segments(data, segments_info):
    """
    Plot each segment identified in the ramp analysis against time.

    Parameters:
    - data: DataFrame containing the ramp data, including force measurements and elapsed time.
    - segments_info: A list of dictionaries, where each dictionary contains 'start', 'end',
                     and 'direction' keys defining each segment's properties.
    """
    plt.figure(figsize=(12, 6))
    for segment in segments_info:
        segment_data = data.iloc[segment['start']:segment['end']+1]
        plt.plot(segment_data['ElapsedTime'], segment_data['ForceX'], label=f"Segment {segment['start']} to {segment['end']} ({segment['direction']})")
    
    plt.xlabel('Time (seconds)')
    plt.ylabel('ForceX')
    plt.title('Ramp Test Analysis - ForceX vs. Time')
    plt.legend()
    plt.show()

# 4. calculate_max_min_force(data)
def calculate_max_min_force(data, column_name='ForceX'):
    """
    Calculate the maximum and minimum average force values, excluding the first 25%
    and last 25% of readings in each ramp segment to account for actuator latency.

    Parameters:
    - data: DataFrame containing the ramp data.
    - column_name: The name of the column to analyze (default is 'ForceX').

    Returns:
    - A tuple with maximum and minimum average force values.
    """
    # Identify ramp up and down segments
    data['RampDirectionChange'] = data['Actuation'].diff().ne(0).cumsum()
    segments = data.groupby(['RampDirectionChange', 'RampDirection'], as_index=False)

    max_force = float('-inf')
    min_force = float('inf')

    for _, segment in segments:
        if segment['RampDirection'].iloc[0] in ['up', 'down']:
            # Calculate the indices to exclude the first 25% and last 25% of the segment
            valid_range_start = int(len(segment) * 0.25)
            valid_range_end = int(len(segment) * 0.75)

            segment_data = data.iloc[segment.index[valid_range_start]:segment.index[valid_range_end]]
            # Calculate the average force in the valid range of the segment
            avg_force = segment_data[column_name].mean()

            # Update max and min forces if applicable
            max_force = max(max_force, avg_force)
            min_force = min(min_force, avg_force)

    return max_force, min_force

# 5. calculate_hysteresis(segments_info, data)
def calculate_hysteresis(segments_info, data):
    """
    Calculate the hysteresis for each pair of ramp up and down segments, adjusting for device latency.
    
    Removing the first 25% of data from each segment to account for device latency. 
    It then uses the maximum force values in the remaining data to 
    calculate hysteresis for each pair of segments.

    Parameters:
    - segments_info: A list of dictionaries, where each dictionary contains 'start' and 'end'
                     indices defining a segment's location within the data.
    - data: DataFrame containing the ramp data, including force measurements.

    Returns:
    - A dictionary with keys as tuples representing the segment pair (up, down) and values
      as the adjusted hysteresis for each pair.
    """
    hysteresis_values = {}
    # Define the percentage of points to skip at the start of each segment to account for latency
    latency_skip_percentage = 0.25

    for i in range(0, len(segments_info)-1, 2):
        # Calculate the number of points to skip for latency
        skip_points_up = int((segments_info[i]['end'] - segments_info[i]['start'] + 1) * latency_skip_percentage)
        skip_points_down = int((segments_info[i+1]['end'] - segments_info[i+1]['start'] + 1) * latency_skip_percentage)

        # Extract the up and down segments, adjusting for latency
        up_segment = data.iloc[segments_info[i]['start'] + skip_points_up:segments_info[i]['end']+1]
        down_segment = data.iloc[segments_info[i+1]['start'] + skip_points_down:segments_info[i+1]['end']+1]

        # If after adjusting for latency there are no data points left in the segment, continue to the next pair
        if up_segment.empty or down_segment.empty:
            continue

        # Use the maximum force values in the adjusted segments to calculate hysteresis
        max_force_up = up_segment['ForceX'].max()
        max_force_down = down_segment['ForceX'].max()
        hysteresis = abs(max_force_up - max_force_down)

        # Store the calculated hysteresis value for the segment pair
        hysteresis_values[(i, i+1)] = hysteresis

    return hysteresis_values

# 6. calculate_sensitivity(segments_info, data)
def calculate_sensitivity(segments_info, data):
    """
    Calculate the sensitivity of the device as the change in output force per unit change in input signal.

    This function assumes that each segment in `segments_info` corresponds to a single increment in the
    input signal and that the device is allowed to stabilize after each increment. The first 25% of data
    points in each segment are discarded to account for transient response.

    Parameters:
    - segments_info: A list of dictionaries, where each dictionary contains 'start' and 'end'
                     indices defining a segment's location within the data, along with the input signal value.
    - data: DataFrame containing the ramp data, including force measurements and input signal values.

    Returns:
    - The calculated sensitivity as the average change in force per unit change in input signal.
    """
    changes = []
    for i in range(1, len(segments_info)):
        # Calculate the index to start from by discarding the first 25% of the segment to account for latency
        start_index_prev = segments_info[i-1]['start'] + int((segments_info[i-1]['end'] - segments_info[i-1]['start']) * 0.25)
        start_index_current = segments_info[i]['start'] + int((segments_info[i]['end'] - segments_info[i]['start']) * 0.25)

        prev_segment = data.iloc[start_index_prev:segments_info[i-1]['end']+1]
        current_segment = data.iloc[start_index_current:segments_info[i]['end']+1]

        if prev_segment.empty or current_segment.empty:
            continue

        # Calculate average forces for the stabilized parts of each segment
        avg_force_prev = prev_segment['ForceX'].mean()
        avg_force_current = current_segment['ForceX'].mean()

        # Get the actuation (input) change between segments
        input_change = data.loc[segments_info[i]['start'], 'Actuation'] - data.loc[segments_info[i-1]['start'], 'Actuation']

        if input_change != 0:
            force_change = avg_force_current - avg_force_prev
            changes.append(force_change / input_change)

    # Calculate the average sensitivity across all changes
    sensitivity = sum(changes) / len(changes) if changes else None
    return sensitivity

# 7. calculate_output_force_resolution(segments_info, data)
def calculate_output_force_resolution(segments_info, data):
    """
    Estimate the Output Force Resolution as the smallest detectable change in output force 
    across small, consistent increments in the input signal, after accounting for transient 
    effects and system stabilization.

    Parameters:
    - segments_info: A list of dictionaries, where each dictionary contains 'start' and 'end'
                     indices defining a segment's location within the data, along with the input signal value.
    - data: DataFrame containing the ramp data, including force measurements and input signal values.

    Returns:
    - The estimated average Output Force Resolution across all analyzed segments.
    """
    force_changes = []
    for i in range(1, len(segments_info)):
        # Adjust for latency by discarding the first 25% of the data in each segment
        segment_start = segments_info[i]['start'] + int((segments_info[i]['end'] - segments_info[i]['start']) * 0.25)
        segment_end = segments_info[i]['end']

        current_segment = data.iloc[segment_start:segment_end+1]

        if current_segment.empty:
            continue  # Skip this segment if it becomes empty after adjustment

        # Calculate the average force for the stabilized part of the current segment
        avg_force = current_segment['ForceX'].mean()

        # Calculate the change in force from the previous segment's average to this segment's average
        if i > 0:  # Ensure there's a previous segment to compare with
            prev_segment_avg_force = data.iloc[segments_info[i-1]['start'] + int((segments_info[i-1]['end'] - segments_info[i-1]['start']) * 0.25):segments_info[i-1]['end']+1]['ForceX'].mean()
            force_change = abs(avg_force - prev_segment_avg_force)
            force_changes.append(force_change)

    # Determine the smallest detectable change in force, representing the output force resolution
    output_force_resolution = min(force_changes) if force_changes else None
    return output_force_resolution

# 8. calculate_dynamic_range(max_force, min_force)
def calculate_dynamic_range(max_force, min_force):
    """
    Calculate the dynamic range of the device in decibels (dB).

    Parameters:
    - max_force: The maximum force measured.
    - min_force: The minimum force measured (must be greater than 0).

    Returns:
    - The dynamic range in decibels (dB).
    """
    if min_force <= 0:
        raise ValueError("Minimum force must be greater than 0 to calculate dynamic range.")
    dynamic_range_db = 20 * math.log10(max_force / min_force)
    return dynamic_range_db


if __name__ == "__main__":
    filepath = 'path_to_your_data.csv'  # Update with the actual path to your data file
    data = load_and_prepare_data(filepath)
    segments_info = identify_segments(data)

    # Plotting segments
    plot_segments(data, segments_info)

    # Calculate maximum and minimum forces
    max_force, min_force = calculate_max_min_force(data)
    print(f"Maximum Force: {max_force}, Minimum Force: {min_force}")

    # Calculate hysteresis adjusted for latency
    hysteresis_values = calculate_hysteresis(segments_info, data)
    print(f"Hysteresis Values: {hysteresis_values}")

    # Calculate sensitivity
    sensitivity = calculate_sensitivity(segments_info, data)
    print(f"Sensitivity: {sensitivity}")

    # Calculate output force resolution
    output_force_resolution = calculate_output_force_resolution(segments_info, data)
    print(f"Output Force Resolution: {output_force_resolution}")

    # Calculate dynamic range
    dynamic_range = calculate_dynamic_range(max_force, min_force)
    print(f"Dynamic Range (dB): {dynamic_range}")

