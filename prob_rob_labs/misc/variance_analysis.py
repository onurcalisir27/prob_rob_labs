#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

def analyze_measurement_errors(csv_file):
    """
    Analyze measurement errors and create variance model
    """
    # Load data
    df = pd.read_csv(csv_file)

    # Clean column names (remove spaces)
    df.columns = df.columns.str.strip()

    # Basic statistics
    print("="*50)
    print("Basic Statistics:")
    print("="*50)
    print(f"Number of measurements: {len(df)}")
    print(f"\nRange Error:")
    print(f"  Mean: {df['range_error'].mean():.4f} m")
    print(f"  Std:  {df['range_error'].std():.4f} m")
    print(f"  Variance: {df['range_error'].var():.6f} m²")

    print(f"\nBearing Error:")
    print(f"  Mean: {np.degrees(df['bearing_error'].mean()):.2f}°")
    print(f"  Std:  {np.degrees(df['bearing_error'].std()):.2f}°")
    print(f"  Variance: {df['bearing_error'].var():.6f} rad²")

    # Check if errors are zero-mean (unbiased)
    range_bias = df['range_error'].mean()
    bearing_bias = df['bearing_error'].mean()

    if abs(range_bias) > 0.02:  # 2cm threshold
        print(f"\nWARNING: Range has bias of {range_bias:.3f}m - consider removing")
    if abs(bearing_bias) > np.radians(1):  # 1 degree threshold
        print(f"WARNING: Bearing has bias of {np.degrees(bearing_bias):.1f}° - consider removing")

    return df

def model_range_variance(df):
    """
    Find relationship between distance and range error variance
    """
    # Bin measurements by distance
    distance_bins = np.arange(0, 11, 1)  # 0-10m in 1m bins
    variances = []
    distances = []

    for i in range(len(distance_bins)-1):
        mask = (df['measured_range'] >= distance_bins[i]) & \
               (df['measured_range'] < distance_bins[i+1])
        errors_in_bin = df.loc[mask, 'range_error']

        if len(errors_in_bin) > 5:  # Need enough samples
            var = errors_in_bin.var()
            variances.append(var)
            distances.append((distance_bins[i] + distance_bins[i+1])/2)
            print(f"Distance {distance_bins[i]:.0f}-{distance_bins[i+1]:.0f}m: "
                  f"σ² = {var:.6f}, σ = {np.sqrt(var):.3f}m, "
                  f"n = {len(errors_in_bin)}")

    # Fit different models
    distances = np.array(distances)
    variances = np.array(variances)

    # Model 1: Constant variance
    const_var = np.mean(variances)
    print(f"\nModel 1 (Constant): σ² = {const_var:.6f}")

    # Model 2: Linear with distance (σ² = a*d + b)
    if len(distances) > 1:
        linear_fit = np.polyfit(distances, variances, 1)
        print(f"Model 2 (Linear): σ² = {linear_fit[0]:.6f}*d + {linear_fit[1]:.6f}")

    # Model 3: Quadratic with distance (σ² = a*d²)
    if len(distances) > 0:
        # Force through origin for quadratic
        quad_fit = np.sum(variances * distances**2) / np.sum(distances**4)
        print(f"Model 3 (Quadratic): σ² = {quad_fit:.6f}*d²")

    # Plot the models
    plt.figure(figsize=(10, 6))
    plt.scatter(distances, variances, s=100, label='Measured Variance')

    d_plot = np.linspace(0, 10, 100)
    plt.plot(d_plot, [const_var]*100, 'r--', label=f'Constant: {const_var:.4f}')

    if len(distances) > 1:
        plt.plot(d_plot, linear_fit[0]*d_plot + linear_fit[1], 'g--',
                label=f'Linear: {linear_fit[0]:.4f}*d + {linear_fit[1]:.4f}')

    plt.xlabel('Distance (m)')
    plt.ylabel('Range Error Variance (m²)')
    plt.title('Range Error Variance vs Distance')
    plt.legend()
    plt.grid(True)
    plt.show()

    return const_var, linear_fit if len(distances) > 1 else None

def model_bearing_variance(df):
    """
    Find relationship between bearing angle and bearing error variance
    """
    # Bin by bearing angle
    bearing_bins = np.radians(np.arange(-40, 41, 10))  # -40° to 40° in 10° bins
    variances = []
    bearings = []

    for i in range(len(bearing_bins)-1):
        mask = (df['measured_bearing'] >= bearing_bins[i]) & \
               (df['measured_bearing'] < bearing_bins[i+1])
        errors_in_bin = df.loc[mask, 'bearing_error']

        if len(errors_in_bin) > 5:
            var = errors_in_bin.var()
            variances.append(var)
            bearings.append((bearing_bins[i] + bearing_bins[i+1])/2)
            print(f"Bearing {np.degrees(bearing_bins[i]):.0f}° to "
                  f"{np.degrees(bearing_bins[i+1]):.0f}°: "
                  f"σ² = {var:.6f}, σ = {np.degrees(np.sqrt(var)):.1f}°")

    # Simple model: constant with increase at edges
    center_variance = np.mean([v for i, v in enumerate(variances)
                               if abs(bearings[i]) < np.radians(20)])
    edge_variance = np.mean([v for i, v in enumerate(variances)
                             if abs(bearings[i]) >= np.radians(20)])

    print(f"\nBearing variance (center): {center_variance:.6f} rad²")
    print(f"Bearing variance (edge): {edge_variance:.6f} rad²")

    return center_variance, edge_variance

def generate_variance_model(csv_file):
    """
    Main function to generate the complete variance model
    """
    df = analyze_measurement_errors(csv_file)

    print("\n" + "="*50)
    print("VARIANCE MODELING")
    print("="*50)

    print("\n--- Range Variance Model ---")
    range_const, range_linear = model_range_variance(df)

    print("\n--- Bearing Variance Model ---")
    bearing_center, bearing_edge = model_bearing_variance(df)

    # Generate final model code
    print("\n" + "="*50)
    print("SUGGESTED IMPLEMENTATION:")
    print("="*50)

    print("""
import numpy as np

class MeasurementVarianceModel:
    def __init__(self):
        # Range variance model (choose one based on your data)
        self.range_var_constant = {:.6f}  # Simple constant model
        {}

        # Bearing variance model
        self.bearing_var_center = {:.6f}  # rad²
        self.bearing_var_edge = {:.6f}    # rad²
        self.bearing_edge_threshold = 0.35  # ~20 degrees

    def get_covariance(self, measured_range, measured_bearing):
        # Range variance (pick the model that fits best)
        # Option 1: Constant
        sigma_r_squared = self.range_var_constant

        # Option 2: Linear with distance
        # sigma_r_squared = self.range_var_slope * measured_range + self.range_var_base

        # Bearing variance
        if abs(measured_bearing) < self.bearing_edge_threshold:
            sigma_theta_squared = self.bearing_var_center
        else:
            sigma_theta_squared = self.bearing_var_edge

        # Return 2x2 covariance matrix
        Qt = np.diag([sigma_r_squared, sigma_theta_squared])
        return Qt
    """.format(
        range_const,
        f"self.range_var_slope = {range_linear[0]:.6f}  # Linear model slope\n        self.range_var_base = {range_linear[1]:.6f}   # Linear model intercept"
        if range_linear is not None else "# Linear model not fitted (insufficient data)",
        bearing_center,
        bearing_edge
    ))

    # Create plots
    plt.figure(figsize=(12, 5))

    # Plot 1: Range errors vs distance
    plt.subplot(1, 2, 1)
    plt.scatter(df['measured_range'], df['range_error'], alpha=0.5)
    plt.xlabel('Distance (m)')
    plt.ylabel('Range Error (m)')
    plt.title('Range Errors vs Distance')
    plt.grid(True)

    # Plot 2: Bearing errors vs bearing
    plt.subplot(1, 2, 2)
    plt.scatter(np.degrees(df['measured_bearing']),
                np.degrees(df['bearing_error']), alpha=0.5)
    plt.xlabel('Bearing (degrees)')
    plt.ylabel('Bearing Error (degrees)')
    plt.title('Bearing Errors vs Bearing Angle')
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        # Use most recent file
        import glob
        files = glob.glob('measurement_errors_*.csv')
        if files:
            csv_file = sorted(files)[-1]
            print(f"Using file: {csv_file}")
        else:
            print("No measurement CSV files found!")
            sys.exit(1)

    generate_variance_model(csv_file)
