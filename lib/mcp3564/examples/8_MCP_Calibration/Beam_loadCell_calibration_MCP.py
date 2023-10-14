import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from sklearn.metrics import mean_squared_error, r2_score

# Load the data
data = pd.read_csv('calibrationWeightSingleBeamMCP.csv')
y = data['Weight'].values  # Weight as the dependent variable
x = data['Reading'].values.reshape(-1, 1)  # ADC reading as the independent variable

offset = data[data['Weight'] == 0]['Reading'].values[0]

# Single Scaling Factor (excluding the zero reading)
x_offset = x - offset
non_zero_readings = x_offset.flatten() != 0
individual_scale_factors = y[non_zero_readings] / x_offset[non_zero_readings].flatten()
scaling_factor = np.mean(individual_scale_factors)
y_pred_single_scale = x_offset.flatten() * scaling_factor

# Linear Regression
linear_regressor = LinearRegression()
linear_regressor.fit(x, y)
y_pred_linear = linear_regressor.predict(x)

# Polynomial Regression (2nd Degree)
poly = PolynomialFeatures(degree=2)
x_poly = poly.fit_transform(x)
poly_regressor = LinearRegression()
poly_regressor.fit(x_poly, y)
y_pred_poly = poly_regressor.predict(x_poly)

# Plotting the results
plt.scatter(x, y, color='red', label='Data Points', marker='o')
plt.plot(x, y_pred_single_scale, color='purple', label='Single Scaling Factor', linestyle='dashed')
plt.plot(x, y_pred_linear, color='blue', label='Linear Regression', marker='^')
plt.plot(x, y_pred_poly, color='green', label='Polynomial Regression', marker='s')
plt.title('Calibration Techniques MCP34')
plt.xlabel('ADC Reading')
plt.ylabel('Weight (known)')
plt.legend()
plt.show()

# Error calculations
mse_single_scale = mean_squared_error(y, y_pred_single_scale)
mse_linear = mean_squared_error(y, y_pred_linear)
mse_poly = mean_squared_error(y, y_pred_poly)

r2_single_scale = r2_score(y, y_pred_single_scale)
r2_linear = r2_score(y, y_pred_linear)
r2_poly = r2_score(y, y_pred_poly)

print(f"Mean Squared Error (Single Scaling Factor): {mse_single_scale}")
print(f"Mean Squared Error (Linear Regression): {mse_linear}")
print(f"Mean Squared Error (Polynomial Regression): {mse_poly}")

# Determine which method is best
min_mse = min(mse_linear, mse_poly, mse_single_scale)
if min_mse == mse_linear:
    print("Linear Regression is the best fit for the data.")
elif min_mse == mse_poly:
    print("Polynomial Regression is the best fit for the data.")
else:
    print("Single Scaling Factor is the best fit for the data.")

print(f"R^2 value (Single Scaling Factor): {r2_single_scale}")
print(f"R^2 value (Linear Regression): {r2_linear}")
print(f"R^2 value (Polynomial Regression): {r2_poly}")

# Print equations
print(f"Single Scaling Factor: y = {scaling_factor}x ")
print(f"Linear Regression Equation: y = {linear_regressor.coef_[0]}x + {linear_regressor.intercept_}")
print(f"Polynomial Regression Equation: y = {poly_regressor.coef_[2]}x^2 + {poly_regressor.coef_[1]}x + {poly_regressor.intercept_}")

# Compute the RMSE for each calibration technique using the original dataset
rmse_single_scale_original = np.sqrt(mse_single_scale)
rmse_linear_original = np.sqrt(mse_linear)
rmse_poly_original = np.sqrt(mse_poly)

print(f"Root Mean Squared Error (Single Scaling Factor): {rmse_single_scale_original} grams")
print(f"Root Mean Squared Error (Linear Regression): {rmse_linear_original} grams")
print(f"Root Mean Squared Error (Polynomial Regression): {rmse_poly_original} grams")




