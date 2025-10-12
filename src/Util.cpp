#include <vector>
#include <stdexcept> // For std::runtime_error

// Function to perform linear interpolation on arrays
double interpolateLinear(const std::vector<double>& x_data,
                         const std::vector<double>& y_data,
                         double x_interp) {
    // Basic error checking
    if (x_data.size() != y_data.size() || x_data.empty()) {
        throw std::runtime_error("Input arrays must have the same non-zero size.");
    }

    // Handle edge cases where x_interp is outside the data range
    if (x_interp <= x_data.front()) {
        return y_data.front();
    }
    if (x_interp >= x_data.back()) {
        return y_data.back();
    }

    // Find the interval where x_interp lies
    for (size_t i = 0; i < x_data.size() - 1; ++i) {
        if (x_interp >= x_data[i] && x_interp <= x_data[i+1]) {
            // Linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            double x1 = x_data[i];
            double y1 = y_data[i];
            double x2 = x_data[i+1];
            double y2 = y_data[i+1];

            return y1 + (x_interp - x1) * (y2 - y1) / (x2 - x1);
        }
    }

    // This part should ideally not be reached if edge cases and loop logic are correct
    throw std::runtime_error("Could not find interpolation interval.");
}