#include <iostream>
#include <vector>
#include <cmath>

// Class representing a B-spline curve
class BSpline {
public:
    // Constructor takes a vector of control points as input
    BSpline(const std::vector<std::vector<double>>& controlPoints) : controlPoints_(controlPoints) {
        buildKnotVector(); // Build the knot vector based on the number of control points
    }

    // Function to get a smooth path by evaluating the B-spline curve at multiple points
    std::vector<std::vector<double>> getSmoothPath(int resolution) {
        std::vector<std::vector<double>> smoothedPath; // Vector to store the smooth path

        int n = controlPoints_.size() - 1; // Number of control points minus one (used for loop limits)
        double interval = static_cast<double>(knotVector_.back()) / (resolution - 1); // Calculate the interval between evaluation points

        // Loop through the resolution points and evaluate the B-spline curve
        for (int i = 1; i < resolution - 1; ++i) {
            double t = i * interval; // Calculate the parameter value 't' based on the interval
            std::vector<double> point = evaluate(t, n); // Evaluate the B-spline curve at 't'
            smoothedPath.push_back(point); // Add the evaluated point to the smooth path
        }

        smoothedPath.push_back(controlPoints_[n]);  // Add the last control point as a separate point to complete the path

        return smoothedPath; // Return the smooth path
    }

private:
    std::vector<std::vector<double>> controlPoints_; // Vector to store the control points
    std::vector<double> knotVector_; // Vector to store the knot vector
    int degree_ = 3; // The degree of the B-spline curve (degree 3 means cubic B-spline)

    // Function to build the knot vector based on the number of control points
    void buildKnotVector() {
        int m = controlPoints_.size(); // Number of control points
        int n = m + degree_ + 1; // Total number of knots (including the duplicated ones at the end)
        knotVector_.resize(n); // Resize the knot vector to store 'n' elements

        // Calculate and set the values of the knots in the knot vector
        for (int i = 0; i < n; ++i) {
            if (i < degree_)
                knotVector_[i] = 0.0; // First 'degree_' knots are set to 0
            else if (i >= degree_ && i <= m)
                knotVector_[i] = i - degree_ + 1.0; // Knots from 'degree_' to 'm' are linearly spaced from 1 to 'm - degree_ + 1'
            else if (i > m)
                knotVector_[i] = m - degree_ + 2.0; // Last 'degree_' knots are set to 'm - degree_ + 2'
        }
    }

    // Function to evaluate the B-spline curve at a given parameter value 't'
    std::vector<double> evaluate(double t, int n) {
        std::vector<double> point(2, 0.0); // Vector to store the evaluated point (2D point)

        double denominator = 0.0; // Used to calculate the weighted sum of control points

        // Loop through all control points and calculate their contributions to the evaluated point
        for (int i = 0; i <= n; ++i) {
            double weight = basisFunction(i, degree_, t); // Calculate the weight for the i-th control point at parameter value 't'
            point[0] += controlPoints_[i][0] * weight; // Weighted sum of control points in x-coordinate
            point[1] += controlPoints_[i][1] * weight; // Weighted sum of control points in y-coordinate
            denominator += weight; // Sum of weights for normalization
        }

        // Normalize the point by dividing by the sum of weights (to account for varying weights at different parameter values)
        if (denominator != 0.0) {
            point[0] /= denominator;
            point[1] /= denominator;
        }

        return point; // Return the evaluated point on the B-spline curve
    }

    // Function to calculate the basis function value at parameter value 't'
    double basisFunction(int i, int k, double t) {
        if (k == 0) {
            // Base case for the recursive calculation of basis functions: linear interpolation between knots
            if (t >= knotVector_[i] && t < knotVector_[i + 1])
                return 1.0; // Basis function is 1 between knots, and 0 elsewhere
            else
                return 0.0;
        }

        double factor1 = 0.0;
        double factor2 = 0.0;

        // Calculate the two factors required for the recursive calculation of basis functions
        if (knotVector_[i + k] - knotVector_[i] != 0.0)
            factor1 = (t - knotVector_[i]) / (knotVector_[i + k] - knotVector_[i]);
        if (knotVector_[i + k + 1] - knotVector_[i + 1] != 0.0)
            factor2 = (knotVector_[i + k + 1] - t) / (knotVector_[i + k + 1] - knotVector_[i + 1]);

        // Recursive calculation of basis function using the two factors and the previous level basis functions
        double result = factor1 * basisFunction(i, k - 1, t) + factor2 * basisFunction(i + 1, k - 1, t);
        return result;
    }
};
