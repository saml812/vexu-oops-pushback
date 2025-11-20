#ifndef EXPODRIVE_HPP
#define EXPODRIVE_HPP

/**
 * @brief Lemlib's exponential drive controller
 */
class ExpoDrive {
 private:
  double a_;  /// Curvature parameter [1.0, 1.3]
  double d_;  /// Deadzone range [0, s]
  double s_;  /// Scale factor [1, 127]
  double n_;  /// Minimum output [0, s]

  // Private helper methods for internal calculations
  double calculateG(double x) const;  ///< Calculates the exponential gain
  double calculateI(double x) const;  ///< Calculates the inverse exponential function

 public:
  /**
   * @brief Construct a new ExpoDrive object
   * 
   * @param a Curvature parameter (default: 1.028)
   * @param d Deadzone range (default: 32.2)
   * @param s Scale factor (default: 127.0)
   * @param n Minimum output (default: 27.7)
   */
  ExpoDrive(double a = 1.028, double d = 32.2, double s = 127.0, double n = 27.7);

  /**
   * @brief Calculate the exponential drive output for a given input
   * 
   * @param x Input value
   * @return double Processed output value after applying exponential curve
   */
  double calculate(double x) const;

  // Parameter setters
  void setA(double a);  /// Set curvature parameter
  void setD(double d);  /// Set deadzone range
  void setS(double s);  /// Set scale factor
  void setN(double n);  /// Set minimum output

  // Parameter getters
  double getA() const { return a_; }  /// Get current curvature parameter
  double getD() const { return d_; }  /// Get current deadzone range
  double getS() const { return s_; }  /// Get current scale factor
  double getN() const { return n_; }  /// Get current minimum output
};

#endif  // EXPODRIVE_HPP