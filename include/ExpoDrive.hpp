#ifndef EXPODRIVE_HPP
#define EXPODRIVE_HPP

class ExpoDrive {
 private:
  double a_;  // curvature [1.0, 1.3]
  double d_;  // deadzone [0, s]
  double s_;  // scale [1, 127]
  double n_;  // minimum output [0, s]

  double calculateG(double x) const;
  double calculateI(double x) const;

 public:
  // Constructor with default parameters
  ExpoDrive(double a = 1.028, double d = 32.2, double s = 127.0, double n = 27.7);

  // Calculate exponential drive output
  double calculate(double x) const;

  // Getters and setters with validation
  void setA(double a);
  void setD(double d);
  void setS(double s);
  void setN(double n);

  // Getters
  double getA() const { return a_; }
  double getD() const { return d_; }
  double getS() const { return s_; }
  double getN() const { return n_; }
};

#endif  // EXPODRIVE_HPP