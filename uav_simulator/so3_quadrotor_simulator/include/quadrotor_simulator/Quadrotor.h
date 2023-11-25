#ifndef __QUADROTOR_SIMULATOR_QUADROTOR_H__
#define __QUADROTOR_SIMULATOR_QUADROTOR_H__

#include <Eigen/Core>
#include <boost/array.hpp>

namespace QuadrotorSimulator
{

// 此处实现与ego-planner不一样

class Quadrotor
{
public:
  struct State
  {
    Eigen::Vector3d x;
    Eigen::Vector3d v;
    Eigen::Matrix3d R;  // 旋转矩阵
    Eigen::Vector3d omega;  // 三轴角速度（什么坐标系？）
    Eigen::Array4d  motor_rpm;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  Quadrotor();

  const Quadrotor::State& getState(void) const;

  void setState(const Quadrotor::State& state);

  void setStatePos(const Eigen::Vector3d& Pos);

  double getMass(void) const;
  void setMass(double mass);

  double getGravity(void) const;
  void setGravity(double g);

  const Eigen::Matrix3d& getInertia(void) const;
  void setInertia(const Eigen::Matrix3d& inertia);

  double getArmLength(void) const;
  void setArmLength(double d);

  double getPropRadius(void) const;
  void setPropRadius(double r);

  double getPropellerThrustCoefficient(void) const;
  void setPropellerThrustCoefficient(double kf);

  double getPropellerMomentCoefficient(void) const;
  void setPropellerMomentCoefficient(double km);

  double getMotorTimeConstant(void) const;
  void setMotorTimeConstant(double k);

  const Eigen::Vector3d& getExternalForce(void) const;
  void setExternalForce(const Eigen::Vector3d& force);   // ?

  const Eigen::Vector3d& getExternalMoment(void) const;
  void setExternalMoment(const Eigen::Vector3d& moment); // ?

  double getMaxRPM(void) const;
  void setMaxRPM(double max_rpm); // 最大转速

  double getMinRPM(void) const;
  void setMinRPM(double min_rpm); // 最小转速

  // Inputs are desired RPM for the motors
  // Rotor numbering is:
  //   *1*    Front
  // 3     4
  //    2
  // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
  void setInput(double u1, double u2, double u3, double u4);

  // Runs the actual dynamics simulation with a time step of dt
  void step(double dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 22> InternalState; // 就是状态的一维表示
  void operator()(const Quadrotor::InternalState& x,
                  Quadrotor::InternalState&       dxdt, const double /* t */);

  Eigen::Vector3d getAcc() const;

private:
  void updateInternalState(void);

  double          alpha0; // AOA ？？
  double          g_;     // gravity
  double          mass_;
  Eigen::Matrix3d J_; // Inertia 转动惯量
  double          kf_;  // 拉力系数
  double          km_;  // 扭力系数
  double          prop_radius_; // 桨叶半径
  double          arm_length_;  // 臂长
  double          motor_time_constant_; // unit: sec ？？
  double          max_rpm_; // 最大转速
  double          min_rpm_; // 最小转速

  Quadrotor::State state_;

  Eigen::Vector3d acc_;

  Eigen::Array4d  input_;
  Eigen::Vector3d external_force_;
  Eigen::Vector3d external_moment_;

  InternalState internal_state_;
};
}
#endif
