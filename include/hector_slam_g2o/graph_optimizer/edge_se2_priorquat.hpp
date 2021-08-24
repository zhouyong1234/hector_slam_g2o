#ifndef EDGE_SE2_PRIORQUAT_HPP
#define EDGE_SE2_PRIORQUAT_HPP

#include <g2o/core/base_unary_edge.h>

#include "vertex_se2.hpp"


class EdgeSE2PriorQuat : public g2o::BaseUnaryEdge<3, Eigen::Quaterniond, VertexSE2>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE2PriorQuat() : g2o::BaseUnaryEdge<3, Eigen::Quaterniond, VertexSE2>() {}

  void computeError() override {
    const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);

    double angle = v1->estimate().rotation().angle();
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << cos(angle), -sin(angle), 0,
                       sin(angle),  cos(angle), 0,
                       0,           0,          1;
    Eigen::Quaterniond estimate =  Eigen::Quaterniond(rotation_matrix);

    // std::cout << "estimate: " << estimate.x() << " " << estimate.y() << " " << estimate.z() << " " << estimate.w() << std::endl;

    // Eigen::Quaterniond estimate1 = Eigen::Quaterniond(cos(angle / 2), 0, 0, sin(angle / 2));

    // std::cout << "estimate1: " << estimate1.x() << " " << estimate1.y() << " " << estimate1.z() << " " << estimate1.w() << std::endl;

    if(estimate.w() < 0)
    {
        estimate.coeffs() = -estimate.coeffs();
    }

    // if(_measurement.coeffs().dot(estimate.coeffs()) < 0.0) {
    //   estimate.coeffs() = -estimate.coeffs();
    // }

    _error = estimate.vec() - _measurement.vec();

    // std::cout << "error: " << std::endl << _error << std::endl;
  }

  void setMeasurement(const Eigen::Quaterniond& m) override {
    _measurement = m;
    if(m.w() < 0.0) {
      _measurement.coeffs() = -m.coeffs();
    }
  }
  
  virtual bool read(std::istream& in) {}
  virtual bool write(std::ostream& out) const {}

};

#endif
