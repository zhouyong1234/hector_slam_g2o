#ifndef EDGE_SE2_PRIORVEC_HPP
#define EDGE_SE2_PRIORVEC_HPP

#include <g2o/core/base_unary_edge.h>

#include "vertex_se2.hpp"


class EdgeSE2PriorVec : public BaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, VertexSE2>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE2PriorVec() : BaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, VertexSE2>() {}
    
    void computeError() override 
    {
        const VertexSE2* v1 = static_cast<VertexSE2*>(_vertices[0]);

        Eigen::Vector3d direction = _measurement.head<3>();
        Eigen::Vector3d measurement = _measurement.tail<3>();

        double angle = v1->estimate().rotation().angle();
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << cos(angle), -sin(angle), 0,
                        sin(angle),  cos(angle), 0,
                        0,           0,          1;

        Eigen::Vector3d estimate = rotation_matrix.inverse() * direction;

        _error = estimate - measurement;
    }

    void setMeasurement(const Eigen::Matrix<double, 6, 1>& m) override
    {
        _measurement.head<3>() = m.head<3>().normalized();
        _measurement.tail<3>() = m.tail<3>().normalized();
    }

    virtual bool read(std::istream& in) override {}
    virtual bool write(std::ostream& out) const override {}
};


#endif