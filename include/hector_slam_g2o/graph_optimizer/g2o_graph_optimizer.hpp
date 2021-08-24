#ifndef G2O_GRAPH_OPTIMIZER_HPP
#define G2O_GRAPH_OPTIMIZER_HPP

#include <g2o/core/hyper_graph.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/robust_kernel_factory.h>


#include <Eigen/Core>
#include <cmath>
#include <iostream>

#include "se2.h"
#include "vertex_se2.hpp"
#include "edge_se2.hpp"
#include "edge_se2_priorquat.hpp"
#include "edge_se2_priorvec.hpp"


class G2oGraphOptimizer
{
private:
    /* data */

public:
    g2o::SparseOptimizer optimizer;
    g2o::RobustKernelFactory* robust_kernel_factory;

public:
    G2oGraphOptimizer()
    {
        typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,3> > Block;
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
        Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));

        // optimizer.setVerbose(true);
        optimizer.setAlgorithm(solver);

        robust_kernel_factory = g2o::RobustKernelFactory::instance();
    }

    ~G2oGraphOptimizer() {}

    int optimize(int num_iterations)
    {
        optimizer.initializeOptimization();
        optimizer.optimize(num_iterations);
        // optimizer.save("pose_graph.g2o");
        // optimizer.clear();
        // optimizer.pop();
    }

    VertexSE2* add_se2_node(const SE2& pose)
    {
        VertexSE2* vertex = new VertexSE2();
        vertex->setId(static_cast<int>(optimizer.vertices().size()));
        vertex->setEstimate(pose);
        // vertex->setMarginalized(true);
        optimizer.addVertex(vertex);
        return vertex;
    }

    EdgeSE2* add_se2_edge(VertexSE2* v1, VertexSE2* v2, const SE2& relative_pose, const Eigen::Matrix3d& information_matrix)
    {
        EdgeSE2* edge = new EdgeSE2();
        edge->setMeasurement(relative_pose);
        edge->setInformation(information_matrix);
        edge->setVertex(0, v1);
        edge->setVertex(1, v2);
        // edge->vertices()[0] = v1;
        // edge->vertices()[1] = v2;
        optimizer.addEdge(edge);
        return edge;
    }

    EdgeSE2PriorQuat* add_se2_prior_quat_edge(VertexSE2* v_se2, const Eigen::Quaterniond& quat, const Eigen::Matrix3d& information_matrix)
    {
        EdgeSE2PriorQuat* edge = new EdgeSE2PriorQuat();
        edge->setMeasurement(quat);
        edge->setInformation(information_matrix);
        edge->vertices()[0] = v_se2;
        optimizer.addEdge(edge);
        return edge;
    }


    EdgeSE2PriorVec* add_se2_prior_vec_edge(VertexSE2* v_se2, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::Matrix3d& information_matrix)
    {
        Eigen::Matrix<double, 6, 1> m;
        m.head<3>() = direction;
        m.tail<3>() = measurement;

        EdgeSE2PriorVec* edge = new EdgeSE2PriorVec();
        edge->setMeasurement(m);
        edge->setInformation(information_matrix);
        edge->vertices()[0] = v_se2;
        optimizer.addEdge(edge);
        return edge;
    }
    

    void add_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size)
    {
        if(kernel_type == "NONE")
        {
            return;
        }

        g2o::RobustKernel* kernel = robust_kernel_factory->construct(kernel_type);

        if(kernel == nullptr)
        {
            std::cerr << "invalid robust kernel type..." << std::endl;
            return;
        }

        kernel->setDelta(kernel_size);

        g2o::OptimizableGraph::Edge* edge_ = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
        edge_->setRobustKernel(kernel);

    }
};



#endif