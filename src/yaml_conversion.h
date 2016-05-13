/**
 * \file yaml_conversion.h
 *
 *  Created on: May 12, 2016
 *      \author: jsola
 */

#ifndef YAML_CONVERSION_H_
#define YAML_CONVERSION_H_

// Yaml
#include <yaml-cpp/yaml.h>

// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// stl
#include <iostream>

namespace YAML
{

/**\brief Bridge YAML <--> Eigen::Matrix <> with all template possibilities except full dynamic.
 *
 */
template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct convert<Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> >
{
        static Node encode(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
        {
            Node node;//(NodeType::Sequence);

            int nValues = matrix.rows() * matrix.cols();

            for (int i = 0; i < nValues; ++i)
            {
                node.push_back(matrix(i / matrix.cols(), i % matrix.cols()));
            }

            return node;
        }

        static bool decode(const Node& node, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
        {
            if (_Rows == Eigen::Dynamic && _Cols == Eigen::Dynamic)
            {
                std::cout << "Bridge to Eigen::Matrix: Matrices may only be dynamic in one dimension!"
                        << std::endl;
                return false;
            }

            int nSize = node.size();

            // If one dimension is dynamic -> calculate and resize

            // _Rows is Dynamic
            if (_Rows == Eigen::Dynamic)
            {
                if (nSize % _Cols != 0)
                {
                    std::cout << "Bridge to Eigen::Matrix: Input size of dynamic row matrix is not a multiple of fixed column size" << std::endl;
                    return false;
                }

                int nDynRows = nSize / _Cols;
                matrix.resize(nDynRows, Eigen::NoChange);
            }

            // _Cols is Dynamic
            if (_Cols == Eigen::Dynamic)
            {
                if (nSize % _Rows != 0)
                {
                    std::cout << "Bridge to Eigen::Matrix: Input size of dynamic column matrix is not a multiple of fixed row size!" << std::endl;
                    return false;
                }

                int nDynCols = nSize / _Rows;
                matrix.resize(Eigen::NoChange, nDynCols);
            }

            // final check for good size
            if (nSize != matrix.rows() * matrix.cols())
            {
                std::cout << "Bridge to Eigen::Matrix. Wrong input size!" << std::endl;
                return false;
            }
            else // Fill the matrix
                for (int i = 0; i < matrix.rows(); i++)
                    for (int j = 0; j < matrix.cols(); j++)
                        matrix(i, j) = node[(int)(i * matrix.cols() + j)].as<_Scalar>();
            return true;
        }
};

/**\brief Bridge YAML <--> Eigen::Quaternion with real component last
 *
 * WARNING: Beware of Eigen constructor order!
 *
 * We use the x-y-z-w order, with the real part at the end. This is consistent with ROS Quaternion.msg,
 * which is good for compatibility against ROS messages and YAML configuration.
 *
 */
template<typename _Scalar, int _Options>
struct convert<Eigen::Quaternion<_Scalar, _Options> >
{
        static Node encode(const Eigen::Quaternion<_Scalar, _Options>& quaternion)
        {
            Node node(NodeType::Sequence);

            node[0] = quaternion.x();
            node[1] = quaternion.y();
            node[2] = quaternion.z();
            node[3] = quaternion.w();

            return node;
        }

        static bool decode(const Node& node, Eigen::Quaternion<_Scalar, _Options>& quaternion)
        {

            int nSize = node.size(); // Sequence check is implicit
            if (nSize != 4)
            {
                std::cout
                        << "Bridge to Eigen::QuaternionBase< Eigen::Quaternion<_Scalar,_Options> >: Wrong input size!"
                        << std::endl;
                return false;
            }
            else
            {
                quaternion.x() = node[0].as<_Scalar>();
                quaternion.y() = node[1].as<_Scalar>();
                quaternion.z() = node[2].as<_Scalar>();
                quaternion.w() = node[3].as<_Scalar>();
            }
            return true;
        }
};

} // namespace YAML

#endif /* YAML_CONVERSION_H_ */
