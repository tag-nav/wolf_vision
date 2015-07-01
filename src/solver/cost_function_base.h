/*
 * cost_function_base.h
 *
 *  Created on: Jun 25, 2015
 *      Author: jvallve
 */

#ifndef TRUNK_SRC_SOLVER_COST_FUNCTION_BASE_H_
#define TRUNK_SRC_SOLVER_COST_FUNCTION_BASE_H_

#include "wolf.h"

class CostFunctionBase
{
    protected:
        unsigned int n_blocks_;
        Eigen::MatrixXs J_0_;
        Eigen::MatrixXs J_1_;
        Eigen::MatrixXs J_2_;
        Eigen::MatrixXs J_3_;
        Eigen::MatrixXs J_4_;
        Eigen::MatrixXs J_5_;
        Eigen::MatrixXs J_6_;
        Eigen::MatrixXs J_7_;
        Eigen::MatrixXs J_8_;
        Eigen::MatrixXs J_9_;
        Eigen::VectorXs residual_;
        std::vector<Eigen::MatrixXs*> jacobians_;
        std::vector<unsigned int> block_sizes_;

    public:
        CostFunctionBase(const unsigned int &_measurement_size,
                         const unsigned int &_block_0_size,
                         const unsigned int &_block_1_size,
                         const unsigned int &_block_2_size,
                         const unsigned int &_block_3_size,
                         const unsigned int &_block_4_size,
                         const unsigned int &_block_5_size,
                         const unsigned int &_block_6_size,
                         const unsigned int &_block_7_size,
                         const unsigned int &_block_8_size,
                         const unsigned int &_block_9_size) :
            n_blocks_(10),
            J_0_(_measurement_size, _block_0_size),
            J_1_(_measurement_size, _block_1_size),
            J_2_(_measurement_size, _block_2_size),
            J_3_(_measurement_size, _block_3_size),
            J_4_(_measurement_size, _block_4_size),
            J_5_(_measurement_size, _block_5_size),
            J_6_(_measurement_size, _block_6_size),
            J_7_(_measurement_size, _block_7_size),
            J_8_(_measurement_size, _block_8_size),
            J_9_(_measurement_size, _block_9_size),
            residual_(_measurement_size),
            jacobians_({&J_0_,&J_1_,&J_2_,&J_3_,&J_4_,&J_5_,&J_6_,&J_7_,&J_8_,&J_9_}),
            block_sizes_({_block_0_size, _block_1_size, _block_2_size, _block_3_size, _block_4_size, _block_5_size, _block_6_size, _block_7_size, _block_8_size, _block_9_size})
            {
                for (unsigned int i = 1; i<n_blocks_; i++)
                {
                    if (block_sizes_.at(i) == 0)
                    {
                        n_blocks_ = i;
                        jacobians_.resize(n_blocks_);
                        block_sizes_.resize(n_blocks_);
                        break;
                    }
                }
            }

        virtual ~CostFunctionBase()
        {

        }

        virtual void evaluateResidualJacobians() = 0;


        void getResidual(Eigen::VectorXs &residual)
        {
            residual.resize(residual_.size());
            residual = residual_;
        }

        std::vector<Eigen::MatrixXs*> getJacobians()
        {
            return jacobians_;
        }

        void getJacobians(std::vector<Eigen::MatrixXs>& jacobians)
        {
            jacobians.resize(n_blocks_);
            for (unsigned int i = 0; i<n_blocks_; i++)
                jacobians.at(i) = (*jacobians_.at(i));
        }
};

#endif /* TRUNK_SRC_SOLVER_COST_FUNCTION_BASE_H_ */
