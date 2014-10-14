/*
 *
 *  Created on: 24/07/2014
 *      \author: acorominas
 */

#ifndef CONSTRAINER_BASE_H
#define CONSTRAINER_BASE_H_

//wolf
#include "wolf.h"

//namespaces
using namespace std;
using namespace Eigen;

/** \brief Constrainer base class
 *
 * Constrainer base class. 
 * 
 */
template<class StateT>
class ConstrainerBase
{
    protected: 
        bool is_terminus_; ///< Indicates if the constrainer is terminus or not
        unsigned int dimension_; ///< dimensionality of the constraint
        VectorXs expected_error_local_; ///< Local storage for expectation
        Map<VectorXs> expected_error_; ///< Remote (Mapped) expectation
        
    public:
        /** \brief Constructor with local storage
         * 
         * Constructor with local storage.
         * \param _is_term Indicates if the constrainer is terminus or not
         * \param _loc Placement within Wolf tree. NodeLocation type declared at file wolf.h
         * \param _dim dimension of the error vector.
         * 
         **/
        ConstrainerBase(const bool _is_term, const unsigned int _dim) :
            is_terminus_(_is_term),
            dimension_(_dim),
            expected_error_local_(_dim),
            expected_error_(expected_error_local_.data(), _dim)
        {
            //
        };

        /** \brief Constructor with remote storage
         * 
         * Constructor with remote storage. 
         * Local vector is not allocated, so it remains to size 0.
         * \param _is_term Indicates if the constrainer is terminus or not
         * \param _dim dimension of the mapped vector (size of the mapping).
         * \param _storage remote vector where error will be stored.
         * \param _idx index to remote vector from which start the mapping.
         * 
         **/
        ConstrainerBase(const bool _is_term, const unsigned int _dim, const VectorXs & _storage, const unsigned int _idx, ) :
            is_terminus_(_is_term),
            dimension_(_dim),
            expected_error_(
                ((_idx + _dim <= _storage.size()) ? _storage.data() + _idx : NULL), 
                ((_idx + _dim <= _storage.size()) ? _dim : 0))
        {
            //
        };
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~ConstrainerBase()
        {
            //
        };
        
        /** \brief Calls expectation recursively
         * 
         * Calls expectation recursively up to find a terminus constrainer.
         * When a terminus constrainer is found, it calls computeExpectation().
         * 
         **/
        virtual void expectation()
        {
            if (is_terminus_)
            {
                computeExpectation();
            }
            else
            {
                
            }
            
            switch(this->location_)
            {
                case TOP:
                case MID: 
                    for (auto iter = this->down_node_map_.begin(); iter != this->down_node_map_.end(); ++iter)
                        iter->second->expectation();
                    break;
                    
                case BOTTOM:
                    computeExpectation();
                    break;
                    
                default:
                    break;
                    
            }
        };
        
        /** \brief Actually computes expectation
         * 
         * Actually computes expectation and places the result to mapped vector storage
         * This method has to be reimplemented for those nodes placed at the bottom, that will be required to compute an expectation
         * 
         **/        
        virtual void computeExpectation()
        {
            //
        };
};
#endif
