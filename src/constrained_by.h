/**
 * \file constrained_by.h
 *
 *  Created on: 14/03/2016
 *      \author: jsola
 */

#ifndef CONSTRAINED_BY_H_
#define CONSTRAINED_BY_H_

// Wolf includes
#include "wolf.h"

class ConstrainedBy
{
    public:
        ConstrainedBy();
        virtual ~ConstrainedBy();

        void addConstrainedBy(ConstraintBase* _ctr_ptr)
        {
            constrained_by_list_.push_back(_ctr_ptr);
        }
        void removeConstrainedBy(ConstraintBase* _ctr_ptr)
        {
            constrained_by_list_.remove(_ctr_ptr);
        }
        unsigned int getHits() const
        {
            return constrained_by_list_.size();
        }
        ConstraintBaseList* getConstrainedByListPtr()
        {
            return &constrained_by_list_;
        }


    private:
        ConstraintBaseList constrained_by_list_;
};

#endif /* CONSTRAINED_BY_H_ */
