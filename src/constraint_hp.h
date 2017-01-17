/*
 * constraint_hp.h
 *
 *  Created on: Nov 28, 2016
 *      Author: jsola
 */

#ifndef CONSTRAINT_HP_H_
#define CONSTRAINT_HP_H_

#include "constraint_sparse.h"

namespace wolf
{

class ConstraintHP : public ConstraintSparse<2, 3, 4, 4>
{
    public:
        ConstraintHP()
        {
            // TODO Auto-generated constructor stub
        }
        virtual ~ConstraintHP()
        {
            // TODO Auto-generated destructor stub
        }
};

} /* namespace wolf */

#endif /* CONSTRAINT_HP_H_ */
