#ifndef _name_cap_H_
#define _name_cap_H_

//Wolf includes
#include "wolf.h"
#include "base_header_file"

namespace wolf
{

WOLF_PTR_TYPEDEFS(class_name);

class class_name : public base_class()
{
    public:

        /** \brief Class constructor
         */
        class_name();

        /** \brief Class Destructor
         */
        virtual ~class_name();
};

} // namespace wolf

#endif /* _name_cap_H_ */
