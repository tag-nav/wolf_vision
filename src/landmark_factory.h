/*
 * \landmark_factory.h
 *
 *  Created on: 09/03/2016
 *      \author: jsola
 */

#ifndef LANDMARK_FACTORY_H_
#define LANDMARK_FACTORY_H_

template<class LmkDerived>
class LandmarkMaker
{
    public:
        LandmarkMaker();
        virtual ~LandmarkMaker();

        virtual LandmarkBase* newLandmark()
        {
            return new LmkDerived;
        }
};

class LandmarkFactory
{
    public:
        LandmarkFactory();
        virtual ~LandmarkFactory();

        bool addMaker(LandmarkMaker<>* _lmk_maker);

        LandmarkBase* newLandmark(LandmarkType _lmk_type);
};

#endif /* LANDMARK_FACTORY_H_ */
