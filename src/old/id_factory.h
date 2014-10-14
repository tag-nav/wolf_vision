/*
 * if_factory.h
 *
 *  Created on: May 28, 2014
 *      \author: jsola
 */

#ifndef ID_FACTORY_H_
#define ID_FACTORY_H_

#include <list>
#include <set>
#include <iostream>

/*
 Forward declarations (cannot forward default parameters):

 template<typename> class IdCollectorNone;
 template<typename> class IdCollectorList;
 template<typename> class IdCollectorSet;
 template<typename, template<typename> class> class IdFactory;

 */

/**
 @ingroup wolf
 \brief Collector of the last ID only

 This none collector only collects the last id,
 so it can overflow even if not all ids are used
 Memory complexity: constant
 Time complexity: constant
 */
template<typename T>
class IdCollectorNone
{
    public:
        bool empty()
        {
            return true;
        }         ///< does the collector have free ids
        T get()
        {
            return 0;
        }                 ///< get an id
        bool collect(T id)
        {
            return false;
        }  ///< tries to collect the id
        T size()
        {
            return 0;
        }                ///< number of free ids
        void clear()
        {
        }                       ///< clear the free ids
        T defrag(T lastid)
        {
            return lastid;
        } ///< convert some free ids here into a decreased lastid if possible
};

/**
 @ingroup wolf
 \brief collector of unused ids

 This collector correctly collects all ids, but does not perform all sanity checks,
 in particular it does not check if the released id was really taken
 Memory complexity: linear in number of collected ids
 Time complexity: constant
 */
template<typename T>
class IdCollectorList
{
    private:
        typedef std::list<T> storage_t;
        storage_t freed;
    public:
        bool empty()
        {
            return freed.empty();
        }
        T get()
        {
            T res = freed.front();
            freed.pop_front();
            return res;
        }
        bool collect(T id)
        {
            freed.push_back(id);
            return true;
        }
        T size()
        {
            return freed.size();
        }
        void clear()
        {
            freed.clear();
        }
        T defrag(T lastid)
        {
            return lastid;
        } // would be too expensive to do something (at least n*log(n))
};

/**
 @ingroup kernel
 \brief Collector of unused ids, with sanity checks

 This collector correctly collects all ids, defragments, and performs all sanity checks
 Memory complexity: linear in number of collected ids, with defragmentation
 Time complexity: constant except releaseId that is logarithmic in number of collected ids, and defrag that can be linear in some cases
 */
template<typename T>
class IdCollectorSet
{
    private:
        typedef std::set<T> storage_t;
        storage_t freed;
    public:
        bool empty()
        {
            return freed.empty();
        }
        T get()
        {
            T res = *freed.begin();
            freed.erase(freed.begin());
            return res;
        }
        bool collect(T id)
        {
            return freed.insert(id).second;
        }
        T size()
        {
            return freed.size();
        }
        void clear()
        {
            freed.clear();
        }
        T defrag(T lastid)
        {
            typename storage_t::reverse_iterator rit;
            for (rit = freed.rbegin(); !freed.empty() && *rit == lastid; --lastid)
            {
                freed.erase(--rit.base());
                if (!freed.empty())
                    ++rit;
            }
            return lastid;
        }
};

/**
 * @ingroup kernel
 * \brief This class allows to create an unique id.
 * \param T the id storage type
 * \param IdCollector the id collector type
 */
template<typename T = unsigned, template<typename > class IdCollector = IdCollectorNone>
class IdFactory
{
    public:
        IdFactory() :
                m_lastId(0)
        {
        }
        virtual ~IdFactory()
        {
        }

        typedef T storage_t;

        /// Call this to get an unique id, starting from 1, 0 means overflow
        T getId()
        {
            T id = (freed.empty() ? ++m_lastId : freed.get());
//std::cout << "IdFactory::getId " << id << std::endl;
            return id;
        }

        /// Call this when you want to stop using an id
        bool releaseId(T id)
        {
//std::cout << "IdFactory::releaseId " << id << std::endl;
            if (id == 0 || id > m_lastId)
                return false;
            if (id == m_lastId)
                m_lastId = freed.defrag(m_lastId - 1);
            else
                return freed.collect(id);
            return true;
        }

        /// Call this when you want to reset the factory without destroying it
        void reset()
        {
            m_lastId = 0;
            freed.clear();
        }

        /// Call this to get the number of ids currently used
        T countUsed()
        {
            return m_lastId - freed.size();
        }

        /// Call this to get the number of ids currently collected
        T countCollected()
        {
            return freed.size();
        }

    private:
        T m_lastId;
        IdCollector<T> freed;
};

typedef IdFactory<unsigned, IdCollectorNone> IdFactoryNone;
typedef IdFactory<unsigned, IdCollectorList> IdFactoryList;
typedef IdFactory<unsigned, IdCollectorSet> IdFactorySet;

#endif /* ID_FACTORY_H_ */
