
// wolf
#include "map_base.h"
#include "landmark_base.h"
#include "factory.h"

// YAML
#include <yaml-cpp/yaml.h>

// stl
#include <fstream>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>



namespace wolf {

MapBase::MapBase() :
    NodeLinked(MID, "MAP", "BASE")
{
    //std::cout << "MapBase::MapBase(): " << __LINE__ << std::endl;
}

MapBase::~MapBase()
{
	//std::cout << "deleting MapBase " << nodeId() << std::endl;
}

LandmarkBase* MapBase::addLandmark(LandmarkBase* _landmark_ptr)
{
	//std::cout << "MapBase::addLandmark" << std::endl;
    addDownNode(_landmark_ptr);
    _landmark_ptr->registerNewStateBlocks();
    return _landmark_ptr;
}

void MapBase::addLandmarkList(LandmarkBaseList _landmark_list)
{
	//std::cout << "MapBase::addLandmarkList" << std::endl;
	LandmarkBaseList lmk_list_copy = _landmark_list; //since _landmark_list will be empty after addDownNodeList()
	addDownNodeList(_landmark_list);
    for (auto landmark_ptr : lmk_list_copy)
        landmark_ptr->registerNewStateBlocks();
}

void MapBase::removeLandmark(LandmarkBase* _landmark_ptr)
{
    removeDownNode(_landmark_ptr->nodeId());
}

void MapBase::removeLandmark(const LandmarkBaseIter& _landmark_iter)
{
    removeDownNode(_landmark_iter);
}

void MapBase::load(const std::string& _map_file_dot_yaml)
{
    YAML::Node map = YAML::LoadFile(_map_file_dot_yaml);

    unsigned int nlandmarks = map["nlandmarks"].as<unsigned int>();

    assert(nlandmarks == map["landmarks"].size() && "Number of landmarks in map file does not match!");

    for (unsigned int i = 0; i < nlandmarks; i++)
    {
        YAML::Node lmk_node = map["landmarks"][i];
        LandmarkBase* lmk_ptr = LandmarkFactory::get().create(lmk_node["type"].as<std::string>(), lmk_node);
        addLandmark(lmk_ptr);
    }

}

void MapBase::save(const std::string& _map_file_yaml, const std::string& _map_name)
{
    YAML::Emitter emitter;

    emitter << YAML::BeginMap;
    emitter << "map name"   << _map_name;
    emitter << "date-time" << dateTimeNow(); // Get date and time for archiving purposes

    emitter << "nlandmarks" << getLandmarkListPtr()->size();

    emitter << "landmarks"  << YAML::BeginSeq;

    for (LandmarkBase* lmk_ptr : *getLandmarkListPtr())
    {
        emitter << YAML::Flow << lmk_ptr->saveToYaml();
    }
    emitter << YAML::EndSeq << YAML::EndMap;


    std::ofstream fout(_map_file_yaml);
    fout << emitter.c_str();
    fout.close();
}

std::string MapBase::dateTimeNow()
{
    // Get date and time for archiving purposes
    std::time_t rawtime;
    std::time(&rawtime);
    const std::tm* timeinfo = std::localtime(&rawtime);
    char time_char[30];
    std::strftime(time_char, sizeof(time_char), "%d/%m/%Y at %H:%M:%S", timeinfo);
    std::string date_time(time_char);
    return date_time;
}

} // namespace wolf
