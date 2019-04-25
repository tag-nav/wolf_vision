#ifndef PARAMS_SERVER_HPP
#define PARAMS_SERVER_HPP
#include <vector>
#include <regex>
#include <map>
#include "base/utils/converter.h"
namespace wolf{
class paramsServer{
    struct ParamsInitSensor{
        std::string _type;
        std::string _name;
    };
    struct ParamsInitProcessor{
        std::string _type;
        std::string _name;
        std::string _name_assoc_sensor;
    };
    std::map<std::string, std::string> _params;
    std::map<std::string,ParamsInitSensor> _paramsSens;
    std::map<std::string,ParamsInitProcessor> _paramsProc;
public:
    paramsServer(){
        _params = std::map<std::string, std::string>();
        _paramsSens = std::map<std::string,ParamsInitSensor>();
        _paramsProc = std::map<std::string,ParamsInitProcessor>();
    }
    paramsServer(std::map<std::string, std::string> params,
                 std::vector<std::array<std::string,2>> sensors,
                 std::vector<std::array<std::string,3>> procs){
        _params = params;
        _paramsSens = std::map<std::string,ParamsInitSensor>();
        _paramsProc = std::map<std::string,ParamsInitProcessor>();
        for(auto it : sensors) {
            ParamsInitSensor pSensor = {it.at(0), it.at(1)};
            _paramsSens.insert(std::pair<std::string,ParamsInitSensor>(it.at(1), pSensor));
        }
        for(auto it : procs) {
            ParamsInitProcessor pProcs = {it.at(0), it.at(1), it.at(2)};
            _paramsProc.insert(std::pair<std::string,ParamsInitProcessor>(it.at(1), pProcs));
        }
    }
    ~paramsServer(){
        //
    }
    void print(){
        for(auto it : _params)
            std::cout << it.first << "~~" << it.second << std::endl;
    }
    void addInitParamsSensor(std::string type, std::string name){
        ParamsInitSensor params = {type, name};
        _paramsSens.insert(std::pair<std::string, ParamsInitSensor>(type + "/" + name + "/", params));
    }
    void addInitParamsProcessor(std::string type, std::string name, std::string name_assoc_sensor){
        ParamsInitProcessor params = {type, name, name_assoc_sensor};
        _paramsProc.insert(std::pair<std::string, ParamsInitProcessor>(type + "/" + name + "/", params));
    }
    void addParam(std::string key, std::string value){
        _params.insert(std::pair<std::string, std::string>(key, value));
    }
    template<typename T>
    T getParam(std::string key, std::string def_value) const {
        if(_params.find(key) != _params.end()){
            return converter<T>::convert(_params.find(key)->second);
        }else{
            return converter<T>::convert(def_value);
        }
    }
    template<typename T>
    T getParam(std::string key) const {
        if(_params.find(key) != _params.end()){
            return converter<T>::convert(_params.find(key)->second);
        }else{
            throw std::runtime_error("The following key: '" + key + "' has not been found in the parameters server and no default value was provided.");
        }
    }
    std::vector<ParamsInitSensor> getSensors(){
        std::vector<ParamsInitSensor> rtn = std::vector<ParamsInitSensor>();
        std::transform(this->_paramsSens.begin(), this->_paramsSens.end(), back_inserter(rtn), [](const std::pair<std::string,ParamsInitSensor> v){return v.second;});
        return rtn;
    }
    std::vector<ParamsInitProcessor> getProcessors(){
        std::vector<ParamsInitProcessor> rtn = std::vector<ParamsInitProcessor>();
        std::transform(this->_paramsProc.begin(), this->_paramsProc.end(), back_inserter(rtn), [](const std::pair<std::string,ParamsInitProcessor> v){return v.second;});
        return rtn;
    }
};
}
#endif