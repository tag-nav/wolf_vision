#ifndef PARSER_YAML_HPP
#define PARSER_YAML_HPP
#include "yaml-cpp/yaml.h"
#include <vector>
#include <regex>
#include <map>
#include <iostream>
#include <algorithm>

using namespace std;
namespace {
    // std::string remove_ws( const std::string& str ){
    //     std::string str_no_ws ;
    //     for( char c : str ) if( !std::isspace(c) ) str_no_ws += c ;
    //     return str_no_ws ;
    // }
    string parseSequence(YAML::Node n){
        assert(n.Type() != YAML::NodeType::Map && "Trying to parse as a Sequence a Map node");
        if(n.Type() == YAML::NodeType::Scalar) return n.Scalar();
        string aux = "[";
        bool first = true;
        for(auto it : n){
            if(first) {
                aux = aux + parseSequence(it);
                first = false;
            }else{
                aux = aux + "," + parseSequence(it);
            }
        }
        aux = aux + "]";
        return aux;
    }
    std::string mapToString(std::map<std::string,std::string> _map){
        std::string result = "";
        auto v = std::vector<string>();
        std::transform(_map.begin(), _map.end(), back_inserter(v), [](const std::pair<string,string> p){return "{" + p.first + ":" + p.second + "}";});
        auto concat = [](string ac, string str)-> string {
                          return ac + str + ",";
                      };
        string aux = "";
        string accumulated = std::accumulate(v.begin(), v.end(), aux, concat);
        if(accumulated.size() > 1) accumulated = accumulated.substr(0,accumulated.size() - 1);
        else accumulated = "";
        return "[" + accumulated + "]";
    }
}
class parserYAML {
    struct ParamsInitSensor{
        string _type;
        string _name;
        YAML::Node n;
    };
    struct ParamsInitProcessor{
        string _type;
        string _name;
        string _name_assoc_sensor;
        YAML::Node n;
    };
    map<string, string> _params;
    string _active_name;
    vector<ParamsInitSensor> _paramsSens;
    vector<ParamsInitProcessor> _paramsProc;
    vector<string> _files;
    string _file;
public:
    parserYAML(){
        _params = map<string, string>();
        _active_name = "";
        _paramsSens = vector<ParamsInitSensor>();
        _paramsProc = vector<ParamsInitProcessor>();
        _file = "";
        _files = vector<string>();
    }
    parserYAML(string file){
        _params = map<string, string>();
        _active_name = "";
        _paramsSens = vector<ParamsInitSensor>();
        _paramsProc = vector<ParamsInitProcessor>();
        _files = vector<string>();
        _file = file;
    }
    ~parserYAML(){
        //
    }
    void walkTree(string file);
    void walkTree(string file, vector<string>& tags);
    void walkTree(string file, vector<string>& tags, string hdr);
    void walkTreeR(YAML::Node n, vector<string>& tags, string hdr);
    void updateActiveName(string tag);
    void parseFirstLevel(string file);
    string tagsToString(vector<string>& tags);
    vector<array<string, 2>> sensorsSerialization();
    vector<array<string, 3>> processorsSerialization();
    vector<string> getFiles();
    map<string,string> getParams();
    void parse();
    map<string, string> fetchAsMap(YAML::Node);
};
string parserYAML::tagsToString(vector<std::string> &tags){
    string hdr = "";
    for(auto it : tags){
        hdr = hdr + "/" + it;
    }
    return hdr;
}
void parserYAML::walkTree(string file){
    YAML::Node n = YAML::LoadFile(file);
    vector<string> hdrs = vector<string>();
    walkTreeR(n, hdrs, "");
}
void parserYAML::walkTree(string file, vector<string>& tags){
    YAML::Node n = YAML::LoadFile(file);
    walkTreeR(n, tags, "");
}
void parserYAML::walkTree(string file, vector<string>& tags, string hdr){
    YAML::Node n = YAML::LoadFile(file);
    walkTreeR(n, tags, hdr);
}
void parserYAML::walkTreeR(YAML::Node n, vector<string>& tags, string hdr){
    switch (n.Type()) {
    case YAML::NodeType::Scalar : {
        regex r("^@.*");
        if(regex_match(n.Scalar(), r)){
            string str = n.Scalar();
            // cout << "SUBSTR " << str.substr(1,str.size() - 1);
            walkTree(str.substr(1,str.size() - 1), tags, hdr);
        }else{
            // std::copy(tags.begin(), tags.end(), std::ostream_iterator<string>(std::cout, "¬"));
            // cout << "«»" << n.Scalar() << endl;
            _params.insert(pair<string,string>(hdr, n.Scalar()));
        }
        break;
    }
    case YAML::NodeType::Sequence : {
        // cout << tags[tags.size() - 1] << "«»" << kv << endl;
        // std::vector<double> vi = n.as<std::vector<double>>();
        string aux = parseSequence(n);
        _params.insert(pair<string,string>(hdr, aux));
        break;
    }
    case YAML::NodeType::Map : {
        for(const auto& kv : n){
            //If the key's value starts with a $ (i.e. $key) then its value is parsed as a literal map,
            //otherwise the parser recursively parses the map
            regex r("^\\$.*");
            if(not regex_match(kv.first.as<string>(), r)){
                regex rr("follow");
                if(not regex_match(kv.first.as<string>(), rr)) {
                    tags.push_back(kv.first.as<string>());
                    if(tags.size() == 2) this->updateActiveName(kv.first.as<string>());
                    walkTreeR(kv.second, tags, hdr +"/"+ kv.first.as<string>());
                    tags.pop_back();
                    if(tags.size() == 1) this->updateActiveName("");
                }else{
                    walkTree(kv.second.as<string>(), tags, hdr);
                }
            }else{
                string key = kv.first.as<string>();
                key = key.substr(1,key.size() - 1);
                auto fm = fetchAsMap(kv.second);
                _params.insert(pair<string,string>(hdr + "/" + key, mapToString(fm)));
            }
        }
        break;
    }
    default:
        assert(1 == 0 && "Unsupported node Type at walkTreeR");
        break;
    }
}
void parserYAML::updateActiveName(string tag){
    this->_active_name = tag;
}
void parserYAML::parseFirstLevel(string file){
    YAML::Node n = YAML::LoadFile(file);
    YAML::Node n_config = n["config"];
    assert(n_config.Type() == YAML::NodeType::Map && "trying to parse config node but found a non-Map node");
    for(const auto& kv : n_config["sensors"]){
        ParamsInitSensor pSensor = {kv["type"].Scalar(), kv["name"].Scalar(), kv};
        _paramsSens.push_back(pSensor);
    }
    for(const auto& kv : n_config["processors"]){
        ParamsInitProcessor pProc = {kv["type"].Scalar(), kv["name"].Scalar(), kv["sensorname"].Scalar(), kv};
        _paramsProc.push_back(pProc);
    }
    YAML::Node n_files = n["files"];
    assert(n_files.Type() == YAML::NodeType::Sequence && "trying to parse files node but found a non-Sequence node");
    for(const auto& kv : n_files){
        _files.push_back(kv.Scalar());
    }
}
vector<array<string, 2>> parserYAML::sensorsSerialization(){
    vector<array<string, 2>> aux = vector<array<string, 2>>();
    for(auto it : this->_paramsSens)
        aux.push_back({{it._type,it._name}});
    return aux;
}
vector<array<string, 3>> parserYAML::processorsSerialization(){
    vector<array<string, 3>> aux = vector<array<string, 3>>();
    for(auto it : this->_paramsProc)
        aux.push_back({{it._type,it._name,it._name_assoc_sensor}});
    return aux;
}
vector<string> parserYAML::getFiles(){
    return this->_files;
}
map<string,string> parserYAML::getParams(){
    map<string,string> rtn = _params;
    return rtn;
}
void parserYAML::parse(){
    this->parseFirstLevel(this->_file);
    for(auto it : _paramsSens){
        vector<string> tags = vector<string>();
        // this->walkTreeR(it.n , tags , it._type + "/" + it._name);
        this->walkTreeR(it.n , tags , it._name);
    }
    for(auto it : _paramsProc){
        vector<string> tags = vector<string>();
        // this->walkTreeR(it.n , tags , it._type + "/" + it._name);
        this->walkTreeR(it.n , tags , it._name);
    }
}
map<string, string> parserYAML::fetchAsMap(YAML::Node n){
    assert(n.Type() == YAML::NodeType::Map && "trying to fetch as Map a non-Map node");
    auto m = map<string, string>();
    for(const auto& kv : n){
        string key = kv.first.as<string>();
        switch (kv.second.Type()) {
        case YAML::NodeType::Scalar : {
            string value = kv.second.Scalar();
            m.insert(pair<string,string>(key, value));
            break;
        }
        case YAML::NodeType::Sequence : {
            // cout << tags[tags.size() - 1] << "«»" << kv << endl;
            // std::vector<double> vi = n.as<std::vector<double>>();
            string aux = parseSequence(kv.second);
            m.insert(pair<string,string>(key, aux));
            break;
        }
        case YAML::NodeType::Map : {
            m = fetchAsMap(kv.second);
            auto rtn = vector<string>();
            std::transform(m.begin(), m.end(), back_inserter(rtn), [](const std::pair<string,string> v){return "{" + v.first + ":" + v.second + "}";});
            auto concat = [](string ac, string str)-> string {
                              return ac + str + ",";
                          };
            string aux = "";
            string accumulated = std::accumulate(rtn.begin(), rtn.end(), aux, concat);
            if(accumulated.size() > 1) accumulated = accumulated.substr(0,accumulated.size() - 1);
            else accumulated = "";
            m.insert(pair<string,string>(key, "[" + accumulated + "]"));
            break;
        }
        default:
            assert(1 == 0 && "Unsupported node Type at fetchAsMap");
            break;
        }
    }
    return m;
}
#endif