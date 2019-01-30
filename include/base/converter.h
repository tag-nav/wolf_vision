#ifndef CONVERTER_H
#define CONVERTER_H
#include <vector>
// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <regex>
#include <iostream>

namespace utils{
    template <typename A>
    using list = std::vector<A>;
    // template <typename A>
    // class toString<A>{};
    static inline std::vector<std::string> splitter(std::string val){
        std::vector<std::string> cont = std::vector<std::string>();
        std::stringstream ss(val);
        std::string token;
        while (std::getline(ss, token, ',')) {
            cont.push_back(token);
        }
        return cont;
    }
    static inline std::vector<std::string> getMatches(std::string val, std::regex exp){
        std::smatch res;
        auto v = std::vector<std::string>();
        std::string::const_iterator searchStart( val.cbegin() );
        while ( std::regex_search( searchStart, val.cend(), res, exp ) ) {
            v.push_back(res[0]);
            searchStart = res.suffix().first;
        }
        return v;
    }
    static inline std::vector<std::string> pairSplitter(std::string val){
        std::regex exp("(\\{[^\\{:]:.*?\\})");
        // std::smatch res;
        // string val = "[{x:1},{y:[1,23,3]},{z:3}]";
        // auto v = std::vector<std::string>();
        // std::string::const_iterator searchStart( val.cbegin() );
        // while ( regex_search( searchStart, val.cend(), res, exp ) ) {
        //     v.push_back(res[0]);
        //     searchStart = res.suffix().first;
        // }
        // return v;
        return getMatches(val, exp);
    }
    static inline std::array<std::string,2> splitMatrixStringRepresentation(std::string matrix){
        std::regex rgx("\\[\\[((?:[0-9]+,?)+)\\],((?:(?:[0-9]+\\.)?[0-9]+,?)+)\\]");
        std::regex rgxStatic("\\[((?:(?:[0-9]+\\.)?[0-9]+,?)+)\\]");
        std::smatch matches;
        std::smatch matchesStatic;
        std::array<std::string,2> values = {{"[]","[]"}};
        if(std::regex_match(matrix, matches, rgx)) {
            values[0] = "[" + matches[1].str() + "]";
            values[1] = "[" + matches[2].str() + "]";
        }else if(std::regex_match(matrix, matchesStatic, rgxStatic)){
            values[1] = "[" + matchesStatic[1].str() + "]";
        }else{
            throw std::runtime_error("Invalid string representation of a Matrix. Correct format is [([num,num],)?(num(,num)*)?]. String provided: " + matrix);
        }
        return values;
    }
    static inline std::string splitMapStringRepresentation(std::string str_map){
        std::smatch mmatches;
        std::regex rgxM("\\[((?:(?:\\{[^\\{:]+:[^:\\}]+\\}),?)*)\\]");
        // auto v = std::vector<std::string>();
        std::string result = "";
        if(std::regex_match(str_map, mmatches, rgxM)) {
            // v = splitter(mmatches[1].str());
            result = mmatches[1].str();
        } else {
            throw std::runtime_error("Invalid string representation of a Map. Correct format is [({id:value})?(,{id:value})*]. String provided: " + str_map);
        }
        return result;
    }
}
namespace wolf{

template<typename T>
struct converter{
    static T convert(std::string val){
        assert(1 == 0 && "There is no general convert for arbitrary T !!!");
    }
};
template<typename A>
struct converter<utils::list<A>>{
    static utils::list<A> convert(std::string val){
        std::regex rgxP("\\[([^,]+)?(,[^,]+)*\\]");
        utils::list<A> result = utils::list<A>();
        if(std::regex_match(val, rgxP)) {
            std::string aux = val.substr(1,val.size()-2);
            auto l = utils::getMatches(aux, std::regex("([^,]+),?"));
            for(auto it : l){
                result.push_back(converter<A>::convert(it));
            }
        } else throw std::runtime_error("Invalid string format representing a list-like structure. Correct format is [(value)?(,value)*]. String provided: " + val);
        return result;
    }
    static std::string convert(utils::list<A> val){
        std::string aux = "";
        bool first = true;
        for(auto it : val){
            if(not first) aux += "," + it;
            else{
                first = false;
                aux = it;
            }
        }
        return "[" + aux + "]";
    }
};
template<>
struct converter<int>{
    static int convert(std::string val){
        return stod(val);
    }
};
template<>
struct converter<double>{
    static double convert(std::string val){
        return stod(val);
    }
};
template<>
struct converter<bool>{
    static bool convert(std::string val){
        if(val == "true") return true;
        else if (val == "false") return false;
        else throw std::runtime_error("Invalid conversion to bool (Must be either \"true\" or \"false\"). String provided: " + val);
    }
};
template<>
struct converter<std::string>{
    static std::string convert(std::string val){
        return val;
    }
    template<typename T>
    static std::string convert(T val){
        assert(1 == 0 && "There is no general convert to string for arbitrary T !!!");
        return "";
    }
    static std::string convert(int val){
        return std::to_string(val);
    }
    static std::string convert(double val){
        return std::to_string(val);
    }
    // template<typename A>
    // static std::string convert(std::vector<A> val){
    //     std::string result = "";
    //     bool first = true;
    //     for(auto it : val){
    //         if(not first) result += "," + converter<std::string>::convert(it);
    //         else{
    //             first = false;
    //             result = converter<std::string>::convert(it);
    //         }
    //     }
    //     return "[" + result + "]";
    // }
    template<typename A>
    static std::string convert(utils::list<A> val){
        std::string result = "";
        for(auto it : val){
            result += "," + converter<std::string>::convert(it);
        }
        if(result.size() > 0) result = result.substr(1,result.size());
        return "[" + result + "]";
    }
    template<typename A, typename B>
    static std::string convert(std::pair<A,B> val){
        return "{" + converter<std::string>::convert(val.first) + ":" + converter<std::string>::convert(val.second) + "}";
    }
    template<typename A, typename B>
    static std::string convert(std::map<A,B> val){
        std::string result = "";
        for(auto it : val){
            result += "," + converter<std::string>::convert(it);
        }
        if(result.size() > 0) result = result.substr(1,result.size());
        return "[" + result + "]";
    }
    template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    static std::string convert(Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> val){
        std::string result = "";
        for(int i = 0; i < val.rows() ; ++i){
            for(int j = 0; j < val.cols(); ++j){
                result += "," + converter<std::string>::convert(val(i,j));
            }
        }
        if(result.size() > 0) result = result.substr(1,result.size());
        if(_Rows == Eigen::Dynamic and _Cols == Eigen::Dynamic){
            result = "[" + std::to_string(val.rows()) + "," + std::to_string(val.cols()) + "]," + result;
        }
        return "[" + result + "]";
    }
};
template<typename A, typename B>
struct converter<std::pair<A,B>>{
    static std::pair<A,B> convert(std::string val){
        std::regex rgxP("\\{([^\\{:]+):([^\\}]+)\\}");
        std::smatch matches;
        if(std::regex_match(val, matches, rgxP)) {
            return std::pair<A,B>(converter<A>::convert(matches[1].str()), converter<B>::convert(matches[2].str()));
        } else throw std::runtime_error("Invalid string format representing a pair. Correct format is {identifier:value}. String provided: " + val);
    }
    static std::vector<A> convert(std::vector<std::string> val){
        std::vector<A> rtn = std::vector<A>();
        for(auto it : val)
            rtn.push_back(converter<A>::convert(it));
        return rtn;
    }
};
// template<typename A>
// struct converter<std::vector<A>>{
//     static std::vector<A> convert(std::string val){
//         auto aux = utils::
//             std::vector<A> rtn = std::vector<A>();
//         for(auto it : aux)
//             rtn.push_back(converter<A>::convert(it));
//         return rtn;
//     }
//     static std::vector<A> convert(std::vector<std::string> val){
//         std::vector<A> rtn = std::vector<A>();
//         for(auto it : val)
//             rtn.push_back(converter<A>::convert(it));
//         return rtn;
//     }
// };

// TODO: WARNING!! For some reason when trying to specialize converter to std::array
// it defaults to the generic T type, thus causing an error!

template<typename A, int _size>
struct converter<std::array<A, _size>>{
    static std::array<A, _size> convert(std::string val){
        // std::vector<std::string> aux = utils::splitter(val);
        auto aux = converter<utils::list<A>>::convert(val);
        std::array<A,_size> rtn = std::array<A, _size>();
        if(_size != aux.size()) throw std::runtime_error("Error in trying to transform literal string to Array. Invalid argument size. Required size "
                                                         + std::to_string(_size) + "; provided size " + std::to_string(aux.size()));
        for (int i = 0; i < _size; ++i) {
            rtn[i] = aux[i];
        }
        return rtn;
    }
    // static std::vector<A> convert(std::vector<std::string> val){
    //     std::vector<A> rtn = std::vector<A>();
    //     for(auto it : val)
    //         rtn.push_back(converter<A>::convert(it));
    //     return rtn;
    // }
};
template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct converter<Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>>{
    typedef Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>  Matrix;
    static Matrix convert(std::string val){
        auto splittedValues = utils::splitMatrixStringRepresentation(val);
        auto dimensions = converter<std::vector<int>>::convert(splittedValues[0]);
        auto values = converter<std::vector<_Scalar>>::convert(splittedValues[1]);
        Matrix m = Matrix();
        if(_Rows == Eigen::Dynamic and _Cols == Eigen::Dynamic) {
            if(dimensions.size() != 2) throw std::runtime_error("Invalid string representing Eigen Matrix. Missing dimensions. String provided: " + val);
            m.resize(dimensions[0],dimensions[1]);
        }else if(_Rows == Eigen::Dynamic){
            int nrows = (int) values.size() / _Cols;
            m.resize(nrows, _Cols);
        }else if(_Cols == Eigen::Dynamic){
            int ncols = (int) values.size() / _Rows;
            m.resize(_Rows, ncols);
        }
        if(m.rows()*m.cols() != values.size()) throw std::runtime_error("The literal string provides " + std::to_string(values.size()) + " values but "
                                                                        + "the Eigen matrix is of dimensions "
                                                                        + std::to_string(m.rows()) + "x" + std::to_string(m.cols()));
        else{
            for (int i = 0; i < m.rows(); i++)
                for (int j = 0; j < m.cols(); j++)
                    m(i, j) = values[(int)(i * m.cols() + j)];
        }
        return m;
    }
};
template<typename A>
struct converter<std::map<std::string,A>>{
    static std::map<std::string,A> convert(std::string val){
        auto str_map = utils::splitMapStringRepresentation(val);
        auto v = utils::pairSplitter(str_map);
        auto map = std::map<std::string, A>();
        for(auto it : v){
            auto p = converter<std::pair<std::string, A>>::convert(it);
            map.insert(std::pair<std::string, A>(p.first, p.second));
        }
        return map;
    }
};
    // template<typename A>
    // struct converter<utils::toString<A>>{
    //     static std::string convert(T val){
    //         return std::to_string(val);
    //     }
    // }
}
#endif