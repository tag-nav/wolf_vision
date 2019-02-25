#ifndef LOADER_HPP
#define LOADER_HPP
#include <dlfcn.h>
class Loader{
protected:
    std::string path_;
public:
    Loader(std::string _file){
        this->path_ = _file;
    }
    virtual void load() = 0;
    virtual void close() = 0;
};
class LoaderRaw: public Loader{
    void* resource_;
public:
    LoaderRaw(std::string _file):
        Loader(_file)
    {
        //
    }
    ~LoaderRaw(){
        this->close();
    }
    void load(){
        this->resource_ = dlopen(this->path_.c_str(), RTLD_LAZY);
        if(not this->resource_)
            throw std::runtime_error("Couldn't load resource with path " + this->path_);
    }
    void close(){
        if(this->resource_) dlclose(this->resource_);
    }
};
// class LoaderPlugin: public Loader{
//     ClassLoader* resource_;
//     void load(){
//         this->resource_ = new ClassLoader(this->path_);
//     }
//     void close(){
//         delete this->resource_;
//     }
// };
#endif