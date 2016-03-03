// Wolf includes
#include "processor_image_point_brisk.h"

// OpenCV includes

#include "unistd.h"

ProcessorORB::ProcessorORB()
{
    std::cout << "ProcessorORB constructor" << std::endl;
}

ProcessorORB::~ProcessorORB()
{
    std::cout << "ProcessorORB destructor" << std::endl;
}

void ProcessorORB::extractFeatures(CaptureBase *_capture_ptr)
{

    //uses a capture
    //call to image extractor, extract features in the image
}

void ProcessorORB::establishConstraints(CaptureBase *_capture_ptr)
{
    //not used yet
}
