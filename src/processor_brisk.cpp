// Wolf includes
#include "processor_brisk.h"

// OpenCV includes

// other includes
#include <bitset>

/** Public */

/** The "main" class of this processor is "process" */

//Constructor
ProcessorBrisk::ProcessorBrisk(unsigned int _image_rows, unsigned int _image_cols, int _threshold, int _octaves, float _pattern_scales,
                               unsigned int _grid_width, unsigned int _grid_height, unsigned int _min_features_th) :
    ProcessorTracker(PRC_TRACKER_BRISK, false),
    brisk_(_threshold, _octaves, _pattern_scales),
    act_search_grid_(_image_rows,_image_cols,_grid_width, _grid_height),
    min_features_th_(_min_features_th)
{
    brisk_.create("Feature2D.BRISK");
}

//Destructor
ProcessorBrisk::~ProcessorBrisk()
{

}


// Tracker function. Returns the number of successful tracks.
unsigned int ProcessorBrisk::processKnownFeatures()
{

    // Cambiar esta funcion y añadir otra que se la llame por medio de dos listas, una de entrada y otra de salida. Para el caso normal, pasar
    // "getLastPtr()->getFeatureListPtr()" y otra lista con los resultados del matching. Luego, después de encontrar new features, se volverá
    // a llamar a processKnownFeatures con las nuevas features en una lista, para que se haga el matching con el incoming, que también ha de
    // devolver una lista con los resultados del matching. Al acabar, hacer un splice de la captura last con las nuevas features de last Y de
    // incoming con las utilizadas en el matching, para que cuando se haga el reset, el last pase entero (con las new features) a origin y
    // incoming pase a ser last (con las features del matching)


    std::cout << std::endl << "<---- processKnownFeatures ---->" << std::endl << std::endl;

    act_search_grid_.renew();

    unsigned int n_tracks = 0;
    std::cout << "Nbr of features in incoming: " << getIncomingPtr()->getFeatureListPtr()->size() << std::endl;
    n_tracks = track(*(getLastPtr()->getFeatureListPtr()), *(getIncomingPtr()->getFeatureListPtr()));
    std::cout << "Nbr of features in incoming: " << getIncomingPtr()->getFeatureListPtr()->size() << std::endl;

    return n_tracks;
}

void ProcessorBrisk::drawFeaturesLastFrame(cv::Mat _image, Eigen::Vector2i _feature_point_last)
{
    cv::Point point;
    point.x = _feature_point_last[0];
    point.y = _feature_point_last[1];
    cv::circle(_image,point,2,cv::Scalar(250.0,180.0,70.0),-1,8,0);
    cv::imshow("Keypoint drawing",_image);
}

void ProcessorBrisk::drawRoiLastFrame(cv::Mat _image,cv::Rect _roi)
{
    cv::rectangle(_image,_roi,cv::Scalar(88.0,70.0,254.0),1,8,0);
    cv::imshow("Keypoint drawing",_image);
}


void ProcessorBrisk::drawFeatures(CaptureBase* const _last_ptr)
{
    cv::KeyPoint _kp;
    cv::Mat image = image_last_;

    //TODO: Why is it 0?
    std::cout << "size: " << getLastPtr()->getFeatureListPtr()->size() << std::endl;
    for (std::list<FeatureBase*>::iterator feature_iter=getLastPtr()->getFeatureListPtr()->begin();feature_iter != getLastPtr()->getFeatureListPtr()->end(); ++feature_iter)
    {

        _kp = ((FeaturePointImage*)*feature_iter)->getKeypoint();
        cv::Point point;
        point.x = _kp.pt.x;
        point.y = _kp.pt.y;
        cv::circle(image,point,2,cv::Scalar(88.0,250.0,154.0),-1,8,0);
        std::cout << "keypoint: " << _kp.pt << std::endl;
    }
    cv::imshow("Keypoint drawing",image);

}


unsigned int ProcessorBrisk::briskDetect(cv::Mat _image, cv::Rect &_roi, std::vector<cv::KeyPoint> &_new_keypoints, cv::Mat & new_descriptors)
{
    std::cout << std::endl << "<---- briskImplementation ---->" << std::endl << std::endl;

    cv::Mat _image_roi = _image(_roi);

    //Brisk Algorithm
    //brisk_.create("Feature2D.BRISK");  //TODO: look if this can be done in the constructor
    brisk_.detect(_image_roi, _new_keypoints);
    brisk_.compute(_image_roi, _new_keypoints,new_descriptors);

    //double dist_ham = cv::norm(new_descriptors,new_descriptors,'NORM_HAMMING');

    if(_new_keypoints.size()!=0)
    {
            for(unsigned int i = 0; i <= (_new_keypoints.size()-1);i++)
            {
                _new_keypoints[i].pt.x = _new_keypoints[i].pt.x + _roi.x;
                _new_keypoints[i].pt.y = _new_keypoints[i].pt.y + _roi.y;
            }

            return new_descriptors.rows;  //number of features
    }
    else
    {
        return 0;
    }
}

void ProcessorBrisk::addNewFeaturesInCapture(std::vector<cv::KeyPoint> _new_keypoints, cv::Mat new_descriptors)
{
    for(unsigned int i = 0; i <= (_new_keypoints.size()-1);i++)
    {
        FeaturePointImage* point_ptr = new FeaturePointImage(_new_keypoints[i],(new_descriptors(cv::Range(i,i+1),cv::Range(0,new_descriptors.cols))),false);
        addNewFeature(point_ptr);
    }
    std::cout << "size of new list: " << getNewFeaturesList().size() << std::endl;
}


// This is intended to create Features that are not among the Features already known in the Map. Returns the number of detected Features.
unsigned int ProcessorBrisk::detectNewFeatures()
{
    std::cout << std::endl << "<---- detectNewFeatures ---->" << std::endl << std::endl;

    cv::Rect roi;

    ///START OF THE SEARCH
    unsigned int n_features = 0;
    unsigned int n_iterations = 0;
    while((n_features < 1) && (n_iterations <5))
    {
        bool roi_exists = act_search_grid_.pickRoi(roi);

        if(roi_exists==false)
        {
            break;
        }

        std::vector<cv::KeyPoint> new_keypoints;
        cv::Mat new_descriptors;

        n_features = briskDetect(image_last_, roi, new_keypoints, new_descriptors);

        if (n_features == 0)
        {
            act_search_grid_.blockCell(roi);
        }
        else
        {
            addNewFeaturesInCapture(new_keypoints,new_descriptors);
        }


        std::cout << "NFL size: " << getNewFeaturesList().size() << std::endl;

        /** To read the values in the new list */

        /**
        for (FeatureBase* feature_ptr : getNewFeaturesList())
        {
            std::cout << "newFeaturesList Feature, keypoint: " << ((FeaturePointImage*)feature_ptr)->getKeypoint().pt << std::endl;
            std::cout << "newFeaturesList Feature, descriptor: [";
            for (float desc_ptr : ((FeaturePointImage*)feature_ptr)->getDescriptor())
            {
                std::cout << ", " << desc_ptr;
            }
            std::cout << "]" << std::endl;
        }
        */

        n_iterations++;
    }
    std::cout << "n_features: " << n_features << std::endl;

    return n_features;
}


//Vote for KeyFrame generation. If a KeyFrame criterion is validated, this function returns true
bool ProcessorBrisk::voteForKeyFrame()
{
    std::cout << "Thereshold: " << min_features_th_ << std::endl;
    std::cout << "Feature_list size: " << ((CaptureImage*)getIncomingPtr())->getFeatureListPtr()->size() << std::endl;
    std::cout << "<--------------------------------> voteForKeyFrame?: " << (((CaptureImage*)getIncomingPtr())->getFeatureListPtr()->size() < min_features_th_) << std::endl;
    return (getIncomingPtr()->getFeatureListPtr()->size() < min_features_th_);
    //return 0;
}

void ProcessorBrisk::process(CaptureBase* const _incoming_ptr)
{
    std::cout << std::endl << "<---- process ---->" << std::endl << std::endl;
    image_incoming_ = ((CaptureImage*)_incoming_ptr)->getImage();
    image_last_ = ((CaptureImage*)getLastPtr())->getImage();
    ProcessorTracker::process(_incoming_ptr);
    std::cout << std::endl << "<---- end process ---->" << std::endl << std::endl;
    drawFeatures(getLastPtr());
    cv::waitKey(30);
}


/** Protected */

//Initialize one landmark
LandmarkBase* ProcessorBrisk::createLandmark(FeatureBase* _feature_ptr)
{
    return new LandmarkBase(LANDMARK_POINT,new StateBlock(Eigen::Vector3s::Zero()),new StateBlock(Eigen::Vector3s::Zero()));
}

//Create a new constraint
ConstraintBase* ProcessorBrisk::createConstraint(FeatureBase* _feature_ptr, NodeBase* _feat_or_lmk_ptr)
{
    return nullptr;
}


/** Private */

unsigned int ProcessorBrisk::track(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out)
{
    std::cout << std::endl << "<---- processFeaturesForMatching ---->" << std::endl << std::endl;

    ///PROJECTION OF LANDMARKS (features in this case)

    unsigned int feature_roi_width = 30;
    unsigned int feature_roi_heigth = 30;

    unsigned int feature_roi_x = 0;
    unsigned int feature_roi_y = 0;

    unsigned int n_features = 0;
    //int tolerance = 20;

    std::vector<cv::KeyPoint> new_keypoints;
    cv::Mat new_descriptors;

    std::cout << "feature_list_in (FFM): " << _feature_list_in.size() << std::endl;
    unsigned int n_last_capture_feat = 0;
    for(auto feature_base_ptr : _feature_list_in)
    {
        FeaturePointImage* feature_ptr = (FeaturePointImage*)feature_base_ptr;
        Eigen::Vector2i feature_point = {feature_ptr->getKeypoint().pt.x,feature_ptr->getKeypoint().pt.y}; //TODO: Look if this is the right order
        act_search_grid_.hitCell(feature_point);
        std::cout << std::endl << "Last feature keypoint: " << feature_ptr->getKeypoint().pt << std::endl;

        feature_roi_x = (feature_ptr->getKeypoint().pt.x)-(feature_roi_heigth/2);
        feature_roi_y = (feature_ptr->getKeypoint().pt.y)-(feature_roi_width/2);




//        Eigen::Vector2i feature_roi_point = {feature_roi_x,feature_roi_y};
        cv::Rect roi_for_matching(feature_roi_x,feature_roi_y,feature_roi_width,feature_roi_heigth);

        drawFeaturesLastFrame(image_incoming_,feature_point);
//        //drawFeaturesLastFrame(_image,feature_roi_point);
        drawRoiLastFrame(image_incoming_,roi_for_matching);


        n_features = briskDetect(image_incoming_, roi_for_matching, new_keypoints, new_descriptors);
        std::cout << "n_features: " << n_features << std::endl;

        //float euclidean_distance = 0;
        //float euclidean_distance_min_value = 1000;
        double f_min = 100;
        unsigned int row = 0;

        // Comparison of position

        if(n_features != 0)
        {
            /* //POSIBLE PROBLEMA: Brisk deja una distancia a la hora de detectar. Si es muy pequeño el roi puede que no detecte nada
            for(unsigned int i = 0; i <= (new_keypoints.size()-1);i++)
            {

                        euclidean_distance = sqrt(pow((feature_ptr->getKeypoint().pt.x-new_keypoints[i].pt.x),2)+
                                                  pow((feature_ptr->getKeypoint().pt.y-new_keypoints[i].pt.y),2));
                        std::cout << "euclidean distance [" << i << "]: " << euclidean_distance << std::endl;
                        if(euclidean_distance < euclidean_distance_min_value)
                        {
                            euclidean_distance_min_value = euclidean_distance;
                            row = i;
                        }
                        std::cout << "euclidean distance min_value: " << euclidean_distance_min_value << std::endl;
                //new FeaturePointImage(_new_keypoints[i],(new_descriptors(cv::Range(i,i+1),cv::Range(0,new_descriptors.cols))),false);
            } */


            std::vector<float> feature_descriptor = feature_ptr->getDescriptor();

            //POSIBLE PROBLEMA: Brisk deja una distancia a la hora de detectar. Si es muy pequeño el roi puede que no detecte nada
            for(unsigned int i = 0; i <= (new_keypoints.size()-1);i++)
            {

                // Comparison of descriptors

                const unsigned char* mat_row = new_descriptors.ptr<unsigned char>(i);

                std::cout << "size: " << (feature_ptr->getDescriptor().size()) << std::endl;



                const unsigned char* feat_desc[64];
                unsigned char gh[64];

                for(unsigned int j = 0; j<= feature_descriptor.size()-1;j++)
                {
                    float n = feature_descriptor[j];
                    char buf[8];
                    sprintf(buf,"%d", (int)n);
                    std::cout << "====buf[" << j << "]: " << buf << std::endl;
                    const unsigned char* b = (unsigned char*)buf;
                    std::cout << "b: " << b << std::endl;
                    feat_desc[j] = b;
                    std::cout << "feat_desc: " << feat_desc << std::endl;

                    gh[j] = (unsigned char)feature_descriptor[j];
                    std::cout << "gh[" << j << "]: " << gh << std::endl;

                    int h = mat_row[j];
                    std::cout << "h[" << j << "]: " << h << std::endl;
                }

                const uchar* k = (uchar*)feat_desc;
                const uchar* test = (uchar*)gh;
                //std::cout << "k: " << k << std::endl;
                //cv::Mat d = feature_ptr->getDescriptor();
                const uchar* a = (uchar*)mat_row;

                //=========================

                char prueba1[64] = {248,191,231,113,112,0,0,67,195,199,255,255,255,255,254,251,255,255,252,49,143,243,56,15,0,224,115,231,25,101,252,225,
                                   255,255,255,255,255,253,243,156,231,220,243,57,8,225,63,58,217,100,242,247,255,255,255,247,249,108,118,190,159,207,25,9};
                char prueba2[64] = {252,191,227,112,104,64,0,1,193,194,13,127,248,243,199,23,158,120,228,49,255,227,48,14,0,160,51,231,8,100,28,227,
                                   252,255,239,63,230,252,251,255,255,156,147,49,1,225,188,187,17,239,255,255,255,255,249,179,249,254,255,62,27,141,136,187};

                const unsigned char* p1 = (unsigned char*)prueba1;
                const unsigned char* p2 = (unsigned char*)prueba2;

                const uchar* pr1 = (uchar*)p1;
                const uchar* pr2 = (uchar*)p2;


                //std::cout << "p1: " << std::hex << k << std::endl;

                double f_pr = cv::normHamming(pr1,pr2,64);
                std::cout << "========================================================================= f_pr: " << f_pr << std::endl;


                //==========================

                double f = cv::normHamming(test,a,64);
                //double dist_ham = cv::normHamming(b,b,4);
                std::cout << "========================================================================= dist_ham: " << f << std::endl;

                if(f < f_min)
                {
                    f_min = f;
                    row = i;
                }


            }




            if(f_min < 230)
            {
                FeaturePointImage* point_ptr = new FeaturePointImage(new_keypoints[row],(new_descriptors(cv::Range(row,row+1),cv::Range(0,new_descriptors.cols))),true);
                _feature_list_out.push_back(point_ptr);
                n_last_capture_feat++;
            }
        }
    }
    std::cout << "n_last_capture_feat: " << n_last_capture_feat << std::endl;
    std::cout << "n_last_capture_size: " << _feature_list_in.size() << std::endl;






    return n_last_capture_feat;
}
