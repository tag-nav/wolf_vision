
//C includes for sleep, time and main args
#include "unistd.h"
#include <time.h>
#include <sys/time.h>
#include <cstdlib>

//GLUT
#include <GL/glut.h>

//faramotics includes
#include "faramotics/window.h"
#include "faramotics/sceneRender.h"
#include "faramotics/dynamicSceneRender.h"
#include "faramotics/rangeSector.h"
#include "faramotics/rangeScan2D.h"
#include "btr-headers/pose3d.h"

//namespaces
using namespace std;

//3D models. Eiffel Tower from http://3dmag.org/en/market/item/258/
enum {CAMPUS = 1, EIFFEL_TOWER};
//const unsigned int environment_id = CAMPUS;

//function to travel around each model
void motionCampus(unsigned int ii, Cpose3d & pose)
{
    if (ii<=40)
    {
        //pose.rt.setEuler( pose.rt.head()-2*M_PI/180., pose.rt.pitch(), pose.rt.roll() );
        pose.pt(0) = pose.pt(0) + 0.1;
    }
    if ( (ii>40) && (ii<=80) )
    {
        pose.pt(0) = pose.pt(0) + 0.1;
    }
    if ( (ii>80) && (ii<=120) )
    {
        pose.pt(0) = pose.pt(0) + 0.1;
    }
    if ( (ii>120) && (ii<=170) )
    {
        pose.rt.setEuler( pose.rt.head()+1.8*M_PI/180., pose.rt.pitch(), pose.rt.roll() );
        pose.moveForward(0.2);
    }
    if ( (ii>170) && (ii<=220) )
    {
        pose.rt.setEuler( pose.rt.head()-1.8*M_PI/180., pose.rt.pitch(), pose.rt.roll());
    }
    if ( (ii>220) && (ii<=310) ) 
    {
        pose.pt(0) = pose.pt(0) + 0.1;
    }
    if ( (ii>310) && (ii<=487) ) 
    {
        pose.rt.setEuler( pose.rt.head()-1.*M_PI/180., pose.rt.pitch() , pose.rt.roll() );
        pose.moveForward(0.1);
    }
    if ( (ii>487) && (ii<=582) ) 
    {
        pose.moveForward(0.2);
    }
    if ( (ii>582) && (ii<=700) ) 
    {
        pose.pt(2) = pose.pt(2) + 0.001;
        pose.rt.setEuler( pose.rt.head()-1.*M_PI/180., pose.rt.pitch(), pose.rt.roll());
        pose.moveForward(0.1);
    }    
}


int main(int argc, char** argv)
{
    CdynamicSceneRender *myRender;
    CrangeScan2D *myScanner; 
    Cpose3d viewPoint;
    Cpose3d devicePose;
    vector<float> myScan;
    string modelFileName;
    unsigned int ii;
    timeval t1,t2;
    double dt;
    
    if (argc != 2)
    {
        cout << "Invalid number of arguments!" << endl;
        cout << "Call test as: visualizationTest environmentID printFlag" << endl;
        cout << "      printFlag: 0->Don't print scan values ; 1->Print scan values" << endl;
        cout << "EXIT PROGRAM" << endl;
        return -1;
    }
    bool print_flag = (bool)atoi(argv[1]);
    
    //model and initial view point
	modelFileName = "../models/campusNordUPC.obj";
	devicePose.setPose(2,8,0.2,0,0,0);
    
    //glut initialization
    glutInit(&argc, argv);
    
    //create a viewer for the 3D model and scan points
    myRender = new CdynamicSceneRender(1200,700,90*M_PI/180,90*700.0*M_PI/(1200.0*180.0),0.2,100);
    myRender->loadAssimpModel(modelFileName,true); //with wireframe
    
    //create scanner and load 3D model
    myScanner = new CrangeScan2D(HOKUYO_UTM30LX_180DEG);//HOKUYO_UTM30LX_180DEG or LEUZE_RS4
    myScanner->loadAssimpModel(modelFileName);
    
    //main loop after a pause
    sleep(1);
    for (ii=0; ii<1100; ii++)
    {
        //get init time 
        gettimeofday(&t1, NULL); 
        
        //moves the device position
        motionCampus(ii, devicePose);
        
        //compute scan
        myScan.clear();
        myScanner->computeScan(devicePose,myScan);
        
        //print scan
        if ( print_flag == true )
        {
            cout << endl;
            for (unsigned int jj = 0; jj < myScan.size(); jj++ ) cout << myScan[jj] << ",";
            cout << endl << endl;
        }
        
        //draws the device frame, scan hits and depth image
        myRender->drawPoseAxis(devicePose);
        myRender->drawScan(devicePose,myScan,180.*M_PI/180.,90.*M_PI/180.); //draw scan with leuze aperture params
                    
        //locate visualization view point, somewhere behind the device
        viewPoint.setPose(devicePose);
        viewPoint.rt.setEuler( viewPoint.rt.head(), viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
        viewPoint.moveForward(-5);
        
        //Set view point and render the scene
        myRender->setViewPoint(viewPoint);
        myRender->render();

        //get end time 
        gettimeofday(&t2, NULL);                 
        
        //sleep to have time to see something
        dt = (t2.tv_sec+t2.tv_usec/1e6) - (t2.tv_sec+t2.tv_usec/1e9);
        dt = dt*1e6; //dt in milliseconds
        if (dt < 50000) usleep(50000-dt);
    }
    
    //delete objects
    delete myRender;
    delete myScanner;

    return 0;
}
