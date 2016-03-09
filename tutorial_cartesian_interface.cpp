// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Cartesian Interface to control a limb
// in the operational space.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
//#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/iKin/iKinFwd.h>

#include <gsl/gsl_math.h>

#include <stdio.h>


#include <csignal>
#include <cmath>
#include <limits>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <deque>
#include <set>



#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/****************************************************************/
class Obstacle
{
    Integrator I;

public:
    double radius;
    Vector v;

    /****************************************************************/
    Obstacle(const Vector &x0, const double r,
             const Vector &v0, const double dt) :
             I(dt,x0), radius(r), v(v0) { }

    /****************************************************************/
    Vector move()
    {
        return I.integrate(v);
    }

    /****************************************************************/
    Vector getPosition() const
    {
        return I.get();
    }

    /****************************************************************/
    void setPosition(const Vector &x)
    {
        I.reset(x);
    }

    /****************************************************************/
    void setVelocity(const Vector &_v)
    {
        v = _v;
    }

    /****************************************************************/
    void setRadius(const double r)
    {
        radius = r;
    }

    /****************************************************************/
    string toString() const
    {
        ostringstream str;
        str<<I.get().toString(3,3)<<" "<<radius;
        return str.str();
    }
};
/****************************************************************/

class CtrlThread: public RateThread,
                  public CartesianEvent
{
protected:
    PolyDriver         client;
    ICartesianControl *icart;

    Vector xd;
    Vector od;

    int startup_context_id;

    double t;
    double t0;
    double t1;

    double yCoef; //mine ==> for trajectory modification
    BufferedPort<Bottle> plannerPortIn; // Input for trajectory from planner
    std::vector<Vector> bestTrajRoot;
    std::vector<double> valueTrajStored;
    int indexViaPoints;

    // the event callback attached to the "motion-ongoing"
    virtual void cartesianEventCallback()
    {
//        fprintf(stdout,"20%% of trajectory attained\n");

//        Bottle info;
//        icart->getInfo(info);

//        printf("check cartesianEvenCallback(), event type: %s\n",
//               cartesianEventParameters.type.c_str());
        if (cartesianEventParameters.type=="motion-done")
        {

            if (indexViaPoints<=bestTrajRoot.size() && bestTrajRoot.size()!=0)
            {
                indexViaPoints++;
                if (indexViaPoints<bestTrajRoot.size())
                {
                    xd=bestTrajRoot[indexViaPoints];
                    printf("A via point is reached, continues to the next via point %d-th: [%f, %f, %f]\n",
                           indexViaPoints+1,xd[0], xd[1], xd[2]);


                }
                else if (indexViaPoints==bestTrajRoot.size())
                {
//                    xd.clear();
                    printf("Trajectory done!!!\n");
                    printf("================================\n");
                }
            }
            else
                xd.clear(); // Clear the last value of xd or it will effect the next run

        }
//        else if (info.find("events").asList()->toString().c_str()=="motion-ongoing")
//        {
//            fprintf(stdout,"20%% of trajectory attained\n");
//        }
    }


public:
    CtrlThread(const double period) : RateThread(int(period*1000.0))
    {
        // we wanna raise an event each time the arm is at 20%
        // of the trajectory (or 70% far from the target)
//        cartesianEventParameters.type="motion-ongoing";
        cartesianEventParameters.type="motion-done";
//        cartesianEventParameters.motionOngoingCheckPoint=0.2;
    }

    virtual bool threadInit()
    {
        // open a client interface to connect to the cartesian server of the simulator
        // we suppose that:
        //
        // 1 - the iCub simulator is running
        //     (launch iCub_SIM)
        //
        // 2 - the cartesian server is running
        //     (launch simCartesianControl)
        //     
        // 3 - the cartesian solver for the left arm is running too
        //     (launch iKinCartesianSolver --context simCartesianControl --part left_arm)
        //
        Property option("(device cartesiancontrollerclient)");
        option.put("remote","/icubSim/cartesianController/left_arm");
        option.put("local","/cartesian_client/left_arm");

        if (!client.open(option))
            return false;

        // open the view
        client.view(icart);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        icart->storeContext(&startup_context_id);

        // set trajectory time
        icart->setTrajTime(1.0);

        // get the torso dofs
        Vector newDof, curDof;
        icart->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=1;
        newDof[1]=0;
        newDof[2]=1;

        // impose some restriction on the torso pitch
        limitTorsoPitch();

        // send the request for dofs reconfiguration
        icart->setDOF(newDof,curDof);

        // print out some info about the controller
        Bottle info;
        icart->getInfo(info);
        fprintf(stdout,"info = %s\n",info.toString().c_str());

        // register the event, attaching the callback
        icart->registerEvent(*this);

        xd.resize(3);
        od.resize(4);

        yCoef = 0.01; //mine: initialization
        string port4Planner = "/tutorial_cartesian_interface/upperPlanner:i";
        string portOfPlanner = "/upperPlanner/bestCartesianTrajectory:o";
        plannerPortIn.open(port4Planner);
        yarp::os::Network::connect(portOfPlanner,port4Planner);


        indexViaPoints = 0;

        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Thread started successfully\n");
        else
            fprintf(stdout,"Thread did not start\n");

        t=t0=t1=Time::now();
    }

    virtual void run()
    {
        t=Time::now();

//        generateTarget();

//        //mine: for modify the direction of trajectory, i.e. reapeating straight line in y axis
//        if (xd[1]>.31)
//        {
//          yCoef=-0.01;
//          t0=t;
//        }

//        else if (xd[1]<-.31)
//        {
//          yCoef=0.01;
//          t0=t;
//        }

        receiveTrajectory();

        generateTarget();
        // go to the target :)
        // (in streaming)
//        printf("check \n");
        icart->goToPose(xd,od);

        // some verbosity
//        printStatus();
    }

    virtual void threadRelease()
    {    
        // we require an immediate stop
        // before closing the client for safety reason
        icart->stopControl();

        // it's a good rule to restore the controller
        // context as it was before opening the module
        icart->restoreContext(startup_context_id);

        client.close();

        plannerPortIn.close();
    }

    void generateTarget()
    {   
        // translational target part: a circular trajectory
        // in the yz plane centered in [-0.3,-0.1,0.1] with radius=0.1 m
        // and frequency 0.1 Hz
//        xd[0]=-0.3;
//        xd[1]=-0.1+0.1*cos(2.0*M_PI*0.1*(t-t0));
//        xd[2]=+0.1+0.1*sin(2.0*M_PI*0.1*(t-t0));

        // mine: to generation a straight line trajectory
//        xd[0]=-0.3;
//        xd[1]=-0.3 + yCoef*(t-t0);
//        xd[2]=0.1;

//        xd[0]=-0.026;
//        xd[1]=-0.3;
//        xd[2]=0.3024;

//        xd[0]=-0.156;
//        xd[1]=-0.0579;
//        xd[2]=0.254;

//        xd[0]=-0.1652;
//        xd[1]=0.222852;
//        xd[2]=0.133712;
                 
        // we keep the orientation of the left arm constant:
        // we want the middle finger to point forward (end-effector x-axis)
        // with the palm turned down (end-effector y-axis points leftward);
        // to achieve that it is enough to rotate the root frame of pi around z-axis
        od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;
    }

    // Mine to process message from plannerPortIn
    void receiveTrajectory()
    {

        int nViaPoints, nDim;

        std::vector<double> valueTraj;
        Bottle* inMsg = plannerPortIn.read(false);
        if (inMsg != NULL)
        {
            printf("================================\n");
            printf("receiveTrajectory...\n");
            printf("indexViaPoints = %d \n", indexViaPoints);

            printf("[cartesianInterface] receives message\n");
            Bottle* inMsgOpened = inMsg->get(0).asList();
            nViaPoints = inMsgOpened->get(0).asInt();
            nDim = inMsgOpened->get(1).asInt();
            for (int i=2; i<nViaPoints*nDim+2; i++)
            {
                valueTraj.push_back(inMsgOpened->get(i).asDouble());
            }
            printf("valueTraj = ");
            for (int i=0; i<valueTraj.size(); i++)
                printf("%f, ",valueTraj[i]);
            cout<<endl;
            printf("valueTrajStored = ");
            for (int i=0; i<valueTrajStored.size(); i++)
                printf("%f, ",valueTrajStored[i]);
            cout<<endl;
            // Check if the received trajectory is new
            if (valueTraj!= valueTrajStored)
            {
                printf("check\n");
                indexViaPoints = 0;
                printf("indexViaPoints = %d \n", indexViaPoints);
                bestTrajRoot.clear();
                for (int i=0; i<nViaPoints; i++)
                {
                    Vector viaPoint(nDim, 0.0);
                    for (int j=0; j<nDim; j++)
                    {
                        viaPoint[j]=valueTraj[3*i+j];
                    }
                    bestTrajRoot.push_back(viaPoint);
                }
                for (int j=0; j<bestTrajRoot.size(); j++)
                  //for (int i=0; i<nDim; i++)
                {
                    printf("vertex in root [%i] = [%g,%g,%g]\n",
                       j, bestTrajRoot[j][0], bestTrajRoot[j][1], bestTrajRoot[j][2]);
                }
                printf("Trajectory has %d via points, in %d dimension space\n", nViaPoints, nDim);
                printf("receive trajectory including %d via points \n",bestTrajRoot.size());
                if (bestTrajRoot.size()!=0)
                {
                    xd = bestTrajRoot[indexViaPoints];
                    printf("xd = %f, %f, %f\n", xd[0], xd[1], xd[2]);
                }
                valueTrajStored.clear();
                for (int i=0; i<valueTraj.size(); i++)
                {
                    valueTrajStored.push_back(valueTraj[i]);
                }
            }
            inMsg->clear(); //Clear received message or it will effect the next period

        }
    }


    void limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;

        // sometimes it may be helpful to reduce
        // the range of variability of the joints;
        // for example here we don't want the torso
        // to lean out more than 30 degrees forward

        // we keep the lower limit
        icart->getLimits(axis,&min,&max);
        icart->setLimits(axis,min,MAX_TORSO_PITCH);
    }

    void printStatus()
    {        
        if (t-t1>=PRINT_STATUS_PER)
        {
            Vector x,o,xdhat,odhat,qdhat;

            // we get the current arm pose in the
            // operational space
            icart->getPose(x,o);

            // we get the final destination of the arm
            // as found by the solver: it differs a bit
            // from the desired pose according to the tolerances
            icart->getDesired(xdhat,odhat,qdhat);

            double e_x=norm(xdhat-x);
            double e_o=norm(odhat-o);

            fprintf(stdout,"+++++++++\n");
            fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
            fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
            fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
            fprintf(stdout,"od        [rad] = %s\n",od.toString().c_str());
            fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
            fprintf(stdout,"o         [rad] = %s\n",o.toString().c_str());
            fprintf(stdout,"norm(e_x)   [m] = %g\n",e_x);
            fprintf(stdout,"norm(e_o) [rad] = %g\n",e_o);

            fprintf(stdout,"time        [s] = %g\n",t-t0); //mine
            fprintf(stdout,"yCoef       [m] = %g\n",yCoef); //mine
            fprintf(stdout,"---------\n\n");

            t1=t;
        }
    }
};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;

public:
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new CtrlThread(CTRL_THREAD_PER);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main()
{   
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return 1;
    }



    CtrlModule mod;

    ResourceFinder rf;
    return mod.runModule(rf);
}



