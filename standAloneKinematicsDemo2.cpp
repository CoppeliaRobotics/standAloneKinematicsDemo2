#include "ik.h"
#include <stdio.h>
#include <stdlib.h>

extern "C" {
    #include "extApi.h"
}

// For double-precision, define IK_DOUBLE in the project settings

int main(int argc, char* argv[])
{
    int portNb=0;
    int simMotorHandles[7];
    if (argc>=9)
    { // We get the port and the motor handles (for the visual feedback, not for IK!) via command-line arguments
        portNb=atoi(argv[1]);
        simMotorHandles[0]=atoi(argv[2]);
        simMotorHandles[1]=atoi(argv[3]);
        simMotorHandles[2]=atoi(argv[4]);
        simMotorHandles[3]=atoi(argv[5]);
        simMotorHandles[4]=atoi(argv[6]);
        simMotorHandles[5]=atoi(argv[7]);
        simMotorHandles[6]=atoi(argv[8]);
    }
    else
    {
        printf("Indicate following arguments: 'portNumber motor1Handle motor2Handle .. motor7Handle'!\n");
        extApi_sleepMs(5000);
        return 0;
    }

    // Read the kinematic file:
    FILE *file;
#ifdef __APPLE__
    file=fopen("../Resources/lbr_iiwa_7_r800.ik","rb");
#else
    file=fopen("lbr_iiwa_7_r800.ik","rb");
#endif
    unsigned char* data=nullptr;
    int dataLength=0;
    if (file)
    {
        fseek(file,0,SEEK_END);
        unsigned long fl=ftell(file);
        dataLength=(int)fl;
        fseek(file,0,SEEK_SET);
        data=new unsigned char[dataLength];
        fread((char*)data,dataLength,1,file);
        fclose(file);
    }
    else
    {
        printf("The kinematic content file 'lbr.ik' could not be read!\n");
        extApi_sleepMs(5000);
        return 0;
    }

    // Initialize the robot model:
    ikCreateEnvironment();
    ikLoad(data,dataLength);

    // Connect to CoppeliaSim at the above specified port, via the remote API. CoppeliaSim is just used for visual feed-back, not IK calculation!
    int clientID=simxStart("127.0.0.1",portNb,true,true,2000,5);
    if (clientID!=-1)
    {
        float simulationStep;
        simxGetFloatingParameter(clientID,sim_floatparam_simulation_time_step,&simulationStep,simx_opmode_streaming);

//      simxSynchronous(clientID,1); // We enable the synchronous mode, so that we can trigger each simulation step from here

        int ikGroupHandle;
        ikGetIkGroupHandle("LBR_iiwa_7_R800",&ikGroupHandle);
        int motorHandles[7];
        ikGetObjectHandle("LBR_iiwa_7_R800_joint1",motorHandles+0);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint2",motorHandles+1);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint3",motorHandles+2);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint4",motorHandles+3);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint5",motorHandles+4);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint6",motorHandles+5);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint7",motorHandles+6);
        int targetHandle;
        ikGetObjectHandle("LBR_iiwa_7_R800_target",&targetHandle);
        int baseHandle;
        ikGetObjectHandle("LBR_iiwa_7_R800",&baseHandle);

        simReal v=0.0;

        // Get the initial target dummy transformation, of the embedded model, and change it a bit:
        C7Vector transf;
        ikGetObjectTransformation(targetHandle,baseHandle,&transf);
        transf.X+=C3Vector(simReal(0.2),simReal(0.3),-simReal(0.2));
        ikSetObjectTransformation(targetHandle,baseHandle,&transf);

        while (simxGetConnectionId(clientID)!=-1)
        {
            // find a config for the given end-effector pose:
            simReal config[7];
            ikGetConfigForTipPose(ikGroupHandle,7,motorHandles,(simReal)0.4,1000,config,nullptr,nullptr,nullptr);

            // Read the corresponding motor angles and send them to CoppeliaSim:
            simxPauseCommunication(clientID,1); // Temporarily pause the remote API communication, in order to send all following commands at once
            simxSetJointPosition(clientID,simMotorHandles[0],(float)config[0],simx_opmode_oneshot);
            simxSetJointPosition(clientID,simMotorHandles[1],(float)config[1],simx_opmode_oneshot);
            simxSetJointPosition(clientID,simMotorHandles[2],(float)config[2],simx_opmode_oneshot);
            simxSetJointPosition(clientID,simMotorHandles[3],(float)config[3],simx_opmode_oneshot);
            simxSetJointPosition(clientID,simMotorHandles[4],(float)config[4],simx_opmode_oneshot);
            simxSetJointPosition(clientID,simMotorHandles[5],(float)config[5],simx_opmode_oneshot);
            simxSetJointPosition(clientID,simMotorHandles[6],(float)config[6],simx_opmode_oneshot);
            simxPauseCommunication(clientID,0); // Unpause the remote API communication

            printf(".");
        }
        ikEraseEnvironment(); // Erase the IK environment
        simxFinish(clientID); // End the remote API
    }
    return(0);
}

