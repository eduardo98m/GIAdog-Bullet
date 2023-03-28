#define _USE_MATH_DEFINES
#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "SharedMemory/PhysicsClientC_API.h"
#include "btBulletDynamicsCommon.h"
#include "Utils/b3Clock.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <cmath>
#include <math.h>


int main(int argc, char* argv[])
{
	btScalar fixedTimeStep = 1. / 240.;

    std::string path = "C:/Users/edued/Documents/Universidad/Quadruped/StaticLibTest/ThirdParty/bullet3/data/";
	
    b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();
    
    sim->setAdditionalSearchPath("C:/Users/edued/Documents/Universidad/Quadruped/StaticLibTest/ThirdParty/bullet3/data");
    sim->connect(eCONNECT_GUI);  //eCONNECT_GUI);//DIRECT);
    //Can also use eCONNECT_DIRECT,eCONNECT_SHARED_MEMORY,eCONNECT_UDP,eCONNECT_TCP, for example:
    //sim->connect(eCONNECT_UDP, "localhost", 1234);
    sim->configureDebugVisualizer(COV_ENABLE_GUI, 1);
    sim->configureDebugVisualizer( COV_ENABLE_SHADOWS, 1);//
    sim->configureDebugVisualizer( COV_ENABLE_KEYBOARD_SHORTCUTS,1);
	sim->configureDebugVisualizer( COV_ENABLE_MOUSE_PICKING, 1);
    //sim->setTimeOut(3);
    //syncBodies is only needed when connecting to an existing physics server that has already some bodies
    sim->syncBodies();

    sim->setTimeStep(fixedTimeStep);

    btQuaternion q = sim->getQuaternionFromEuler(btVector3(0.1, 0.2, 0.3));
    btVector3 rpy;
    rpy = sim->getEulerFromQuaternion(q);

    sim->setGravity(btVector3(0, 0, -9.8));
    btQuaternion orn;
    orn.setEulerZYX(0, 0, 0);

    b3RobotSimulatorLoadUrdfFileArgs args;
    args.m_startPosition.setValue(0, 0, 0);
    args.m_startOrientation = orn;
    args.m_useMultiBody = false;  //true : false;//false : true;
    args.m_forceOverrideFixedBase = true;

    int plane_id = sim->loadURDF(path + "plane.urdf", args);
    
    // args.m_startPosition.setValue(1.66, 1.66, 1.66);
    // args.m_startOrientation.setEulerZYX(M_PI, M_PI/2, M_PI/4);
    // args.m_useMultiBody = false;
    // args.m_forceOverrideFixedBase = false;
    // int cube_id  = sim->loadURDF(path + "cube.urdf", args);
    btVector3 initial_pos = btVector3(1.66, 1.66, 10.66);
    args.m_startPosition = initial_pos;
    args.m_startOrientation.setEulerZYX(0, 3.1415 * 0.23, 0);
    args.m_useMultiBody = true;
    args.m_forceOverrideFixedBase = false;
    sim->loadURDF(path +"quadruped/minitaur.urdf", args);

    double distance = 1.5;
    double yaw = 50;
    sim->resetDebugVisualizerCamera(distance, yaw, 20, initial_pos);
    sim->setRealTimeSimulation(false);
    sim->renderScene();

	
	int enableSim = 1;
	std::cout << "Simulando\n" ;
	while (sim->canSubmitCommand())
	{	

		b3KeyboardEventsData keyEvents;
		sim->getKeyboardEvents(&keyEvents);
		if (keyEvents.m_numKeyboardEvents)
		{
		
			//printf("num key events = %d]\n", keyEvents.m_numKeyboardEvents);
			//m_keyState is a flag combination of eButtonIsDown,eButtonTriggered, eButtonReleased
			for (int i=0;i<keyEvents.m_numKeyboardEvents;i++)
			{
				if (keyEvents.m_keyboardEvents[i].m_keyCode=='i' && keyEvents.m_keyboardEvents[i].m_keyState & eButtonTriggered)
				{
					enableSim = !enableSim;
				}
			}
		}
	
        //printf(".");
        if (enableSim)
        {   
            
            sim->stepSimulation();

            btVector3 rpy = sim->getEulerFromQuaternion(orn);

            
        }
		
		sim->renderScene();
		b3Clock::usleep(1000. * 1000. * fixedTimeStep);
	}

	printf("sim->disconnect\n");

    
    sim->disconnect();
    printf("delete sim\n");
    delete sim;
	

	printf("exit\n");
}
