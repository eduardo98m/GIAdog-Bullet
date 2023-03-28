//todo: turn this into a gtest, comparing maximal and reduced coordinates contact behavior etc

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "SharedMemory/PhysicsClientC_API.h"
#include "btBulletDynamicsCommon.h"
#include "Utils/b3Clock.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#define ASSERT_EQ(a, b) assert((a) == (b));
//#include "MinitaurSetup.h"
#define NUM_SIM 1
int main(int argc, char* argv[])
{
	b3RobotSimulatorClientAPI* sims[1];
	btScalar fixedTimeStep = 1. / 240.;
	for (int i = 0; i < NUM_SIM; i++)
	{
		b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();
		
		sims[i] = sim;
		sim->setAdditionalSearchPath("C:/Users/edued/Documents/Universidad/Quadruped/StaticLibTest/ThirdParty/bullet3/data");
		sim->connect(eCONNECT_GUI);  //eCONNECT_GUI);//DIRECT);
		//Can also use eCONNECT_DIRECT,eCONNECT_SHARED_MEMORY,eCONNECT_UDP,eCONNECT_TCP, for example:
		//sim->connect(eCONNECT_UDP, "localhost", 1234);
		sim->configureDebugVisualizer(COV_ENABLE_GUI, 1);
		sim->configureDebugVisualizer( COV_ENABLE_SHADOWS, 1);//COV_ENABLE_WIREFRAME
		//sim->setTimeOut(3);
		//syncBodies is only needed when connecting to an existing physics server that has already some bodies
		sim->syncBodies();

		sim->setTimeStep(fixedTimeStep);

		btQuaternion q = sim->getQuaternionFromEuler(btVector3(0.1, 0.2, 0.3));
		btVector3 rpy;
		rpy = sim->getEulerFromQuaternion(q);

		sim->setGravity(btVector3(0, 0, -9.8));
		btQuaternion orn;
		orn.setEulerZYX(0, 3.1415 * 0.23, 0);

		b3RobotSimulatorLoadUrdfFileArgs args;
		args.m_startPosition.setValue(0, 0, 0);
		args.m_startOrientation = orn;
		args.m_useMultiBody = i == 0 ? false : true;  //true : false;//false : true;
		args.m_forceOverrideFixedBase = true;

		int plane_id = sim->loadURDF("plane.urdf", args);
		
		args.m_startPosition.setValue(1.66, 1.66, 1.66);
		args.m_startOrientation.setEulerZYX(0, 3.1415 * 0.23, 0);
		int cube_id  = sim->loadURDF("cube.urdf", args);

		//args.m_startPosition.setValue(-1.66, 1.66, -1.66);
		//args.m_startOrientation.setEulerZYX(0, 3.1415 * 0.23, 0);
		//sim->loadURDF("kuka_lwr/kuka.urdf", args);

		std::cout << "KUKA KARGADA\n" ;

		double distance = 1.5;
		double yaw = 50;
		sim->resetDebugVisualizerCamera(distance, yaw, 20, btVector3(0, 0, 0.1));
		sim->setRealTimeSimulation(false);
		sims[0]->renderScene();

	}
	int enableSim = 1;
	std::cout << "Simulando\n" ;
	while (sims[0]->canSubmitCommand())
	{	

		b3KeyboardEventsData keyEvents;
		sims[0]->getKeyboardEvents(&keyEvents);
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
		for (int i = 0; i < NUM_SIM; i++)
		{
			sims[i]->setGravity(btVector3(0, 0, -10));
			//printf(".");
			if (enableSim)
			{
				sims[i]->stepSimulation();
				
			}
		}
		sims[0]->renderScene();
		b3Clock::usleep(1000. * 1000. * fixedTimeStep);
	}

	printf("sim->disconnect\n");

	for (int i = 0; i < NUM_SIM; i++)
	{
		sims[i]->disconnect();
		printf("delete sim\n");
		delete sims[i];
	}

	printf("exit\n");
}
