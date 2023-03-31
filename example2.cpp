#define _USE_MATH_DEFINES
//#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
//#include "SharedMemory/PhysicsClientC_API.h"
#include "btBulletDynamicsCommon.h"
#include "Utils/b3Clock.h"
#include "RoboSim.hpp"

#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <random>
#include <iomanip>
#include "omp.h"
#include <chrono>

int createSlope(RoboSim* sim, double slope, double roughness) 
{   
    // WORK IN PROGRESS


    // Gather noise data
    //std::vector<double> height_map, noise_map;
    
    int resolution = 90;
    double terrain_x_size = 0.25;
    double terrain_y_size = 0.25;

    std::vector<float> height_map(resolution * resolution);

    // create a uniform random distribution between -1 and 1
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0);
    
    for (int y = 0; y < resolution; y++){
        for (int x = 0; x < resolution; x++){
            height_map[y*resolution + x] = roughness * dis(gen) + x * terrain_x_size * slope/ resolution;
        }
    }


    
    auto map_id = sim->loadHeightfield(height_map,
                                    resolution, 
                                    resolution,
                                    terrain_x_size,
                                    terrain_y_size,
                                    1.0);
    
    return map_id;
}

RoboSim* init_sim(bool use_gui, btScalar fixedTimeStep = 1. / 240.){
    const std::string   path = "C:/Users/edued/Documents/Universidad/Quadruped/GIAdog Bullet/ThirdParty/bullet3Robotics/data/";
    RoboSim* sim = new RoboSim();
    
    if (use_gui){
        sim->connect(eCONNECT_GUI);  
        sim->configureDebugVisualizer( COV_ENABLE_GUI,               1);
        sim->configureDebugVisualizer( COV_ENABLE_SHADOWS,           1);//
        sim->configureDebugVisualizer( COV_ENABLE_KEYBOARD_SHORTCUTS,1);
        sim->configureDebugVisualizer( COV_ENABLE_MOUSE_PICKING,     1);
        sim->syncBodies();

        double distance = 4.5;
        double yaw      = 50;
        btVector3 initial_pos = btVector3(0.0, 0.0, 0.0);
        sim->resetDebugVisualizerCamera(distance, -30, yaw, initial_pos);
        sim->setRealTimeSimulation(false);
    }
    else{
        sim->connect(eCONNECT_DIRECT);
        sim->setRealTimeSimulation(false);
    }
    sim->setTimeStep(fixedTimeStep);

    int plane_id = createSlope(sim, 1.0, 0.3);

    int base_idx = sim->loadURDF("mini_ros/urdf/spot.urdf", 
                                btVector3(0, 0, 0.5),       // position 
                                btQuaternion(0, 0, 0, 1), // orientation
                                false,                    // useMultiBody
                                false                     // useFixedBase
                                );


   return sim;

}
RoboSim* init_sim_gui(btScalar fixedTimeStep = 1. / 240.){
    RoboSim* sim = new RoboSim();
    sim->connect(eCONNECT_GUI);  
    sim->configureDebugVisualizer( COV_ENABLE_GUI,               1);
    sim->configureDebugVisualizer( COV_ENABLE_SHADOWS,           1);//
    sim->configureDebugVisualizer( COV_ENABLE_KEYBOARD_SHORTCUTS,1);
    sim->configureDebugVisualizer( COV_ENABLE_MOUSE_PICKING,     1);
    sim->syncBodies();

    double distance = 4.5;
    double yaw      = 50;
    btVector3 initial_pos = btVector3(0.0, 0.0, 0.0);
    sim->resetDebugVisualizerCamera(distance, -30, yaw, initial_pos);
    sim->setRealTimeSimulation(false);
    sim->setTimeStep(fixedTimeStep);
    int plane_id = createSlope(sim, 1.0, 0.3);

    int base_idx = sim->loadURDF("anymal_c/urdf/anymal.urdf", 
                                btVector3(0, 0, 1.4),       // position 
                                btQuaternion(0, 0, 0, 1), // orientation
                                false,                    // useMultiBody
                                false                     // useFixedBase
                                );
    sim->setGravity(btVector3(0, 0, -9.81));
    
    return sim;
}

RoboSim* init_sim_no_gui(btScalar fixedTimeStep = 1. / 240.){
    RoboSim* sim = new RoboSim();
    sim->connect(eCONNECT_DIRECT);  
    sim->setRealTimeSimulation(false);
    sim->setTimeStep(fixedTimeStep);
    int plane_id = createSlope(sim, 1.0, 0.3);

    int base_idx = sim->loadURDF("anymal_c/urdf/anymal.urdf", 
                                btVector3(0, 0, 1.4),       // position 
                                btQuaternion(0, 0, 0, 1), // orientation
                                false,                    // useMultiBody
                                false                     // useFixedBase
                                );
    sim->setGravity(btVector3(0, 0, -9.81));
    
    return sim;
}

int main(int argc, char* argv[])
{
	
    btScalar fixedTimeStep = 1. / 240.;
    
    const int N_SIMS = 100;
    omp_set_num_threads(8);

    //Create an array of simulators


    RoboSim* sims[N_SIMS];
#pragma omp parallel for schedule(auto)
    for (int i = 0; i < N_SIMS; i++){
        if (i == -1){
            sims[i] = init_sim_gui();
        }
        else{
            sims[i] = init_sim_no_gui();
        }
    }
    printf("Sims initialized\n");

    
	// int enableSim = 1;
	// while (sims[0]->canSubmitCommand())
	// {	

	// 	// b3KeyboardEventsData keyEvents;
	// 	// sims[0]->getKeyboardEvents(&keyEvents);
	// 	// if (keyEvents.m_numKeyboardEvents)
	// 	// {
		
	// 	// 	for (int i=0;i<keyEvents.m_numKeyboardEvents;i++)
	// 	// 	{
	// 	// 		if (keyEvents.m_keyboardEvents[i].m_keyCode=='i' && keyEvents.m_keyboardEvents[i].m_keyState & eButtonTriggered)
	// 	// 		{
	// 	// 			enableSim = !enableSim;
	// 	// 		}
	// 	// 	}
	// 	// }
    //     sims[0]->renderScene();
    //     //sim->stepSimulation();
    //     for (int i = 0; i < N_SIMS; i++){
    //         #pragma omp parallel for schedule(auto)
    //         sims[i]->stepSimulation();
    //     }


		
	// 	//b3Clock::usleep(1000. * 1000. * fixedTimeStep);
	// }
    double Total_Simulation_Time = 10.0;
    int N_STEPS = Total_Simulation_Time / fixedTimeStep;
    printf("N_STEPS: %d\n", N_STEPS);

    // Get the current time
    
    auto start = std::chrono::high_resolution_clock::now();
    
    printf("Start simulation\n");
    for (int i = 0; i < N_STEPS; i++){
#pragma omp parallel for schedule(auto)
        for (int j = 0; j < N_SIMS; j++){
            sims[j]->stepSimulation();
        }
    }

    printf("End simulation\n");

    auto stop = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    
    double seconds = duration.count() / 1000000.0;
    
    std::cout << "Time taken by function: "
              << seconds << " seconds" << std::endl;
    double real_time_factor = (Total_Simulation_Time * N_SIMS) /  seconds ;

    std::cout << "Elapsed time: " << seconds << "s\n";
    std::cout << "Real time factor: " << real_time_factor << "\n";
    




	printf("sim->disconnect\n");
    for (int i = 0; i < N_SIMS; i++){
        sims[i]->disconnect();
        delete sims[i];
    }
    //sim->disconnect();
    //delete sim;
    printf("delete sim\n");
	printf("exit\n");
}
