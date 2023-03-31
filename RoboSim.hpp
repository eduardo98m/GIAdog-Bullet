#ifndef ROBO_SIM
#define ROBO_SIM

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "btBulletDynamicsCommon.h"
#include <vector>

//
class RoboSim : public b3RobotSimulatorClientAPI
{
public:
	RoboSim();

	virtual ~RoboSim();

	//virtual bool connect(int mode, const std::string& hostName = "localhost", int portOrKey = -1);

	//virtual void renderScene();

	//virtual void debugDraw(int debugDrawMode);

	//virtual bool mouseMoveCallback(float x, float y);

	//virtual bool mouseButtonCallback(int button, int state, float x, float y);

    virtual int loadURDF(const std::string &fileName, const btVector3 &pos, const btQuaternion &orn, bool useMultiBody = true, bool useFixedBase = false, double globalScaling = -1, int flags = 0);
    
    /*
    * Set the gravity acceleration.
    * @param gravityAcceleration The gravity acceleration.
    */
    virtual void setGravity(const btVector3 &gravityAcceleration);
    
    /*
    * Load a heightfield from a vector of heights.
    * @param height_map A vector of heights.
    * @param numHeightfieldRows The number of rows in the heightfield.
    * @param numHeightfieldColumns The number of columns in the heightfield.
    * @param meshScaleX The scale of the heightfield in the x direction.
    * @param meshScaleY The scale of the heightfield in the y direction.
    * @param meshScaleZ The scale of the heightfield in the z direction.
    * @param heightfieldTextureScaling The scaling of the heightfield texture.
    * @param replaceHeightfieldIndex The index of the heightfield to replace. If -1 (default), a new heightfield is created.
    */
    virtual int loadHeightfield(std::vector<float> height_map, 
                                int numHeightfieldRows, 
                                int numHeightfieldColumns, 
                                double meshScaleX                = 1.0, 
                                double meshScaleY                = 1.0, 
                                double meshScaleZ                = 1.0, 
                                double heightfieldTextureScaling = 1.0, 
                                int replaceHeightfieldIndex      = -1,
                                const btVector3 &pos             = btVector3(0,0,0), 
                                const btQuaternion &orn          = btQuaternion(0,0,0,1));


};

#endif  //ROBO_SIM
