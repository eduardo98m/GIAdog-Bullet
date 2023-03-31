#include "RoboSim.hpp"
#include <stdio.h>
#include "SharedMemory/PhysicsClientC_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include <vector>
//#include "SharedMemory/SharedMemoryPublic.h"

RoboSim::RoboSim()
{
}

RoboSim::~RoboSim()
{
}

int RoboSim::loadURDF(const std::string &fileName, 
                      const btVector3 &pos, 
                      const btQuaternion &orn, 
                      bool useMultiBody, 
                      bool useFixedBase, 
                      double globalScaling, 
                      int flags){
    
    b3SharedMemoryStatusHandle statusHandle;

    b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
    
    int statusType;

    flags |= URDF_USE_INERTIA_FROM_FILE;
    flags |= URDF_ENABLE_CACHED_GRAPHICS_SHAPES;
    flags |= URDF_IGNORE_VISUAL_SHAPES;
    b3SharedMemoryCommandHandle command = b3LoadUrdfCommandInit(sm, fileName.c_str());

    b3LoadUrdfCommandSetFlags(command, flags);

    // setting the initial position, orientation and other arguments are
    // optional
    b3LoadUrdfCommandSetStartPosition(command, pos.getX(), pos.getY(), pos.getZ());

    b3LoadUrdfCommandSetStartOrientation(command, orn.getX(), orn.getY(), orn.getZ(), orn.getW());
            
    if (useMultiBody)
    {
        b3LoadUrdfCommandSetUseMultiBody(command, 1);
    }
    if (useFixedBase)
    {
        b3LoadUrdfCommandSetUseFixedBase(command, 1);
    }
    if (globalScaling > 0)
    {
        b3LoadUrdfCommandSetGlobalScaling(command, globalScaling);
    }
    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    statusType = b3GetStatusType(statusHandle);
    if (statusType != CMD_URDF_LOADING_COMPLETED)
    {
        printf("Cannot load URDF file.");
        return NULL;
    }
    int bodyUniqueId = b3GetStatusBodyIndex(statusHandle);

    return bodyUniqueId;
}


void RoboSim::setGravity(const btVector3 &gravityAcceleration){

    b3SharedMemoryStatusHandle statusHandle;
    
    b3SharedMemoryCommandHandle command;

    b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;

    command = b3InitPhysicsParamCommand(sm);

    b3PhysicsParamSetGravity(command, gravityAcceleration.getX(), gravityAcceleration.getY(), gravityAcceleration.getZ());
 
    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
}

 int RoboSim::loadHeightfield(std::vector<float> height_map, 
                              int numHeightfieldRows, 
                              int numHeightfieldColumns, 
                              double meshScaleX, 
                              double meshScaleY,
                              double meshScaleZ,
                              double heightfieldTextureScaling,
                              int replaceHeightfieldIndex,
                              const btVector3 &pos, 
                              const btQuaternion &orn){

    b3SharedMemoryStatusHandle statusHandle;
    
    b3SharedMemoryCommandHandle command;

    b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;

    command = b3CreateCollisionShapeCommandInit(sm);



    // Check if the size of the heightfield data matches the number of rows and columns
    if (height_map.size() != numHeightfieldColumns*numHeightfieldRows)
    {
        printf("Size of heightfieldData doesn't match numHeightfieldColumns*numHeightfieldRows");
        return NULL;
    }
    
    
    double meshScale[3] = {meshScaleX, meshScaleY, meshScaleZ};
    int shapeIndex = b3CreateCollisionShapeAddHeightfield2(sm, 
                                                           command, 
                                                           meshScale, 
                                                           heightfieldTextureScaling, 
                                                           height_map.data(), 
                                                           numHeightfieldRows, 
                                                           numHeightfieldColumns, 
                                                           replaceHeightfieldIndex);
    
    double  collisionFramePosition    [3] = {pos.getX(), pos.getY(), pos.getZ()};
    double  collisionFrameOrientation [4] = {orn.getX(), orn.getY(), orn.getZ(), orn.getW()};
    double baseInertialFramePosition[3] = {0, 0, 0};
	double baseInertialFrameOrientation[4] = {0, 0, 0, 1};

    

    // b3CreateCollisionShapeSetChildTransform(command, 
    //                                         shapeIndex, 
    //                                         collisionFramePosition, 
    //                                         collisionFrameOrientation);
    

    // int flags = 0;
    // b3CreateCollisionSetFlag(command, shapeIndex, flags);

    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);

    int statusType = b3GetStatusType(statusHandle);
    printf("statusType = %d \n", statusType);
    if (statusType != CMD_CREATE_COLLISION_SHAPE_COMPLETED)
    {
        printf("Cannot create heightfield.");
        return NULL;
    }
    // Part B: Create a multi-body base using the heightfield shape
    
    printf("shapeIndex = %d \n", shapeIndex);
    shapeIndex = b3GetStatusCollisionShapeUniqueId(statusHandle);
    printf("shapeIndex = %d\n", shapeIndex);
    
    b3SharedMemoryCommandHandle command_b = b3CreateMultiBodyCommandInit(sm);
    printf("Debug A\n");
    int baseIndex = b3CreateMultiBodyBase(command_b, 
                                            0.0,
                                            shapeIndex, 
                                            -1, 
                                            collisionFramePosition, 
                                            collisionFrameOrientation, 
                                            baseInertialFramePosition, 
                                            baseInertialFrameOrientation);
    printf("Debug B\n");
    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command_b);
    statusType = b3GetStatusType(statusHandle);
    printf("Debug c status handle:%d\n", statusType);




    return b3GetStatusCollisionShapeUniqueId(statusHandle);
    
 }