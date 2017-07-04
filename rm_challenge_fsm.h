#define ZBY_PC 1
#define MANIFOLD 2
#define CURRENT_COMPUTER ZBY_PC

#include <sstream>
#include <ros/assert.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
// C++标准库
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
// boost asio
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#if CURRENT_COMPUTER == MANIFOLD
#include <dji_sdk/dji_drone.h>
using namespace DJI::onboardSDK;
#endif

using namespace std;
using namespace boost::asio;
// const definition
class RMChallengeFSM
{
  public:
    enum TASK_STATE
    {
        TAKE_OFF,
        GO_UP,
        GO_TO_SETPOINT,
        IDLE,
        TRACK_LINE,
        LAND,
        CONTROL_GRASPPER,
        GO_TO_LAND_POINT,
    };
    enum GRASPPER_STATE
    {
        GRASPPER_OPEN,
        GRASPPER_CLOSE,
    };
    enum UAV_STATE
    {
        UAV_FLY,
        UAV_LAND,
    };
    enum LAND_POINT_TYPE
    {
        BASE_LAND_POINT,
        PILLAR_LAND_POINT,
    };
    enum PREPARE_TO_LAND_TYPE
    {
        PREPARE_AT_HIGH,
        PREPARE_AT_LOW,
    };
    enum LINE_TYPE
    {
        YELLOW_LINE,
        VIRTUAL_LINE,
    };
    RMChallengeFSM()
    {
    }
    ~RMChallengeFSM();
    void run();
    void initialize();
    void updateState();

  public:
    boost::asio::serial_port *m_serial_port;
    boost::system::error_code m_err_code;
    boost::asio::io_service m_io_service;
    TASK_STATE m_state;
    static const int GOAL_NUMBER = 5;
    /**
    *takeoff point id,0 is start point
    *1,2,4,5 are pillar,3, 6 are base
    */
    float m_goal_height[GOAL_NUMBER];
    float m_takeoff_points[GOAL_NUMBER + 1][2];
    float m_setpoints[GOAL_NUMBER + 1][2];
    /**subscribe from dji's nodes*/
    UAV_STATE m_uav_state;                      ///<update outside
    float m_current_height_from_guidance;       ///<height need to update outside
    float m_current_position_from_guidance[2];  ///<position need to update outside
    /**subscribe from vision node about circle and triangle*/
    bool m_discover_pillar_circle;
    float m_landpoint_position_error[2];  ///<need to update outside
    float m_current_height_from_vision;
    int m_pillar_triangle[4];
    /**subscribe from  vision node about base*/
    bool m_discover_base;
    /**subscribe from vision node about detectLine*/
    float m_distance_to_line[2];  ///<update outside
    float m_line_normal[2];       ///<update outside
    LAND_POINT_TYPE m_land_point_type;
    PREPARE_TO_LAND_TYPE m_prepare_to_land_type;
    GRASPPER_STATE m_graspper_state = GRASPPER_CLOSE;  ///<update inside
    int m_graspper_control_time = 0;
    int m_current_takeoff_point_id = 0;  ///<current position, need to update inside
    ros::Time m_takeoff_time;
    /**uav state checking method*/
    void transferToTask( TASK_STATE task_state );  // tested
    bool isTakeoffTimeout();                       // tested
    bool isTakingoff();                            // tested
    bool isOnLand();                               // tested
    bool closeToGoalHeight();                      // tested
    bool farFromTakeoffPoint();                    // tested
    bool discoverLandPoint();                      /// tested
    bool discoverTriangle();                       /// tested
    bool discoverYellowLine();                     // tested
    bool closeToSetPoint();                        // tested
    bool readyToLand();                            // tested
    bool finishGraspperTask();                     // tested
                                                   /**uav control method*/
  public:
    void droneTakeoff();
    void droneLand();
    void controlDroneVelocity( float x, float y, float z, float yaw );
    void controlGraspper();     // tested
    void openGraspper();        // tested
    void closeGraspper();       // tested
    void updateTakeoffTime();   // tested
    void droneGoUp();           // tested
    void droneGoToSetPoint();   // tested
    void droneTrackLine();      // tested
    void droneHover();          // tested
    void dronePrepareToLand();  // tested
    void calculateNormalVelocity( float &x, float &y,
                                  LINE_TYPE lint_type );  // tested
    void calculateTangentialVelocity( float &x, float &y,
                                      LINE_TYPE lint_type );  // tested
    void calculateYawRate( float &yaw );        // tested,confirm yaw direction
    void unitifiyVector( float &x, float &y );  // tested
    void navigateByTriangle( float &x, float &y, float &z );  // tested
    void navigateByCircle( float &x, float &y, float &z );    // tested
    /**update state variables from subscription*/
  public:
    /**update from dji's nodes*/
    void setDroneState( int state );
    void setHeightFromGuidance( float height );
    void setPositionFromGuidance( float x, float y );  //考虑漂移，m_bias
    /**update from topic about circle and triangle*/
    void setCircleVariables( bool is_circle_found, float position_error[2],
                             float height );
    void setTriangleVariables( int pillar_triangle[4] );
    /**update from topic about base */
    void setBaseVariables( bool is_base_found, float position_error[2] );
    /**update from topic about detectLine*/
    void setLineVariables( float distance_to_line[2], float line_normal[2] );
};
