#include "rm_challenge_fsm.h"

RMChallengeFSM::~RMChallengeFSM()
{
    delete m_serial_port;
#if CURRENT_COMPUTER == MANIFOLD
    m_drone->release_sdk_permission_control();
    delete m_drone;
#endif
}
void RMChallengeFSM::run()
{
    ROS_INFO_STREAM( "running: state is:" << m_state );
	switch(m_state)
	{
		test1
		case TAKE_OFF:  //
		{
			// send take off command to uav until state change
			if ( !isTakingoff() )
			{
				/* send take off command to uav*/
				closeGraspper();
				droneTakeoff();
				updateTakeoffTime();
				ros::Duration( 1.0 ).sleep();
			}
			else
			{
				if ( isTakingoff() && !isTakeoffTimeout() )
				{
					/* wait for time out */
					ros::Duration( 0.5 ).sleep();
				}
				else if ( isTakeoffTimeout() )
				{
					/* state change to GO_UP*/
					transferToTask( GO_UP );
				}
			}
		}
		break;
		case  GO_UP:  //
		{
			/* not reach goal height */
			if ( !closeToGoalHeight() )
			{
				/* uav go up*/
				droneGoUp();
			}
			else if ( closeToGoalHeight() )
			{
				/* change state to setpoint*/
				transferToTask( GO_TO_SETPOINT );
			}
		}
		break;
		case GO_TO_SETPOINT :  //
		{
			if ( !farFromTakeoffPoint() )
			{
				droneGoToSetPoint();
			}
			else if ( discoverLandPoint() )
			{
				transferToTask( GO_TO_LAND_POINT );
			}
			else if ( discoverYellowLine() )
			{
				transferToTask( TRACK_LINE );
			}
			else if ( !discoverLandPoint() && !discoverYellowLine() &&
						!closeToSetPoint() )
			{
				droneGoToSetPoint();
			}
			else
			{
				transferToTask( IDLE );
			}
		}
		break;
		case  TRACK_LINE :
		{
			if ( !discoverLandPoint() )
			{
				/* track detectLine */
				droneTrackLine();
			}
			else
			{
				/* state change to land */
				transferToTask( GO_TO_LAND_POINT );
			}
		}
		break;
		case GO_TO_LAND_POINT :
		{
			if ( !readyToLand() )
			{
				/* go to land point */
				dronePrepareToLand();
			}
			else
			{
				/* land*/
				transferToTask( LAND );
			}
		}
		break;
		case LAND:
		{
			if ( !isOnLand() )
			{
				/* continue to land */
				openGraspper();
				droneLand();
			}
			else if ( isOnLand() )
			{
				droneHover();
				droneDropDown();
				transferToTask( CONTROL_GRASPPER );
			}
		}
		break;
		case CONTROL_GRASPPER :  //
		{
			if ( !finishGraspperTask() )
			{
				/* continue graspper control */
				controlGraspper();
			}
			else
			{
				/* state change to take off*/
				closeGraspper();
				transferToTask( TAKE_OFF );
			}
		}
		break;
		case  IDLE :  //
		{
			/* code */
			if ( discoverLandPoint() )
			{
				transferToTask( GO_TO_LAND_POINT );
			}
			else if ( discoverYellowLine() )
			{
				/* track detectLine*/
				transferToTask( TRACK_LINE );
			}
			else
			{
				// do nothing, wait
				droneHover();
				ros::Duration( 0.5 ).sleep();
			}
		}
		break;
	}
}
void anotherTest()
{
	//here is another test function
}
void RMChallengeFSM::resetAllState()
{
    m_state = TAKE_OFF;
    m_current_position_from_guidance[0] = 0.0;
    m_current_position_from_guidance[1] = 0.0;
    m_prepare_to_land_type = PREPARE_AT_HIGH;
    m_graspper_control_time = 0;
    m_current_takeoff_point_id = 0;
    m_land_counter = 0;
}
void RMChallengeFSM::initialize( ros::NodeHandle &node_handle )
{
    /*initialize serial port*/
    m_serial_port = new boost::asio::serial_port( m_io_service );
    m_serial_port->open( "/dev/ttyTHS0", m_err_code );
    m_serial_port->set_option( serial_port::baud_rate( 9600 ), m_err_code );
    m_serial_port->set_option(
        serial_port::flow_control( serial_port::flow_control::none ), m_err_code );
    m_serial_port->set_option( serial_port::parity( serial_port::parity::none ),
                               m_err_code );
    m_serial_port->set_option( serial_port::stop_bits( serial_port::stop_bits::one ),
                               m_err_code );
    m_serial_port->set_option( serial_port::character_size( 8 ), m_err_code );

/*initialize dji sdk*/
#if CURRENT_COMPUTER == MANIFOLD
    m_drone = new DJIDrone( node_handle );
    m_drone->request_sdk_permission_control();
#endif

    /*initialize parameter*/
    for ( int i = 0; i < TAKEOFF_POINT_NUMBER; ++i )
    {
        /* only set for one time */
        m_goal_height[i] = 2.0;
        m_takeoff_points[i][0] = 0.0;
        m_takeoff_points[i][1] = 0.0;
        m_setpoints[i][0] = 8.0;
        m_setpoints[i][1] = 0.7;
    }

    /*initialize  state*/
    resetAllState();

    /*just test some function, need to be delete later*/
    // m_current_height_from_guidance = 5;
    //    m_current_position_from_guidance[0] = 8;
    //    m_current_position_from_guidance[1] = 5;
    //    m_setpoints[0][0] = 8;
    //    m_setpoints[0][1] = 6;
    // m_distance_to_line[0] = 0.2;
    // m_pillar_triangle[0] = 1;
    //    m_discover_pillar_circle = true;
    // m_discover_base = true;
    // m_current_takeoff_point_id = 2;
    //    m_landpoint_position_error[0] = 0.03;
    //    m_landpoint_position_error[1] = -0.03;
    // m_current_height_from_vision = 0;
    //    m_distance_to_line[0] = 0.2;
    //    m_distance_to_line[1] = 0.2;
    //    m_line_normal[0] = 2;
    //    m_line_normal[1] = -7;
    //    m_pillar_triangle[1] = 1;
    //    m_pillar_triangle[2] = 1;
    // m_pillar_triangle[3] = 1;
    // this->unitifiyVector(m_line_normal[0], m_line_normal[1]);
    //    while ( !finishGraspperTask() )
    //    {
    //        controlGraspper();
    //    }
    //    ros::Duration( 1 ).sleep();
}
void RMChallengeFSM::transferToTask( TASK_STATE task_state )
{
    if ( task_state == TAKE_OFF )
    {
        m_state = TAKE_OFF;
    }
    else if ( task_state == GO_UP )
    {
        m_state = GO_UP;
    }
    else if ( task_state == GO_TO_SETPOINT )
    {
        m_state = GO_TO_SETPOINT;
    }
    else if ( task_state == IDLE )
    {
        m_state = IDLE;
    }
    else if ( task_state == TRACK_LINE )
    {
        m_state = TRACK_LINE;
    }
    else if ( task_state == LAND )
    {
        m_state = LAND;
    }
    else if ( task_state == CONTROL_GRASPPER )
    {
        m_state = CONTROL_GRASPPER;
    }
    else if ( task_state == GO_TO_LAND_POINT )
    {
        m_state = GO_TO_LAND_POINT;
    }
}
bool RMChallengeFSM::isTakeoffTimeout()
{
    double t = ros::Time::now().toSec() - m_takeoff_time.toSec();
    ROS_INFO_STREAM( "time now is:" << ros::Time::now().toSec() );
    ROS_INFO_STREAM( "takeoff time is:" << m_takeoff_time.toSec() );
    // static double time_threshold = 100;
    ROS_INFO_STREAM( "Taking off time is:" << t );
    if ( PA_TAKEOFF_TIME < t )
    {
        ROS_INFO_STREAM( "Time is enough" );
        return true;
    }
    else
    {
        ROS_INFO_STREAM( "Time is too less" );
        return false;
    }
}
bool RMChallengeFSM::isTakingoff()
{
    if ( m_uav_state == UAV_FLY )
    {
        ROS_INFO_STREAM( "Is taking off" );
        return true;
    }
    else
    {
        ROS_INFO_STREAM( "Wait to take off" );
        return false;
    }
}
bool RMChallengeFSM::isOnLand()
{
    if ( m_uav_state == UAV_LAND )
    {
        ROS_INFO_STREAM( "on land" );
        if ( m_current_takeoff_point_id < TAKEOFF_POINT_NUMBER - 1 )
        {
            m_current_takeoff_point_id += 1;
        }
        return true;
    }
    else
    {
        ROS_INFO_STREAM( "still not on land" );
        return false;
    }
}
bool RMChallengeFSM::closeToGoalHeight()
{
    float height_error = fabs( m_current_height_from_guidance -
                               m_goal_height[m_current_takeoff_point_id] );
    // static float height_threshold = 10;
    if ( height_error < PA_TAKEOFF_HEIGHT_THRESHOLD )
    {
        ROS_INFO_STREAM( "Take off height error :" << height_error << "is "
                                                                      "small" );
        return true;
    }
    else
    {
        ROS_INFO_STREAM( "Take off height error:" << height_error << "is too "
                                                                     "large" );
        return false;
    }
}
bool RMChallengeFSM::farFromTakeoffPoint()
{
    double pos_error =
        sqrt( pow( m_current_position_from_guidance[0] -
                       m_takeoff_points[m_current_takeoff_point_id][0],
                   2 ) +
              pow( m_current_position_from_guidance[1] -
                       m_takeoff_points[m_current_takeoff_point_id][1],
                   2 ) );
    // static double takeoff_pos_err_threshold = 5;
    if ( pos_error > PA_TAKEOFF_POSITION_ERROR )
    {
        ROS_INFO_STREAM( "distance to takeoff point is:" << pos_error << ",fa"
                                                                         "r" );
        return true;
    }
    else
    {
        ROS_INFO_STREAM( "distance to takeoff point is:" << pos_error << ",too "
                                                                         "close" );
        return false;
    }
}
bool RMChallengeFSM::discoverTriangle()
{
    int triangle_num = m_pillar_triangle[0] + m_pillar_triangle[1] +
                       m_pillar_triangle[2] + m_pillar_triangle[3];
    if ( triangle_num != 0 )
    {
        ROS_INFO_STREAM( "discover triangle" << triangle_num );
        return true;
    }
    else
    {
        ROS_INFO_STREAM( "no triangle" );
        return false;
    }
}
bool RMChallengeFSM::discoverLandPoint()
{
    if ( m_current_takeoff_point_id == 2 || m_current_takeoff_point_id == 5 )
    {
        if ( m_discover_base )
        {
            m_land_point_type = BASE_LAND_POINT;
            ROS_INFO_STREAM( "discover base" );
            return true;
        }
        else
        {
            ROS_INFO_STREAM( "no base" );
            return false;
        }
    }
    else
    {
        if ( m_discover_pillar_circle || discoverTriangle() )
        {
            m_land_point_type = PILLAR_LAND_POINT;
            ROS_INFO_STREAM( "circle:" << m_discover_pillar_circle );
            return true;
        }
        else
        {
            ROS_INFO_STREAM( "no circle" );
            return false;
        }
    }
}
bool RMChallengeFSM::discoverYellowLine()
{
    if ( fabs( m_distance_to_line[0] ) > 0.0001 ||
         fabs( m_distance_to_line[1] ) > 0.0001 )
    {
        ROS_INFO_STREAM( "discover yellow detectLine" );
        return true;
    }
    else
    {
        ROS_INFO_STREAM( "no yellow detectLine found" );
        return false;
    }
}
bool RMChallengeFSM::closeToSetPoint()
{
    float disp_x = m_current_position_from_guidance[0] -
                   m_takeoff_points[m_current_takeoff_point_id][0];
    float disp_y = m_current_position_from_guidance[1] -
                   m_takeoff_points[m_current_takeoff_point_id][1];
    double pos_error =
        sqrt( pow( disp_x - m_setpoints[m_current_takeoff_point_id][0], 2 ) +
              pow( disp_y - m_setpoints[m_current_takeoff_point_id][1], 2 ) );
    // static double setpoint_pos_err_threshold = 2;
    if ( pos_error < PA_SETPOINT_POSITION_ERROR )
    {
        ROS_INFO_STREAM( "Error to setpoint is :" << pos_error << ",close" );
        return true;
    }
    else
    {
        ROS_INFO_STREAM( "Error to setpoint is :" << pos_error << ",too far" );
        return false;
    }
}
bool RMChallengeFSM::finishGraspperTask()
{
    // static int graspper_control_time_threshold = 6;
    if ( m_graspper_control_time >= PA_GRASPPER_CONTROL_TIME )
    {
        m_graspper_control_time = 0;
        ROS_INFO_STREAM( "graspper  finish" );
        return true;
    }
    else
    {
        ROS_INFO_STREAM( "graspper not finish" );
        return false;
    }
}
void RMChallengeFSM::droneTakeoff()
{
#if CURRENT_COMPUTER == MANIFOLD
    m_drone->takeoff();
#endif
}
void RMChallengeFSM::droneLand()
{
#if CURRENT_COMPUTER == MANIFOLD
    m_drone->landing();
#endif
}
void RMChallengeFSM::openGraspper()
{
    boost::asio::write( *m_serial_port, boost::asio::buffer( "b" ),
                        m_err_code );  // open graspper
    m_graspper_state = GRASPPER_OPEN;
}
void RMChallengeFSM::closeGraspper()
{
    boost::asio::write( *m_serial_port, boost::asio::buffer( "a" ),
                        m_err_code );  // close graspper
    m_graspper_state = GRASPPER_CLOSE;
}
void RMChallengeFSM::controlGraspper()
{
    m_graspper_control_time++;
    if ( m_graspper_state == GRASPPER_CLOSE )
    {
        openGraspper();
    }
    else
    {
        closeGraspper();
    }
    ROS_INFO_STREAM( "graspper state is:" << m_graspper_state );
}
void RMChallengeFSM::updateTakeoffTime()
{
    m_takeoff_time = ros::Time::now();
    ROS_INFO_STREAM( "Taking off time is :" << m_takeoff_time );
}
void RMChallengeFSM::controlDroneVelocity( float x, float y, float z, float yaw )
{
#if CURRENT_COMPUTER == MANIFOLD
    m_drone->attitude_control( 0x4B, x, y, z, yaw );
#endif
    ros::Duration( 20 / 1000 ).sleep();
}
void RMChallengeFSM::droneGoUp()
{
    // static float go_up_velocity = 0.2;
    if ( m_goal_height[m_current_takeoff_point_id] > m_current_height_from_guidance )
    {
        controlDroneVelocity( 0.0, 0.0, PA_GO_UP_VELOCITY, 0.0 );
        ROS_INFO_STREAM( "go up" );
    }
    else
    {
        controlDroneVelocity( 0.0, 0.0, -PA_GO_UP_VELOCITY, 0.0 );
        ROS_INFO_STREAM( "go down" );
    }
}
void RMChallengeFSM::droneGoToSetPoint()
{
    float vt_x, vt_y;
    calculateTangentialVelocity( vt_x, vt_y, VIRTUAL_LINE );
    ROS_INFO_STREAM( "vtx:" << vt_x << "vt_y" << vt_y );
    float vn_x, vn_y;
    calculateNormalVelocity( vn_x, vn_y, VIRTUAL_LINE );
    ROS_INFO_STREAM( "vnx:" << vn_x << "vn_y" << vn_y );
    controlDroneVelocity( vt_x + vn_x, vt_y + vn_y, 0.0, 0.0 );
}
void RMChallengeFSM::droneTrackLine()
{
    float vt_x, vt_y;
    float vn_x, vn_y;
    float v_z, yaw;
    // static float goal_height = 10;
    // static float height_threshold = 0.5;
    // static float const_vz = 10;
    calculateTangentialVelocity( vt_x, vt_y, YELLOW_LINE );
    calculateNormalVelocity( vn_x, vn_y, YELLOW_LINE );
    calculateYawRate( yaw );
    if ( fabs( m_current_height_from_guidance - PA_FLYING_HEIGHT ) >
         PA_FLYING_HEIGHT_THRESHOLD )
    {
        v_z = PA_FLYING_HEIGHT > m_current_height_from_guidance ?
                  PA_FLYING_Z_VELOCITY :
                  -PA_FLYING_Z_VELOCITY;
    }
    ROS_INFO_STREAM( "\n t:" << vt_x << "," << vt_y << "\n"
                             << "n:" << vn_x << "," << vn_y << "\n"
                             << "yaw:" << yaw << "\n"
                             << "vz:" << v_z );
    controlDroneVelocity( vt_x + vn_x, vt_y + vn_y, v_z, yaw );
}
void RMChallengeFSM::droneHover()
{
    controlDroneVelocity( 0, 0, 0, 0 );
    ros::Duration( 0.5 ).sleep();
}

void RMChallengeFSM::droneDropDown()
{
    for ( int i = 0; i < 200; i++ )
        controlDroneVelocity( 0.0, 0.0, -1.2, 0.0 );
}

bool RMChallengeFSM::readyToLand()
{
    // static float height_error_threshold = 1;
    // static float landpoint_pos_err_threshold = 0.03;
    // static float goal_land_height = 0;
    float land_err = sqrt( pow( m_landpoint_position_error[0], 2 ) +
                           pow( m_landpoint_position_error[1], 2 ) );
    float height_error;
    if ( m_land_point_type == BASE_LAND_POINT )
    {
        height_error = fabs( PA_LAND_HEIGHT - m_current_height_from_guidance );
        if ( height_error < PA_LAND_HEIGHT_THRESHOLD &&
             land_err < PA_LAND_POSITION_THRESHOLD_LOW )
        {
            ROS_INFO_STREAM( "ready to land at base," << land_err << ","
                                                      << height_error );
            return true;
        }
        else
        {
            ROS_INFO_STREAM( "error too big," << land_err << "," << height_error );
            return false;
        }
    }
    else if ( m_land_point_type == PILLAR_LAND_POINT )
    {
        height_error = fabs( PA_LAND_HEIGHT - m_current_height_from_vision );
        if ( m_prepare_to_land_type == PREPARE_AT_LOW &&
             land_err < PA_LAND_POSITION_THRESHOLD_LOW )
        {
            m_land_counter++;
            if ( m_land_counter >= PA_LAND_COUNT )
            {
                m_prepare_to_land_type = PREPARE_AT_HIGH;
                m_land_counter = 0;
                ROS_INFO_STREAM( "ready to land at pillar ," << land_err << ","
                                                             << height_error );
                return true;
            }
            else
            {
                ROS_INFO_STREAM( "counter not enough," << land_err << ","
                                                       << height_error << ","
                                                       << m_land_counter );
                return false;
            }
        }
        else
        {
            ROS_INFO_STREAM( "error too big," << land_err << "," << height_error );
            return false;
        }
    }
}
void RMChallengeFSM::dronePrepareToLand()
{
    // static float const_vz = 10;
    float vx, vy, vz;
    if ( m_land_point_type == BASE_LAND_POINT )
    {
        // static float goal_height = 10;
        // static float height_threshold = 10;
        // static float kp_base = 10KP_BASE
        if ( fabs( m_current_height_from_guidance - PA_LAND_HEIGHT ) >
             PA_BASE_HEIGHT_THRESHOLD )
        {
            vz = PA_LAND_HEIGHT > m_current_height_from_guidance ?
                     PA_LAND_Z_VELOCITY :
                     -PA_LAND_Z_VELOCITY;
        }
        else
        {
            vz = 0;
        }
        vx = PA_KP_BASE * m_landpoint_position_error[0];
        vy = PA_KP_BASE * m_landpoint_position_error[1];
    }
    else if ( m_land_point_type == PILLAR_LAND_POINT )
    {
        if ( m_discover_pillar_circle )
        {
            navigateByCircle( vx, vy, vz );
        }
        else if ( discoverTriangle() )
        {
            navigateByTriangle( vx, vy, vz );
        }
    }
    ROS_INFO_STREAM( "v are:" << vx << "," << vy << "," << vz );
    controlDroneVelocity( vx, vy, vz, 0.0 );
}
void RMChallengeFSM::navigateByCircle( float &vx, float &vy, float &vz )
{
    if ( m_prepare_to_land_type == PREPARE_AT_HIGH )
    {
        ROS_INFO_STREAM( "navigate high" );
        // static float xy_threshold_high = 0.3;
        // static float pillar_goal_height = 0;
        // static float height_threshold = 0.05;
        // static float v_min = 0.12;
        // static float k_p_pillar_height = 0.3;
        float land_err = sqrt( pow( m_landpoint_position_error[0], 2 ) +
                               pow( m_landpoint_position_error[1], 2 ) );
        if ( land_err > PA_LAND_POSITION_THRESHOLD_HIGH )
        {
            vx = PA_KP_PILLAR_HIGH * m_landpoint_position_error[0];
            vy = PA_KP_PILLAR_HIGH * m_landpoint_position_error[1];
            vz = 0;
            if ( fabs( vx ) < PA_V_MIN_HIGH )
                vx = fabs( vx ) / ( vx + 0.0001 ) * PA_V_MIN_HIGH;
            if ( fabs( vy ) < PA_V_MIN_HIGH )
                vy = fabs( vy ) / ( vy + 0.0001 ) * PA_V_MIN_HIGH;
        }
        else
        {
            vx = vy = 0.0;
            // static float const_vz = 0.15;
            float height_error = PA_LAND_HEIGHT - m_current_height_from_vision;
            if ( fabs( height_error ) > PA_LAND_HEIGHT_THRESHOLD )
            {
                vz = fabs( height_error ) / ( height_error + 0.0000000001 ) *
                     PA_LAND_Z_VELOCITY;
            }
            else
            {
                vz = 0.0;
                droneHover();
                m_prepare_to_land_type = PREPARE_AT_LOW;
            }
        }
    }
    else if ( m_prepare_to_land_type == PREPARE_AT_LOW )
    {
        ROS_INFO_STREAM( "navigate ai low" );
        // static float xy_threshold_high = 0.3;
        // static float v_min = 0.036;
        // static float k_p_pillar_low = 0.3;
        vx = PA_KP_PILLAR_LOW * m_landpoint_position_error[0];
        vy = PA_KP_PILLAR_LOW * m_landpoint_position_error[1];
        vz = 0;
        if ( fabs( vx ) < PA_V_MIN_LOW )
            vx = fabs( vx ) / ( vx + 0.0001 ) * PA_V_MIN_LOW;
        if ( fabs( vy ) < PA_V_MIN_LOW )
            vy = fabs( vy ) / ( vy + 0.0001 ) * PA_V_MIN_LOW;
    }
}
void RMChallengeFSM::navigateByTriangle( float &x, float &y, float &z )
{
    int triangle_sum = m_pillar_triangle[0] + m_pillar_triangle[1] +
                       m_pillar_triangle[2] + m_pillar_triangle[3];
    // static float const_v = 0.15;
    x = y = z = 0.0;
    if ( triangle_sum == 1 )
    {
        if ( m_pillar_triangle[0] == 1 )
        {
            y = -PA_LAND_TRIANGLE_VELOCITY;
        }
        else if ( m_pillar_triangle[1] == 1 )
        {
            x = -PA_LAND_TRIANGLE_VELOCITY;
        }
        else if ( m_pillar_triangle[2] == 1 )
        {
            y = PA_LAND_TRIANGLE_VELOCITY;
        }
        else if ( m_pillar_triangle[3] == 1 )
        {
            x = PA_LAND_TRIANGLE_VELOCITY;
        }
    }
    else if ( triangle_sum == 2 )
    {
        if ( m_pillar_triangle[0] == 1 )
        {
            y = -PA_LAND_TRIANGLE_VELOCITY;
        }
        else if ( m_pillar_triangle[2] == 1 )
        {
            y = PA_LAND_TRIANGLE_VELOCITY;
        }
        if ( m_pillar_triangle[1] == 1 )
        {
            x = -PA_LAND_TRIANGLE_VELOCITY;
        }
        else if ( m_pillar_triangle[3] == 1 )
        {
            x = PA_LAND_TRIANGLE_VELOCITY;
        }
    }
    else if ( triangle_sum == 3 )
    {
        if ( m_pillar_triangle[0] == 0 )
        {
            y = PA_LAND_TRIANGLE_VELOCITY;
        }
        else if ( m_pillar_triangle[1] == 0 )
        {
            x = PA_LAND_TRIANGLE_VELOCITY;
        }
        else if ( m_pillar_triangle[2] == 0 )
        {
            y = -PA_LAND_TRIANGLE_VELOCITY;
        }
        else if ( m_pillar_triangle[3] == 0 )
        {
            x = -PA_LAND_TRIANGLE_VELOCITY;
        }
    }
}
void RMChallengeFSM::unitifiyVector( float &x, float &y )
{
    float unit_x = x;
    float unit_y = y;
    x = unit_x / sqrt( pow( unit_x, 2 ) + pow( unit_y, 2 ) );
    y = unit_y / sqrt( pow( unit_x, 2 ) + pow( unit_y, 2 ) );
    // ROS_INFO_STREAM("vector is :" << x << "," << y);
}
void RMChallengeFSM::calculateNormalVelocity( float &x, float &y,
                                              LINE_TYPE line_type )
{
    // static float k_n = 1;
    if ( line_type == VIRTUAL_LINE )
    {
        static float xc = m_current_position_from_guidance[0];
        static float yc = m_current_position_from_guidance[1];
        static float x0 = m_takeoff_points[m_current_takeoff_point_id][0];
        static float y0 = m_takeoff_points[m_current_takeoff_point_id][1];
        static float xs = m_setpoints[m_current_takeoff_point_id][0];
        static float ys = m_setpoints[m_current_takeoff_point_id][1];
        float t = ( ( xc - x0 ) * ( xs - x0 ) + ( yc - y0 ) * ( ys - y0 ) ) /
                  ( pow( xs - x0, 2 ) + pow( ys - y0, 2 ) );
        float x_n = ( x0 - xc ) + t * ( xs - x0 );
        float y_n = ( y0 - yc ) + t * ( ys - y0 );
        unitifiyVector( x_n, y_n );
        x = PA_KN * x_n;
        y = PA_KN * y_n;
    }
    else if ( line_type == YELLOW_LINE )
    {
        unitifiyVector( m_distance_to_line[0], m_distance_to_line[1] );
        x = PA_KN * m_distance_to_line[0];
        y = PA_KN * m_distance_to_line[1];
    }
}
void RMChallengeFSM::calculateTangentialVelocity( float &x, float &y,
                                                  LINE_TYPE line_type )
{
    // static float k_t = 1;
    if ( line_type == VIRTUAL_LINE )
    {
        static float x0 = m_takeoff_points[m_current_takeoff_point_id][0];
        static float y0 = m_takeoff_points[m_current_takeoff_point_id][1];
        static float xs = m_setpoints[m_current_takeoff_point_id][0];
        static float ys = m_setpoints[m_current_takeoff_point_id][1];
        x = PA_KT * ( xs - x0 ) / sqrt( pow( xs - x0, 2 ) + pow( ys - y0, 2 ) );
        y = PA_KT * ( ys - y0 ) / sqrt( pow( xs - x0, 2 ) + pow( ys - y0, 2 ) );
    }
    else if ( line_type == YELLOW_LINE )
    {
        unitifiyVector( m_line_normal[0], m_line_normal[1] );
        if ( m_current_takeoff_point_id == 3 || m_current_takeoff_point_id == 4 )
        {
            x = -PA_KT * m_line_normal[0];
            y = -PA_KT * m_line_normal[1];
        }
        else  // 0,1,2,5
        {
            x = PA_KT * m_line_normal[0];
            y = PA_KT * m_line_normal[1];
        }
    }
}
void RMChallengeFSM::calculateYawRate( float &yaw )
{
    // static float angle_threshold = 5;
    // static float yaw_rate = 10;
    float angle_to_line_1 = 57.3 * acos( m_line_normal[0] );
    float angle_to_line_2 =
        57.3 * acos( m_line_normal[0] * 0.5 - m_line_normal[1] * sqrt( 0.75 ) );
    ROS_INFO_STREAM( "angle 1 and 2 are :" << angle_to_line_1 << ","
                                           << angle_to_line_2 );
    if ( fabs( angle_to_line_1 ) < fabs( angle_to_line_2 ) )
    {
        if ( fabs( angle_to_line_1 ) > PA_ANGLE_THRESHOLD )
        {
            yaw = m_line_normal[1] > 0 ? -PA_YAW_RATE : PA_YAW_RATE;
        }
    }
    else
    {
        if ( fabs( angle_to_line_2 ) > PA_ANGLE_THRESHOLD )
        {
            float sign = m_line_normal[0] * sqrt( 0.75 ) + m_line_normal[1] * 0.5;
            yaw = sign > 0 ? -PA_YAW_RATE : PA_YAW_RATE;
        }
    }
}
void RMChallengeFSM::setDroneState( int state )
{
    if ( state == 1 )
    {
        m_uav_state = UAV_LAND;
    }
    else if ( state == 3 )
    {
        m_uav_state = UAV_FLY;
    }
    ROS_INFO_STREAM( "uav_state is:" << m_uav_state );
}
void RMChallengeFSM::setHeightFromGuidance( float height )
{
    m_current_height_from_guidance = height;
    ROS_INFO_STREAM( "height from guidance is:" << m_current_height_from_guidance );
}
/**
*set position from guidance
*when on land, bias of guidance position is updated
*when flying, use real time gudance position and bias to
*update actual guidance position
*/
void RMChallengeFSM::setPositionFromGuidance( float x, float y )
{
    if ( m_uav_state == UAV_LAND )
    {
        /* update guidance bias*/
        m_guidance_bias[0] = x - m_current_position_from_guidance[0];
        m_guidance_bias[1] = y - m_current_position_from_guidance[1];
        ROS_INFO_STREAM( "guidance bisa is:" << m_guidance_bias[0] << ","
                                             << m_guidance_bias[1] );
    }
    else if ( m_uav_state == UAV_FLY )
    {
        /* update actual position */
        m_current_position_from_guidance[0] = x - m_guidance_bias[0];
        m_current_position_from_guidance[1] = y - m_guidance_bias[1];
        ROS_INFO_STREAM( "pos from gui is:" << m_current_position_from_guidance[0]
                                            << ","
                                            << m_current_position_from_guidance[1] );
    }
}
void RMChallengeFSM::setCircleVariables( bool is_circle_found,
                                         float position_error[2], float height )
{
    m_discover_pillar_circle = is_circle_found;
    if ( is_circle_found )
    {
        m_landpoint_position_error[0] = position_error[0];
        m_landpoint_position_error[1] = position_error[1];
        m_current_height_from_vision = height;
    }
    else
    {
        m_landpoint_position_error[0] = m_landpoint_position_error[1] =
            m_current_height_from_vision = 0;
    }
    ROS_INFO_STREAM( "circle var is:" << m_discover_pillar_circle << ","
                                      << m_landpoint_position_error[0] << ","
                                      << m_landpoint_position_error[1] << ","
                                      << m_current_height_from_vision );
}
void RMChallengeFSM::setTriangleVariables( int pillar_triangle[4] )
{
    for ( int i = 0; i < 4; ++i )
    {
        m_pillar_triangle[i] = pillar_triangle[i];
    }
    ROS_INFO_STREAM( "triangle is:"
                     << pillar_triangle[0] << "," << pillar_triangle[1] << ","
                     << pillar_triangle[2] << "," << pillar_triangle[3] );
}
void RMChallengeFSM::setBaseVariables( bool is_base_found, float position_error[2] )
{
    m_discover_base = is_base_found;
    if ( is_base_found )
    {
        m_landpoint_position_error[0] = position_error[0];
        m_landpoint_position_error[1] = position_error[1];
    }
    else
    {
        m_landpoint_position_error[0] = 0;
        m_landpoint_position_error[1] = 0;
    }
    ROS_INFO_STREAM( "base var is:" << m_discover_base << ","
                                    << m_landpoint_position_error[0] << ","
                                    << m_landpoint_position_error[1] );
}
void RMChallengeFSM::setLineVariables( float distance_to_line[2],
                                       float line_normal[2] )
{
    m_distance_to_line[0] = distance_to_line[0];
    m_distance_to_line[1] = distance_to_line[1];
    m_line_normal[0] = line_normal[0];
    m_line_normal[1] = line_normal[1];
    ROS_INFO_STREAM( "distance to line is:" << m_distance_to_line[0] << ","
                                            << m_distance_to_line[1]
                                            << ",line vector is" << m_line_normal[0]
                                            << "," << m_line_normal[1] );
}
