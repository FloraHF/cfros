#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <crazyflie_controller/Mocap.h>


#include "pid.hpp"


double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}



class Controller

{

public:

    Controller(
        // const std::string& worldFrame,
        // const std::string& frame,
        const ros::NodeHandle& n)
        : m_pubNav()
        , m_pubCmdVtemp()
        , m_pidX(
            get(n, "PIDs/X/kp"),
            get(n, "PIDs/X/kd"),
            get(n, "PIDs/X/ki"),
            get(n, "PIDs/X/minOutput"),
            get(n, "PIDs/X/maxOutput"),
            get(n, "PIDs/X/integratorMin"),
            get(n, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(n, "PIDs/Y/kp"),
            get(n, "PIDs/Y/kd"),
            get(n, "PIDs/Y/ki"),
            get(n, "PIDs/Y/minOutput"),
            get(n, "PIDs/Y/maxOutput"),
            get(n, "PIDs/Y/integratorMin"),
            get(n, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(n, "PIDs/Z/kp"),
            get(n, "PIDs/Z/kd"),
            get(n, "PIDs/Z/ki"),
            get(n, "PIDs/Z/minOutput"),
            get(n, "PIDs/Z/maxOutput"),
            get(n, "PIDs/Z/integratorMin"),
            get(n, "PIDs/Z/integratorMax"),
            "z")
        , m_pidVx(
            get(n, "PIDs/Vx/kp"),
            get(n, "PIDs/Vx/kd"),
            get(n, "PIDs/Vx/ki"),
            get(n, "PIDs/Vx/minOutput"),
            get(n, "PIDs/Vx/maxOutput"),
            get(n, "PIDs/Vx/integratorMin"),
            get(n, "PIDs/Vx/integratorMax"),
            "x")
        , m_pidVy(
            get(n, "PIDs/Vy/kp"),
            get(n, "PIDs/Vy/kd"),
            get(n, "PIDs/Vy/ki"),
            get(n, "PIDs/Vy/minOutput"),
            get(n, "PIDs/Vy/maxOutput"),
            get(n, "PIDs/Vy/integratorMin"),
            get(n, "PIDs/Vy/integratorMax"),
            "x")
        , m_pidVz(
            get(n, "PIDs/Vz/kp"),
            get(n, "PIDs/Vz/kd"),
            get(n, "PIDs/Vz/ki"),
            get(n, "PIDs/Vz/minOutput"),
            get(n, "PIDs/Vz/maxOutput"),
            get(n, "PIDs/Vz/integratorMin"),
            get(n, "PIDs/Vz/integratorMax"),
            "x")                                
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state(Idle)
        , m_goal()
        , m_cmdV()
        , m_dronePositionWorld()
        , m_droneVelocityWorld()
        , m_droneEuler()
        , m_Cbe()
        , m_subscribeGoal()
        , m_subscribeCmdV()
        , m_subscribeDroneState()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_serviceGame()
        , m_thrust(0)
        , m_startZ(0)
        , m_trimThrust(43000)
    {
        ros::NodeHandle nh;
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_pubCmdVtemp = nh.advertise<geometry_msgs::Twist>("cmdVtemp", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        m_subscribeCmdV = nh.subscribe("cmdV", 1, &Controller::cmdVChanged, this);
        m_subscribeDroneState = nh.subscribe("mocap", 1, &Controller::droneMoved, this);
        m_serviceTakeoff = nh.advertiseService("cftakeoff", &Controller::takeoff, this);
        m_serviceGame = nh.advertiseService("cfplay", &Controller::play, this);
        m_serviceLand = nh.advertiseService("cfland", &Controller::land, this);
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }


private:
    void goalChanged(
        const geometry_msgs::PoseStamped& msg)
    {
        m_goal[0] = msg.pose.position.x;
        m_goal[1] = msg.pose.position.y;
        m_goal[2] = msg.pose.position.z;
    }

    void cmdVChanged(
        const geometry_msgs::Twist& msg)
    {
        m_cmdV[0] = msg.linear.x;
        m_cmdV[1] = msg.linear.y;
        m_cmdV[2] = msg.linear.z;
    }

    void droneMoved(
        const crazyflie_controller::Mocap& msg)
    {
        m_dronePositionWorld[0] = msg.position[0];
        m_dronePositionWorld[1] = msg.position[1];
        m_dronePositionWorld[2] = msg.position[2];
        m_droneVelocityWorld[0] = msg.velocity[0];
        m_droneVelocityWorld[1] = msg.velocity[1];
        m_droneVelocityWorld[2] = msg.velocity[2];
        m_droneEuler = quaternion_to_euler_w(msg);
        m_Cbe = quaternion_to_Cbe(msg);
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;
        m_startZ = m_dronePositionWorld[2];
        return true;
    }


    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;
        return true;
    }

    bool play(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("playing requested!");
        m_state = Playing;
        return true;
    }

    void pidXYZReset()
    {
        m_pidX.reset();
        m_pidY.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    void pidVReset()
    {
        m_pidVx.reset();
        m_pidVy.reset();
        m_pidVz.reset();
    }

    void pidReset()
    {
        pidXYZReset();
        pidVReset();
    }

    Eigen::Vector3d quaternion_to_euler_w(const crazyflie_controller::Mocap& mocap)
    {
       float quat[4];
       quat[0] = mocap.quaternion[0];
       quat[1] = mocap.quaternion[1];
       quat[2] = mocap.quaternion[2];
       quat[3] = mocap.quaternion[3];
       Eigen::Vector3d ans;
       ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
       ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
       ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
       return ans;
    }

    Eigen::Matrix3d quaternion_to_Cbe(const crazyflie_controller::Mocap& mocap)
    {
        Eigen::Matrix3d m_Cbe;
        Eigen::Matrix3d Ceb;
        Eigen::Matrix<double, 3, 4> L;// Quaternion auxiliary matirx
        Eigen::Matrix<double, 3, 4> R;// Quaternion auxiliary matirx
        double q0, q1, q2, q3;
        q0 = mocap.quaternion[0];
        q1 = mocap.quaternion[1];
        q2 = mocap.quaternion[2];
        q3 = mocap.quaternion[3];
         // update the auxiliary matrix
        /*
        L = [-q1 q0 q3 -q2;
             -q2 -q3 q0 q1;
             -q3 q2 -q1 q0]
        R = [-q1 q0 -q3 q2;
             -q2 q3 q0 -q1;
             -q3 -q2 q1 q0]
        R_IB = RL^T
        */
        L(0,0) = - q1;
        L(1,0) = - q2;
        L(2,0) = - q3;


        L(0,1) = q0;
        L(1,2) = q0;
        L(2,3) = q0;

        L(0,2) = q3;
        L(0,3) = - q2;
        L(1,1) = - q3;
        L(1,3) = q1;
        L(2,1) = q2;
        L(2,2) = - q1;

        R(0,0) = - q1;
        R(1,0) = - q2;
        R(2,0) = - q3;

        R(0,1) = q0;
        R(1,2) = q0;
        R(2,3) = q0;

        R(0,2) = - q3;
        R(0,3) =  q2;
        R(1,1) =  q3;
        R(1,3) = - q1;
        R(2,1) = - q2;
        R(2,2) =  q1;

        Ceb = R * L.transpose();
        m_Cbe = Ceb.transpose();

        return m_Cbe;
    }

    void iteration(const ros::TimerEvent& e)
    {

        float dt = e.current_real.toSec() - e.last_real.toSec();
        switch(m_state)
        {
        case TakingOff:
            {
                if (m_dronePositionWorld[2] > m_startZ + 0.05 || m_thrust > m_trimThrust)
                {
                    pidReset();
                    m_pidZ.setIntegral((m_thrust-m_trimThrust)/m_pidZ.ki());
                    m_state = Automatic;
                    m_thrust = m_trimThrust;
                    ROS_INFO("Automatic!");
                }
                else
                {
                    m_thrust += 12000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }

            }
            break;
        case Landing:
            {
                 m_goal[0] = m_dronePositionWorld[0];
                 m_goal[1] = m_dronePositionWorld[1];
                 m_goal[2] = m_startZ + 0.05;
                if (m_dronePositionWorld[2] <= m_startZ + 0.1) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }

            // intentional fall-thru
        case Automatic:
            {
//                std::cout<<"automatic";
//                pidVReset();
                Eigen::Vector3d positionErr = m_goal - m_dronePositionWorld;

                Eigen::Vector3d cmdV;
                cmdV[0] = m_pidX.update(0.0, positionErr[0]);
                cmdV[1] = m_pidY.update(0.0, positionErr[1]);
                cmdV[2] = m_pidZ.update(0.0, positionErr[2]);
//                std::cout<<"cmdVz, dz\t"<<cmdV[2]<<"\t\t"<<positionErr[2]<<std::endl;

                geometry_msgs::Twist cmdVtemp;
                cmdVtemp.linear.x = cmdV[0];
                cmdVtemp.linear.y = cmdV[1];
                cmdVtemp.linear.z = cmdV[2];
//                ROS_INFO("publish intermeidate velocity cmd");
                m_pubCmdVtemp.publish(cmdVtemp);

                Eigen::Vector3d velocityErrBody = m_Cbe*(cmdV - m_droneVelocityWorld);
                geometry_msgs::Twist msg;
                msg.linear.x = m_pidVx.update(0, velocityErrBody[0]);
                msg.linear.y = m_pidVy.update(0, velocityErrBody[1]);
                msg.linear.z =  m_pidVz.update(0, velocityErrBody[2]) + m_trimThrust;
                msg.angular.z = m_pidYaw.update(0.0, m_droneEuler[2]);
                m_pubNav.publish(msg);
            }
            break;

        case Playing:
            {
                Eigen::Vector3d positionErr = m_goal - m_dronePositionWorld;
                Eigen::Vector3d cmdV;
                cmdV[0] = m_cmdV[0];
                cmdV[1] = m_cmdV[1];
                cmdV[2] = m_pidZ.update(0.0, positionErr[2]);

                Eigen::Vector3d velocityErrBody = m_Cbe*(cmdV - m_droneVelocityWorld);
                geometry_msgs::Twist msg;
                msg.linear.x = m_pidVx.update(0, velocityErrBody[0]);
                msg.linear.y = m_pidVy.update(0, velocityErrBody[1]);
                msg.linear.z = m_pidVz.update(0, velocityErrBody[2]) + m_trimThrust;
                msg.angular.z = m_pidYaw.update(0.0, m_droneEuler[2]);
                m_pubNav.publish(msg);                
            }
            break;

        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        }
    }

private:
    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
        Playing = 4,
    };

private:
    ros::Publisher m_pubNav;
    ros::Publisher m_pubCmdVtemp;
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidVx;
    PID m_pidVy;
    PID m_pidVz;
    PID m_pidYaw;
    State m_state;
    Eigen::Vector3d m_goal;
    Eigen::Vector3d m_cmdV;
    Eigen::Vector3d m_dronePositionWorld;
    Eigen::Vector3d m_droneVelocityWorld;
    Eigen::Vector3d m_droneEuler;
    Eigen::Matrix3d m_Cbe;
    ros::Subscriber m_subscribeGoal;
    ros::Subscriber m_subscribeCmdV;
    ros::Subscriber m_subscribeDroneState;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    ros::ServiceServer m_serviceGame;
    float m_thrust;
    float m_trimThrust;
    float m_startZ;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  // Read parameters
  ros::NodeHandle n("~");
  // std::string worldFrame;
  // n.param<std::string>("worldFrame", worldFrame, "/world");
  // std::string frame;
  // n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 50.0);

  // Controller controller(worldFrame, frame, n);
  Controller controller(n);
  controller.run(frequency);

  return 0;

}
