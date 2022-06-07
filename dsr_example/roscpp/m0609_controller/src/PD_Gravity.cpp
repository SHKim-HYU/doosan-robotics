/*
 * PD_Gravity.cpp
 *
 *  Created on: Jun 4, 2022
 *      Author: Sunhong Kim
 */


// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
#include <kdl/chainfksolvervel_recursive.hpp> // forward kinematics


#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <cmath>

#define _USE_MATH_DEFINES

#include "Trajectory.h"

#include "DRFLEx.h"
using namespace DRAFramework;

#define R2D 180/M_PI
#define D2R M_PI/180
#define num_taskspace 6
#define SaveDataMax 97

namespace  m0609_controller
{
    // class PD_Gravity : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    class PD_Gravity : public controller_interface::Controller<hardware_interface::PositionJointInterface>
    {
    public:
        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
        {
            // ********* 1. Get joint name / gain from the parameter server *********
            // 1.0 Control objective & Inverse Kinematics mode
            ROS_INFO("#######Initialize start##########");
            if (!n.getParam("ctr_obj", ctr_obj_))
            {
                ROS_ERROR("Could not find control objective");
                return false;
            }

            if (!n.getParam("ik_mode", ik_mode_))
            {
                ROS_ERROR("Could not find control objective");
                return false;
            }

            // 1.1 Joint Name
            if (!n.getParam("joints", joint_names_))
            {
                ROS_ERROR("Could not find joint name");
                return false;
            }
            n_joints_ = joint_names_.size();
            ROS_INFO("#########joint info is obtained########");
            if (n_joints_ == 0)
            {
                ROS_ERROR("List of joint names is empty.");
                return false;
            }
            else
            {
                ROS_INFO("Found %d joint names", n_joints_);
                for (int i = 0; i < n_joints_; i++)
                {
                    ROS_INFO("%s", joint_names_[i].c_str());
                }
            }

            // 1.2 Gain
            // 1.2.1 Joint Controller
            Kp_.resize(n_joints_);
            Kd_.resize(n_joints_);
            Ki_.resize(n_joints_);


            std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);

            for (size_t i = 0; i < n_joints_; i++)
            {
                std::string si = std::to_string(i + 1);
                if (n.getParam("/dsr01/PD_Gravity/gains/joint" + si + "/pid/p", Kp[i]))
                {
                    Kp_(i) = Kp[i];
                }
                else
                {
                    std::cout << "/dsr01/PD_Gravity/gains/joint" + si + "/pid/p" << std::endl;
                    ROS_ERROR("Cannot find pid/p gain");
                    return false;
                }

                if (n.getParam("/dsr01/PD_Gravity/gains/joint" + si + "/pid/i", Ki[i]))
                {
                    Ki_(i) = Ki[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/i gain");
                    return false;
                }

                if (n.getParam("/dsr01/PD_Gravity/gains/joint" + si + "/pid/d", Kd[i]))
                {
                    Kd_(i) = Kd[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/d gain");
                    return false;
                }
            }

            // 2. ********* urdf *********
            urdf::Model urdf;
            if (!urdf.initParam("/dsr01/robot_description"))
            {
                ROS_ERROR("Failed to parse urdf file");
                return false;
            }
            else
            {
                ROS_INFO("Found robot_description");
            }

            // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
            for (int i = 0; i < n_joints_; i++)
            {
                try
                {
                    joints_.push_back(hw->getHandle(joint_names_[i]));
                }
                catch (const hardware_interface::HardwareInterfaceException &e)
                {
                    ROS_ERROR_STREAM("Exception thrown: " << e.what());
                    return false;
                }

                urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
                if (!joint_urdf)
                {
                    ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                    return false;
                }
                joint_urdfs_.push_back(joint_urdf);
            }

            // 4. ********* KDL *********
            // 4.1 kdl parser
            if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
            {
                ROS_ERROR("Failed to construct kdl tree");
                return false;
            }
            else
            {
                ROS_INFO("Constructed kdl tree");
            }

            // 4.2 kdl chain
            std::string root_name, tip_name1;
            if (!n.getParam("root_link", root_name))
            {
                ROS_ERROR("Could not find root link name");
                return false;
            }
            if (!n.getParam("tip_link1", tip_name1))
            {
                ROS_ERROR("Could not find tip link name");
                return false;
            }
            if (!kdl_tree_.getChain(root_name, tip_name1, kdl_chain_))
            {
                ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
                ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name1);
                ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
                ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
                ROS_ERROR_STREAM("  The segments are:");

                KDL::SegmentMap segment_map = kdl_tree_.getSegments();
                KDL::SegmentMap::iterator it;

                for (it = segment_map.begin(); it != segment_map.end(); it++)
                    ROS_ERROR_STREAM("    " << (*it).first);

                return false;
            }
            else
            {
                ROS_INFO("Got kdl chain");
            }
            gravity_ = KDL::Vector::Zero();
            gravity_(2) = -9.81; // 0: x-axis 1: y-axis 2: z-axis

            M_.resize(kdl_chain_.getNrOfJoints());
            C_.resize(kdl_chain_.getNrOfJoints());
            G_.resize(kdl_chain_.getNrOfJoints());

            id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

            jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
            fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

            // ********* 5. 각종 변수 초기화 *********

            // 5.1 KDL Vector 초기화 (사이즈 정의 및 값 0)
            x_cmd_.data = Eigen::VectorXd::Zero(num_taskspace);

            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qdot_.data = Eigen::VectorXd::Zero(n_joints_);

            q_chain[0].data = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());

            J_.resize(kdl_chain_.getNrOfJoints());

            TrajFlag_j.resize(n_joints_);
            TrajFlag_j.setZero();

            res_trad.resize(6);

            // ********* 6. ROS 명령어 *********
            // 6.1 publisher
            pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
            pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
            pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

            pub_xd_ = n.advertise<std_msgs::Float64MultiArray>("xd", 1000);
            pub_x_ = n.advertise<std_msgs::Float64MultiArray>("x", 1000);
            pub_ex_ = n.advertise<std_msgs::Float64MultiArray>("ex", 1000);

            pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000);

            // 6.2 subsriber
            sub_x_cmd_ = n.subscribe<std_msgs::Float64MultiArray>("command",
                    1, &PD_Gravity::commandCB, this);
            event = 0; // subscribe 받기 전: 0
            // subscribe 받은 후: 1

            return true;
        }

        void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
        {
            if (msg->data.size() != num_taskspace)
            {
                ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match DOF of Task Space (" << 2 << ")! Not executing!");
                return;
            }

            for (int i = 0; i < num_taskspace; i++)
            {
                x_cmd_(i) = msg->data[i];
            }

            event = 1;  // subscribe 받기 전: 0
                        // subscribe 받은 후: 1
        }

        void starting(const ros::Time &time) override
        {
            t = 0.0;
            ROS_INFO("Starting Computed Torque Controller with Closed-Loop Inverse Kinematics");
            //xd={0.048723,-0.30684,1.23653};

            qd.setZero();
            qd_dot.setZero();
            qd_old.setZero();

            q_flag=0;
            traj5th_joint = new hyuCtrl::Trajectory();
            ROS_INFO("Trajectory instance has made");
            // Drfl = new CDRFLEx;
            Drfl->set_robot_mode(ROBOT_MODE_AUTONOMOUS);

            // Connecting
            cout << "Connecting..." << endl;
            Drfl->connect_rt_control("192.168.12.249");

            // Setting
            cout << "Setting..." << endl;
            string version = "v1.0";
            float period = 0.001;
            int losscount = 4;
            Drfl->set_rt_control_output(version, period, losscount);
            cout << "Set." << endl;

            // Starting
            cout << "Starting..." << endl;
        	  Drfl->start_rt_control();

            memcpy(q, Drfl->read_data_rt()->actual_joint_position, sizeof(float)*NUMBER_OF_JOINT);
            memcpy(qdot, Drfl->read_data_rt()->actual_joint_velocity, sizeof(float)*NUMBER_OF_JOINT);
            memcpy(torque_g, Drfl->read_data_rt()->gravity_torque, sizeof(float)*NUMBER_OF_JOINT);

            for (int i = 0; i < n_joints_; i++)
            {
                // joints_[i].setCommand(torque_g[i]);
                joints_[i].setCommand(qd[i]);

            }
            ROS_INFO("Starting is done");
        }

        void update(const ros::Time &time, const ros::Duration &period) override
        {
            // ********* 0. Get states from gazebo *********
            // 0.1 sampling time
            double dt = period.toSec();
            t = t + 0.001;

            // 0.2 joint state
            memcpy(q, Drfl->read_data_rt()->actual_joint_position, sizeof(float)*NUMBER_OF_JOINT);
            memcpy(qdot, Drfl->read_data_rt()->actual_joint_velocity, sizeof(float)*NUMBER_OF_JOINT);
            memcpy(torque_g, Drfl->read_data_rt()->gravity_torque, sizeof(float)*NUMBER_OF_JOINT);




            //Control->InvDynController(q, qdot, dq, dqdot, dqddot, torque, dt);
            //torque[0]=-5.0;torque[1]=3.0;torque[2]=0.0;torque[3]=2.0;torque[4]=0.0;



            id_solver_->JntToMass(q_, M_);
            id_solver_->JntToCoriolis(q_, qdot_, C_);
            id_solver_->JntToGravity(q_, G_); // output은 머지? , id_solver는 어디에서?
            /////////////Trajectory for Joint Space//////////////
            if(Motion==1 &&TrajFlag_j(0)==0)
            {
                traj_q[0]=0.3; traj_q[1]=0.3; traj_q[2]= 0.3; traj_q[3]=0.3; traj_q[4]=0.3; traj_q[5]=0.3;
                Motion++;
                for(int i=0;i<n_joints_;i++)
                {
                    TrajFlag_j(i)=1;
                }
            }
            else if(Motion==2 &&TrajFlag_j(0)==0)
            {
                traj_q[0]=0.0; traj_q[1]=0.0; traj_q[2]= 0.0; traj_q[3]=0.0; traj_q[4]=0.0; traj_q[5]=0.0;
                Motion++;
                for(int i=0;i<n_joints_;i++)
                {
                    TrajFlag_j(i)=1;
                }
            }
            else if(Motion==3 &&TrajFlag_j(0)==0)
            {
                traj_q[0]=-0.3; traj_q[1]=-0.3; traj_q[2]= -0.3; traj_q[3]=-0.3; traj_q[4]=-0.3; traj_q[5]=-0.3;
                Motion=1;
                for(int i=0;i<n_joints_;i++)
                {
                    TrajFlag_j(i)=1;
                }
            }
            for (int i = 0; i < n_joints_; i++) {
                if (TrajFlag_j(i)==2)
                {
                    traj5th_joint->Polynomial5th(i, t, &TrajFlag_j(i), res_tra);
                    qd[i] = res_tra[0];
                    qd_dot[i] = res_tra[1];
                    qd_ddot[i]=res_tra[2];
                }
                else if(TrajFlag_j(i)==1) {
                    traj5th_joint->SetPolynomial5th(i, q[i], traj_q(i), t, 2.0, res_tra);
                    qd[i] = res_tra[0];
                    qd_dot[i] = res_tra[1];
                    qd_ddot[i]=res_tra[2];
                    TrajFlag_j(i)=2;
                }
            }


               for(int i=0;i<n_joints_;i++)
               {
                   dq[i]=qd[i];
                   dqdot[i]=qd_dot[i];
               }
            /*
            if(traj_flag==0)
                Control->PD_Gravity(q, qdot, qd, qd_dot, torque);
            else
                Control->PD_Gravity(q, qdot, qd, qd_dot, torque);
            */
            for (int i = 0; i < n_joints_; i++)
            {
                // joints_[i].setCommand(torque_g[i]);
                joints_[i].setCommand(qd[i]);


                //joints_[i].setCommand(0.0);
            }

            // ********* 4. data 저장 *********
            save_data();

            // ********* 5. state 출력 *********
            print_state();
        }

        void stopping(const ros::Time &time) override
        {
            delete Drfl;
            delete traj5th_joint;
        }

        static void save_data()
        {

        }

        void print_state()
        {
            static int count = 0;
            if (count > 99)
            {
                printf("*********************************************************\n\n");
                printf("*** Simulation Time (unit: sec)  ***\n");
                printf("t = %f\n", t);
                printf("\n");

                printf("*** Command from Subscriber in Task Space (unit: m) ***\n");
                if (event == 0)
                {
                    printf("No Active!!!\n");
                }
                else
                {
                    printf("Active!!!\n");
                }

                printf("*** States in Joint Space (unit: deg) ***\n");
                for(int i=0; i < n_joints_; i++)
                {
                    printf("Joint ID:%d \t", i);
//                    printf("q: %0.3f, ", q_(i) * R2D);
//                    printf("dq: %0.3f, ", qd_(i) * R2D);
//                    printf("qdot: %0.3f, ", qdot_(i) * R2D);
//                    printf("dqdot: %0.3f, ", qd_dot_(i) * R2D);
                    printf("q: %0.3f, ", q_(i));
                    printf("dq: %0.3f, ", qd(i));
                    printf("qdot: %0.3f, ", qdot_(i));
                    printf("dqdot: %0.3f, ", qd_dot(i));
                    printf("torque: %0.3f", torque_d[i]);
                    printf("\n");
                }

                count = 0;
            }
            count++;
        }

    private:
        // others
        double t;
        int ctr_obj_;
        int ik_mode_;
        int event;

        //Joint handles
        unsigned int n_joints_;                               // joint 숫자
        std::vector<std::string> joint_names_;                // joint name ??
        std::vector<hardware_interface::JointHandle> joints_; // ??
        std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

        // kdl
        KDL::Tree kdl_tree_;   // tree?
        KDL::Chain kdl_chain_; // chain?

        // kdl M,C,G
        KDL::JntSpaceInertiaMatrix M_; // intertia matrix
        KDL::JntArray C_;              // coriolis
        KDL::JntArray G_;              // gravity torque vector
        KDL::Vector gravity_;

        // kdl and Eigen Jacobian
        KDL::Jacobian J_;
        //KDL::Jacobian J_inv_;
        //Eigen::Matrix<double, num_taskspace, num_taskspace> J_inv_;
        Eigen::MatrixXd J_inv_;
        Eigen::Matrix<double, num_taskspace, num_taskspace> J_transpose_;

        // kdl solver
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
        boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_; //Solver to compute the forward kinematics (velocity)
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Solver to compute the jacobian
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_;               // Solver To compute the inverse dynamics

        // Joint Space State
        KDL::JntArray qd_, qd_dot_, qd_ddot_;
        KDL::JntArray qd_old_;
        KDL::JntArray q_, qdot_;

        KDL::JntArray q_chain[2];
        KDL::JntArray qdot_chain[2];

        MatrixXd q_dot;

        double q[NUMBER_OF_JOINT], qdot[NUMBER_OF_JOINT];
        double dq[NUMBER_OF_JOINT] = {0.0,};
        double dqdot[NUMBER_OF_JOINT] = {0.0,};
        double dqddot[NUMBER_OF_JOINT] = {0.0,};
        double torque_g[NUMBER_OF_JOINT] = {0.0,};
        double torque_d[NUMBER_OF_JOINT] = {0.0,};
        double torque_d_tmp[NUMBER_OF_JOINT] = {0.0,};


        Matrix<double,6,1> qd;
        Matrix<double,6,1> qd_old;
        Matrix<double,6,1> qd_dot;
        Matrix<double,6,1> qd_dot_old;
        Matrix<double,6,1> qd_ddot;
        KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
        KDL::Frame x_;
        KDL::Twist ex_temp_;
        int q_flag;

        // Input
        KDL::JntArray x_cmd_;

        //Trajectory
        int Motion = 1;
        VectorXi TrajFlag_j;

        Matrix<double,6,1> traj_q;
        Vector3d traj_x;
        VectorXd traj_xd, res_trad;
        double res_tra[3]={0.0,};
        Matrix3d ROT, ROTD;
        int traj_flag=0;
        double t_buf;
        int t_flag=0;

        // gains
        KDL::JntArray Kp_, Ki_, Kd_;
        double K_regulation_, K_tracking_;

        // save the data
        double SaveData_[SaveDataMax];

        // ros subscriber
        ros::Subscriber sub_x_cmd_;

        // ros publisher
        ros::Publisher pub_qd_, pub_q_, pub_e_;
        ros::Publisher pub_xd_, pub_x_, pub_ex_;
        ros::Publisher pub_SaveData_;

        // ros message
        std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
        std_msgs::Float64MultiArray msg_xd_, msg_x_, msg_ex_;
        std_msgs::Float64MultiArray msg_SaveData_;

        hyuCtrl::Trajectory *traj5th_joint;
        CDRFLEx *Drfl;
        bool g_bHasControlAuthority = FALSE;
        bool g_TpInitailizingComplted = FALSE;
        bool g_mStat = FALSE;
        bool g_Stop = FALSE;
        bool moving = FALSE;
        bool bAlterFlag = FALSE;
    };
}

PLUGINLIB_EXPORT_CLASS(m0609_controller::PD_Gravity,controller_interface::ControllerBase)
