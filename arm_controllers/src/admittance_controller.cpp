#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>
#include <geometry_msgs/WrenchStamped.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

#include <boost/scoped_ptr.hpp>

// from computed torque clik
#include <boost/lexical_cast.hpp>
//
#include <math.h>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

//#define SaveDataMax 97
#define num_taskspace 6
#define A 0.1
#define b 2.5
#define f 1
#define t_set 1

#include <string>
#include <iostream>
//



#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI
#define JointMax 6
#define SaveDataMax 7

namespace arm_controllers{

class AdmittanceController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    ~AdmittanceController() {sub_q_cmd_.shutdown(); sub_forcetorque_sensor_.shutdown();}

    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
    {
        // List of controlled joints
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n_joints_ = joint_names_.size();

        if(n_joints_ == 0)
        {
            ROS_ERROR("List of joint names is empty.");
            return false;
        }

        // urdf
        urdf::Model urdf;
        if (!urdf.initParam("elfin/robot_description"))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }

        // joint handle
        for(int i=0; i<n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException& e)
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

        // kdl parser
        if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_)){
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }

        // kdl chain
        std::string root_name, tip_name;
        if (!n.getParam("root_link", root_name))
        {
            ROS_ERROR("Could not find root link name");
            return false;
        }
        if (!n.getParam("tip_link", tip_name))
        {
            ROS_ERROR("Could not find tip link name");
            return false;
        }
        if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
            ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
            ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for( it=segment_map.begin(); it != segment_map.end(); it++ )
                ROS_ERROR_STREAM( "    "<<(*it).first);

            return false;
        }

        gravity_ = KDL::Vector::Zero();
        gravity_(2) = -9.81;
        G_.resize(n_joints_);

        // inverse dynamics solver
        id_solver_.reset( new KDL::ChainDynParam(kdl_chain_, gravity_) );
        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        //ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, fk_solver_, ik_vel_solver_));

        // command and state
        tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
        tau_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
        q_cmd_sp_.data = Eigen::VectorXd::Zero(n_joints_);
        q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
        q_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
        qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
        q_cmd_end_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        q_init_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_old_.data = Eigen::VectorXd::Zero(n_joints_);
        qddot_.data = Eigen::VectorXd::Zero(n_joints_);

        for (size_t i = 0; i < 6; i++)
        {
            Xc_dot_(i) = 0.0;
            Xc_dot_old_(i) = 0.0;
            Xc_ddot_(i) = 0.0;
        }

        // gains
        Mbar_.resize(n_joints_);
        Mbar_dot_.resize(n_joints_);
        Ramda_.resize(n_joints_);
        Alpha_.resize(n_joints_);
        Omega_.resize(n_joints_);

        Xr_dot_ = 0.0;
        Xe_dot_ = 0.0;
        Xe_ddot_ = 0.0;
        Fd_ = 0.0;
        Fd_temp_ = 0.0;
        Fd_old_ = 0.0;
        Fe_ = 0.0;
        Fe_old_ = 0.0;
        M_ = 0.0;
        B_ = 0.0;
        del_B_ = 0.0;
        B_buffer_ = 0.0;
        PI_ = 0.0;
        PI_old_ = 0.0;

        filt_old_ = 0.0;
        filt_ = 0.0;
        tau_ = 1.0/(2*PI*9.0);

        f_cur_buffer_ = 0.0;

        experiment_mode_ = 0;

        std::vector<double> Mbar(n_joints_), Ramda(n_joints_), Alpha(n_joints_), Omega(n_joints_);
        for (size_t i=0; i<n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i+1);
            if ( n.getParam("/elfin/admittance_controller/joint" + si + "/tdc/mbar", Mbar[i]) )
            {
                Mbar_(i) = Mbar[i];
            }
            else
            {
                std::cout << "/elfin/admittance_controller/joint" + si + "/tdc/mbar" << std::endl;
                ROS_ERROR("Cannot find tdc/mbar gain");
                return false;
            }

            if ( n.getParam("/elfin/admittance_controller/joint" + si + "/tdc/r", Ramda[i]) )
            {
                Ramda_(i) = Ramda[i];
            }
            else
            {
                ROS_ERROR("Cannot find tdc/r gain");
                return false;
            }

            if ( n.getParam("/elfin/admittance_controller/joint" + si + "/tdc/a", Alpha[i]) )
            {
                Alpha_(i) = Alpha[i];
            }
            else
            {
                ROS_ERROR("Cannot find tdc/a gain");
                return false;
            }

            if ( n.getParam("/elfin/admittance_controller/joint" + si + "/tdc/w", Omega[i]) )
            {
                Omega_(i) = Omega[i];
            }
            else
            {
                ROS_ERROR("Cannot find tdc/w gain");
                return false;
            }
        }

        if (!n.getParam("/elfin/admittance_controller/aic/fd", Fd_temp_))
        {
            ROS_ERROR("Cannot find aci/fd");
            return false;
        }

        if (!n.getParam("/elfin/admittance_controller/aic/m", M_))
        {
            ROS_ERROR("Cannot find aci/m");
            return false;
        }

        if (!n.getParam("/elfin/admittance_controller/aic/b", B_))
        {
            ROS_ERROR("Cannot find aci/b");
            return false;
        }

        if (!n.getParam("/elfin/admittance_controller/mode", experiment_mode_))
        {
            ROS_ERROR("Cannot find mode");
            return false;
        }

        // command
        sub_q_cmd_ = n.subscribe("command", 1, &AdmittanceController::commandCB, this);
        sub_forcetorque_sensor_ = n.subscribe<geometry_msgs::WrenchStamped>("/elfin/elfin/ft_sensor_topic", 1, &AdmittanceController::updateFTsensor, this);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000);


        // ---------------------from Control clik-------------

        // ********* 1.  joint name / gain from the parameter server *********
        // 1.0 Control objective & Inverse Kinematics mode
        if (!n.getParam("ctr_obj", cctr_obj_))
        {
            ROS_ERROR("Could not find control objective");
            return false;
        }

        if (!n.getParam("ik_mode", iik_mode_))
        {
            ROS_ERROR("Could not find control objective");
            return false;
        }

        // 1.1 Joint Name
        if (!n.getParam("joints", jjoint_names_))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }

        // had to subtract 1 manually. Sorry!
        nn_joints_ = jjoint_names_.size()-1;

        if (nn_joints_ == 0)
        {
            ROS_ERROR("List of joint names is empty.");
            return false;
        }
        else
        {
            ROS_INFO("Found %d joint names", nn_joints_);
            for (int i = 0; i < nn_joints_; i++)
            {
                ROS_INFO("%s", jjoint_names_[i].c_str());
            }
        }

        // 1.2 Gain
        // 1.2.1 Joint Controller
        KKp_.resize(nn_joints_);
        KKd_.resize(nn_joints_);
        KKi_.resize(nn_joints_);

        std::vector<double> KKp(nn_joints_), KKi(nn_joints_), KKd(nn_joints_);

        for (size_t i = 0; i < nn_joints_; i++)
        {
            ROS_INFO("%d",i);
            std::string si = boost::lexical_cast<std::string>(i + 1);
            if (n.getParam("/elfin/admittance_controller/gains/elfin_joint" + si + "/pid/p", KKp[i]))
            {
                KKp_(i) = KKp[i];
            }
            else
            {
                std::cout << "/elfin/admittance_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/admittance_controller/gains/elfin_joint" + si + "/pid/i", KKi[i]))
            {
                KKi_(i) = KKi[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/admittance_controller/gains/elfin_joint" + si + "/pid/d", KKd[i]))
            {
                KKd_(i) = KKd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }

        // 1.2.2 Closed-loop Inverse Kinematics Controller
        if (cctr_obj_ == 1)
        {
            if (!n.getParam("/elfin/admittance_controller/clik_gain/K_regulation", KK_regulation_))
            {
                ROS_ERROR("Cannot find clik regulation gain");
                return false;
            }
        }

        else if (cctr_obj_ == 2)
        {
            if (!n.getParam("/elfin/admittance_controller/clik_gain/K_tracking", KK_tracking_))
            {
                ROS_ERROR("Cannot find clik tracking gain");
                return false;
            }
        }
        /*
        // 2. ********* urdf *********
        urdf::Model uurdf;
        if (!uurdf.initParam("elfin/robot_description"))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        else
        {
            ROS_INFO("Found robot_description");
        }
*/
        // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
        for (int i = 0; i < nn_joints_; i++)
        {
            try
            {
                jjoints_.push_back(hw->getHandle(jjoint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr jjoint_urdf = urdf.getJoint(jjoint_names_[i]);
            if (!jjoint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", jjoint_names_[i].c_str());
                return false;
            }
            jjoint_urdfs_.push_back(jjoint_urdf);
        }

        // 4. ********* KDL *********
        // 4.1 kdl parser
        if (!kdl_parser::treeFromUrdfModel(urdf, kkdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }
        else
        {
            ROS_INFO("Constructed kdl tree");
        }

        // 4.2 kdl chain
        std::string rroot_name, ttip_name;

        rroot_name="world";
        ttip_name="elfin_link6";

        /*
        if (!n.getParam("root_link", root_name))
        {
            ROS_ERROR("Could not find root link name");
            return false;
        }
        if (!n.getParam("tip_link", tip_name))
        {
            ROS_ERROR("Could not find tip link name");
            return false;
        }
        */

        if (!kdl_tree_.getChain(rroot_name, ttip_name, kkdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << rroot_name << " --> " << ttip_name);
            ROS_ERROR_STREAM("  Tree has " << kkdl_tree_.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << kkdl_tree_.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap ssegment_map = kkdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = ssegment_map.begin(); it != ssegment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            return false;
        }
        else
        {
            ROS_INFO("Got kdl chain");
        }

        // 4.3 inverse dynamics solver 초기화
        ggravity_ = KDL::Vector::Zero();
        ggravity_(2) = -9.81; // 0: x-axis 1: y-axis 2: z-axis

        iid_solver_.reset(new KDL::ChainDynParam(kkdl_chain_, gravity_));

        // 4.4 jacobian solver 초기화
        jjnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kkdl_chain_));

        // 4.5 forward kinematics solver 초기화
        ffk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kkdl_chain_));

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 KDL Vector 초기화 (사이즈 정의 및 값 0)
        ttau_d_.data = Eigen::VectorXd::Zero(nn_joints_);
        xx_cmd_.data = Eigen::VectorXd::Zero(num_taskspace);

        qqd_.data = Eigen::VectorXd::Zero(nn_joints_);
        qqd_dot_.data = Eigen::VectorXd::Zero(nn_joints_);
        qqd_ddot_.data = Eigen::VectorXd::Zero(nn_joints_);
        qqd_old_.data = Eigen::VectorXd::Zero(nn_joints_);

        qq_.data = Eigen::VectorXd::Zero(nn_joints_);
        qqdot_.data = Eigen::VectorXd::Zero(nn_joints_);

        ee_.data = Eigen::VectorXd::Zero(nn_joints_);
        ee_dot_.data = Eigen::VectorXd::Zero(nn_joints_);
        ee_int_.data = Eigen::VectorXd::Zero(nn_joints_);


        // 5.2 KDL Matrix 초기화 (사이즈 정의 및 값 0)
        JJ_.resize(nn_joints_);
        // J_inv_.resize(kdl_chain_.getNrOfJoints());
        MM_.resize(nn_joints_);
        CC_.resize(nn_joints_);
        GG_.resize(nn_joints_);

        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
        ppub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        ppub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        ppub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

        ppub_xd_ = n.advertise<std_msgs::Float64MultiArray>("xd", 1000);
        ppub_x_ = n.advertise<std_msgs::Float64MultiArray>("x", 1000);
        ppub_ex_ = n.advertise<std_msgs::Float64MultiArray>("ex", 1000);

        ppub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        // 6.2 subsriber
        //ssub_x_cmd_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &Computed_Torque_Controller_CLIK::commandCB, this);
        eevent = 0; // subscribe 받기 전: 0
        // subscribe 받은 후: 1


        mmsg_x_.layout.dim.push_back(std_msgs::MultiArrayDimension());


        mmsg_x_.data.resize(6);


        for(int i=0;i<6;i++){
            mmsg_x_.data[i]=0;
        }

        k_stiff_damp_sub = n.subscribe<std_msgs::Float64MultiArray>("admittance_gains", 1, &AdmittanceController::admit_gains, this);

        // admittance params
        z=0;
        zdot=0;
        zddot=0;
        z_pre=0;
        zdot_pre=0;
        zddot_pre=0;
        K_damp=20;
        K_stiff=50;
        M_ad=5;

        publisher_ = n.advertise<std_msgs::Float64MultiArray>("publish_admittance", 1000);

        msg_temp.data.resize(6);

        for(int i=0;i<6;i++){
            msg_temp.data[i]=0;
        }
        return true;
    } // end of initialize


    void admit_gains(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        K_stiff = msg->data[0];
        K_damp  = msg->data[1];
        M_ad= msg->data[2];
    }

    void starting(const ros::Time& time)
    {
        // get joint positions
        for(size_t i=0; i<n_joints_; i++)
        {
            ROS_INFO("JOINT %d", (int)i);
            q_(i) = joints_[i].getPosition();
            q_init_(i) = q_(i);
            qdot_(i) = joints_[i].getVelocity();
        }

        time_ = 0.0;
        total_time_ = 0.0;

        ROS_INFO("Starting Adaptive Impedance Controller");
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if(msg->data.size()!=n_joints_)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
            return;
        }

        for (unsigned int i = 0; i<n_joints_; i++)
            q_cmd_sp_(i) = msg->data[i];
    }

    //void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
        // Convert Wrench msg to KDL wrench
        geometry_msgs::Wrench f_meas = msg->wrench;

        f_cur_[0] = f_meas.force.x;
        f_cur_buffer_ = f_meas.force.y;
        f_cur_[2] = f_meas.force.z;
        f_cur_[3] = f_meas.torque.x;
        f_cur_[4] = f_meas.torque.y;
        f_cur_[5] = f_meas.torque.z;

        if (experiment_mode_ == 1 || experiment_mode_ == 2)
            f_cur_[1] = first_order_lowpass_filter();
    }

    // load gain is not permitted during controller loading?
    void loadGainCB()
    {

    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
        //---------------------

        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        double ddt = period.toSec();
        tt = tt + 0.001;

        //----------------------------

        // simple trajectory interpolation from joint command setpoint
        dt_ = period.toSec();

        if(total_time_ < 5.0)
        {
            task_init();
        }
        else if(total_time_ >= 5.0 && total_time_ < 6.0)
        {
            task_via();
        }
        else if(total_time_ >= 6.0 && total_time_ < 16.0)
        {
            task_ready();
        }
        else if (total_time_ >= 16.0 && total_time_ < 17.0)
        {
            task_via();
        }
        else if (total_time_ >= 17.0 && total_time_ < 27.0)
        {
            task_freespace();
        }
        else if (total_time_ >= 27.0 && total_time_ < 28.0)
        {
            task_via();
            //        }
            //        else if (total_time_ >= 28.0 && total_time_ < 48.0)
            //        {
            //            //task_contactspace();
        }
        else if (total_time_ >= 48.0 && total_time_ < 49.0)
        {
            task_via();
        }
        else if (total_time_ >= 49.0 && total_time_ < 59.0)
        {
            task_homming();
        }

        // get joint states
        for (size_t i=0; i<n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }


        //fk_solver_->JntToCart(qq_, xx_);

        for (size_t i=0; i<nn_joints_; i++)
        {
            qq_(i) = jjoints_[i].getPosition();
            qqdot_(i) = jjoints_[i].getVelocity();
        }

        // 0.3 end-effector state by Compute forward kinematics
        ffk_pos_solver_->JntToCart(qq_, xx_);

        for(int i=0 ; i<3; i++){
            mmsg_x_.data[i]=xx_.p(i);
        }

        ppub_x_.publish(mmsg_x_);

        //        if (total_time_ >= 28.0 && total_time_ < 48.0)
        if (total_time_ >= 28.0)
        {

            if (flag!=2)
            {
                xxd_= xx_;
                xx=xx_.p(0)-0.02;
                //yy=xx_.p(1);
                zz=xx_.p(2)-0.05;

                xxd_dot_(0) = 0;
                xxd_dot_(1) = 0;
                xxd_dot_(2) = 0;
                xxd_dot_(3) = 0;
                xxd_dot_(4) = 0;
                xxd_dot_(5) = 0;

                xxd_.p(2)=zz;

                flag = 2;
            }



            double amp=0.18;
            double ttt=(total_time_-28)/30;
            ttt=ttt-floor(ttt);
            xxd_.p(0) = xx+amp*sin(PI*ttt);

            //xxd_dot_(0) = amp*PI*cos(PI*ttt);
            //xxd_ddot_(0)=-1*amp*PI*PI*sin(PI*ttt);


            // ADMITTACE CONTROL IMPLEMENTATION***********************************************************

            double F =abs(f_cur_[1]);
            //double F =f_cur_[1];

            zddot= (F-K_damp*zdot_pre-K_stiff*z_pre)/M_ad;
            zdot=zdot_pre+zddot*ddt;
            z=z_pre+zdot*ddt;
            z_pre=z;
            zdot_pre=zdot;

           xxd_.p(2) = zz+z;
           xxd_dot_(2)=0+zdot;
           xxd_ddot_(2)=0+zddot;


            // publisher    *************************************
            msg_temp.data[0]=zz;
            msg_temp.data[1]=zdot;
            msg_temp.data[2]=zddot;
            msg_temp.data[3]=xxd_.p(2);
            msg_temp.data[4]=F;
            msg_temp.data[5]=total_time_;

            publisher_.publish(msg_temp);


            // ********* 2. Inverse Kinematics *********
            // *** 2.0 Error Definition in Task Space ***
            eex_temp_ = diff(xx_, xxd_);
            eex_(0) = eex_temp_(0);
            eex_(1) = eex_temp_(1);
            eex_(2) = eex_temp_(2);
            eex_(3) = eex_temp_(3);
            eex_(4) = eex_temp_(4);
            eex_(5) = eex_temp_(5);

            // *** 2.1 computing Jacobian J(q) ***
            jjnt_to_jac_solver_->JntToJac(qq_, JJ_);

            xxdot_ = JJ_.data * qqdot_.data;
            eex_dot_ = xxd_dot_ - xxdot_;

            // *** 2.1 computing Jacobian J(q) ***
            jjnt_to_jac_solver_->JntToJac(qq_, JJ_);

            // *** 2.2 computing Jacobian transpose/inversion ***
            JJ_transpose_ = JJ_.data.transpose();
            JJ_inv_ = JJ_.data.inverse();

            if (tt < t_set)
            {
                qqd_.data = qqd_old_.data;
            }
            else
            {
                qqd_.data = qqd_old_.data + JJ_transpose_ * KK_regulation_ * eex_ * ddt;
                qqd_old_.data = qqd_.data;
            }


            // ********* 3. Motion Controller in Joint Space*********
            // *** 3.1 Error Definition in Joint Space ***
            ee_.data = qqd_.data - qq_.data;
            ee_dot_.data = qqd_dot_.data - qqdot_.data;
            ee_int_.data = qqd_.data - qq_.data; // (To do: e_int 업데이트 필요요 (Update required))


            // *** 3.2 Compute model(M,C,G) ***
            iid_solver_->JntToMass(qq_, MM_);
            iid_solver_->JntToCoriolis(qq_, qqdot_, CC_);
            iid_solver_->JntToGravity(qq_, GG_); // output은 머지? , id_solver는 어디에서?

            // *** 3.3 Apply Torque Command to Actuator ***

            aaux_d_.data =MM_.data*(JJ_inv_*(xxd_ddot_ + KKd_.data.cwiseProduct(eex_dot_)+KKp_.data.cwiseProduct(eex_)));


            //aaux_d_.data = MM_.data * (qqd_ddot_.data + KKp_.data.cwiseProduct(ee_.data) + KKd_.data.cwiseProduct(ee_dot_.data));
            ccomp_d_.data = CC_.data + GG_.data; //(n(q,q_dot))
            ttau_d_.data = aaux_d_.data + ccomp_d_.data;


            for (int i = 0; i < nn_joints_; i++)
            {
                jjoints_[i].setCommand(ttau_d_(i));
            }


        }else {

            qddot_.data = (qdot_.data - qdot_old_.data) / dt_;

            // error
            KDL::JntArray q_error;
            KDL::JntArray q_error_dot;
            KDL::JntArray ded;
            KDL::JntArray tde;
            KDL::JntArray s;

            q_error.data = Eigen::VectorXd::Zero(n_joints_);
            q_error_dot.data = Eigen::VectorXd::Zero(n_joints_);
            ded.data = Eigen::VectorXd::Zero(n_joints_);
            tde.data = Eigen::VectorXd::Zero(n_joints_);
            s.data = Eigen::VectorXd::Zero(n_joints_);



            double db = 0.001;

            q_error.data = q_cmd_.data - q_.data;
            q_error_dot.data = qdot_cmd_.data - qdot_.data;

            for(size_t i=0; i<n_joints_; i++)
            {
                s(i) = q_error_dot(i) + Ramda_(i)*q_error(i);

                if(db < Mbar_(i) && Mbar_(i) > Ramda_(i)/Alpha_(i))
                {
                    Mbar_dot_(i) = Alpha_(i)*s(i)*s(i) - Alpha_(i)*Omega_(i)*Mbar_(i);
                }

                if(Mbar_(i) < db)
                {
                    Mbar_(i) = db;
                }

                if(Mbar_(i) > Ramda_(i)/Alpha_(i))
                {
                    Mbar_(i) = Ramda_(i)/Alpha_(i) - db;
                }

                Mbar_(i) = Mbar_(i) + dt_*Mbar_dot_(i);
            }

            // torque command
            for(size_t i=0; i<n_joints_; i++)
            {
                ded(i) = qddot_cmd_(i) + 2.0 * Ramda_(i)*q_error(i) + Ramda_(i)*Ramda_(i)*q_error_dot(i);
                tde(i) = tau_cmd_old_(i) - Mbar_(i)*qddot_(i);
                tau_cmd_(i) = Mbar_(i)*ded(i) + tde(i);
            }

            for(size_t i=0; i<n_joints_; i++)
            {
                joints_[i].setCommand(tau_cmd_(i));
            }

        }



        tau_cmd_old_.data = tau_cmd_.data;
        qdot_old_.data = qdot_.data;

        KDL::Frame Xdes;

        fk_solver_->JntToCart(q_cmd_, Xdes);

        SaveData_[0] = total_time_;
        SaveData_[1] = 0.122;
        SaveData_[2] = Xdes.p(2);
        SaveData_[3] = Fd_;
        SaveData_[4] = f_cur_(1);
        SaveData_[5] = f_cur_buffer_;
        SaveData_[6] = B_buffer_;

        msg_SaveData_.data.clear();

        for (size_t i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]);
        }
        pub_SaveData_.publish(msg_SaveData_);

        time_ = time_ + dt_;
        total_time_ = total_time_ + dt_;

    }

    void stopping(const ros::Time& time) { }

    void task_via()
    {
        time_ = 0.0;

        for (size_t i = 0; i < n_joints_; i++)
        {
            q_init_(i) = joints_[i].getPosition();
            q_cmd_(i) = q_cmd_end_(i);
            qdot_cmd_(i) = 0.0;
            qddot_cmd_(i) = 0.0;
        }
    }

    void task_init()
    {
        for (size_t i=0; i<n_joints_; i++)
        {
            q_cmd_(i) = 0.0;
            qdot_cmd_(i) = 0.0;
            qddot_cmd_(i) = 0.0;
        }

        q_cmd_end_.data = q_cmd_.data;
    }

    void task_ready()
    {
        for (size_t i=0; i<n_joints_; i++)
        {
            if (i == 2 || i == 4)
            {
                q_cmd_(i) = trajectory_generator_pos(q_init_(i), PI/2.0, 10.0);
                qdot_cmd_(i) = trajectory_generator_vel(q_init_(i), PI/2.0, 10.0);
                qddot_cmd_(i) = trajectory_generator_acc(q_init_(i), PI/2.0, 10.0);
            }
            else
            {
                q_cmd_(i) = q_init_(i);
                qdot_cmd_(i) = 0.0;
                qddot_cmd_(i) = 0.0;
            }
        }

        q_cmd_end_.data = q_cmd_.data;
    }

    void task_freespace()
    {
        KDL::Frame start;
        KDL::Twist target_vel;
        KDL::JntArray cart_cmd;

        cart_cmd.data = Eigen::VectorXd::Zero(3);

        fk_solver_->JntToCart(q_init_, start);

        for (size_t i=0; i<6; i++)
        {
            if(i == 2)
            {
                target_vel(i) = trajectory_generator_vel(start.p(i), 0.122, 10.0);
            }
            else
            {
                target_vel(i) = 0.0;
            }
        }

        ik_vel_solver_->CartToJnt(q_cmd_, target_vel, qdot_cmd_);

        for (size_t i=0; i<n_joints_; i++)
        {
            q_cmd_(i) = q_cmd_(i) + qdot_cmd_(i)*dt_;
            qddot_cmd_(i) = (qdot_cmd_(i) - qdot_cmd_old_(i))/dt_;
            qdot_cmd_old_(i) = qdot_cmd_(i);
        }

        q_cmd_end_.data = q_cmd_.data;
    }

    void task_contactspace()
    {
        KDL::Frame start;

        if (experiment_mode_ == 0 || experiment_mode_ == 1)
            Fd_ = Fd_temp_;
        else if (experiment_mode_ == 2)
            Fd_ = Fd_temp_ * 1 / 5 * sin(2 * PI * 0.2 * total_time_) + Fd_temp_;

        fk_solver_->JntToCart(q_init_, start);

        for (size_t i = 0; i < 6; i++)
        {
            if (i == 0)
            {
                Xc_dot_(i) = trajectory_generator_vel(start.p(i), 0.5, 20.0);
            }
            else if (i == 2)
            {
                Fe_ = f_cur_[1];
                PI_ = PI_old_ + dt_ * (Fd_old_ - Fe_old_) / B_;
                B_buffer_ = B_ + del_B_;
                Xc_ddot_(i) = 1/M_*((Fe_ - Fd_) - (B_ + del_B_)*Xc_dot_old_(i));
                Xc_dot_(i) = Xc_dot_old_(i) + Xc_ddot_(i)*dt_;
                del_B_ = B_ / Xc_dot_(i) * PI_;
                Xc_dot_old_(i) = Xc_dot_(i);
                Fd_old_ = Fd_;
                Fe_old_ = Fe_;
                PI_old_ = PI_;
            }
            else
            {
                Xc_ddot_(i) = 0.0;
                Xc_dot_(i) = 0.0;
            }
        }

        ik_vel_solver_->CartToJnt(q_cmd_, Xc_dot_, qdot_cmd_);

        for (size_t i=0; i<n_joints_; i++)
        {
            q_cmd_(i) = q_cmd_(i) + qdot_cmd_(i)*dt_;
            qddot_cmd_(i) = (qdot_cmd_(i) - qdot_cmd_old_(i))/dt_;
            qdot_cmd_old_(i) = qdot_cmd_(i);
        }

        q_cmd_end_.data = q_cmd_.data;
    }

    void task_homming()
    {
        for (size_t i = 0; i < n_joints_; i++)
        {
            q_cmd_(i) = trajectory_generator_pos(q_init_(i), 0.0, 10.0);
            qdot_cmd_(i) = trajectory_generator_vel(q_init_(i), 0.0, 10.0);
            qddot_cmd_(i) = trajectory_generator_acc(q_init_(i), 0.0, 10.0);
        }

        q_cmd_end_.data = q_cmd_.data;
    }

    double trajectory_generator_pos(double dStart, double dEnd, double dDuration)
    {
        double dA0 = dStart;
        double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
        double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
        double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

        return dA0 + dA3*time_*time_*time_ + dA4*time_*time_*time_*time_ + dA5*time_*time_*time_*time_*time_;
    }

    double trajectory_generator_vel(double dStart, double dEnd, double dDuration)
    {
        double dA0 = dStart;
        double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
        double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
        double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

        return 3.0*dA3*time_*time_ + 4.0*dA4*time_*time_*time_ + 5.0*dA5*time_*time_*time_*time_;
    }

    double trajectory_generator_acc(double dStart, double dEnd, double dDuration)
    {
        double dA0 = dStart;
        double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
        double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
        double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

        return 6.0*dA3*time_ + 12.0*dA4*time_*time_ + 20.0*dA5*time_*time_*time_;
    }

    double first_order_lowpass_filter()
    {
        filt_ = (tau_ * filt_old_ + dt_*f_cur_buffer_)/(tau_ + dt_);
        filt_old_ = filt_;

        return filt_;
    }






private:
    // joint handles
    unsigned int n_joints_;
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

    // kdl
    KDL::Tree 	kdl_tree_;
    KDL::Chain	kdl_chain_;
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_;
    boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
    boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
    KDL::JntArray G_;
    KDL::Vector gravity_;

    // tdc gain
    KDL::JntArray Mbar_, Mbar_dot_, Ramda_, Alpha_, Omega_;

    // cmd, state
    KDL::JntArray q_init_;
    KDL::JntArray tau_cmd_, tau_cmd_old_;
    KDL::JntArray q_cmd_, q_cmd_old_, qdot_cmd_, qdot_cmd_old_, qddot_cmd_;
    KDL::JntArray q_cmd_end_;
    KDL::JntArray q_cmd_sp_;
    KDL::JntArray q_, qdot_, qdot_old_, qddot_;
    KDL::Wrench f_cur_;
    double f_cur_buffer_;

    KDL::Twist Xc_dot_, Xc_dot_old_, Xc_ddot_;

    double Xr_dot_, Xe_dot_;
    double Xe_ddot_;

    double Fd_, Fd_temp_, Fd_old_, Fe_, Fe_old_;
    double M_, B_, del_B_, B_buffer_;
    double PI_, PI_old_;

    double dt_;
    double time_;
    double total_time_;

    double filt_old_;
    double filt_;
    double tau_;

    int experiment_mode_;

    // topic
    ros::Subscriber sub_q_cmd_;
    ros::Subscriber sub_forcetorque_sensor_;

    double SaveData_[SaveDataMax];

    ros::Publisher pub_SaveData_;

    std_msgs::Float64MultiArray msg_SaveData_;


    // new from computed torque controller:


    // others
    double tt;
    int cctr_obj_;
    int iik_mode_;
    int eevent;

    //Joint handles
    unsigned int nn_joints_;                               // joint 숫자
    std::vector<std::string> jjoint_names_;                // joint name ??
    std::vector<hardware_interface::JointHandle> jjoints_; // ??
    std::vector<urdf::JointConstSharedPtr> jjoint_urdfs_;  // ??

    // kdl
    KDL::Tree kkdl_tree_;   // tree?
    KDL::Chain kkdl_chain_; // chain?

    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix MM_; // intertia matrix
    KDL::JntArray CC_;              // coriolis
    KDL::JntArray GG_;              // gravity torque vector
    KDL::Vector ggravity_;

    // kdl and Eigen Jacobian
    KDL::Jacobian JJ_;
    // KDL::Jacobian J_inv_;
    // Eigen::Matrix<double, num_taskspace, num_taskspace> J_inv_;
    Eigen::MatrixXd JJ_inv_;
    Eigen::Matrix<double, num_taskspace, num_taskspace> JJ_transpose_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> ffk_pos_solver_; //Solver to compute the forward kinematics (position)
    // boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_; //Solver to compute the forward kinematics (velocity)
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jjnt_to_jac_solver_; //Solver to compute the jacobian
    boost::scoped_ptr<KDL::ChainDynParam> iid_solver_;               // Solver To compute the inverse dynamics

    // Joint Space State
    KDL::JntArray qqd_, qqd_dot_, qqd_ddot_;
    KDL::JntArray qqd_old_;
    KDL::JntArray qq_, qqdot_;
    KDL::JntArray ee_, ee_dot_, ee_int_;

    // Task Space State
    // ver. 01
    KDL::Frame xxd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
    KDL::Frame xx_;
    KDL::Twist eex_temp_;

    // KDL::Twist xd_dot_, xd_ddot_;
    Eigen::Matrix<double, num_taskspace, 1> eex_;
    Eigen::Matrix<double, num_taskspace, 1> xxd_dot_, xxd_ddot_;
    Eigen::Matrix<double, num_taskspace, 1> xxdot_;
    Eigen::Matrix<double, num_taskspace, 1> eex_dot_, eex_int_;

    // ver. 02
    // Eigen::Matrix<double, num_taskspace, 1> xd_, xd_dot_, xd_ddot_;
    // Eigen::Matrix<double, num_taskspace, 1> x_, xdot_;
    // KDL::Frame x_temp_;
    // Eigen::Matrix<double, num_taskspace, 1> ex_, ex_dot_, ex_int_;

    // Input
    KDL::JntArray xx_cmd_;

    // Torque
    KDL::JntArray aaux_d_;
    KDL::JntArray ccomp_d_;
    KDL::JntArray ttau_d_;

    // gains
    KDL::JntArray KKp_, KKi_, KKd_;
    double KK_regulation_, KK_tracking_;

    // save the data
    double SSaveData_[SaveDataMax];

    // ros subscriber
    ros::Subscriber ssub_x_cmd_;

    // ros publisher
    ros::Publisher ppub_qd_, ppub_q_, ppub_e_;
    ros::Publisher ppub_xd_, ppub_x_, ppub_ex_;
    ros::Publisher ppub_SaveData_;

    // ros message
    std_msgs::Float64MultiArray mmsg_qd_, mmsg_q_, mmsg_e_;
    std_msgs::Float64MultiArray mmsg_xd_, mmsg_x_, mmsg_ex_;
    std_msgs::Float64MultiArray mmsg_SaveData_;

    // magni
    int flag = 0;
    double xx;
    double zz;

    // admittance params
    double z_pre;
    double zdot_pre;
    double zddot_pre;
    double z;
    double zdot;
    double zddot;
    double K_damp;
    double K_stiff;
    double M_ad;
    int window;

    ros::Subscriber k_stiff_damp_sub;

    std_msgs::Float64MultiArray msg_temp;
    ros::Publisher publisher_;


};
}

PLUGINLIB_EXPORT_CLASS(arm_controllers::AdmittanceController, controller_interface::ControllerBase)
