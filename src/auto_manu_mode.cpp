#include <cassert>
#include <string>
#include <memory>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

namespace
{
    //
    // Ros I/O stuff
    //
    
    ros::Publisher pub_twist;
    ros::Subscriber tw_manu_sub;
    ros::Subscriber tw_auto_sub;
    ros::Subscriber ads_manu_sub;
    ros::Subscriber ads_auto_sub;
    ros::ServiceServer joy_mode_manu_button;
    ros::ServiceServer joy_mode_auto_button;
    
    const std::string veh_controller = "/vehicle_controller";
    const std::string dir_controller = "/direction_controller";
    const std::string output_twist = "cmd_vel";
    std::string input_abutton_service_name = "limo_auto_mode";
    std::string input_bbutton_service_name = "limo_manu_mode";
    constexpr const int input_queue_size = 10;
    constexpr const int output_queue_size = 10;
    std::string str_manu ("manu");
    std::string str_auto ("auto");

    //
    // Parameters
    //

    const std::string twist_publish_message_type = "twist";
    const std::string all_publish_message_type = "all";
    std::string publish_message_twist("/cmd_vel_");
    bool twist_mode(false);
    bool use_joy(false);
    bool cmd_manu_mode(true);
    bool cmd_auto_mode(false);
    std::string sub_twist_manu("/joy_vel_");
    std::string sub_twist_auto("/cws_vel_");
    std::string pub_twist_name("/cmd_vel_");
    std::string start_command(str_manu);

    void initialize_params()
    {
        ros::NodeHandle node_private("~");
        
        node_private.param<bool>("select/twist", twist_mode, bool(true));
        node_private.param<std::string>("select/start_command_mode", start_command, "manu");

        node_private.param<bool>("use_joy", use_joy, bool(true));
        std::cout<<"use_joy = " << use_joy <<std::endl;
        
        if(twist_mode)
        {
            node_private.param<std::string>("subscribe/twist_manu", sub_twist_manu, "/joy_vel");
            node_private.param<std::string>("subscribe/twist_auto", sub_twist_auto, "/cws_vel");
            node_private.param<std::string>("publish/twist_cmd", pub_twist_name, "/cmd_vel");
            
            
            std::cout << "DBG : DECLARE SUBSCRIBE " << std::endl;
            ROS_INFO_STREAM("sub_twist_manu : " << sub_twist_manu);
            ROS_INFO_STREAM("sub_twist_auto : " << sub_twist_auto);
        }

        
        if (start_command.compare(str_manu) == 0)
        {
            if (use_joy)
            {
                std::cout << "DBG : start_command=MANU" << std::endl;
                cmd_manu_mode = true;
                cmd_auto_mode = false;
            }else
            {
                cmd_manu_mode = false;
                cmd_auto_mode = true;                
            }
        }
        else if (start_command.compare(str_auto) == 0)
        {
            if (use_joy)
            {                        
                std::cout << "DBG : start_command=AUTO" << std::endl;
                cmd_manu_mode = false;
                cmd_auto_mode = true;
            }else
            {
                cmd_manu_mode = false;
                cmd_auto_mode = true;                
            }
        }
        else
        {
            std::cout << "DBG : start_command=OTHER" << std::endl;
            cmd_manu_mode = false;
            cmd_auto_mode = false;
        }
        
        // TODO
        node_private.param<std::string>("service/manu_action_name", input_bbutton_service_name, "toto");
        node_private.param<std::string>("service/auto_action_name", input_abutton_service_name, "titi");
        std::cout << "DBG : DECLARE A ET B BOUTONS" << std::endl;
        ROS_INFO_STREAM("input_bbutton_service_name : " << input_bbutton_service_name);
        ROS_INFO_STREAM("input_abutton_service_name : " << input_abutton_service_name);
        //
    }
    
    void callback_twist_manu(const geometry_msgs::Twist& tw)
    {
        if(cmd_manu_mode)
            pub_twist.publish(tw);
    }
    
    void callback_twist_auto(const geometry_msgs::Twist& tw)
    {
        if(cmd_auto_mode )
            pub_twist.publish(tw);
    }

    
    bool setManuMode(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
    {
        if (use_joy)
        {
            std::cout << "DBG : setManuMode called since XBox paddle A Touch" << std::endl;
            cmd_manu_mode = true;
            cmd_auto_mode = false;
        }else
        {
            cmd_manu_mode = false;
            cmd_auto_mode = true;                
        }
        response.success = true;
        return true;
    }

    bool setAutoMode(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
    {
        if (use_joy)
        {
            std::cout << "DBG : setAutoMode called since XBox paddle B Touch" << std::endl;
            cmd_manu_mode = false;
            cmd_auto_mode = true;
        }else
        {
            cmd_manu_mode = false;
            cmd_auto_mode = true;                
        }
        response.success = true;
        
        return true;
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_switch");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    initialize_params();

    if(twist_mode)
    {
        tw_manu_sub = private_nh.subscribe(sub_twist_manu, 10, callback_twist_manu);
        tw_auto_sub = private_nh.subscribe(sub_twist_auto, 10, callback_twist_auto);
        pub_twist = private_nh.advertise<geometry_msgs::Twist>(pub_twist_name, 10);
    }

    
    joy_mode_manu_button = nh.advertiseService(input_abutton_service_name, &setAutoMode);
    joy_mode_auto_button = nh.advertiseService(input_bbutton_service_name, &setManuMode);
    
    ros::spin();

    return 0;
}
