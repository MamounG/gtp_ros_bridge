#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <gtp_ros_msg/requestAction.h>
#include <msgClient.hh>
#include <jsoncpp/json/json.h>

#include <string>
#include <iostream>

#include "std_msgs/String.h"
#include <gtp_ros_msg/GTPTraj.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <toaster_msgs/ObjectListStamped.h>
#include <toaster_msgs/RobotListStamped.h>
#include <toaster_msgs/HumanListStamped.h>
#include <tf/transform_datatypes.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

msgClient client;
ros::Publisher trajectory;

bool updateObjectList = false;
bool updateRobotList = false;
bool updateHumanList = false;


std::map<std::string,std::string> jointCorrespondances;



class GtpRosBridge
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<gtp_ros_msg::requestAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  gtp_ros_msg::requestFeedback feedback_;
  gtp_ros_msg::requestResult result_;

  

  


public:

  GtpRosBridge(std::string name) :
    as_(nh_, "gtp_ros_server", boost::bind(&GtpRosBridge::executeCB, this, _1), false),
    action_name_(name)
  {
  
        jointCorrespondances["torso_lift_link"] = "Torso";
        jointCorrespondances["head_pan_link"] = "pan_cam";
        jointCorrespondances["head_tilt_link"] = "tilt_cam";
        jointCorrespondances["laser_tilt_mount_link"] = "laser-jnt";
        jointCorrespondances["r_shoulder_pan_link"] = "right-Arm1";
        jointCorrespondances["r_shoulder_lift_link"] = "right-Arm2";
        jointCorrespondances["r_upper_arm_roll_link"] = "right-Arm3";
        jointCorrespondances["r_elbow_flex_link"] = "right-Arm4";
        jointCorrespondances["r_forearm_roll_link"] = "right-Arm5";
        jointCorrespondances["r_wrist_flex_link"] = "right-Arm6";
        jointCorrespondances["r_wrist_roll_link"] = "right-Arm7";
        jointCorrespondances["r_gripper_link"] = "fingerJointGripper_0";
        jointCorrespondances["l_shoulder_pan_link"] = "left-Arm1";
        jointCorrespondances["l_shoulder_lift_link"] = "left-Arm2";
        jointCorrespondances["l_upper_arm_roll_link"] = "left-Arm3";
        jointCorrespondances["l_elbow_flex_link"] = "left-Arm4";
        jointCorrespondances["l_forearm_roll_link"] = "left-Arm5";
        jointCorrespondances["l_wrist_flex_link"] = "left-Arm6";
        jointCorrespondances["l_wrist_roll_link"] = "left-Arm7";
        jointCorrespondances["l_gripper_link"] = "fingerJointGripper_1";
        
  
        as_.start();
  }

  ~GtpRosBridge(void)
  {
  }

  void executeCB(const gtp_ros_msg::requestGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    ROS_INFO("%s: Computing the request", action_name_.c_str());
    

    //int f = str.find("taskAlternativeId");
    //std::cout << str.c_str()[f] << std::endl; //163
    //int vir = str.rfind(",");
    //std::cout << vir << std::endl; //164
    


        
        
    //std::string altIdStr = str.substr(str.find("taskAlternativeId") + 19, str.rfind(",") - str.find("taskAlternativeId") - 19);
    //std::string IdStr = str.substr(str.find("taskId") + 8, str.rfind("}") - 1 - str.find("taskId") - 8);
    
    Json::FastWriter wrt;
    if (goal->req.requestType == "planning")
    {
        client.sendMessage("move3d", "{\"ClearGTPInputs\":null}");
        std::cout << client.getBlockingMessage().second << std::endl;
        
        
        //_result = wrt.write(res);

        
        for (unsigned int i = 0; i< goal->req.involvedAgents.size(); i++)
        {
           Json::Value request(Json::objectValue);
           Json::Value input(Json::objectValue);
           
           input["key"] = goal->req.involvedAgents.at(i).actionKey;
           input["value"] = goal->req.involvedAgents.at(i).agentName;
           request["AddGTPAgent"] = input;
           
           client.sendMessage("move3d", wrt.write(request));
           ROS_INFO("planner answer: %s", client.getBlockingMessage().second.c_str());
        }
        
        for (unsigned int i = 0; i< goal->req.involvedObjects.size(); i++)
        {
           Json::Value request(Json::objectValue);
           Json::Value input(Json::objectValue);
           
           input["key"] = goal->req.involvedObjects.at(i).actionKey;
           input["value"] = goal->req.involvedObjects.at(i).objectName;
           request["AddGTPObject"] = input;
           
           client.sendMessage("move3d", wrt.write(request));
           ROS_INFO("planner answer: %s", client.getBlockingMessage().second.c_str());
        }
        
        for (unsigned int i = 0; i< goal->req.data.size(); i++)
        {
           Json::Value request(Json::objectValue);
           Json::Value input(Json::objectValue);
           
           input["key"] = goal->req.data.at(i).dataKey;
           input["value"] = goal->req.data.at(i).dataValue;
           request["AddGTPData"] = input;
           
           client.sendMessage("move3d", wrt.write(request));
           ROS_INFO("planner answer: %s", client.getBlockingMessage().second.c_str());
        }
        
        for (unsigned int i = 0; i< goal->req.points.size(); i++)
        {
        
           Json::Value request(Json::objectValue);
           Json::Value input(Json::objectValue);
           
           input["x"] = goal->req.points.at(i).value.x;
           input["y"] = goal->req.points.at(i).value.y;
           input["z"] = goal->req.points.at(i).value.z;
           
           input["key"] = goal->req.points.at(i).pointKey;
           request["AddGTPPoint"] = input;
           
           client.sendMessage("move3d", wrt.write(request));
           ROS_INFO("planner answer: %s", client.getBlockingMessage().second.c_str());
        }
        
        Json::Value request(Json::objectValue);
        Json::Value input(Json::objectValue);
        
        input["computeMotionPlan"] = true;
        input["previousTaskAlternativeId"] = (int)goal->req.predecessorId.alternativeId;
        input["previousTaskId"] = (int)goal->req.predecessorId.actionId;
        input["type"] = goal->req.actionName;
        request["PlanGTPTask"] = input;
        
        client.sendMessage("move3d", wrt.write(request));
        
        std::string str = client.getBlockingMessage().second;
        //ROS_INFO("%s",str.c_str());
        
        Json::Value root,nullval,tmpVal;
        Json::Reader reader;
        
        bool parsedSuccess = reader.parse(str, root, false);
        if(!parsedSuccess || root.size() <= 0)
        {
            ROS_INFO("Failed to parse JSON");
            result_.ans.success = false;
            as_.setSucceeded(result_);
            return;
        }
        
        ROS_INFO("%s", root.toStyledString().c_str());
        
        tmpVal = root.get("PlanGTPTask", nullval );
        
        if (tmpVal == nullval)
        {
           ROS_INFO("Failed to find PlanGTPTask");
           result_.ans.success = false;
           as_.setSucceeded(result_);
           return;
        }
        

        std::string report = "";// = tmpVal.get("returnReport", "Null" ).asString();
        if (tmpVal.isMember("planningInformation") && tmpVal["planningInformation"].isMember("returnReport"))
            report = tmpVal["planningInformation"]["returnReport"].asString();
            
        
        if (report == "OK")
           result_.ans.success = true;
        else
        {
           ROS_INFO("Failed to compute, reason: %s", report.c_str());
           result_.ans.success = false;
           as_.setSucceeded(result_);
           return;
        }
          
        result_.ans.identifier.actionId = tmpVal.get("taskId", "Null" ).asInt();
        result_.ans.identifier.alternativeId = tmpVal.get("taskAlternativeId", "Null" ).asInt();
        
        
        ROS_INFO("planner answer: %s", str.c_str());
        as_.setSucceeded(result_);
    }
    else if (goal->req.requestType == "details")
    {
        Json::Value request(Json::objectValue);
        Json::Value input(Json::objectValue);
        
        input["taskId"] = (int)goal->req.loadAction.actionId;
        input["alternativeId"] = (int)goal->req.loadAction.alternativeId;
        request["GetGTPDetails"] = input;
        
        client.sendMessage("move3d", wrt.write(request));
        std::string res = client.getBlockingMessage().second;
        //ROS_INFO("planner answer: %s", res.c_str());
        
        
        Json::Value root, answer, nullval;
        Json::Reader reader;
  
        reader.parse(res, root, false);
        answer = root.get("GetGTPDetails", nullval );
        
        if (answer == nullval)
        {
          ROS_INFO("no GetGTPDetails");
          result_.ans.success = false;
          as_.setSucceeded(result_);
          return;
        }
        
        if (answer.get("status", "NULL").asString() != "OK")
        {
          ROS_INFO("get details failed");
          result_.ans.success = false;
          as_.setSucceeded(result_);
          return;
        }
        
        const Json::Value& subTrajs = answer["subTrajs"];
        result_.ans.subTrajs.clear();
        for (Json::ValueConstIterator it = subTrajs.begin(); it != subTrajs.end(); ++it)
        {
            const Json::Value& subTraj = *it;
            // rest as before
            gtp_ros_msg::SubTraj subMsg;
            subMsg.agent = subTraj["agent"].asString();
            subMsg.armId = subTraj["armId"].asInt();
            subMsg.subTrajId = subTraj["subTrajId"].asInt();
            subMsg.subTrajName = subTraj["subTrajName"].asString();
            subMsg.subTrajType = subTraj["subTrajType"].asString();
            result_.ans.subTrajs.push_back(subMsg);
        }
        result_.ans.success = true;
        as_.setSucceeded(result_);
        
    }
    else if (goal->req.requestType == "load")
    {
        Json::Value request(Json::objectValue);
        Json::Value input(Json::objectValue);
        
        input["taskId"] = (int)goal->req.loadAction.actionId;
        input["alternativeId"] = (int)goal->req.loadAction.alternativeId;
        input["subTrajId"] = (int)goal->req.loadSubTraj;
        request["GetGTPTraj"] = input;
        
        client.sendMessage("move3d", wrt.write(request));
        std::string res = client.getBlockingMessage().second;
        //ROS_INFO("planner answer: %s", res.c_str());
        
        
        Json::Value root;
        Json::Reader reader;
        
        reader.parse(res, root, false);
        
        gtp_ros_msg::GTPTraj t;
        for (unsigned int i = 0; i < root["GetGTPTraj"]["confs"].size(); i++)
        {
            trajectory_msgs::JointTrajectoryPoint Jtpt;
            for (unsigned int j = 6; j < root["GetGTPTraj"]["confs"][i].size(); j++)
            {
               Jtpt.positions.push_back(root["GetGTPTraj"]["confs"][i][j].asDouble());
               Jtpt.velocities.push_back(0);
               Jtpt.accelerations.push_back(0);
            }
            t.traj.points.push_back(Jtpt);
        }
        
        
        t.name = "traj";
        
       //TODO: check the robot name
       //TODO: do the same thing for navigation
        
        if (root["GetGTPTraj"]["agent"] == "PR2_ROBOT")
        {
            t.traj.joint_names.push_back("navX");
            t.traj.joint_names.push_back("navY");
            t.traj.joint_names.push_back("dummyZ");
            t.traj.joint_names.push_back("dummyRX");
            t.traj.joint_names.push_back("dummyRY");
            t.traj.joint_names.push_back("RotTheta");
            t.traj.joint_names.push_back("torso_lift_joint");
            t.traj.joint_names.push_back("head_pan_joint");
            t.traj.joint_names.push_back("head_tilt_joint");
            t.traj.joint_names.push_back("laser_tilt_mount_joint");
            
            t.traj.joint_names.push_back("r_shoulder_pan_joint");
            t.traj.joint_names.push_back("r_shoulder_lift_joint");
            t.traj.joint_names.push_back("r_upper_arm_roll_joint");
            t.traj.joint_names.push_back("r_elbow_flex_joint");
            t.traj.joint_names.push_back("r_forearm_roll_joint");
            t.traj.joint_names.push_back("r_wrist_flex_joint");
            t.traj.joint_names.push_back("r_wrist_roll_joint");
            t.traj.joint_names.push_back("r_gripper_joint");
            
            t.traj.joint_names.push_back("dummyGripper");
            
            
            t.traj.joint_names.push_back("l_shoulder_pan_joint");
            t.traj.joint_names.push_back("l_shoulder_lift_joint");
            t.traj.joint_names.push_back("l_upper_arm_roll_joint");
            t.traj.joint_names.push_back("l_elbow_flex_joint");
            t.traj.joint_names.push_back("l_forearm_roll_joint");
            t.traj.joint_names.push_back("l_wrist_flex_joint");
            t.traj.joint_names.push_back("l_wrist_roll_joint");
            t.traj.joint_names.push_back("l_gripper_joint");
        }
        else if (root["GetGTPTraj"]["agent"] == "KUKA_ROBOT")
        {
            t.traj.joint_names.push_back("arm0");
            t.traj.joint_names.push_back("arm1");
            t.traj.joint_names.push_back("arm2");
            t.traj.joint_names.push_back("arm3");
            t.traj.joint_names.push_back("arm4");
            t.traj.joint_names.push_back("arm5");
            t.traj.joint_names.push_back("arm6");
        }    
        
        
        
        //t.traj.points.
        trajectory.publish(t);
        
        result_.ans.success = true;
        as_.setSucceeded(result_);
        
    }
    else if (goal->req.requestType == "update")
    {
       updateObjectList = true;
       while (updateObjectList)
       {
         usleep(500000);
         ROS_INFO("Waiting for objects updates");
       }
       updateRobotList = true;
       while (updateRobotList)
       {
         usleep(500000);
         ROS_INFO("Waiting for robots updates");
       }
       updateHumanList = true;
       while (updateHumanList)
       {
         usleep(500000);
         ROS_INFO("Waiting for humans updates");
       }
       result_.ans.success = true;
       as_.setSucceeded(result_);
       ROS_INFO("Done updating");
    }
    else if (goal->req.requestType == "addAttachemnt")
    {
        Json::Value request(Json::objectValue);
        Json::Value input(Json::objectValue);

        input["taskId"] = (int)goal->req.loadAction.actionId;
        input["alternativeId"] = (int)goal->req.loadAction.alternativeId;

        request["SetGTPAttachementFromTask"] = input;

        client.sendMessage("move3d", wrt.write(request));
        std::string res = client.getBlockingMessage().second;

        Json::Value root;
        Json::Reader reader;

        reader.parse(res, root, false);
        if (root["SetGTPAttachementFromTask"]["status"].asString() == "OK")
        {
            ROS_INFO("Attachment added successfully");
        }
        else
        {
            ROS_INFO("Failed to add attachment");
        }
        result_.ans.success = true;
        as_.setSucceeded(result_);
    }
    else if (goal->req.requestType == "removeAttachment")
    {
        Json::Value request(Json::objectValue);
        Json::Value input(Json::objectValue);


        request["ClearAllAttachements"] = input;

        client.sendMessage("move3d", wrt.write(request));
        std::string res = client.getBlockingMessage().second;

        Json::Value root;
        Json::Reader reader;

        reader.parse(res, root, false);
        if (root["ClearAllAttachements"]["status"].asString() == "OK")
        {
            ROS_INFO("Attachment removed successfully");
        }
        else
        {
            ROS_INFO("Failed to remove attachment");
        }
        result_.ans.success = true;
        as_.setSucceeded(result_);
    }
    else
    {
        ROS_INFO("request type: \"%s\" not supported, the supported types are: planning, details, load and update", goal->req.requestType.c_str());
        result_.ans.success = false;
        result_.ans.identifier.actionId = -1;
        result_.ans.identifier.alternativeId = -1;
        as_.setSucceeded(result_);
    }
        
  }
};



void updateObjectPosesCB(const toaster_msgs::ObjectListStamped::ConstPtr& msg)
{
  if (updateObjectList)
  {
    Json::Value request(Json::objectValue);
    Json::Value input(Json::arrayValue);
    for (unsigned int i = 0; i < msg->objectList.size(); i++)
    {
      Json::Value obj(Json::objectValue);
      Json::Value conf(Json::arrayValue);
      conf.append(msg->objectList[i].meEntity.pose.position.x);
      conf.append(msg->objectList[i].meEntity.pose.position.y);
      conf.append(msg->objectList[i].meEntity.pose.position.z);
      
      tf::Quaternion q(msg->objectList[i].meEntity.pose.orientation.x,
                       msg->objectList[i].meEntity.pose.orientation.y,
                       msg->objectList[i].meEntity.pose.orientation.z,
                       msg->objectList[i].meEntity.pose.orientation.w);
      tf::Matrix3x3 m(q);      
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      conf.append(roll);
      conf.append(pitch);
      conf.append(yaw);
      
      obj["name"] = msg->objectList[i].meEntity.id;
      obj["conf"] = conf;

      input.append(obj);
      ROS_INFO("I heard about: [%s]", msg->objectList[i].meEntity.id.c_str());
    }
    
    request["UpdateGTPObjects"] = input;
    
    Json::FastWriter wrt;
    client.sendMessage("move3d", wrt.write(request));
    std::string res = client.getBlockingMessage().second;
    updateObjectList = false;
  }
  //ROS_INFO("I heard about: [%s]", msg->objectList[0].meEntity.id.c_str());
}

void updateRobotPosesCB(const toaster_msgs::RobotListStamped::ConstPtr& msg)
{
  if (updateRobotList)
  {
    Json::Value request(Json::objectValue);
    Json::Value input(Json::arrayValue);
    for (unsigned int i = 0; i < msg->robotList.size(); i++)
    {
      Json::Value rob(Json::objectValue);
      Json::Value dofs(Json::objectValue);
      Json::Value conf(Json::arrayValue);
      
      
      conf.append(msg->robotList[i].meAgent.meEntity.pose.position.x);
      conf.append(msg->robotList[i].meAgent.meEntity.pose.position.y);
      conf.append(0);
      
      tf::Quaternion q(msg->robotList[i].meAgent.meEntity.pose.orientation.x,
                       msg->robotList[i].meAgent.meEntity.pose.orientation.y,
                       msg->robotList[i].meAgent.meEntity.pose.orientation.z,
                       msg->robotList[i].meAgent.meEntity.pose.orientation.w);
      tf::Matrix3x3 m(q);      
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      conf.append(0);
      conf.append(0);
      conf.append(yaw);
      
      //Json::Value joint0(Json::objectValue);
      dofs["base"] = conf;
      
      for (unsigned int j = 0; j < msg->robotList.at(i).meAgent.skeletonJoint.size(); j++)
      {
         //Json::Value oneJoint(Json::objectValue);
         dofs[jointCorrespondances[msg->robotList.at(i).meAgent.skeletonJoint.at(j).meEntity.name]] = msg->robotList.at(i).meAgent.skeletonJoint.at(j).position;
         std::cout << msg->robotList.at(i).meAgent.skeletonJoint.at(j).meEntity.name << std::endl;
         std::cout << jointCorrespondances[msg->robotList.at(i).meAgent.skeletonJoint.at(j).meEntity.name] << std::endl;
         std::cout << msg->robotList.at(i).meAgent.skeletonJoint.at(j).position << std::endl;
         
         //conf.append(msg->robotList.at(i).meAgent.skeletonJoint.at(j).position);
      }
      
      

      
      
      rob["name"] = msg->robotList[i].meAgent.meEntity.name;
      rob["conf"] = dofs;

      input.append(rob);
      ROS_INFO("I heard about: [%s]", msg->robotList[i].meAgent.meEntity.id.c_str());
    }
    
    ROS_INFO("I heard about: [%s]", input.toStyledString().c_str());
    request["UpdateGTPRobots"] = input;
    
    Json::FastWriter wrt;
    client.sendMessage("move3d", wrt.write(request));
    std::string res = client.getBlockingMessage().second;
    updateRobotList = false;
  }
  //ROS_INFO("I heard about: [%s]", msg->robotList[0].meAgent.meEntity.id.c_str());
}

void updateHumanPosesCB(const toaster_msgs::HumanListStamped::ConstPtr& msg)
{
  if (updateHumanList)
  {
    Json::Value request(Json::objectValue);
    Json::Value input(Json::arrayValue);
    for (unsigned int i = 0; i < msg->humanList.size(); i++)
    {
      Json::Value rob(Json::objectValue);
      Json::Value conf(Json::arrayValue);
      
      conf.append(msg->humanList[i].meAgent.meEntity.pose.position.x);
      conf.append(msg->humanList[i].meAgent.meEntity.pose.position.y);
      conf.append(msg->humanList[i].meAgent.meEntity.pose.position.z);
      
      tf::Quaternion q(msg->humanList[i].meAgent.meEntity.pose.orientation.x, msg->humanList[i].meAgent.meEntity.pose.orientation.y, msg->humanList[i].meAgent.meEntity.pose.orientation.z, msg->humanList[i].meAgent.meEntity.pose.orientation.w);
      tf::Matrix3x3 m(q);      
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      conf.append(roll);
      conf.append(pitch);
      conf.append(yaw);
      
      Json::Value joints(Json::objectValue);
      for (unsigned int j = 0; j < msg->humanList.at(i).meAgent.skeletonJoint.size(); j++)
      {
         Json::Value joint(Json::objectValue);
         Json::Value poseConf(Json::arrayValue);
         //joint["jointName"] = msg->humanList.at(i).meAgent.skeletonJoint.at(j).meEntity.id;
         poseConf.append(msg->humanList.at(i).meAgent.skeletonJoint.at(j).meEntity.pose.position.x);
         poseConf.append(msg->humanList.at(i).meAgent.skeletonJoint.at(j).meEntity.pose.position.y);
         poseConf.append(msg->humanList.at(i).meAgent.skeletonJoint.at(j).meEntity.pose.position.z);
         
         tf::Quaternion q(msg->humanList.at(i).meAgent.skeletonJoint.at(j).meEntity.pose.orientation.x,
                          msg->humanList.at(i).meAgent.skeletonJoint.at(j).meEntity.pose.orientation.y,
                          msg->humanList.at(i).meAgent.skeletonJoint.at(j).meEntity.pose.orientation.z,
                          msg->humanList.at(i).meAgent.skeletonJoint.at(j).meEntity.pose.orientation.w);
         tf::Matrix3x3 m(q);      
         double roll, pitch, yaw;
         m.getRPY(roll, pitch, yaw);
         poseConf.append(roll);
         poseConf.append(pitch);
         poseConf.append(yaw);
         
         
         //joint["pose"] = poseConf;
         joints[msg->humanList.at(i).meAgent.skeletonJoint.at(j).meEntity.name] = poseConf;
         //joints.append(joint);
         
         //conf.append(msg->robotList.at(i).meAgent.skeletonJoint.at(j).position);
         
      }
      
      
      rob["name"] = msg->humanList[i].meAgent.meEntity.id;
      rob["conf"] = conf;
      rob["joints"] = joints;

      input.append(rob);
      ROS_INFO("I heard about: [%s]", msg->humanList[i].meAgent.meEntity.id.c_str());
    }
    
    request["UpdateGTPHumans"] = input;
    
    Json::FastWriter wrt;
    client.sendMessage("move3d", wrt.write(request));
    std::string res = client.getBlockingMessage().second;
    updateHumanList = false;
  }
}



int main(int argc, char** argv)
{
  std::string server = "localhost";
  int port = 5500;
  ROS_INFO("Starting node GTP_ROS with server: %s and port: %d",server.c_str(),port);
  

  ros::init(argc, argv, "gtp_ros_server_node");
  
  ros::NodeHandle n;
  trajectory = n.advertise<gtp_ros_msg::GTPTraj>("gtp_trajectory", 1000);
  
  ros::Rate loop_rate(10);
  


  ros::Subscriber subObj = n.subscribe("/pdg/objectList", 10, updateObjectPosesCB);
  ros::Subscriber subRob = n.subscribe("/pdg/robotList", 10, updateRobotPosesCB);
  ros::Subscriber subHum = n.subscribe("/pdg/humanList", 10, updateHumanPosesCB);

  client.connect("gtp_ros_node", server, port);
  if(client.isConnected())
  {
     client.sendMessage("move3d", "{\"InitGTP\":null}");
     bool testMove3d = false;
     int nbTestTime = 0;
     int nbTestType = 0;
     int maxtest = 3;
     while (!testMove3d && nbTestTime < maxtest && nbTestType < maxtest)
     {
        std::pair<std::string,std::string> ans = client.getMessageTimeout(2000);
        if (ans.second==STATUS_TIMEOUT)
        {
           nbTestTime++;
           ROS_INFO("The server timed out %d times", nbTestTime);
           continue;
        }
        if (ans.first != "move3d")
        {
           nbTestType++;
           continue;
        }
        testMove3d = true;

        GtpRosBridge bridge(ros::this_node::getName());
        ros::spin();
     }
     if (!testMove3d)
     {
        if (nbTestTime >= maxtest)
            ROS_INFO("no answer from move3d (time out reached): is it launched?");
        if (nbTestType >= maxtest)
            ROS_INFO("answer from other modules, might reconsider this code!!");
     }

  }
  else
  {
     ROS_INFO("Failed to connect to geometric planner, abort");
  }

  return 0;
}
