#include <ros/ros.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <gtp_ros_msg/requestAction.h>
#include <jsoncpp/json/json.h>

#include <string>
#include <fstream>
#include <streambuf>

using namespace std;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_client");
  if (argc != 2) 
  {
    cout << "FAIL" << endl;
    cout << "usage: rosrun gtp_ros_bridge gtp_ros_bridge_test_client [fileName]" << endl;
    return 0;
  }
  string filename = argv[1];
  
  std::ifstream t(filename.c_str());
  std::stringstream buffer;
  buffer << t.rdbuf();
  //cout << buffer.str() << endl;
  
  Json::Value root;
  Json::Value nullVal;
  Json::Reader reader;
  
  bool parsedSuccess = reader.parse(buffer.str(), root, false);
  if(!parsedSuccess || root.size() <= 0)
  {
    // Report failures and their locations
    // in the document.
    cout<<"Failed to parse JSON"<< endl;
    return 0;
  }
  
  //Json::Value res(Json::objectValue);
  

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<gtp_ros_msg::requestAction> ac("gtp_ros_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  gtp_ros_msg::requestGoal goal;
  
  std::string requesttype = root.get("requestType", "Null" ).asString();
  cout << root.get("requestType", "Null" ).asString() << endl;
  if (requesttype == "Null")
  {
    cout << "no requestType" << endl;
    return 0;
  }
  
  goal.req.requestType = requesttype;
  
  if (requesttype == "planning" || requesttype == "updateAndPlan")
  {
  
      std::string type = root.get("actionType", "Null" ).asString();
      if (type == "Null")
      {
        cout << "no actionType" << endl;
        return 0;
      }
      //cout << "type = " << type << endl;
      goal.req.actionName = type;
      
      int Id = root.get("previousId", -170 ).asInt();
      if (Id == -170)
      {
        cout << "no previousId" << endl;
        return 0;
      }
      goal.req.predecessorId.actionId = Id;
      
      int altId = root.get("previousAltId", -170 ).asInt();
      if (altId == -170)
      {
        cout << "no previousAltId" << endl;
        return 0;
      }
      goal.req.predecessorId.alternativeId = altId;
      
      Json::Value inputVal = root.get("input", nullVal );
      if (inputVal == nullVal)
      {
        cout << "no inputs" << endl;
        return 0;
      }
      
      Json::Value agentVal = inputVal.get("agents", nullVal );
      if (agentVal != nullVal)
      {
        for (unsigned int i = 0; i < agentVal.getMemberNames().size(); i++)
        {
          gtp_ros_msg::Ag agent;
          Json::Value tmp = agentVal.get(agentVal.getMemberNames().at(i), nullVal );
          agent.actionKey = tmp.get("key", "null").asString();
          agent.agentName = tmp.get("value", "null").asString();
          goal.req.involvedAgents.push_back(agent);
        }
      }
      
      Json::Value objectVal = inputVal.get("objects", nullVal );
      if (objectVal != nullVal)
      {
        for (unsigned int i = 0; i < objectVal.getMemberNames().size(); i++)
        {
          gtp_ros_msg::Obj object;
          Json::Value tmp = objectVal.get(objectVal.getMemberNames().at(i), nullVal );
          object.actionKey = tmp.get("key", "null").asString();
          object.objectName = tmp.get("value", "null").asString();
          goal.req.involvedObjects.push_back(object);
        }
      }
      
      Json::Value dataVal = inputVal.get("datas", nullVal );
      if (dataVal != nullVal)
      {
        for (unsigned int i = 0; i < dataVal.getMemberNames().size(); i++)
        {
          gtp_ros_msg::Data data;
          Json::Value tmp = dataVal.get(dataVal.getMemberNames().at(i), nullVal );
          data.dataKey = tmp.get("key", "null").asString();
          data.dataValue = tmp.get("value", "null").asString();
          goal.req.data.push_back(data);
        }
      }
      
      Json::Value pointVal = inputVal.get("points", nullVal );
      if (pointVal != nullVal)
      {
        for (unsigned int i = 0; i < pointVal.getMemberNames().size(); i++)
        {
          gtp_ros_msg::Points ptt;
          Json::Value tmp = pointVal.get(pointVal.getMemberNames().at(i), nullVal );
          ptt.pointKey = tmp.get("key", "null").asString();
          if (tmp.get("value", "null").size() != 3)
          {
            cout << "problem in point with id: " << pointVal.getMemberNames().at(i) << endl;
            return 0;
          }
          ptt.value.x = tmp.get("value", "null")[0].asDouble();
          ptt.value.y = tmp.get("value", "null")[1].asDouble();
          ptt.value.z = tmp.get("value", "null")[2].asDouble();
          goal.req.points.push_back(ptt);
        }
      }
      
      
      
      //cout << root.toStyledString() << endl;
      
      //for (unsigned int i = 0; i < root.getMemberNames().size(); i++)
      //{
      //  cout << root.getMemberNames().at(i) << endl;
      //}


      ac.sendGoal(goal);

    //  //wait for the action to return
      bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

      if (finished_before_timeout)
      {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        if (ac.getResult()->ans.success)
        {  
            ROS_INFO("Action computed with success");
            ROS_INFO("id = %lu, altId = %lu", ac.getResult()->ans.identifier.actionId, ac.getResult()->ans.identifier.alternativeId);
            if (requesttype == "updateAndPlan")
            {
                ROS_INFO("nb subtrajs = %lu", ac.getResult()->ans.subTrajs.size());
            }
        }
        else
        {
            ROS_INFO("No path found");
        }
            
      }
      else
        ROS_INFO("Action did not finish before the time out.");

      //exit
      return 0;
  }
  else if (requesttype == "details")
  {
      int actionId = root.get("actionId", -1 ).asInt();
      if (actionId == -1)
      {
        cout << "no actionId 1" << endl;
        return 0;
      }
      
      int altId = root.get("altId", -1 ).asInt();
      if (altId == -1)
      {
        cout << "no actionId 2" << endl;
        return 0;
      }
      
      goal.req.loadAction.actionId = actionId;
      goal.req.loadAction.alternativeId = altId;
      
      ac.sendGoal(goal);

    //  //wait for the action to return
      bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

      if (finished_before_timeout)
      {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        if (ac.getResult()->ans.success)
        {  
            ROS_INFO("Details retrieved with success");
            ROS_INFO("nb subtrajs = %lu", ac.getResult()->ans.subTrajs.size());
        }
        else
        {
            ROS_INFO("No path found");
        }
            
      }
      else
        ROS_INFO("Action did not finish before the time out.");

      //exit
      return 0;
      
  }
  else if (requesttype == "load" || requesttype ==  "loadAndExecute")
  {
      int actionId = root.get("actionId", -1 ).asInt();
      if (actionId == -1)
      {
        cout << "no actionId 3" << endl;
        return 0;
      }
      
      int altId = root.get("altId", -1 ).asInt();
      if (altId == -1)
      {
        cout << "no actionId 4" << endl;
        return 0;
      }
      
      int subTrajId = root.get("subTrajId", -1 ).asInt();
      if (subTrajId == -1)
      {
        cout << "no subTrajId" << endl;
        return 0;
      }
      
      goal.req.loadAction.actionId = actionId;
      goal.req.loadAction.alternativeId = altId;
      goal.req.loadSubTraj = subTrajId;
      
      ac.sendGoal(goal);
  }
  else if (requesttype == "addAttachemnt")
  {
      int actionId = root.get("actionId", -1 ).asInt();
      if (actionId == -1)
      {
        cout << "no actionId 3" << endl;
        return 0;
      }

      int altId = root.get("altId", -1 ).asInt();
      if (altId == -1)
      {
        cout << "no actionId 4" << endl;
        return 0;
      }


      goal.req.loadAction.actionId = actionId;
      goal.req.loadAction.alternativeId = altId;

      ac.sendGoal(goal);
  }
  else if (requesttype == "removeAttachment")
  {
      cout << "sending the goal" << endl;
      ac.sendGoal(goal);
  }
  else if (requesttype == "update")
  {
      cout << "sending the goal" << endl;
      ac.sendGoal(goal);
  }
  else if (requesttype == "test")
  {
      ac.sendGoal(goal);
  }
  else
  {
    ROS_INFO("request type: %s unknown, abort action.", requesttype.c_str());
  }
}













