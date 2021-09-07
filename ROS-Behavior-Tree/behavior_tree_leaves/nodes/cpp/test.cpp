#include <ros/ros.h>
#include "dmp/dmp.h"
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <python2.7/Python.h>
//#include <pyList.h>

using namespace dmp;
using namespace std;

int main(int argc, char **argv)
{
  Py_Initialize();
  if( !Py_IsInitialized())
  {
  cout << "python init fail" << endl;
  return 0;
  }

  PyObject *pMoudle, *pFunc;
  PyObject *pArgs;


  string path = "/home/lni/Ros_ws/catkin_ws/src/ROS-Behavior-Tree/behavior_tree_leaves/nodes/cpp";
  string chdir_cmd = string("sys.path.append(\"") + path + "\")";
  const char* cstr_cmd = chdir_cmd.c_str();
  PyRun_SimpleString("import sys");
  PyRun_SimpleString(cstr_cmd);
  pMoudle = PyImport_Import(PyString_FromString("dmp_jiyuan"));
  pFunc = PyObject_GetAttrString(pMoudle, "DMP_generation");

//  pArgs = PyTuple_New(5);   // 1个参数
  pArgs = PyTuple_New(3);

  PyObject* p_start = PyList_New(7);
  PyList_SetItem(p_start,0, PyFloat_FromDouble(0.2126780700));
  PyList_SetItem(p_start,1, PyFloat_FromDouble(-0.2563742000));
  PyList_SetItem(p_start,2, PyFloat_FromDouble(0.5061635934));
  PyList_SetItem(p_start,3, PyFloat_FromDouble(-0.6831957413));
  PyList_SetItem(p_start,4, PyFloat_FromDouble(0.2288655176));
  PyList_SetItem(p_start,5, PyFloat_FromDouble(-0.6876954690));
  PyList_SetItem(p_start,6, PyFloat_FromDouble(-0.0891016044));

  PyObject* p_end = PyList_New(7);
  PyList_SetItem(p_end,0, PyFloat_FromDouble(0.3563434191));
  PyList_SetItem(p_end,1, PyFloat_FromDouble(0.0003662767));
  PyList_SetItem(p_end,2, PyFloat_FromDouble(0.3371303887));
  PyList_SetItem(p_end,3, PyFloat_FromDouble(-0.9945580586));
  PyList_SetItem(p_end,4, PyFloat_FromDouble(0.1041741315));
  PyList_SetItem(p_end,5, PyFloat_FromDouble(0.0012358004));
  PyList_SetItem(p_end,6, PyFloat_FromDouble(0.0007007868));

  PyTuple_SetItem(pArgs,0, PyString_FromString("/home/lni/Ros_ws/catkin_ws/src/dmp/data/jiyuan/jiyuan0.txt"));
  PyTuple_SetItem(pArgs,1,p_start);
  PyTuple_SetItem(pArgs,2,p_end);

  std::vector<double> list_points;
  PyObject * pValue;

  pValue = PyObject_CallObject(pFunc, pArgs);
  if (pValue)
  {
    cout<<"got pValue correctly!"<<endl;
    if(PyList_Check(pValue))
       {
            cout<<"pValue is list"<<endl;
            int SizeOfList = PyList_Size(pValue);//SizeOfList = 875
            cout<<"got SizeOfList: "<<SizeOfList<<endl; //sizeOfList = 875
            for(int i = 0; i < SizeOfList; i++)
            {
                PyObject *ListItem = PyList_GetItem(pValue, i);//获取List对象中的每一个元素
                double result = PyFloat_AsDouble(ListItem);
//                cout<<"got ListItem "<<i <<": "<<result<<endl;
                list_points.push_back(result);
            }
        }
    else
       ROS_ERROR("something wrong!");
  }
  else
  {
    ROS_ERROR("pValue is null!");
    Py_Finalize();
    return 0;
  }
//  way_point.resize(125,7);
  int x = 0;
  int size = list_points.size()/7;
  float result[size][7];
  geometry_msgs::Pose target_point;
  std::vector<geometry_msgs::Pose> way_points;
  for (int i = 0;i<size;i++)
  {
    for (int j = 0;j<7;j++)
    {
      result[i][j] = list_points[x];
      x++;
    }
    target_point.position.x = result[i][0];
    target_point.position.y = result[i][1];
    target_point.position.z = result[i][2];
    target_point.orientation.x = result[i][3];
    target_point.orientation.y = result[i][4];
    target_point.orientation.z = result[i][5];
    target_point.orientation.w = result[i][6];
    way_points.push_back(target_point);
  }
  cout<<result[124][0]<<endl;
  cout<<target_point.position.x<<endl;



/*------------------------------------------------
  std::vector<double> way_point;
  for (int i = 0; i<125 ; i++)
  {
    for (int j=0;j<7;j++)
    {
      PyObject* I = PyInt_FromLong(i);
      PyObject* J = PyInt_FromLong(j);
      PyTuple_SetItem(pArgs,3,I);
      PyTuple_SetItem(pArgs,4,J);
      PyObject* pValue;

      pValue = PyObject_CallObject(pFunc, pArgs);
//      cout << "result:" <<endl;
      double result = PyFloat_AsDouble(pValue);

      //cout << "result:" << result<<endl;
//      ros::WallDuration(1);
      way_point.push_back(result);
    }
  }
//  cout << "result:" <<endl;
  way_point.resize(125,7);

  cout<<"******"<<way_point[static_cast<void>(124),1]<<endl;
-------------------------------*/

  Py_Finalize();


  ROS_INFO("Hello world!");

  return 0;
}
