#include <iostream>
#include <ros/ros.h>
#include "humanoid_league_msgs/GoalRelative.h"
#include "humanoid_league_msgs/LineInformationRelative.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>//quick walking node 
#include "humanoid_league_msgs/TeamData.h"
#include "humanoid_league_msgs/ObstaclesRelative.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>

using namespace Eigen;


int main( int argc, char **argv )
{
	ros::init(argc, argv, "test_node");
	ros::NodeHandle n;

	/*通过"testData"Node仿真发送下列messages，其中"visual_odometry" "visualCompass"在bitbots_meta中尚未更新。*/
	ros::Publisher pubTeamData = n.advertise<humanoid_league_msgs::TeamData>("teamdata_topic",10);
	ros::Publisher pubGoalRelative = n.advertise<humanoid_league_msgs::GoalRelative>("goal_relative",10);
	ros::Publisher pubObsRelative = n.advertise<humanoid_league_msgs::ObstaclesRelative>("obstacles_relative",10);
	ros::Publisher pubLineInformationRelative = n.advertise<humanoid_league_msgs::LineInformationRelative>("line_relative",10);
	ros::Publisher pubWlakOdometry = n.advertise<nav_msgs::Odometry>("walk_odometry",10);
	ros::Publisher pubVisualOdometry = n.advertise<nav_msgs::Odometry>("visual_odometry",10);
	ros::Publisher pubPointCloud2 = n.advertise<sensor_msgs::PointCloud2>("line_relative_pc",10);
//	ros::Publisher pubVisualCompass = n.advertise<>();

	Matrix<float, 24, 7> MatrixGoal;
	Matrix<float,  9, 6> MatrixObs;
	Matrix<float,  6, 2> MatrixSegstart;
	Matrix<float,  6, 2> MatrixSegend;
	Matrix<float,  6, 6> MatrixCircle;
	//lx, ly, rx, ry, cx, cy, c
	MatrixGoal << //fake data:七个一组，分别对应上述值。
		4, 2, 4, 4, 4, 3, 1, 	      4, 2, 4, 4, 4, 3, 1, 		4, 2, 4, 4, 4, 3, 1, 		4, 2, 4, 4, 4, 3, 1,
            	4, 2, 4, 4, 4, 3, 1, 	      4, 2, 4, 4, 4, 3, 1, 		4, 2, 4, 4, 4, 3, 1, 		4, 2, 4, 4, 4, 3, 1,
            	4, 2, 4, 4, 4, 3, 1, 	      4, 2, 4, 4, 4, 3, 1, 		4, 2, 4, 4, 4, 3, 1, 		4, 2, 4, 4, 4, 3, 1,
            	4, 2, 4, 4, 4, 3, 1, 	      4, 2, 4, 4, 4, 3, 1, 		4, 2, 4, 4, 4, 3, 1, 		4, 2, 4, 4, 4, 3, 1,
            	3.9, 2, 3.9, 3.9, 4, 3, 1,    3.7, 2, 3.7, 4, 3.7, 3, 1,        3.5, 2, 3.5, 4, 3.5, 3, 1,
            	3.3, 2, 3.3, 4, 3.3, 3, 1,    3.1, 2, 3.1, 4, 3.1, 3, 1,        2.9, 2, 2.9, 4, 2.9, 3, 1,
            	2.7, 2, 2.7, 4, 2.7, 3, 1,    2.5, 2, 2.5, 4, 2.5, 3, 1;

	//x, y, w, h, t, c
	MatrixObs <<    0.5, -2,   0.5, 0.5, 3, 0.7, 	0.8, -2,   0.5, 0.5, 3, 0.7,
            		1,   -2,   0.5, 0.5, 3, 0.7, 	1.2, -2,   0.5, 0.5, 3, 0.7,
            		1.2, -2.2, 0.5, 0.5, 3, 0.7, 	1.2, -2.5, 0.5, 0.5, 3, 0.7,
            		1.2, -2.7, 0.5, 0.5, 3, 0.7, 	1.0, -2.5, 0.5, 0.5, 3, 0.7,
        		0.8, -2.3, 0.5, 0.5, 3, 0.7 ;
	
	//线的起点坐标，两个值一组
	MatrixSegstart <<  0,0,   0,0,   0,0,   0,0,   0,0,   0,0;//TODO:1.make data,2.declear it
	//线的终点，两个值一组
	MatrixSegend   <<  0,0,   0,0,   0,0,   0,0,   0,0,   0,0;
	//三个点确定一个圆，六个值一组，分别是左点，中点，右点
	MatrixCircle <<    0,0,0,0,0,0,   0,0,0,0,0,0,   0,0,0,0,0,0,   0,0,0,0,0,0,   0,0,0,0,0,0,   0,0,0,0,0,0;
	
	humanoid_league_msgs::GoalRelative goal;
	humanoid_league_msgs::TeamData team;
	humanoid_league_msgs::ObstacleRelative obs;
	std::vector<humanoid_league_msgs::ObstacleRelative> Obs;
	humanoid_league_msgs::ObstaclesRelative obstacle;
	humanoid_league_msgs::LineSegmentRelative linesegment;
	std::vector<humanoid_league_msgs::LineSegmentRelative> lineSegment;
	humanoid_league_msgs::LineIntersectionRelative lineinter;
	std::vector<humanoid_league_msgs::LineIntersectionRelative> lineInter;
	humanoid_league_msgs::LineCircleRelative linecircle;
	std::vector<humanoid_league_msgs::LineCircleRelative> lineCircle;
	humanoid_league_msgs::LineInformationRelative line;
	
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		for(int i = 0; i < MatrixGoal.rows(); i++)
		{
			goal.header.seq = i;
			goal.header.stamp = ros::Time::now();
			goal.header.frame_id = "base_footprint";
			goal.left_post.x = MatrixGoal(i,0);
			goal.left_post.y = MatrixGoal(i,1);
			goal.right_post.x = MatrixGoal(i,2);
			goal.right_post.y = MatrixGoal(i,3);
			goal.center_direction.x = MatrixGoal(i,4);
			goal.center_direction.y = MatrixGoal(i,5);
			goal.confidence = MatrixGoal(i,6);;
		}
		
		for(int i = 0; i < MatrixObs.rows(); i++)
		{
			obs.playerNumber = -1;
			obs.position.x = MatrixObs(i,0);//Point
			obs.position.y = MatrixObs(i,1);
			obs.width = MatrixObs(i,2);
			obs.height = MatrixObs(i,3);
			obs.color = MatrixObs(i,4);
			obs.confidence = MatrixObs(i,5);
			Obs.push_back(obs);
		}
		
		
		for(int j = 0; j < 6; j++ )	
		{
			for(int i = 0; i < 6; i++ )
			{	
				linesegment.start.x = MatrixSegstart(i,0);
				linesegment.start.y = MatrixSegstart(i,1);
				linesegment.end.x = MatrixSegend(i,0);
				linesegment.end.y = MatrixSegend(i,1);
				linesegment.confidence = 0;
				lineSegment.push_back(linesegment);
			}
			lineinter.segments = lineSegment;
			lineinter.type = 0;
			lineinter.confidence = 0;
		}
		lineInter.push_back(lineinter);
		for(int i = 0; i < 6; i++ )
		{
			linecircle.left.x = MatrixCircle(i,0);
			linecircle.left.y = MatrixCircle(i,1);
			linecircle.middle.x = MatrixCircle(i,2);
			linecircle.middle.y = MatrixCircle(i,3);
			linecircle.right.x = MatrixCircle(i,4);
			linecircle.right.y = MatrixCircle(i,5);
			linecircle.confidence = 0;
			lineCircle.push_back(linecircle);
		}	
		line.intersections = lineInter;
		line.segments = lineSegment;
		line.circles = lineCircle;
		obstacle.obstacles = Obs;
		pubGoalRelative.publish(goal);
		pubObsRelative.publish(obstacle);
		pubLineInformationRelative.publish(line);
		loop_rate.sleep();
	}
}
                        /*
                        //bitbots_meta TODO;
                        iteam.header.seq = ;
                        team.header.stamp = ros::Time::now();
                        team.header.frame_id = ;
                        team.robot_ids[3];
                        team.role[2];
                        team.action[1];
                        team.state[1];
                        team.robot_positions = ;//数组
                        team.oppgoal_relative = ;//数组
                        team.avg_walking_speed = ;
                        team.time_to_position_at_ball = ;
                        team.max_kicking_distance = ;
                        pubTeamData.publish(team);

                        nav_msgs::Odometry walkodom;
                        walkodom.header.seq = ;
                        walkodom.header.stamp = ros::Time::now();
                        walkodomm.header.frame_id = ;
                        walkodom.string_child_frame_id = ;
                        walkodom.pose = ;//geometry_msgs/PoseWithCovariance
                        walkodom.twist = ;//geometry_msgs/TwistWithCovariance
                        pubWlakOdometry.publish(walkdodm);

                        sensor_msgs::PointCloud2 line;
                        line.header.seq = ;
                        line.header.stamp = ros::Time::now();
                        line.header.frame_id = ;
                        line.height = ;
                        line.width = ;
                        line.fields = ;//sensor_msgs/PointField[]
                        line.is_bigendian = ;//bool
                        line.point_step = ;
                        line.row_step = ;
                        line.data = ;
                        line.is_dense = ;//bool
                        */

