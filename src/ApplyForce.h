/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

//System
#include <stdlib.h>
#include <iostream>
#include <string>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>



class ApplyForce : public Test
{

 private:
  std::vector<geometry_msgs::PointStamped> centroids;
  std::vector<std::string> colors;
  geometry_msgs::PointStamped red_goal;
  geometry_msgs::PointStamped blue_goal;
  std::string red_str = "red";
  std::string blue_str = "blue";
  std::string red_goal_str = "red_goal";
  std::string blue_goal_str = "blue_goal";
  std::string gray_str = "gray";
  void* red_ptr; void* blue_ptr; void* red_goal_ptr; void* blue_goal_ptr; void* gray_ptr;
  double pix_coeff = 50.0;
    
 public:
 ApplyForce(): red_ptr(&red_str), blue_ptr(&blue_str), red_goal_ptr(&red_goal_str), blue_goal_ptr(&blue_goal_str), gray_ptr(&gray_str) { }
	void conv_robot_to_box2d_frame(const double x_in, const double y_in, double& x_out, double& y_out) {
	  x_out = y_in * pix_coeff;
	  y_out = -x_in * pix_coeff;
	  return;
	}

	
	void destroy_all_objects()
	{
	  // delete all objects
	  /*int num_objs = 0;
	  for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
	    {
	      //b->SetAwake(true);
	      num_objs++;
	    }
	  std::cout<< "num_objs: " << num_objs << std::endl;
	  destroy_all_bodies();*/

	  
	  for (b2Body* b = m_world->GetBodyList(); b;)  // remove GetNext() call
	    {
	      //if (b->GetUserData() != NULL) {
	      //CCSprite *s = (CCSprite *)b->GetUserData();
		//[self removeChild:s cleanup:YES];
		b2Body* next = b->GetNext();  // remember next body before *b gets destroyed
		m_world->DestroyBody(b); // do I need to destroy fixture as well(and how?) or it does that for me?
		b = next;  // go to next body
		//}
	    }
	}
	
	void setup_objects(std::vector<geometry_msgs::PointStamped> centroids, std::vector<std::string> colors, geometry_msgs::PointStamped red_goal, geometry_msgs::PointStamped blue_goal)
	{
	  setup_table(red_goal, blue_goal);
	  this->centroids = centroids;
	  this->colors = colors;
	  this->red_goal = red_goal;
	  this->blue_goal = blue_goal;
	  draw_table(centroids, colors, red_goal, blue_goal);
	}
	
	void draw_table (std::vector<geometry_msgs::PointStamped> centroids, std::vector<std::string> colors, geometry_msgs::PointStamped red_goal, geometry_msgs::PointStamped blue_goal)
	{
	  for (int i=0; i<centroids.size(); i++) {
	    b2BodyDef myBodyDef;
	    myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
	    myBodyDef.position.Set(0, 0); //a little to the left
	    
	    double x_out, y_out;
	    conv_robot_to_box2d_frame(centroids[i].point.x, centroids[i].point.y, x_out, y_out);
	    myBodyDef.position.Set(x_out,y_out);
	    
	    b2Body* dynamicBody1 = m_world->CreateBody(&myBodyDef);
	  
	    b2CircleShape circleShape;
	    circleShape.m_p.Set(0, 0); //position, relative to body position
	    circleShape.m_radius = 0.025*pix_coeff; //radius	  

	    b2FixtureDef myFixtureDef;
	    myFixtureDef.shape = &circleShape; //this is a pointer to the shape above
	    dynamicBody1->CreateFixture(&myFixtureDef); //add a fixture to the body
	    if (colors[i] == "red")
	      dynamicBody1->SetUserData(red_ptr);
	    else
	      dynamicBody1->SetUserData(blue_ptr);
	  }
	  
	}
	void setup_table(geometry_msgs::PointStamped red_goal, geometry_msgs::PointStamped blue_goal)
	{
	  m_world->SetGravity(b2Vec2(0.0f, 0.0f));
	  
	  b2Body* ground;	  

	  b2EdgeShape shape;
	  b2FixtureDef sd;
	  sd.shape = &shape;

	  double table_x_len = 1.5;
	  double table_y_len = 0.8;
	  double robot_base_offset = 0.1;

	  b2BodyDef bd;
	  bd.active = false;
	  //bd.position.Set(0.0f, table_y_len/2-robot_base_offset);
	  bd.position.Set(0.0f, 0.0f);
	  ground = m_world->CreateBody(&bd);
	  
	  // Left vertical
	  shape.Set(pix_coeff* b2Vec2(-table_x_len/2,-robot_base_offset), pix_coeff*b2Vec2(-table_x_len/2, table_y_len - robot_base_offset));
	  //shape.Set(pix_coeff*b2Vec2(-table_x_len/2,-table_y_len/2), pix_coeff*b2Vec2(-table_x_len/2, table_y_len/2));

	  ground->CreateFixture(&sd);

	  // Right vertical
	  shape.Set(pix_coeff*b2Vec2(table_x_len/2, -robot_base_offset), pix_coeff*b2Vec2(table_x_len/2, table_y_len - robot_base_offset) );
	  ground->CreateFixture(&sd);

	  // Top horizontal
	  shape.Set(pix_coeff*b2Vec2(-table_x_len/2, table_y_len - robot_base_offset), pix_coeff*b2Vec2(table_x_len/2, table_y_len - robot_base_offset));
	  ground->CreateFixture(&sd);

	  //std::cout<< "TABLE X: " <<pix_coeff*table_x_len/2 << " Y: " << pix_coeff*(table_y_len - robot_base_offset)<<std::endl;
	  
	  // Bottom horizontal
	  shape.Set(pix_coeff*b2Vec2(-table_x_len/2, - robot_base_offset), pix_coeff*b2Vec2(table_x_len/2, - robot_base_offset));
	  ground->CreateFixture(&sd);	  

	  //Robot Base
	  b2BodyDef myBodyDef;
	  myBodyDef.type = b2_staticBody;
	  myBodyDef.position.Set(0.0,0.0);	  
	  b2Body* robot_base_body = m_world->CreateBody(&myBodyDef);	  
	  b2CircleShape circleShape;
	  circleShape.m_p.Set(0, 0); //position, relative to body position
	  circleShape.m_radius = 0.05*pix_coeff; //radius	  
	  b2FixtureDef myFixtureDef;
	  myFixtureDef.shape = &circleShape; //this is a pointer to the shape above
	  robot_base_body->CreateFixture(&myFixtureDef); //add a fixture to the body	  
	  robot_base_body->SetUserData(gray_ptr);

	  //Goal Regions
	  double red_goal_x_box2d; double red_goal_y_box2d; 
	  conv_robot_to_box2d_frame(red_goal.point.x,red_goal.point.y, red_goal_x_box2d, red_goal_y_box2d);
	  b2BodyDef goal_body_def;
	  goal_body_def.type = b2_staticBody;
	  goal_body_def.position.Set(red_goal_x_box2d, red_goal_y_box2d);
	  b2Body* goal_body = m_world->CreateBody(&goal_body_def);  	  
	  b2CircleShape goal_body_shape;
	  goal_body_shape.m_p.Set(0, 0);
	  goal_body_shape.m_radius = 0.25*pix_coeff;
	  b2FixtureDef goal_fixture_def;
	  goal_fixture_def.isSensor = true;
	  goal_fixture_def.shape = &goal_body_shape;
	  goal_body->CreateFixture(&goal_fixture_def);
	  goal_body->SetUserData(red_goal_ptr);

	  double blue_goal_x_box2d; double blue_goal_y_box2d;
	  conv_robot_to_box2d_frame(blue_goal.point.x, blue_goal.point.y, blue_goal_x_box2d, blue_goal_y_box2d);
	  b2BodyDef goal_body_def_2;
	  goal_body_def_2.type = b2_staticBody;
	  goal_body_def_2.position.Set(blue_goal_x_box2d, blue_goal_y_box2d);
	  b2Body* goal_body_2 = m_world->CreateBody(&goal_body_def_2);  	  
	  b2CircleShape goal_body_shape_2;
	  goal_body_shape_2.m_p.Set(0, 0);
	  goal_body_shape_2.m_radius = 0.25*pix_coeff;
	  b2FixtureDef goal_fixture_def_2;
	  goal_fixture_def_2.isSensor = true;
	  goal_fixture_def_2.shape = &goal_body_shape_2;
	  goal_body_2->CreateFixture(&goal_fixture_def_2);
	  goal_body_2->SetUserData(blue_goal_ptr);

	  
	}
	~ApplyForce()
	  {
	    std::cout<<"ApplyForce Destructor"<<std::endl;
	  }
	static Test* Create()
	{
	  return new ApplyForce;
	}
	
	b2Body* m_body;
};

#endif
