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


//ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>



class ApplyForce : public Test
{
public:
  double pix_coeff = 50.0;
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
	
	void setup_objects(std::vector<geometry_msgs::PointStamped> centroids)
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
	    //circleShape.m_p.Set(x_out,y_out); //position, relative to body position
	    circleShape.m_radius = 0.025*pix_coeff; //radius	  

	    b2FixtureDef myFixtureDef;
	    myFixtureDef.shape = &circleShape; //this is a pointer to the shape above
	    dynamicBody1->CreateFixture(&myFixtureDef); //add a fixture to the body	  
	  }
	  
	}
	void setup_table()
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
	  myBodyDef.type = b2_staticBody; //this will be a dynamic body
	  double x_out, y_out;
	  myBodyDef.position.Set(0.0,0.0);	  
	  b2Body* robot_base_body = m_world->CreateBody(&myBodyDef);	  
	  b2CircleShape circleShape;
	  circleShape.m_p.Set(0, 0); //position, relative to body position
	  circleShape.m_radius = 0.05*pix_coeff; //radius	  
	  b2FixtureDef myFixtureDef;
	  myFixtureDef.shape = &circleShape; //this is a pointer to the shape above
	  robot_base_body->CreateFixture(&myFixtureDef); //add a fixture to the body	  


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
