#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

//System
#include <stdlib.h>
#include <iostream>
#include <string>
#include <random>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

struct BodyState {
  b2Vec2 position;
  double radius;
  std::string color;
  int id;
};

struct WorldState {
  std::vector<BodyState> bodies;  
};

struct PushDef {
  b2Vec2 push_start;
  b2Vec2 push_end;
};

class ApplyForce : public Test
{

 private:
  std::vector<geometry_msgs::PointStamped> centroids;
  std::vector<double> radiuses;
  std::vector<double> goal_radiuses;
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
  double ee_long = 0.32;
  double ee_short = 0.08;    
  b2Body* ee_body;
  b2Body* robot_base_body;
  b2Body* red_goal_body;
  b2Body* blue_goal_body;
  std::vector<b2Body*> red_bodies;
  std::vector<b2Body*> blue_bodies;
  
 public:
 ApplyForce(): red_ptr(&red_str), blue_ptr(&blue_str), red_goal_ptr(&red_goal_str), blue_goal_ptr(&blue_goal_str), gray_ptr(&gray_str) { }

  WorldState save_world() {
    WorldState world_state;
    for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {

      int id = get_body_id(b);
      if (id == -1)
	continue;
      BodyState body_state;
      body_state.position = b->GetPosition();
      body_state.radius = get_obj_radius(b);
      body_state.id = id;
      body_state.color = get_body_color(b);      
      world_state.bodies.push_back(body_state);
    }
    return world_state;
  }

  void load_world(WorldState state) {

    set_sensor_status(ee_body, true);
    ee_body->SetTransform(b2Vec2(1000,1000), 0);
        
    //delete all objects with non -1 ID
    std::vector<int> exception_ids;
    exception_ids.push_back(-1);
    destroy_objects_with_exceptions(exception_ids);

        
    //create bodies from scratch
    for (int i=0; i < state.bodies.size(); i++) {      
      b2BodyDef myBodyDef;
      myBodyDef.type = b2_dynamicBody;
      myBodyDef.position.Set(state.bodies[i].position.x,state.bodies[i].position.y);	    
      b2Body* dynamicBody1 = m_world->CreateBody(&myBodyDef);	  
      b2CircleShape circleShape;
      circleShape.m_p.Set(0, 0);
      circleShape.m_radius = state.bodies[i].radius;      
      b2FixtureDef myFixtureDef;
      myFixtureDef.shape = &circleShape; //this is a pointer to the shape above
      myFixtureDef.friction = 10.0;
      dynamicBody1->CreateFixture(&myFixtureDef); //add a fixture to the body
      bodyUserData* myStruct = new bodyUserData;	    
      myStruct->id = state.bodies[i].id;
      myStruct->str = state.bodies[i].color;
      dynamicBody1->SetUserData(myStruct);      
    }
  }
   
  void destroy_objects_with_exceptions(std::vector<int> exception_ids)
  {
    for (b2Body* b = m_world->GetBodyList(); b;)
      {
	b2Body* next = b->GetNext();
	int id = get_body_id(b);
	if(std::find(exception_ids.begin(), exception_ids.end(), id) == exception_ids.end()) { //does not contain
	  m_world->DestroyBody(b);
	}	
	b = next;
      }
  }

  void destroy_all_objects()
  {
    for (b2Body* b = m_world->GetBodyList(); b;)
      {
	b2Body* next = b->GetNext();
	m_world->DestroyBody(b);
	b = next;
      }
  }
  
  void robot_to_box2d_frame(const double x_in, const double y_in, double& x_out, double& y_out) {
    x_out = y_in * pix_coeff;
    y_out = -x_in * pix_coeff;
    return;
  }
  void box2d_to_robot_frame(const double x_in, const double y_in, double& x_out, double& y_out) {
    x_out = -y_in/pix_coeff;
    y_out = x_in/pix_coeff;
    return;
  }

  b2Vec2 calc_linear_velocity(b2Body* b_from, b2Body* b_to, double out_magnitude) {
    b2Vec2 linear_velocity;
    double diff_x = b_from->GetPosition().x - b_to->GetPosition().x;
    double diff_y = b_from->GetPosition().y - b_to->GetPosition().y;
    double mag = sqrt(diff_x*diff_x + diff_y*diff_y);
    double coeff = out_magnitude/mag;
    linear_velocity.x = diff_x*coeff;
    linear_velocity.y = diff_y*coeff;
    return linear_velocity;
  }

  double calc_angle(b2Body* b_from, b2Body* b_to) {
    double diff_x = b_from->GetPosition().x - b_to->GetPosition().x;
    double diff_y = b_from->GetPosition().y - b_to->GetPosition().y;
    return atan2(diff_y, diff_x) + M_PI/2;
  }

  double get_dist_between_bodies(b2Body* b_from, b2Body* b_to) {
    double diff_x = b_from->GetPosition().x - b_to->GetPosition().x;
    double diff_y = b_from->GetPosition().y - b_to->GetPosition().y;
    double mag = sqrt(diff_x*diff_x + diff_y*diff_y);
    return mag;
  }
       	
  PushDef simulate_push(b2Body* b, bool goal_oriented, bool draw) {    

    PushDef push_def;
    double goal_threshold = 0.05;
    double ee_multiplier = 1.25;
    
    //find collision-free position for EE	  
    b2Body* goal_body;
    if (get_body_color(b) == "red")
      goal_body = red_goal_body;
    else if (get_body_color(b) == "blue")
      goal_body = blue_goal_body;

    double magnitude = 10.0;
    b2Vec2 linear_velocity = calc_linear_velocity(b, goal_body, magnitude);
    double angle = calc_angle(b, goal_body);
    set_sensor_status(ee_body, true);
    set_obj_dimensions(ee_body, ee_long * ee_multiplier, ee_short * ee_multiplier);
    ee_body->SetTransform(b->GetPosition(), angle); //uniform angle sampling

    ee_body->SetActive(true);
    ee_body->SetLinearVelocity(linear_velocity);
	  
    // Find collision-free pre-push pose
    while(true) {
      bool in_collision_1 = coll_check(ee_body, red_bodies);
      bool in_collision_2 = coll_check(ee_body, blue_bodies);
      //bool in_collision_3 = coll_check(ee_body, robot_base_body);
      
      if (!in_collision_1 && !in_collision_2) { //Collision Free!
	ee_body->SetLinearVelocity(b2Vec2(0,0));
	set_obj_dimensions(ee_body, ee_long, ee_short);
	set_sensor_status(ee_body, false);
	break;
      }
      else {
	draw_stuff(true, false);
      }
    }

    push_def.push_start = ee_body->GetPosition();
    
    while (true) {
      double d = get_dist_between_bodies(ee_body, goal_body);
      if (d < goal_threshold*pix_coeff) {
	ee_body->SetLinearVelocity(b2Vec2(0,0));
	set_sensor_status(ee_body, true);
	break;
      }
      else {
	ee_body->SetLinearVelocity(-linear_velocity);
	draw_stuff(true, draw);
      }
    }
    push_def.push_end = ee_body->GetPosition();   
    ee_body->SetActive(false);
    return push_def;
  }


  double calc_heuristic() {
    double dist = 0.0;
    double d = 0.0;
    for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
      if (get_body_color(b) == "red")
	d = get_dist_between_bodies(b, red_goal_body);
      else if (get_body_color(b) == "blue")
	d = get_dist_between_bodies(b, blue_goal_body);      
      dist = dist + d;
    }
    return dist;
  }

  void set_bodies() {
    red_goal_body = get_objects(red_goal_str)[0];
    blue_goal_body = get_objects(blue_goal_str)[0];
    red_bodies = get_objects(red_str);
    blue_bodies = get_objects(blue_str);
    //std::cout << "# red_bodies: " << red_bodies.size() << std::endl;
    //std::cout << "# blue_bodies: " << blue_bodies.size() << std::endl;
  }
  
  void plan(geometry_msgs::PointStamped &obj_centroid, geometry_msgs::PointStamped &placement, bool& goal_reached, int& action_type)
  {    
    bool enable_draw = true;
    double padding_for_gripper = 0.04;

    set_bodies();
    
    std::vector<b2Body*> displaced_bodies = get_displaced_objects();
    std::cout << "# displaced_bodies: " << displaced_bodies.size() << std::endl;

    if (displaced_bodies.empty()) {
      goal_reached = true;
      return;
    }

    //1 = push. 2 = pick&place
    if (rand() % 2 == 0)
      action_type = 1; 
    else
      action_type = 2;
    
          
    if (action_type == 1) {
      //Algo: greedy search for 1 goal-oriented push per object
      WorldState state = save_world();
      std::vector<double> heuristics;
      std::vector<PushDef> push_defs;
      double min_heuristic = -DBL_MAX;
      PushDef min_push_def;
      double heuristic = calc_heuristic();
      std::cout<<"heur INIT: " << heuristic <<std::endl;    

      for (unsigned int i = 0; i < displaced_bodies.size(); i++) {
	PushDef push_def = simulate_push(displaced_bodies[i], true, enable_draw);
	double heuristic = calc_heuristic();
	std::cout<<"heur[" << i<< "]: " << heuristic <<std::endl;
	heuristics.push_back(heuristic);      
	push_defs.push_back(push_def);
	load_world(state);
      }

      auto smallest = std::min_element(heuristics.begin(), heuristics.end());	
      int min_index = std::distance(heuristics.begin(), smallest);	
      min_push_def = push_defs[min_index];

      //visualize selected push
      simulate_push(displaced_bodies[min_index], true, true);
      load_world(state);
      
      double push_start_x = 0.0, push_start_y = 0.0, push_end_x = 0.0, push_end_y = 0.0;
      box2d_to_robot_frame(min_push_def.push_start.x, min_push_def.push_start.y, push_start_x , push_start_y);
      box2d_to_robot_frame(min_push_def.push_end.x, min_push_def.push_end.y, push_end_x , push_end_y);      
      obj_centroid.point.x = push_start_x;
      obj_centroid.point.y = push_start_y;
      placement.point.x = push_end_x;
      placement.point.y = push_end_y;
      goal_reached = false;
      return;
    }
    else if (action_type == 2) {

    }
    else {
      ROS_ERROR("Error: Action type undefined");
      exit(0);
    }
    
    //Algo: select an object for pick&place
    while (true) {

      std::vector<b2Body*> displaced_bodies = get_displaced_objects();
	  
      //choose a random object
      std::vector<b2Body*>::iterator randIt = displaced_bodies.begin();
      std::advance(randIt, std::rand() % displaced_bodies.size());
      b2Body* cur_body = *randIt;

      //choose a placement position in goal region
      b2Body* goal_body;
      if (get_body_color(cur_body) == "red")
	goal_body = red_goal_body;
      else if (get_body_color(cur_body) == "blue")
	goal_body = blue_goal_body;

      //get goal region bounding box
      b2AABB aabb = get_aabb(goal_body);
      b2Vec2 lowerbound = aabb.lowerBound;
      b2Vec2 upperbound = aabb.upperBound;

      //sample a position in goal_body aabb
      double x_sampled = RandomFloat(lowerbound.x, upperbound.x);
      double y_sampled = RandomFloat(lowerbound.y, upperbound.y);

      //get current position of the object, in case placement fails
      double x_init = cur_body->GetPosition().x;
      double y_init = cur_body->GetPosition().y;
      double angle_init = cur_body->GetAngle();

      set_sensor_status(cur_body, true);
      double cur_radius = get_obj_radius(cur_body);
      set_obj_radius(cur_body, cur_radius + padding_for_gripper); //padding
	    
      cur_body->SetTransform(b2Vec2(x_sampled,y_sampled), angle_init); //uniform angle sampling

      //check if proposition satisfies goal
      bool local_goal_satisfied = obj_goal_satisfied(goal_body, cur_body);

      //check if proposition collides with any other object
      bool in_collision_1 = coll_check(cur_body, red_bodies);
      bool in_collision_2 = coll_check(cur_body, blue_bodies);
	    
      cur_body->SetTransform(b2Vec2(x_init,y_init), angle_init);
      set_obj_radius(cur_body, cur_radius);
      set_sensor_status(cur_body, false);

      if (local_goal_satisfied && !in_collision_1 && !in_collision_2)
	{	      
	  double x_out, y_out;
	  box2d_to_robot_frame(x_sampled, y_sampled, x_out, y_out);	    
	  obj_centroid = centroids[get_body_id(cur_body)];
	  placement.point.x = x_out;
	  placement.point.y = y_out;
	  placement.point.z = obj_centroid.point.z;
	  goal_reached = false;
	  return;
	}
    }
  }

  double get_obj_radius(b2Body* b) {
    b2Fixture* F = b->GetFixtureList();
    b2CircleShape* circle = (b2CircleShape*) F->GetShape();
    return circle->m_radius;
  }
  void set_obj_radius(b2Body* b, double radius) {
    b2Fixture* F = b->GetFixtureList();
    b2CircleShape* circle = (b2CircleShape*) F->GetShape();
    circle->m_radius = radius;
  }

  void set_obj_dimensions(b2Body* b, double dim_long, double dim_short) {
    b2Fixture* F = b->GetFixtureList();
    b2PolygonShape* poly = (b2PolygonShape*) F->GetShape();
    poly->SetAsBox(dim_long*pix_coeff*0.5, dim_short*pix_coeff*0.5);
  }
	
  int get_body_id(b2Body* b) {
    if (b->GetUserData() != NULL) {
      bodyUserData* udStruct = (bodyUserData*)b->GetUserData();
      return udStruct->id;
    }
    return -1;
  }
	
  bool coll_check(b2Body* cur_body, std::vector<b2Body*> bodies) {
    for (unsigned int i=0; i < bodies.size(); i++) {
      if (get_body_id(cur_body) == get_body_id(bodies[i])) {		
	continue;
      }
      b2Fixture* f_1 = cur_body->GetFixtureList();
      b2Fixture* f_2 = bodies[i]->GetFixtureList();	      
      bool overlap = b2TestOverlap(f_1->GetShape(), 0, f_2->GetShape(), 0,  cur_body->GetTransform(), bodies[i]->GetTransform());
      if (overlap)
	return true;
    }
    return false;
  }
	
  void set_sensor_status(b2Body* body, bool isSensor) {
    b2Fixture* fixture = body->GetFixtureList();
    while (fixture != NULL)
      {
	fixture->SetSensor(isSensor);
	fixture = fixture->GetNext();
      }
  }
	  	
  b2AABB get_aabb(b2Body* body) {
    b2AABB aabb;
    aabb.lowerBound = b2Vec2(FLT_MAX,FLT_MAX);
    aabb.upperBound = b2Vec2(-FLT_MAX,-FLT_MAX); 
    b2Fixture* fixture = body->GetFixtureList();
    while (fixture != NULL)
      {
	aabb.Combine(aabb, fixture->GetAABB(0));
	fixture = fixture->GetNext();
      }
    return aabb;
  }

  float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
  }

  bool obj_goal_satisfied(b2Body* goal_body, b2Body* test_body) {
    bool global_hit = false;
    b2Vec2 point = test_body->GetPosition();
    b2Fixture* F = goal_body->GetFixtureList();
    while(F != NULL)
      {
	switch (F->GetType())
	  {
	  case b2Shape::e_circle:
	    {
	      b2CircleShape* circle = (b2CircleShape*) F->GetShape();
	      bool hit = circle->TestPoint(goal_body->GetTransform(), point);
	      if (hit) {
		global_hit = true;
		break;
	      }		      
	    }
	  case b2Shape::e_polygon:
	    {
	      b2PolygonShape* polygon = (b2PolygonShape*) F->GetShape();
	      bool hit = polygon->TestPoint(goal_body->GetTransform(), point);		   
	      if (hit) {
		global_hit = true;
		break;
	      }		      
	    }
	    break;
	  }
	F = F->GetNext();
      }
    return global_hit;
  }

  std::vector<b2Body*>  get_displaced_objects() {
    std::vector<b2Body*> displaced_bodies;
    for (unsigned int i=0; i < red_bodies.size(); i++)
      {
	if (!obj_goal_satisfied(red_goal_body, red_bodies[i])) {
	  displaced_bodies.push_back(red_bodies[i]);
	}
      }
    for (unsigned int i=0; i < blue_bodies.size(); i++)
      {
	if (!obj_goal_satisfied(blue_goal_body, blue_bodies[i])) {
	  displaced_bodies.push_back(blue_bodies[i]);
	}
      }
    return displaced_bodies;
  }
  	
  unsigned int count_objs() {
    unsigned int num_objs = 0;
    for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
      num_objs++;
    }
    return num_objs;
  }

  std::string get_body_color (b2Body* b) {
    if (b->GetUserData() != NULL) {
      bodyUserData* udStruct = (bodyUserData*)b->GetUserData();
      std::string color = udStruct->str;
      return color;
    }
    return "null";
  }
	
  std::vector<b2Body*> get_objects(std::string str_in) {	  
    std::vector<b2Body*> result;
    for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
      {
	if (get_body_color(b) == str_in) {
	  result.push_back(b);
	}	      
      }
    return result;
  }
		
  void setup_objects(std::vector<geometry_msgs::PointStamped> centroids, std::vector<double> radiuses, std::vector<std::string> colors, geometry_msgs::PointStamped red_goal, geometry_msgs::PointStamped blue_goal, std::vector<double> goal_radiuses)
  {
    this->centroids = centroids;
    this->colors = colors;
    this->red_goal = red_goal;
    this->blue_goal = blue_goal;
    this->goal_radiuses = goal_radiuses;
    setup_table(red_goal, blue_goal);
    draw_table(centroids, radiuses, colors, red_goal, blue_goal);
  }

	
  void draw_table (std::vector<geometry_msgs::PointStamped> centroids, std::vector<double> radiuses, std::vector<std::string> colors, geometry_msgs::PointStamped red_goal, geometry_msgs::PointStamped blue_goal)
  {
    for (int i=0; i<centroids.size(); i++) {
      b2BodyDef myBodyDef;
      myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
      myBodyDef.position.Set(0, 0); //a little to the left
	    
      double x_out, y_out;
      robot_to_box2d_frame(centroids[i].point.x, centroids[i].point.y, x_out, y_out);
      myBodyDef.position.Set(x_out,y_out);
	    
      b2Body* dynamicBody1 = m_world->CreateBody(&myBodyDef);
	  
      b2CircleShape circleShape;
      circleShape.m_p.Set(0, 0); //position, relative to body position
      circleShape.m_radius = radiuses[i]*pix_coeff; //radius	  
	    
      b2FixtureDef myFixtureDef;
      myFixtureDef.shape = &circleShape; //this is a pointer to the shape above
      dynamicBody1->CreateFixture(&myFixtureDef); //add a fixture to the body

      bodyUserData* myStruct = new bodyUserData;	    
      myStruct->id = i;
      if (colors[i] == "red") {
	myStruct->str = red_str;
	dynamicBody1->SetUserData(myStruct);
      }
      else {
	myStruct->str = blue_str;
	dynamicBody1->SetUserData(myStruct);
      }
    }	  
  }
  
  void setup_table(geometry_msgs::PointStamped red_goal, geometry_msgs::PointStamped blue_goal)
  {
    m_world->SetGravity(b2Vec2(0.0f, 0.0f));

    b2Body* ground;	  

    b2EdgeShape shape;
    b2FixtureDef sd;
    sd.shape = &shape;
    sd.friction = 10.0f;
	  
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
    robot_base_body = m_world->CreateBody(&myBodyDef);	  
    b2CircleShape circleShape;
    circleShape.m_p.Set(0, 0); //position, relative to body position
    circleShape.m_radius = 0.05*pix_coeff; //radius	  
    b2FixtureDef myFixtureDef;
    myFixtureDef.shape = &circleShape; //this is a pointer to the shape above
    robot_base_body->CreateFixture(&myFixtureDef); //add a fixture to the body	  
    bodyUserData* robot_base_data = new bodyUserData;
    robot_base_data->id = -1;
    robot_base_data->str = gray_str;	      
    robot_base_body->SetUserData(robot_base_data);

    //Robot End Effector
    b2BodyDef ee_body_def;
    ee_body_def.type = b2_dynamicBody;
    ee_body_def.position.Set(0.0,0.0);	  
    ee_body = m_world->CreateBody(&ee_body_def);
    b2PolygonShape ee_shape;
    ee_shape.SetAsBox(ee_long*pix_coeff*0.5, ee_short*pix_coeff*0.5);
    b2FixtureDef ee_fixture;
    ee_fixture.isSensor = true;
    ee_fixture.shape = &ee_shape;
    ee_body->CreateFixture(&ee_fixture);
    bodyUserData* ee_body_data = new bodyUserData;
    ee_body_data->id = -1;
    ee_body_data->str = gray_str;	      
    ee_body->SetUserData(ee_body_data);
    ee_body->SetActive(false);
	  
    //Goal Regions
    double red_goal_x_box2d; double red_goal_y_box2d; 
    robot_to_box2d_frame(red_goal.point.x,red_goal.point.y, red_goal_x_box2d, red_goal_y_box2d);
    b2BodyDef goal_body_def;
    goal_body_def.type = b2_staticBody;
    goal_body_def.position.Set(red_goal_x_box2d, red_goal_y_box2d);
    b2Body* goal_body = m_world->CreateBody(&goal_body_def);  	  
    b2CircleShape goal_body_shape;
    goal_body_shape.m_p.Set(0, 0);
    goal_body_shape.m_radius = goal_radiuses[0]*pix_coeff;
    b2FixtureDef goal_fixture_def;
    goal_fixture_def.isSensor = true;
    goal_fixture_def.shape = &goal_body_shape;
    goal_body->CreateFixture(&goal_fixture_def);
    bodyUserData* goal_data_1 = new bodyUserData;
    goal_data_1->id = -1;
    goal_data_1->str = red_goal_str;  
    goal_body->SetUserData(goal_data_1);
	  
    double blue_goal_x_box2d; double blue_goal_y_box2d;
    robot_to_box2d_frame(blue_goal.point.x, blue_goal.point.y, blue_goal_x_box2d, blue_goal_y_box2d);
    b2BodyDef goal_body_def_2;
    goal_body_def_2.type = b2_staticBody;
    goal_body_def_2.position.Set(blue_goal_x_box2d, blue_goal_y_box2d);
    b2Body* goal_body_2 = m_world->CreateBody(&goal_body_def_2);  	  
    b2CircleShape goal_body_shape_2;
    goal_body_shape_2.m_p.Set(0, 0);
    goal_body_shape_2.m_radius = goal_radiuses[1]*pix_coeff;
    b2FixtureDef goal_fixture_def_2;
    goal_fixture_def_2.isSensor = true;
    goal_fixture_def_2.shape = &goal_body_shape_2;
    goal_body_2->CreateFixture(&goal_fixture_def_2);
    bodyUserData* goal_data_2 = new bodyUserData;
    goal_data_2->id = -1;
    goal_data_2->str = blue_goal_str;
    goal_body_2->SetUserData(goal_data_2);	  	  
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
