#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

// System
#include <stdlib.h>
#include <iostream>
#include <string>
#include <random>
#include <ctime>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <push_vs_grasp/object.h>

#include <opencv2/opencv.hpp>

struct BodyState
{
  b2Vec2 position;
  double radius;
  std::string color;
  int id;
};

struct WorldState
{
  std::vector<BodyState> bodies;
};

struct PushDef
{
  b2Vec2 push_start;
  b2Vec2 push_end;
};

void print_push_def(PushDef d)
{
  
  
}

class ApplyForce : public Test
{
private:
  std::vector<geometry_msgs::PointStamped> centroids;
  std::vector<double> radiuses;
  std::vector<double> goal_radiuses;
  std::vector<std::string> colors;
  std::string red_str = "red";
  std::string blue_str = "blue";
  std::string red_goal_str = "red_goal";
  std::string blue_goal_str = "blue_goal";
  std::string gray_str = "gray";
  void* red_ptr;
  void* blue_ptr;
  void* red_goal_ptr;
  void* blue_goal_ptr;
  void* gray_ptr;
  double pix_coeff = 50.0;
  double ee_long = 0.32;
  double ee_short = 0.08;
  b2Body* ee_body;
  b2Body* robot_base_body;
  b2Body* red_goal_body;
  b2Body* blue_goal_body;
  b2Body* ground;
  b2Body* ground2;

public:
  ApplyForce()
    : red_ptr(&red_str)
    , blue_ptr(&blue_str)
    , red_goal_ptr(&red_goal_str)
    , blue_goal_ptr(&blue_goal_str)
    , gray_ptr(&gray_str)
  {
  }

   void reset(std::vector<geometry_msgs::PointStamped> centroids, std::vector<double> radiuses,
             std::vector<std::string> colors, geometry_msgs::PointStamped red_goal,
             geometry_msgs::PointStamped blue_goal, std::vector<double> goal_radiuses, bool transform)
  {
    destroy_all_objects();
    setup_objects(centroids, radiuses, colors, red_goal, blue_goal, goal_radiuses, transform);
    




  }

  void run_safely(const std::function<void()>& f)
  {
    while (true)
    {
      try
      {
        f();
        return;
      }
      catch (...)
      {
        continue;
      }
    }
  }

  void destroy_all_objects()
  {
    for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
    {
      auto destroy = [&]() { m_world->DestroyBody(b); };
      run_safely(destroy);
    }
  }

  void robot_to_box2d_frame(const double x_in, const double y_in, double& x_out, double& y_out)
  {
    x_out = y_in * pix_coeff;
    y_out = -x_in * pix_coeff;
    return;
  }

  void box2d_to_img(const double x_in, const double y_in, double& x_out, double& y_out)
  {
    x_out = (x_in + 37.5) + 10;
    y_out = -(y_in + 10) + 50;
    return;
  }

  void box2d_to_robot_frame(const double x_in, const double y_in, double& x_out, double& y_out)
  {
    x_out = -y_in / pix_coeff;
    y_out = x_in / pix_coeff;
    return;
  }

  b2Vec2 calc_linear_velocity2(float start_x, float start_y, float end_x, float end_y, double mag, double out_magnitude)
  {
    b2Vec2 linear_velocity;

    linear_velocity.x = (end_x - start_x) * out_magnitude/mag;
    linear_velocity.y = (end_y - start_y) * out_magnitude/mag;

    return linear_velocity;
  }

    double calc_angle2(float start_x, float start_y, float end_x, float end_y)
  {
    double diff_x = start_x - end_x;
    double diff_y = start_y - end_y;
    return atan2(diff_y, diff_x) + M_PI / 2;
  }

  double get_dist_between_bodies(b2Body* b_from, b2Body* b_to)
  {
    double diff_x = b_from->GetPosition().x - b_to->GetPosition().x;
    double diff_y = b_from->GetPosition().y - b_to->GetPosition().y;
    double mag = sqrt(diff_x * diff_x + diff_y * diff_y);
    return mag;
  }

  double get_dist_between_point(float start_x, float start_y, float end_x, float end_y)
  {
    double diff_x = start_x - end_x;
    double diff_y = start_y - end_y;
    double mag = sqrt(diff_x * diff_x + diff_y * diff_y);
    return mag;
  }

  double get_dist_moved(float x_start, float y_start, b2Body* b_to)
  {
    double diff_x = x_start - b_to->GetPosition().x;
    double diff_y = y_start - b_to->GetPosition().y;
    double mag = sqrt(diff_x * diff_x + diff_y * diff_y);
    return mag;
  }
  bool coll_check(b2Body* cur_body) {
    //check with tabletop objects
    b2Fixture* f_1 = cur_body->GetFixtureList();
    for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
      if (get_body_id(cur_body) == get_body_id(b) || get_body_id(b) < 0) 
	continue;
      b2Fixture* f_2 = b->GetFixtureList();
      bool overlap = b2TestOverlap(f_1->GetShape(), 0, f_2->GetShape(), 0,  cur_body->GetTransform(), b->GetTransform());
      if (overlap)
	return true;
    }
    return coll_check_with_robot_base(cur_body);
}

  int get_body_id(b2Body* b) {
    if (b->GetUserData() != NULL) {
      bodyUserData* udStruct = (bodyUserData*)b->GetUserData();
      return udStruct->id;
    }
    return -1;
  }  
  std::vector<push_vs_grasp::object> get_all_objects() {
      std::vector<push_vs_grasp::object> result;
        for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
          if (get_body_color(b) == "red" || get_body_color(b) == "blue") {
              b2Vec2 pos = b->GetPosition();
              push_vs_grasp::object curr;
              //double x_out, y_out;
              //box2d_to_img(pos.x, pos.y, x_out, y_out);

              curr.x = pos.x; //x_out;
              curr.y = pos.y; //y_out;
              curr.is_red = get_body_color(b) == "red";
              result.push_back(curr);
          }

        }
        return result;
    }


  cv::Mat push_action(float start_x, float start_y, float end_x, float end_y, double& out_dist, std::vector<push_vs_grasp::object> &objects)
  {
    std::cout << "1" << std::endl;
    b2Vec2 position = b2Vec2(start_x, start_y);
        std::cout << "2" << std::endl;
    red_goal_body = get_objects(red_goal_str)[0];
    blue_goal_body = get_objects(blue_goal_str)[0];
    std::cout << "3" << std::endl;
   
  

    float prev_reward = calc_heuristic();

    PushDef push_def;
    double goal_threshold = 0.02;
    double ee_multiplier = 1.25;

<<<<<<< HEAD
    std::cout << "4" << std::endl;
    double magnitude = 0.5; //* (dist > 0 ? +1 : -1);
=======
    double magnitude = 0.4; //* (dist > 0 ? +1 : -1);
>>>>>>> 535ecdd2aa58056e42139b275a08bef8371c30fb

    float angle = calc_angle2(start_x, start_y, end_x, end_y);
    float dist = get_dist_between_point(start_x, start_y, end_x, end_y);
    std::cout << "5" << std::endl;
    b2Vec2 linear_velocity = calc_linear_velocity2(start_x, start_y, end_x, end_y, dist, magnitude);

    set_sensor_status(ee_body, false);
    set_obj_dimensions(ee_body, ee_long * ee_multiplier, ee_short * ee_multiplier);
    std::cout << "6" << std::endl;
          ee_body->SetTransform(position, angle);

    auto set_pos = [&]() {
          std::cout << "7" << std::endl;
    };
    run_safely(set_pos);

<<<<<<< HEAD
      if (coll_check(ee_body)) {
            std::cout << "8" << std::endl;
        cv::Mat data = get_ocv_img_from_gl_img();
        out_dist = calc_heuristic() - prev_reward;
        objects = get_all_objects();

=======
    for (int i = 0; i < 10000; i++) {
      // std::cout << "moving object" << std::endl;
      position.x -= linear_velocity.x;
      position.y -= linear_velocity.y;
            
      auto set_pos = [&]() {
      ee_body->SetTransform(position, angle);
      };
      run_safely(set_pos);

      if (!coll_check(ee_body)) {
        start_x = position.x;
        start_y = position.y;
        dist = get_dist_between_point(start_x, start_y, end_x, end_y);
        // std::cout << "DIST" << dist << std::endl;
        break;
      }
>>>>>>> 535ecdd2aa58056e42139b275a08bef8371c30fb

    }
      // std::cout << "coll check: " << coll_check(ee_body) << std::endl;

      // if (coll_check(ee_body)) {
        
      //   cv::Mat data = get_ocv_img_from_gl_img();
      //   objects = get_all_objects();

      //   out_dist = calc_heuristic() - prev_reward;

      // return data;
      //   }

      auto set_velo = [&]() {
        ee_body->SetActive(true);
        ee_body->SetLinearVelocity(linear_velocity);
    std::cout << "9" << std::endl;
      };

      run_safely(set_velo);

  using namespace std;

<<<<<<< HEAD
    std::cout << "10" << std::endl;

=======
    clock_t begin = clock();
>>>>>>> 535ecdd2aa58056e42139b275a08bef8371c30fb
    push_def.push_start = ee_body->GetPosition();
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    while (elapsed_secs  < 60)
    {
          std::cout << "11" << std::endl;
      double d = dist - get_dist_moved(start_x, start_y, ee_body);

<<<<<<< HEAD
      // std::cout << ee_body->GetLinearVelocity().Length();
    std::cout << d << "," << dist << std::endl;

    std::cout << "12" << std::endl;
=======
>>>>>>> 535ecdd2aa58056e42139b275a08bef8371c30fb
      bool collision = coll_check_with_robot_base(ee_body);
    // std::cout << d << "," <<  goal_threshold << "," << pix_coeff << std::endl;

      // std::cout << "velocity: " << ee_body->GetLinearVelocity().Length() << std::endl;

      if (d < goal_threshold * pix_coeff || collision || ee_body->GetLinearVelocity().Length() == 0 ) //at goal or stuck
      {
        // std::cout << "d < goal_threshold * pix_coeff" << (d < goal_threshold * pix_coeff) << std::endl;
        // std::cout << "collision" << collision << std::endl;
        // std::cout << "ee_body->GetLinearVelocity().Length() == 0" << (ee_body->GetLinearVelocity().Length() == 0) << std::endl;
        auto set_finished = [&]() {
          ee_body->SetLinearVelocity(b2Vec2(0, 0));
          set_sensor_status(ee_body, true);
              std::cout << "13" << std::endl;
        };
        run_safely(set_finished);

        break;
      }
      else
      {
        auto set_cont_velocity = [&]() { ee_body->SetLinearVelocity(linear_velocity); };
        run_safely(set_cont_velocity);
            std::cout << "14" << std::endl;
      }
      end = clock();
    elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    }

    std::cout  << ee_body->GetPosition().x << "," << ee_body->GetPosition().y << std::endl;
    std::cout << "15" << std::endl;
    push_def.push_end = ee_body->GetPosition();

    auto set_inactive = [&]() { ee_body->SetActive(false); };
    run_safely(set_inactive);
    std::cout << "16" << std::endl;
    cv::Mat data = get_ocv_img_from_gl_img();

    out_dist = calc_heuristic() - prev_reward;
    std::cout << "17" << std::endl;
    std::cout << "current reward" << calc_heuristic() << std::endl;

    std::cout << "difference" << out_dist << std::endl;

    objects = get_all_objects();

    std::cout << "18" << std::endl;

    return data;
  }


  cv::Mat get_ocv_img_from_gl_img()
  {
    cv::Mat img(60, 100, CV_8UC3, cv::Scalar(0, 0, 0));

    for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
    {
      cv::Scalar colour;

      if (get_body_color(b) == "red")
      {
        colour = cv::Scalar(0, 0, 255);

        double x_out, y_out;
        box2d_to_img(b->GetPosition().x, b->GetPosition().y, x_out, y_out);
        cv::circle(img, cv::Point(x_out, y_out), 0.08 * pix_coeff, colour, -1);
      }
      else if (get_body_color(b) == "blue")
      {
        colour = cv::Scalar(255, 0, 0);

        double x_out, y_out;
        box2d_to_img(b->GetPosition().x, b->GetPosition().y, x_out, y_out);
        cv::circle(img, cv::Point(x_out, y_out), 0.08 * pix_coeff, colour, -1);
      }
    }

    return img;
  }

  double calc_heuristic()
  {
    double dist = 0.0;
    double d = 0.0;
    for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
    {
      if (get_body_color(b) == "red" || get_body_color(b) == "blue")
      {
        
        //    double diff_x = b_from->GetPosition().x - b_to->GetPosition().x;
        if (b->GetPosition().x > 37.5 || b->GetPosition().x < -37.5 || b->GetPosition().y < 0 ||
            b->GetPosition().y > 35)
        {
          std::cout << b->GetPosition().x << "," << b->GetPosition().y << std::endl;
          std::cout << get_body_color(b) << "is off the table..." << std::endl;
          d = -10000;
        }
        else if (get_dist_between_bodies(b, get_body_color(b) == "blue" ? blue_goal_body : red_goal_body) < 10)
        {
         
          d = 1;
        }
        else
        {
          d = -get_dist_between_bodies(b, get_body_color(b) == "blue" ? blue_goal_body : red_goal_body);
        }

        dist = dist + d;

      }
    }
      
      return dist;
    }

   void set_obj_dimensions(b2Body * b, double dim_long, double dim_short)
    {
      b2Fixture* F = b->GetFixtureList();
      b2PolygonShape* poly = (b2PolygonShape*)F->GetShape();
      poly->SetAsBox(dim_long * pix_coeff * 0.5, dim_short * pix_coeff * 0.5);
    }

    bool coll_check_with_robot_base(b2Body * cur_body)
    {
      // check with robot base
      b2Fixture* f_1 = cur_body->GetFixtureList();
      b2Fixture* f_2 = robot_base_body->GetFixtureList();
      bool overlap = b2TestOverlap(f_1->GetShape(), 0, f_2->GetShape(), 0, cur_body->GetTransform(),
                                   robot_base_body->GetTransform());
      if (overlap)
        return true;
      return false;
    }

    void set_sensor_status(b2Body * body, bool isSensor)
    {
      b2Fixture* fixture = body->GetFixtureList();
      while (fixture != NULL)
      {
        fixture->SetSensor(isSensor);
        fixture = fixture->GetNext();
      }
    }

   
    float RandomFloat(float a, float b)
    {
      float random = ((float)rand()) / (float)RAND_MAX;
      float diff = b - a;
      float r = random * diff;
      return a + r;
    }
    std::vector<b2Body*> get_objects(std::string str_in)
    {
      std::vector<b2Body*> result;
      for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
      {
        if (get_body_color(b) == str_in)
        {
          result.push_back(b);
        }
      }
      return result;
    }

    

   
    std::string get_body_color(b2Body * b)
    {
      if (b->GetUserData() != NULL)
      {
        bodyUserData* udStruct = (bodyUserData*)b->GetUserData();
        std::string color = udStruct->str;
        return color;
      }
      return "null";
    }

   void setup_objects(std::vector<geometry_msgs::PointStamped> centroids, std::vector<double> radiuses,
                       std::vector<std::string> colors, geometry_msgs::PointStamped red_goal,
                       geometry_msgs::PointStamped blue_goal, std::vector<double> goal_radiuses, bool transform)
    {
      this->centroids = centroids;
      this->colors = colors;
      this->goal_radiuses = goal_radiuses;
      setup_table(red_goal, blue_goal);
      draw_table(centroids, radiuses, colors, red_goal, blue_goal, transform);
    } 

    void draw_table(std::vector<geometry_msgs::PointStamped> centroids, std::vector<double> radiuses,
                    std::vector<std::string> colors, geometry_msgs::PointStamped red_goal,
                    geometry_msgs::PointStamped blue_goal, bool transform)
    {
      for (int i = 0; i < centroids.size(); i++)
      {
        b2BodyDef myBodyDef;
        myBodyDef.type = b2_dynamicBody;  // this will be a dynamic body
        myBodyDef.position.Set(0, 0);     // a little to the left

        double x_out, y_out;
        robot_to_box2d_frame(centroids[i].point.x, centroids[i].point.y, x_out, y_out);
        myBodyDef.position.Set(transform ? x_out : centroids[i].point.x, transform ? y_out : centroids[i].point.y);
        b2Body* dynamicBody1;

        auto create_db1 = [&]() { dynamicBody1 = m_world->CreateBody(&myBodyDef); };
        run_safely(create_db1);

        b2CircleShape circleShape;
        circleShape.m_p.Set(0, 0);                       // position, relative to body position
        circleShape.m_radius = radiuses[i] * pix_coeff;  // radius

        b2FixtureDef myFixtureDef;
        myFixtureDef.friction = 0.99f;
        myFixtureDef.density = 9.0f;
        myFixtureDef.shape = &circleShape;  // this is a pointer to the shape above

        auto create_fdb1 = [&]() {
          dynamicBody1->CreateFixture(&myFixtureDef);
          dynamicBody1->SetActive(true);
        };
        run_safely(create_fdb1);

        b2FrictionJointDef jd;
        jd.localAnchorA.SetZero();
        jd.localAnchorB.SetZero();
        jd.bodyA = ground;
        jd.bodyB = dynamicBody1;
        jd.collideConnected = true;
        jd.maxForce = 5;   // mass * gravity;
        jd.maxTorque = 5;  // mass * radius * grvity;

        auto create_jd = [&]() { m_world->CreateJoint(&jd); };
        run_safely(create_jd);

        bodyUserData* myStruct = new bodyUserData;
        myStruct->id = i;
        if (colors[i] == "red")
        {
          myStruct->str = red_str;
          dynamicBody1->SetUserData(myStruct);
        }
        else
        {
          myStruct->str = blue_str;
          dynamicBody1->SetUserData(myStruct);
        }
      }
    }

    void setup_table(geometry_msgs::PointStamped red_goal, geometry_msgs::PointStamped blue_goal)
    {
      m_world->SetGravity(b2Vec2(0, 0));

      b2EdgeShape shape;
      b2FixtureDef sd;
      sd.shape = &shape;
      sd.friction = 0.99f;
      sd.density = 0.0f;
      sd.restitution = 0.4;

      // b2EdgeShape shape2;
      // b2FixtureDef sd2;
      // sd2.shape = &shape2;
      // sd2.friction = 0.99f;
      // sd2.density = 0.0f;
      // sd2.restitution = 0.4;


      double table_x_len = 1.5;
      double table_y_len = 0.8;
      double robot_base_offset = 0.1;

      // b2BodyDef bd;
      // bd.active = false;
      // //bd.position.Set(0.0f, table_y_len/2-robot_base_offset);
      // bd.position.Set(0.0f, 0.0f);
      b2BodyDef bd;
      bd.position.Set(0.0f, 0.0f);

      // b2BodyDef bd2;
      // bd2.position.Set(0.0f, 0.0f);

      auto create_ground = [&]() { 
        ground = m_world->CreateBody(&bd); };
      //ground2 = m_world->CreateBody(&bd2); };

      run_safely(create_ground);

      //  auto create_ground2f = [&]() {
      //   ground2->CreateFixture(&sd);
      // };



      // // Left vertical
      // shape.Set(pix_coeff * b2Vec2(-table_x_len / 2, -robot_base_offset),
      // pix_coeff * b2Vec2(-table_x_len / 2, table_y_len - robot_base_offset));

      // // shape.Set(pix_coeff * b2Vec2(-table_x_len / 2, -table_y_len / 2),
      // //         pix_coeff * b2Vec2(-table_x_len / 2, table_y_len / 2));

      // // ground2->CreateFixture(&sd);
      // run_safely(create_ground2f);


      // // Right vertical
      // shape.Set(pix_coeff * b2Vec2(table_x_len / 2, -robot_base_offset),
      //           pix_coeff * b2Vec2(table_x_len / 2, table_y_len - robot_base_offset));
      // run_safely(create_ground2f);

      // // Top horizontal
      // shape.Set(pix_coeff * b2Vec2(-table_x_len / 2, table_y_len - robot_base_offset),
      //           pix_coeff * b2Vec2(table_x_len / 2, table_y_len - robot_base_offset));
      // run_safely(create_ground2f);

      // // std::cout<< "TABLE X: " <<pix_coeff*table_x_len/2 << " Y: " << pix_coeff*(table_y_len -
      // // robot_base_offset)<<std::endl;

      // // Bottom horizontal
      // shape.Set(pix_coeff * b2Vec2(-table_x_len / 2, -robot_base_offset),
      //           pix_coeff * b2Vec2(table_x_len / 2, -robot_base_offset));
      // run_safely(create_ground2f);

      auto create_groundf = [&]() {
        ground->CreateFixture(&sd);
        ground->SetActive(true);
       // ground2->SetActive(false);
      };
      run_safely(create_groundf);

      // Robot Base
      b2BodyDef myBodyDef;
      myBodyDef.type = b2_staticBody;
      myBodyDef.position.Set(0.0, 0.0);

      auto create_base = [&]() { robot_base_body = m_world->CreateBody(&myBodyDef); };
      run_safely(create_base);

      b2CircleShape circleShape;
      circleShape.m_p.Set(0, 0);                // position, relative to body position
      circleShape.m_radius = 0.05 * pix_coeff;  // radius
      b2FixtureDef myFixtureDef;
      myFixtureDef.shape = &circleShape;

      // this is a pointer to the shape above
      auto create_rbbf = [&]() {
        robot_base_body->CreateFixture(&myFixtureDef);  // add a fixture to the body

      };
      run_safely(create_rbbf);

      bodyUserData* robot_base_data = new bodyUserData;
      robot_base_data->id = -1;
      robot_base_data->str = gray_str;
      robot_base_body->SetUserData(robot_base_data);

      // Robot End Effector
      b2BodyDef ee_body_def;
      ee_body_def.type = b2_dynamicBody;
      ee_body_def.position.Set(0.0, 0.0);
      
      
      
      auto create_ee_body = [&]() {
      ee_body = m_world->CreateBody(&ee_body_def);
      };
      run_safely(create_ee_body);


      
      b2PolygonShape ee_shape;
      ee_shape.SetAsBox(ee_long * pix_coeff * 0.5, ee_short * pix_coeff * 0.5);
      b2FixtureDef ee_fixture;
      ee_fixture.isSensor = true;
      ee_fixture.shape = &ee_shape;

      auto create_ee = [&]() {
        robot_base_body->CreateFixture(&myFixtureDef);  // add a fixture to the body
        ee_body->CreateFixture(&ee_fixture);
        ee_body->SetActive(false);
      };
      run_safely(create_ee);


      bodyUserData* ee_body_data = new bodyUserData;
      ee_body_data->id = -1;
      ee_body_data->str = gray_str;
      ee_body->SetUserData(ee_body_data);

      

      // Goal Regions
      double red_goal_x_box2d;
      double red_goal_y_box2d;
      robot_to_box2d_frame(red_goal.point.x, red_goal.point.y, red_goal_x_box2d, red_goal_y_box2d);
      b2BodyDef goal_body_def;
      goal_body_def.type = b2_staticBody;
      goal_body_def.position.Set(red_goal_x_box2d, red_goal_y_box2d);
      b2Body* goal_body;

      auto create_body = [&]() { goal_body = m_world->CreateBody(&goal_body_def); };
      run_safely(create_body);

      b2CircleShape goal_body_shape;
      goal_body_shape.m_p.Set(0, 0);
      goal_body_shape.m_radius = goal_radiuses[0] * pix_coeff;
      b2FixtureDef goal_fixture_def;
      goal_fixture_def.isSensor = true;
      goal_fixture_def.shape = &goal_body_shape;

      auto create_g_fixture = [&]() { goal_body->CreateFixture(&goal_fixture_def); };
      run_safely(create_g_fixture);

      bodyUserData* goal_data_1 = new bodyUserData;
      goal_data_1->id = -1;
      goal_data_1->str = red_goal_str;
      goal_body->SetUserData(goal_data_1);

      double blue_goal_x_box2d;
      double blue_goal_y_box2d;
      robot_to_box2d_frame(blue_goal.point.x, blue_goal.point.y, blue_goal_x_box2d, blue_goal_y_box2d);
      b2BodyDef goal_body_def_2;
      goal_body_def_2.type = b2_staticBody;
      goal_body_def_2.position.Set(blue_goal_x_box2d, blue_goal_y_box2d);
      b2Body* goal_body_2;

      auto create_body_2 = [&]() { goal_body_2 = m_world->CreateBody(&goal_body_def_2); };
      run_safely(create_body_2);

      b2CircleShape goal_body_shape_2;
      goal_body_shape_2.m_p.Set(0, 0);
      goal_body_shape_2.m_radius = goal_radiuses[1] * pix_coeff;
      b2FixtureDef goal_fixture_def_2;
      goal_fixture_def_2.isSensor = true;
      goal_fixture_def_2.shape = &goal_body_shape_2;

      auto create_g2_fixture = [&]() { goal_body_2->CreateFixture(&goal_fixture_def_2); };
      run_safely(create_g2_fixture);

      bodyUserData* goal_data_2 = new bodyUserData;
      goal_data_2->id = -1;
      goal_data_2->str = blue_goal_str;
      goal_body_2->SetUserData(goal_data_2);
    }
    ~ApplyForce()
    {
      
    }
    static Test* Create()
    {
      return new ApplyForce;
    }

    b2Body* m_body;
  };

#endif
