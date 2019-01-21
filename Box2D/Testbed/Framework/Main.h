/*
* Copyright (c) 2006-2013 Erin Catto http://www.box2d.org
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

#include "imgui.h"
#include "RenderGL3.h"
#include "DebugDraw.h"
#include "Test.h"

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <glew/glew.h>
#endif

#include <glfw/glfw3.h>
#include <stdio.h>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

double camera_y_offset = 15.0f;

//
struct UIState
{
  bool showMenu;
  int scroll;
  int scrollarea1;
  bool mouseOverMenu;
  bool chooseTest;
};

//
namespace
{
  GLFWwindow* mainWindow = NULL;
  UIState ui;

  int32 testIndex = 0;
  int32 testSelection = 0;
  int32 testCount = 0;
  TestEntry* entry;
  Test* test;
  Settings settings;
  bool rightMouseDown;
  b2Vec2 lastp;
}

//
static void sCreateUI()
{
  ui.showMenu = false;
  ui.scroll = 0;
  ui.scrollarea1 = 0;
  ui.chooseTest = false;
  ui.mouseOverMenu = false;
}

//
static void sRestart()
{
  delete test;
  entry = g_testEntries + testIndex;
  test = entry->createFcn();
}

//
static void sSimulate()
{
  glEnable(GL_DEPTH_TEST);
  test->Step(&settings);

  //test->DrawTitle(entry->name);
  glDisable(GL_DEPTH_TEST);

  g_camera.m_center.Set(0.0f, camera_y_offset);
  
  if (testSelection != testIndex)
    {
      testIndex = testSelection;
      delete test;
      entry = g_testEntries + testIndex;
      test = entry->createFcn();
      g_camera.m_zoom = 1.0f;
      //g_camera.m_center.Set(0.0f, 0.0f);
      //g_camera.m_center.Set(0.0f, -12.5);
    }
}

//int main(int argc, char** argv)
static void setup_box2d()
{
  g_camera.m_width = 800;
  g_camera.m_height = 400;
    
  if (glfwInit() == 0) {
    fprintf(stderr, "Failed to initialize GLFW\n");
  }
    
  mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, "Planner", NULL, NULL);
  if (mainWindow == NULL)
    {
      fprintf(stderr, "Failed to open GLFW mainWindow.\n");
      glfwTerminate();
      //return -1;
    }
  glfwMakeContextCurrent(mainWindow);
  printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

#if defined(__APPLE__) == FALSE
  //glewExperimental = GL_TRUE;
  GLenum err = glewInit();
  if (GLEW_OK != err)
    {
      fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      exit(EXIT_FAILURE);
    }
#endif

  
  g_debugDraw.Create();
   
  sCreateUI();  
  testCount = 0;
  while (g_testEntries[testCount].createFcn != NULL)
    {
      ++testCount;
    }

  testIndex = b2Clamp(testIndex, 0, testCount - 1);
  testSelection = testIndex;

  entry = g_testEntries + testIndex;
  test = entry->createFcn();

  
  // Control the frame rate. One draw per monitor refresh.
  glfwSwapInterval(1);

  //double time1 = glfwGetTime();
  //double frameTime = 0.0;
   
  glClearColor(0.9f, 0.9f, 0.9f, 1.f);
  g_camera.m_center.Set(0.0f, camera_y_offset);
  /*
    while (!glfwWindowShouldClose(mainWindow))
    {
    //glfwGetWindowSize(mainWindow, &g_camera.m_width, &g_camera.m_height);
    //glViewport(0, 0, g_camera.m_width, g_camera.m_height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    sSimulate();

    //glEnable(GL_BLEND);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glDisable(GL_DEPTH_TEST);
    //RenderGLFlush(g_camera.m_width, g_camera.m_height);
	
    glfwSwapBuffers(mainWindow);	
    glfwPollEvents();
    }
  */
  //g_debugDraw.Destroy();
  //RenderGLDestroy();
  //glfwTerminate();
}

static void draw_stuff()
{
  glfwGetWindowSize(mainWindow, &g_camera.m_width, &g_camera.m_height);
  glViewport(0, 0, g_camera.m_width, g_camera.m_height);  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  sSimulate();
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_DEPTH_TEST);
  
  //RenderGLFlush(g_camera.m_width, g_camera.m_height);
  glfwSwapBuffers(mainWindow);  
  glfwPollEvents();

  /*
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  sSimulate();
  glfwSwapBuffers(mainWindow);	
  glfwPollEvents();
  */
}
