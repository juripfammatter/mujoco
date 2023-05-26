#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "stdio.h"

char error[1000];

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// simple controller applying damping to each dof
void mycontroller(const mjModel* m, mjData* d)
{
  if( m->nu==m->nv )
    mju_scl(d->ctrl, d->qvel, -0.1, m->nv);
}


int main(void)
{
   // load model from file and check for errors
   m = mj_loadXML("/Users/juri/git/mujoco/model/simple_box/simple_box.xml", NULL, error, 1000);
   if( !m )
   {
      printf("%s\n", error);
      return 1;
   }

   // make data corresponding to model
   d = mj_makeData(m);


   // init GLFW, create window, make OpenGL context current, request v-sync
   glfwInit();
   GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
   glfwMakeContextCurrent(window);
   glfwSwapInterval(1);

   // initialize visualization data structures
   mjv_defaultCamera(&cam);
   // mjv_defaultPerturb(&pert);
   mjv_defaultOption(&opt);
   mjr_defaultContext(&con);

   // create scene and context
   mjv_makeScene(m, &scn, 1000);
   mjr_makeContext(m, &con, mjFONTSCALE_100);

   // install GLFW mouse and keyboard callbacks
   glfwSetKeyCallback(window, keyboard);
   glfwSetCursorPosCallback(window, mouse_move);
   glfwSetMouseButtonCallback(window, mouse_button);
   glfwSetScrollCallback(window, scroll);

   // install control callback
   mjcb_control = mycontroller;

   // run main loop, target real-time simulation and 60 fps rendering
   while( !glfwWindowShouldClose(window) ) {
      // advance interactive simulation for 1/60 sec
      //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
      //  this loop will finish on time for the next frame to be rendered at 60 fps.
      //  Otherwise add a cpu timer and exit this loop when it is time to render.
      mjtNum simstart = d->time;
      while( d->time - simstart < 1.0/60.0 )
            mj_step(m, d);
            // mj_step1(m, d);
            // // set d->ctrl or d->qfrc_applied or d->xfrc_applied
            // mj_step2(m, d);

      // get framebuffer viewport
      mjrRect viewport = {0, 0, 0, 0};
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

      // update scene and render
      mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
      mjr_render(viewport, &scn, &con);

      // swap OpenGL buffers (blocking call due to v-sync)
      glfwSwapBuffers(window);

      // process pending GUI events, call GLFW callbacks
      glfwPollEvents();
   }


   // close GLFW, free visualization storage
   glfwTerminate();
   mjv_freeScene(&scn);
   mjr_freeContext(&con);

   // // run simulation for 10 seconds
   // while( d->time<10 )
   //    mj_step(m, d);
   //    printf("executed step");

   // free model and data
   mj_deleteData(d);
   mj_deleteModel(m);

   return 0;
}