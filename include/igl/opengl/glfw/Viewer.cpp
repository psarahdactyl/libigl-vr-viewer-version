// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

#include <chrono>
#include <thread>

#include <Eigen/LU>

#include "../gl.h"
#include <GLFW/glfw3.h>

#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>


// Internal global variables used for glfw event handling
static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;

static void glfw_mouse_press(GLFWwindow* window, int button, int action, int modifier)
{

  igl::opengl::glfw::Viewer::MouseButton mb;

  if (button == GLFW_MOUSE_BUTTON_1)
    mb = igl::opengl::glfw::Viewer::MouseButton::Left;
  else if (button == GLFW_MOUSE_BUTTON_2)
    mb = igl::opengl::glfw::Viewer::MouseButton::Right;
  else //if (button == GLFW_MOUSE_BUTTON_3)
    mb = igl::opengl::glfw::Viewer::MouseButton::Middle;

  if (action == GLFW_PRESS)
    __viewer->mouse_down(mb,modifier);
  else
    __viewer->mouse_up(mb,modifier);
}

static void glfw_error_callback(int error, const char* description)
{
  fputs(description, stderr);
}

static void glfw_char_mods_callback(GLFWwindow* window, unsigned int codepoint, int modifier)
{
  __viewer->key_pressed(codepoint, modifier);
}

static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int modifier)
{
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, GL_TRUE);

  if (action == GLFW_PRESS)
    __viewer->key_down(key, modifier);
  else if(action == GLFW_RELEASE)
    __viewer->key_up(key, modifier);
}

static void glfw_window_size(GLFWwindow* window, int width, int height)
{
  int w = width*highdpi;
  int h = height*highdpi;

  __viewer->post_resize(w, h);

}

static void glfw_mouse_move(GLFWwindow* window, double x, double y)
{
  __viewer->mouse_move(x*highdpi, y*highdpi);
}

static void glfw_mouse_scroll(GLFWwindow* window, double x, double y)
{
  using namespace std;
  scroll_x += x;
  scroll_y += y;

  __viewer->mouse_scroll(y);
}

static void glfw_drop_callback(GLFWwindow *window,int count,const char **filenames)
{
}

namespace igl
{
namespace opengl
{
namespace glfw
{

  IGL_INLINE int Viewer::launch(bool resizable /*= true*/, bool fullscreen /*= false*/,
    const std::string &name, int windowWidth /*= 0*/, int windowHeight /*= 0*/)
  {
    // TODO return values are being ignored...
    launch_init(resizable,fullscreen,name,windowWidth,windowHeight);
    launch_rendering(true);
    launch_shut();
    return EXIT_SUCCESS;
  }

  IGL_INLINE int  Viewer::launch_init(bool resizable, bool fullscreen,
    const std::string &name, int windowWidth, int windowHeight)
  {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
    {
      return EXIT_FAILURE;
    }
    glfwWindowHint(GLFW_SAMPLES, 8);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    #ifdef __APPLE__
      glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
      glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif
    if(fullscreen)
    {
      GLFWmonitor *monitor = glfwGetPrimaryMonitor();
      const GLFWvidmode *mode = glfwGetVideoMode(monitor);
      window = glfwCreateWindow(mode->width,mode->height,name.c_str(),monitor,nullptr);
      windowWidth = mode->width;
      windowHeight = mode->height;
    }
    else
    {
      // Set default windows width
      if (windowWidth <= 0 && core_list.size() == 1 && core().viewport[2] > 0)
        windowWidth = core().viewport[2];
      else if (windowWidth <= 0)
        windowWidth = 1280;
      // Set default windows height
      if (windowHeight <= 0 && core_list.size() == 1 && core().viewport[3] > 0)
        windowHeight = core().viewport[3];
      else if (windowHeight <= 0)
        windowHeight = 800;
      window = glfwCreateWindow(windowWidth,windowHeight,name.c_str(),nullptr,nullptr);
    }
    if (!window)
    {
      glfwTerminate();
      return EXIT_FAILURE;
    }
    glfwMakeContextCurrent(window);
    // Load OpenGL and its extensions
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress))
    {
      printf("Failed to load OpenGL and its extensions\n");
      return(-1);
    }
    #if defined(DEBUG) || defined(_DEBUG)
      printf("OpenGL Version %d.%d loaded\n", GLVersion.major, GLVersion.minor);
      int major, minor, rev;
      major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
      minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
      rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
      printf("OpenGL version received: %d.%d.%d\n", major, minor, rev);
      printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
      printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
    #endif
    glfwSetInputMode(window,GLFW_CURSOR,GLFW_CURSOR_NORMAL);
    // Initialize FormScreen
    __viewer = this;
    // Register callbacks
    glfwSetKeyCallback(window, glfw_key_callback);
    glfwSetCursorPosCallback(window,glfw_mouse_move);
    glfwSetWindowSizeCallback(window,glfw_window_size);
    glfwSetMouseButtonCallback(window,glfw_mouse_press);
    glfwSetScrollCallback(window,glfw_mouse_scroll);
    glfwSetCharModsCallback(window,glfw_char_mods_callback);
    glfwSetDropCallback(window,glfw_drop_callback);
    // Handle retina displays (windows and mac)
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    int width_window, height_window;
    glfwGetWindowSize(window, &width_window, &height_window);
    highdpi = windowWidth/width_window;
    glfw_window_size(window,width_window,height_window);
    // Initialize IGL viewer
    init();
    for(auto &core : this->core_list)
    {
      for(auto &data : this->data_list)
      {
        if(data.is_visible & core.id)
        {
          this->core(core.id).align_camera_center(data.V, data.F);
        }
      }
    }
    return EXIT_SUCCESS;
  }

  IGL_INLINE bool Viewer::launch_rendering(bool loop)
  {
    // glfwMakeContextCurrent(window);
    // Rendering loop
    const int num_extra_frames = 5;
    int frame_counter = 0;
    while (!glfwWindowShouldClose(window))
    {
      double tic = get_seconds();
      draw();
      glfwSwapBuffers(window);
      if(core().is_animating || frame_counter++ < num_extra_frames)
      {
        glfwPollEvents();
        // In microseconds
        double duration = 1000000.*(get_seconds()-tic);
        const double min_duration = 1000000./core().animation_max_fps;
        if(duration<min_duration)
        {
          std::this_thread::sleep_for(std::chrono::microseconds((int)(min_duration-duration)));
        }
      }
      else
      {
        glfwWaitEvents();
        frame_counter = 0;
      }
      if (!loop)
        return !glfwWindowShouldClose(window);

      #ifdef __APPLE__
        static bool first_time_hack  = true;
        if(first_time_hack) {
          glfwHideWindow(window);
          glfwShowWindow(window);
          first_time_hack = false;
        }
      #endif
    }
    return EXIT_SUCCESS;
  }

  IGL_INLINE void Viewer::launch_shut()
  {
    for(auto & data : data_list)
    {
      data.meshgl.free();
    }
    core().shut(); // Doesn't do anything
    shutdown_plugins();
    glfwDestroyWindow(window);
    glfwTerminate();
    return;
  }

  IGL_INLINE void Viewer::init()
  {
    core().init(); // Doesn't do anything

    if (callback_init)
      if (callback_init(*this))
        return;

    init_plugins();
  }

  IGL_INLINE void Viewer::init_plugins()
  {
    // Init all plugins
    for (unsigned int i = 0; i<plugins.size(); ++i)
    {
      plugins[i]->init(this);
    }
  }

  IGL_INLINE void Viewer::shutdown_plugins()
  {
    for (unsigned int i = 0; i<plugins.size(); ++i)
    {
      plugins[i]->shutdown();
    }
  }

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
    selected_core_index(0),
    next_core_id(2)
  {
    window = nullptr;
    data_list.front().id = 0;

    core_list.emplace_back(ViewerCore());
    core_list.front().id = 1;

    // Temporary variables initialization
    down = false;
    hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    // C-style callbacks
    callback_init         = nullptr;
    callback_pre_draw     = nullptr;
    callback_post_draw    = nullptr;
    callback_mouse_down   = nullptr;
    callback_mouse_up     = nullptr;
    callback_mouse_move   = nullptr;
    callback_mouse_scroll = nullptr;
    callback_key_down     = nullptr;
    callback_key_up       = nullptr;

    callback_init_data          = nullptr;
    callback_pre_draw_data      = nullptr;
    callback_post_draw_data     = nullptr;
    callback_mouse_down_data    = nullptr;
    callback_mouse_up_data      = nullptr;
    callback_mouse_move_data    = nullptr;
    callback_mouse_scroll_data  = nullptr;
    callback_key_down_data      = nullptr;
    callback_key_up_data        = nullptr;

#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  D,d     Toggle double sided lighting
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  Z       Snap to canonical view
  [,]     Toggle between rotation control types (trackball, two-axis
          valuator with fixed up, 2D mode with no rotation))
  <,>     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // first try to load it with a plugin
    for (unsigned int i = 0; i<plugins.size(); ++i)
    {
      if (plugins[i]->load(mesh_file_name_string))
      {
        return true;
      }
    }

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      data().set_mesh(V,F);
      if(!UV_V.rows() != 0 && UV_F.rows() != 0)
      {
        data().set_uv(UV_V,UV_F);
      }
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    for(int i=0;i<core_list.size(); i++)
        core_list[i].align_camera_center(data().V,data().F);

    for (unsigned int i = 0; i<plugins.size(); ++i)
      if (plugins[i]->post_load())
        return true;

    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    for (unsigned int i = 0; i<plugins.size(); ++i)
      if (plugins[i]->save(mesh_file_name_string))
        return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }

  IGL_INLINE bool Viewer::key_pressed(unsigned int unicode_key,int modifiers)
  {
    for (unsigned int i = 0; i<plugins.size(); ++i)
    {
      if (plugins[i]->key_pressed(unicode_key, modifiers))
      {
        return true;
      }
    }

    if (callback_key_pressed)
      if (callback_key_pressed(*this,unicode_key,modifiers))
        return true;

    switch(unicode_key)
    {
      case 'A':
      case 'a':
      {
        core().is_animating = !core().is_animating;
        return true;
      }
      case 'D':
      case 'd':
      {
        data().double_sided = !data().double_sided;
        return true;
      }
      case 'F':
      case 'f':
      {
        data().set_face_based(!data().face_based);
        return true;
      }
      case 'I':
      case 'i':
      {
        data().dirty |= MeshGL::DIRTY_NORMAL;
        data().invert_normals = !data().invert_normals;
        return true;
      }
      case 'L':
      case 'l':
      {
        core().toggle(data().show_lines);
        return true;
      }
      case 'O':
      case 'o':
      {
        core().orthographic = !core().orthographic;
        return true;
      }
      case 'T':
      case 't':
      {
        core().toggle(data().show_faces);
        return true;
      }
      case 'Z':
      {
        snap_to_canonical_quaternion();
        return true;
      }
      case '[':
      case ']':
      {
        if(core().rotation_type == ViewerCore::ROTATION_TYPE_TRACKBALL)
            core().set_rotation_type(ViewerCore::ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP);
        else
          core().set_rotation_type(ViewerCore::ROTATION_TYPE_TRACKBALL);

        return true;
      }
      case '<':
      case '>':
      {
        selected_data_index =
          (selected_data_index + data_list.size() + (unicode_key=='>'?1:-1))%data_list.size();
        return true;
      }
      case '{':
      case '}':
      {
        selected_core_index =
          (selected_core_index + core_list.size() + (unicode_key=='}'?1:-1))%core_list.size();
        return true;
      }
      case ';':
        data().show_vertid = !data().show_vertid;
        return true;
      case ':':
        data().show_faceid = !data().show_faceid;
        return true;
      default: break;//do nothing
    }
    return false;
  }

  IGL_INLINE bool Viewer::key_down(int key,int modifiers)
  {
    for (unsigned int i = 0; i<plugins.size(); ++i)
      if (plugins[i]->key_down(key, modifiers))
        return true;

    if (callback_key_down)
      if (callback_key_down(*this,key,modifiers))
        return true;

    return false;
  }

  IGL_INLINE bool Viewer::key_up(int key,int modifiers)
  {
    for (unsigned int i = 0; i<plugins.size(); ++i)
      if (plugins[i]->key_up(key, modifiers))
        return true;

    if (callback_key_up)
      if (callback_key_up(*this,key,modifiers))
        return true;

    return false;
  }

  IGL_INLINE void Viewer::select_hovered_core()
  {
    int width_window, height_window;
    glfwGetFramebufferSize(window, &width_window, &height_window);
    for (int i = 0; i < core_list.size(); i++)
    {
      Eigen::Vector4f viewport = core_list[i].viewport;

      if ((current_mouse_x > viewport[0]) &&
          (current_mouse_x < viewport[0] + viewport[2]) &&
          ((height_window - current_mouse_y) > viewport[1]) &&
          ((height_window - current_mouse_y) < viewport[1] + viewport[3]))
      {
        selected_core_index = i;
        break;
      }
    }
  }

  IGL_INLINE bool Viewer::mouse_down(MouseButton button,int modifier)
  {
    // Remember mouse location at down even if used by callback/plugin
    down_mouse_x = current_mouse_x;
    down_mouse_y = current_mouse_y;

    for (unsigned int i = 0; i<plugins.size(); ++i)
      if(plugins[i]->mouse_down(static_cast<int>(button),modifier))
        return true;

    if (callback_mouse_down)
      if (callback_mouse_down(*this,static_cast<int>(button),modifier))
        return true;

    down = true;

    // Select the core containing the click location.
    select_hovered_core();

    down_translation = core().camera_translation;


    // Initialization code for the trackball
    Eigen::RowVector3d center;
    if (data().V.rows() == 0)
    {
      center << 0,0,0;
    }else
    {
      center = data().V.colwise().sum()/data().V.rows();
    }

    Eigen::Vector3f coord =
      igl::project(
        Eigen::Vector3f(center(0),center(1),center(2)),
        core().view,
        core().proj,
        core().viewport);
    down_mouse_z = coord[2];
    down_rotation = core().trackball_angle;

    mouse_mode = MouseMode::Rotation;

    switch (button)
    {
      case MouseButton::Left:
        if (core().rotation_type == ViewerCore::ROTATION_TYPE_NO_ROTATION) {
          mouse_mode = MouseMode::Translation;
        } else {
          mouse_mode = MouseMode::Rotation;
        }
        break;

      case MouseButton::Right:
        mouse_mode = MouseMode::Translation;
        break;

      default:
        mouse_mode = MouseMode::None;
        break;
    }

    return true;
  }

  IGL_INLINE bool Viewer::mouse_up(MouseButton button,int modifier)
  {
    down = false;

    for (unsigned int i = 0; i<plugins.size(); ++i)
      if(plugins[i]->mouse_up(static_cast<int>(button),modifier))
          return true;

    if (callback_mouse_up)
      if (callback_mouse_up(*this,static_cast<int>(button),modifier))
        return true;

    mouse_mode = MouseMode::None;

    return true;
  }

  IGL_INLINE bool Viewer::mouse_move(int mouse_x,int mouse_y)
  {
    if(hack_never_moved)
    {
      down_mouse_x = mouse_x;
      down_mouse_y = mouse_y;
      hack_never_moved = false;
    }
    current_mouse_x = mouse_x;
    current_mouse_y = mouse_y;

    for (unsigned int i = 0; i<plugins.size(); ++i)
      if (plugins[i]->mouse_move(mouse_x, mouse_y))
        return true;

    if (callback_mouse_move)
      if (callback_mouse_move(*this, mouse_x, mouse_y))
        return true;


    if (down)
    {
      // We need the window height to transform the mouse click coordinates into viewport-mouse-click coordinates
      // for igl::trackball and igl::two_axis_valuator_fixed_up
      int width_window, height_window;
      glfwGetFramebufferSize(window, &width_window, &height_window);
      switch (mouse_mode)
      {
        case MouseMode::Rotation:
        {
          switch(core().rotation_type)
          {
            default:
              assert(false && "Unknown rotation type");
            case ViewerCore::ROTATION_TYPE_NO_ROTATION:
              break;
            case ViewerCore::ROTATION_TYPE_TRACKBALL:
              igl::trackball(
                core().viewport(2),
                core().viewport(3),
                2.0f,
                down_rotation,
                down_mouse_x - core().viewport(0),
                down_mouse_y - (height_window - core().viewport(1) - core().viewport(3)),
                mouse_x - core().viewport(0),
                mouse_y - (height_window - core().viewport(1) - core().viewport(3)),
                core().trackball_angle);
              break;
            case ViewerCore::ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP:
              igl::two_axis_valuator_fixed_up(
                core().viewport(2),core().viewport(3),
                2.0,
                down_rotation,
                down_mouse_x - core().viewport(0),
                down_mouse_y - (height_window - core().viewport(1) - core().viewport(3)),
                mouse_x - core().viewport(0),
                mouse_y - (height_window - core().viewport(1) - core().viewport(3)),
                core().trackball_angle);
              break;
          }
          //Eigen::Vector4f snapq = core().trackball_angle;

          break;
        }

        case MouseMode::Translation:
        {
          //translation
          Eigen::Vector3f pos1 = igl::unproject(Eigen::Vector3f(mouse_x, core().viewport[3] - mouse_y, down_mouse_z), core().view, core().proj, core().viewport);
          Eigen::Vector3f pos0 = igl::unproject(Eigen::Vector3f(down_mouse_x, core().viewport[3] - down_mouse_y, down_mouse_z), core().view, core().proj, core().viewport);

          Eigen::Vector3f diff = pos1 - pos0;
          core().camera_translation = down_translation + Eigen::Vector3f(diff[0],diff[1],diff[2]);

          break;
        }
        case MouseMode::Zoom:
        {
          float delta = 0.001f * (mouse_x - down_mouse_x + mouse_y - down_mouse_y);
          core().camera_zoom *= 1 + delta;
          down_mouse_x = mouse_x;
          down_mouse_y = mouse_y;
          break;
        }

        default:
          break;
      }
    }
    return true;
  }

  IGL_INLINE bool Viewer::mouse_scroll(float delta_y)
  {
    // Direct the scrolling operation to the appropriate viewport
    // (unless the core selection is locked by an ongoing mouse interaction).
    if (!down)
      select_hovered_core();
    scroll_position += delta_y;

    for (unsigned int i = 0; i<plugins.size(); ++i)
      if (plugins[i]->mouse_scroll(delta_y))
        return true;

    if (callback_mouse_scroll)
      if (callback_mouse_scroll(*this,delta_y))
        return true;

    // Only zoom if there's actually a change
    if(delta_y != 0)
    {
      float mult = (1.0+((delta_y>0)?1.:-1.)*0.05);
      const float min_zoom = 0.1f;
      core().camera_zoom = (core().camera_zoom * mult > min_zoom ? core().camera_zoom * mult : min_zoom);
    }
    return true;
  }

  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
    igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::draw()
  {
    using namespace std;
    using namespace Eigen;


    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    int width_window, height_window;
    glfwGetWindowSize(window, &width_window, &height_window);

    auto highdpi_tmp = (width_window == 0 ||  width == 0) ? highdpi : (width/width_window);

    if(fabs(highdpi_tmp-highdpi)>1e-8)
    {
      post_resize(width, height);
      highdpi=highdpi_tmp;
    }

    for (auto& core : core_list)
    {
      core.clear_framebuffers();
    }

    for (unsigned int i = 0; i<plugins.size(); ++i)
    {
      if (plugins[i]->pre_draw())
      {
        return;
      }
    }
    if (callback_pre_draw)
    {
      if (callback_pre_draw(*this))
      {
        return;
      }
    }

    for (auto& core : core_list)
    {
      for (auto& mesh : data_list)
      {
        if (mesh.is_visible & core.id)
        {
            if (core.vr) {
                core.drawVR(mesh);
            }
            else {
                core.draw(mesh);
            }
        }
      }
    }
    for (unsigned int i = 0; i<plugins.size(); ++i)
    {
      if (plugins[i]->post_draw())
      {
        break;
      }
    }
    if (callback_post_draw)
    {
      if (callback_post_draw(*this))
      {
        return;
      }
    }
  }

  IGL_INLINE void Viewer::resize(int w,int h)
  {
    if (window) {
      glfwSetWindowSize(window, w/highdpi, h/highdpi);
    }
    post_resize(w, h);
  }

  IGL_INLINE void Viewer::post_resize(int w,int h)
  {
    if (core_list.size() == 1)
    {
      core().viewport = Eigen::Vector4f(0,0,w,h);
    }
    else
    {
      // It is up to the user to define the behavior of the post_resize() function
      // when there are multiple viewports (through the `callback_post_resize` callback)
    }
    for (unsigned int i = 0; i<plugins.size(); ++i)
    {
      plugins[i]->post_resize(w, h);
    }
    if (callback_post_resize)
    {
      callback_post_resize(*this, w, h);
    }
  }

  IGL_INLINE void Viewer::snap_to_canonical_quaternion()
  {
    Eigen::Quaternionf snapq = this->core().trackball_angle;
    igl::snap_to_canonical_view_quat(snapq,1.0f,this->core().trackball_angle);
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;

    this->load_mesh_from_file(fname.c_str());
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    if (visible)
        for (int i = 0; i < core_list.size(); i++)
            data_list.back().set_visible(true, core_list[i].id);
    else
        data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

  IGL_INLINE ViewerCore& Viewer::core(unsigned core_id /*= 0*/)
  {
    assert(!core_list.empty() && "core_list should never be empty");
    int core_index;
    if (core_id == 0)
      core_index = selected_core_index;
    else
      core_index = this->core_index(core_id);
    assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
    return core_list[core_index];
  }

  IGL_INLINE const ViewerCore& Viewer::core(unsigned core_id /*= 0*/) const
  {
    assert(!core_list.empty() && "core_list should never be empty");
    int core_index;
    if (core_id == 0)
      core_index = selected_core_index;
    else
      core_index = this->core_index(core_id);
    assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
    return core_list[core_index];
  }

  IGL_INLINE bool Viewer::erase_core(const size_t index)
  {
    assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if (core_list.size() == 1)
    {
      // Cannot remove last viewport
      return false;
    }
    core_list[index].shut(); // does nothing
    core_list.erase(core_list.begin() + index);
    if (selected_core_index >= index && selected_core_index > 0)
    {
      selected_core_index--;
    }
    return true;
  }

  IGL_INLINE size_t Viewer::core_index(const int id) const {
    for (size_t i = 0; i < core_list.size(); ++i)
    {
      if (core_list[i].id == id)
        return i;
    }
    return 0;
  }

  IGL_INLINE int Viewer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
  {
    core_list.push_back(core()); // copies the previous active core and only changes the viewport
    core_list.back().viewport = viewport;
    core_list.back().id = next_core_id;
    next_core_id <<= 1;
    if (!append_empty)
    {
      for (auto &data : data_list)
      {
        data.set_visible(true, core_list.back().id);
        data.copy_options(core(), core_list.back());
      }
    }
    selected_core_index = core_list.size()-1;
    return core_list.back().id;
  }

  IGL_INLINE int Viewer::append_vrcore(VRApplication* VRapp, Eigen::Vector4f viewport)
  {
      core_list.emplace_back(ViewerCore(VRapp));
      core_list.back().viewport = viewport;
      core_list.back().id = next_core_id;
      next_core_id <<= 1;
      for (auto& data : data_list)
      {
          data.set_visible(true, core_list.back().id);
          data.copy_options(core(), core_list.back());
      }
      selected_core_index = core_list.size() - 1;
      return core_list.back().id;
  }

} // end namespace

//-----------------------------------------------------------------------------
// Purpose: helper to get a string from a tracked device property and turn it
//			into a std::string
//-----------------------------------------------------------------------------
std::string VRApplication::GetTrackedDeviceString(vr::IVRSystem* pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError* peError)
{
    uint32_t requiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
    if (requiredBufferLen == 0)
        return "";

    char* pchBuffer = new char[requiredBufferLen];
    requiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, requiredBufferLen, peError);
    std::string sResult = pchBuffer;
    delete[] pchBuffer;

    return sResult;
}

//-----------------------------------------------------------------------------
// Purpose: helper to get a string from a tracked device type class
//-----------------------------------------------------------------------------
std::string VRApplication::GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class) {

    std::string str_td_class = "Unknown class";

    switch (td_class)
    {
    case vr::TrackedDeviceClass_Invalid:			// = 0, the ID was not valid.
        str_td_class = "invalid";
        break;
    case vr::TrackedDeviceClass_HMD:				// = 1, Head-Mounted Displays
        str_td_class = "hmd";
        break;
    case vr::TrackedDeviceClass_Controller:			// = 2, Tracked controllers
        str_td_class = "controller";
        break;
    case vr::TrackedDeviceClass_GenericTracker:		// = 3, Generic trackers, similar to controllers
        str_td_class = "generic tracker";
        break;
    case vr::TrackedDeviceClass_TrackingReference:	// = 4, Camera and base stations that serve as tracking reference points
        str_td_class = "base station";
        break;
    case vr::TrackedDeviceClass_DisplayRedirect:	// = 5, Accessories that aren't necessarily tracked themselves, but may redirect video output from other tracked devices
        str_td_class = "display redirect";
        break;
    }

    return str_td_class;
}

VRApplication::VRApplication() {
    initOpenVR();
}

IGL_INLINE void VRApplication::initOpenVR() {
    vr::EVRInitError err = vr::VRInitError_None;
    hmd = vr::VR_Init(&err, vr::VRApplication_Scene);

    if (err != vr::VRInitError_None)
    {
        handleVRError(err);
    }

    std::clog << GetTrackedDeviceString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String) << std::endl;
    std::clog << GetTrackedDeviceString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String) << std::endl;

    const std::string& driver = GetTrackedDeviceString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
    const std::string& model = GetTrackedDeviceString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_ModelNumber_String);
    const std::string& serial = GetTrackedDeviceString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);
    const float freq = hmd->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);

    //get the proper resolution of the hmd
    hmd->GetRecommendedRenderTargetSize(&hmdWidth, &hmdHeight);

    fprintf(stderr, "HMD: %s '%s' #%s (%d x %d @ %g Hz)\n", driver.c_str(), model.c_str(), serial.c_str(), hmdWidth, hmdHeight, freq);

    const vr::HmdMatrix34_t& ltMatrix = hmd->GetEyeToHeadTransform(vr::Eye_Left);
    const vr::HmdMatrix34_t& rtMatrix = hmd->GetEyeToHeadTransform(vr::Eye_Right);

    lEyeMat = convertMatrix(ltMatrix);
    //lEyeMat = lEyeMat.reverse().eval();	

    rEyeMat = convertMatrix(rtMatrix);
    //rEyeMat = rEyeMat.reverse().eval();

    const vr::HmdMatrix44_t& rtProj = hmd->GetProjectionMatrix(vr::Eye_Right, nearPlaneZ, farPlaneZ);
    const vr::HmdMatrix44_t& ltProj = hmd->GetProjectionMatrix(vr::Eye_Left, nearPlaneZ, farPlaneZ);

    lProjectionMat = convertMatrix(rtProj);

    printf("l projection: \n%.3f, %.3f, %.3f, %.3f\n%.3f, %.3f, %.3f, %.3f\n%.3f, %.3f, %.3f, %.3f\n%.3f, %.3f, %.3f, %.3f\n\n",
        lProjectionMat(0, 0), lProjectionMat(0, 1), lProjectionMat(0, 2), lProjectionMat(0, 3),
        lProjectionMat(1, 0), lProjectionMat(1, 1), lProjectionMat(1, 2), lProjectionMat(1, 3),
        lProjectionMat(2, 0), lProjectionMat(2, 1), lProjectionMat(2, 2), lProjectionMat(2, 3),
        lProjectionMat(3, 0), lProjectionMat(3, 1), lProjectionMat(3, 2), lProjectionMat(3, 3));

    rProjectionMat = convertMatrix(ltProj);

    vr::VRInput()->SetActionManifestPath("F:/GitHub/libigl-vr-viewer/build/vr_actions.json");

    vr::VRInput()->GetActionSetHandle("/actions/demo", &m_actionsetDemo);

    vr::VRInput()->GetActionHandle("/actions/demo/out/Haptic_Left", &m_rHand[Left].m_actionHaptic);
    vr::VRInput()->GetInputSourceHandle("/user/hand/left", &m_rHand[Left].m_source);
    vr::VRInput()->GetActionHandle("/actions/demo/in/Hand_Left", &m_rHand[Left].m_actionPose);

    vr::VRInput()->GetActionHandle("/actions/demo/out/Haptic_Right", &m_rHand[Right].m_actionHaptic);
    vr::VRInput()->GetInputSourceHandle("/user/hand/right", &m_rHand[Right].m_source);
    vr::VRInput()->GetActionHandle("/actions/demo/in/Hand_Right", &m_rHand[Right].m_actionPose);
    // Initialize the compositor
    vr::IVRCompositor* compositor = vr::VRCompositor();
    if (!compositor) {
        fprintf(stderr, "OpenVR Compositor initialization failed. See log file for details\n");
        vr::VR_Shutdown();
        assert("VR failed" && false);
    }
}

void VRApplication::handleVRError(vr::EVRInitError err)
{
    throw std::runtime_error(vr::VR_GetVRInitErrorAsEnglishDescription(err));
}


Eigen::Matrix4f VRApplication::convertMatrix(vr::HmdMatrix34_t vrmat) {
    Eigen::Matrix4f mat;
    mat <<
        vrmat.m[0][0], vrmat.m[0][1], vrmat.m[0][2], vrmat.m[0][3],
        vrmat.m[1][0], vrmat.m[1][1], vrmat.m[1][2], vrmat.m[1][3],
        vrmat.m[2][0], vrmat.m[2][1], vrmat.m[2][2], vrmat.m[2][3],
        0.0, 0.0, 0.0, 1.0f;

    //mat <<
    //    vrmat.m[0][0], vrmat.m[1][0], vrmat.m[2][0], 0.0,
    //    vrmat.m[0][1], vrmat.m[1][1], vrmat.m[2][1], 0.0,
    //    vrmat.m[0][2], vrmat.m[1][2], vrmat.m[2][2], 0.0,
    //    vrmat.m[0][3], vrmat.m[1][3], vrmat.m[2][3], 1.0f;
    return mat;
}

Eigen::Matrix4f VRApplication::convertMatrix(vr::HmdMatrix44_t vrmat) {
    Eigen::Matrix4f mat;
    mat <<
        vrmat.m[0][0], vrmat.m[0][1], vrmat.m[0][2], vrmat.m[0][3],
        vrmat.m[1][0], vrmat.m[1][1], vrmat.m[1][2], vrmat.m[1][3],
        vrmat.m[2][0], vrmat.m[2][1], vrmat.m[2][2], vrmat.m[2][3],
        vrmat.m[3][0], vrmat.m[3][1], vrmat.m[3][2], vrmat.m[3][3];

    return mat;
}

IGL_INLINE void VRApplication::updatePose() {
    vr::VRCompositor()->WaitGetPoses(trackedDevicePose, vr::k_unMaxTrackedDeviceCount, nullptr, 0);

    validPoseCount = 0;
    //m_strPoseClasses = "";
    for (int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice)
    {
        if (trackedDevicePose[nDevice].bPoseIsValid)
        {
            validPoseCount++;
            mat4DevicePose[nDevice] = convertMatrix(trackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
            if (m_rDevClassChar[nDevice] == 0)
            {
                switch (hmd->GetTrackedDeviceClass(nDevice))
                {
                case vr::TrackedDeviceClass_Controller:        m_rDevClassChar[nDevice] = 'C'; break;
                case vr::TrackedDeviceClass_HMD:               m_rDevClassChar[nDevice] = 'H'; break;
                case vr::TrackedDeviceClass_Invalid:           m_rDevClassChar[nDevice] = 'I'; break;
                case vr::TrackedDeviceClass_GenericTracker:    m_rDevClassChar[nDevice] = 'G'; break;
                case vr::TrackedDeviceClass_TrackingReference: m_rDevClassChar[nDevice] = 'T'; break;
                default:                                       m_rDevClassChar[nDevice] = '?'; break;
                }
            }
            //m_strPoseClasses += m_rDevClassChar[nDevice];
        }
    }

    if (trackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
    {
        hmdPose = mat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd];
        hmdPose = hmdPose.inverse().eval();
    }
}

IGL_INLINE Eigen::Matrix4f VRApplication::getMatrixPoseHmd() {
    return hmdPose;
}

IGL_INLINE Eigen::Matrix4f VRApplication::getMatrixProjectionEye(vr::EVREye eye) {
    return convertMatrix(hmd->GetProjectionMatrix(eye, nearPlaneZ, farPlaneZ));
}

IGL_INLINE Eigen::Matrix4f VRApplication::getMatrixPoseEye(vr::EVREye eye) {
    return convertMatrix(hmd->GetEyeToHeadTransform(eye)).inverse().eval();

    //const vr::HmdMatrix34_t head = m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking;
    //vr::HmdMatrix34_t eyeFromHead;
    //eyeFromHead = hmd->GetEyeToHeadTransform(eye);
    //Eigen::Matrix4f temp, headMat;
    //temp = convertMatrix(eyeFromHead);
    //headMat = convertMatrix(head);
    //Eigen::Matrix4f res = temp * headMat;
    //return res;
}

IGL_INLINE Eigen::Quaternionf VRApplication::EigenGetRotation(Eigen::Matrix4f matrix) {
    Eigen::Quaternionf q;

    q.w() = sqrt(fmax(0, 1 + matrix(0, 0) + matrix(1, 1) + matrix(2, 2))) / 2;
    q.x() = sqrt(fmax(0, 1 + matrix(0, 0) - matrix(1, 1) - matrix(2, 2))) / 2;
    q.y() = sqrt(fmax(0, 1 - matrix(0, 0) + matrix(1, 1) - matrix(2, 2))) / 2;
    q.z() = sqrt(fmax(0, 1 - matrix(0, 0) - matrix(1, 1) + matrix(2, 2))) / 2;
    q.x() = -1 * copysign(q.x(), matrix(1, 2) - matrix(2, 1));
    q.y() = -1 * copysign(q.y(), matrix(2, 0) - matrix(0, 2));
    q.z() = -1 * copysign(q.z(), matrix(0, 1) - matrix(1, 0));

    //printf("%.3f, %.3f, %.3f, %.3f ", q.x(), q.y(), q.z(), q.w());
    return q;
}

IGL_INLINE Eigen::Vector3f VRApplication::GetPosition(Eigen::Matrix4f matrix) {
    Eigen::Vector3f vector;
    vector[0] = matrix(3, 0);
    vector[1] = (matrix(3, 1) - 1.2);
    vector[2] = matrix(3, 2);
    //printf("%.3f, %.3f, %.3f\n", vector[0], vector[1], vector[2]);
    //printf("%.3f, ", matrix.m[3][3]);

    return vector;
}

IGL_INLINE void VRApplication::submitToHMD() {

    //vr::EColorSpace colorSpace = vr::ColorSpace_Gamma;
    vr::Texture_t leftEyeTexture = { (void*)(uintptr_t)leftEyeDesc.resolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
    vr::Texture_t rightEyeTexture = { (void*)(uintptr_t)rightEyeDesc.resolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };

    //vr::Texture_t lt = { (void*)(uintptr_t)ltEyeTexture, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };

    //vr::Texture_t lt = { reinterpret_cast<void*>(intptr_t(lTexture)), vr::TextureType_OpenGL, colorSpace };
    vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);

    //vr::Texture_t rt = { (void*)(uintptr_t)rtEyeTexture, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };


    //vr::Texture_t rt = { reinterpret_cast<void*>(intptr_t(rTexture)), vr::TextureType_OpenGL, colorSpace };
    vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);

    // Tell the compositor to begin work immediately instead of waiting for the next WaitGetPoses() call
    vr::VRCompositor()->PostPresentHandoff();
    //glClearColor(0, 0, 0, 1);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

IGL_INLINE void VRApplication::shut() {
    if (hmd) {
        vr::VR_Shutdown();
        hmd = NULL;
    }
    if (companionWindowProgramID)
    {
        glDeleteProgram(companionWindowProgramID);
    }

    //delete leftEyeDesc;
    glDeleteRenderbuffers(1, &leftEyeDesc.depthBufferId);
    glDeleteTextures(1, &leftEyeDesc.renderTextureId);
    glDeleteFramebuffers(1, &leftEyeDesc.renderFramebufferId);
    glDeleteTextures(1, &leftEyeDesc.resolveTextureId);
    glDeleteFramebuffers(1, &leftEyeDesc.resolveFramebufferId);

    //delete rightEyeDesc;
    glDeleteRenderbuffers(1, &rightEyeDesc.depthBufferId);
    glDeleteTextures(1, &rightEyeDesc.renderTextureId);
    glDeleteFramebuffers(1, &rightEyeDesc.renderFramebufferId);
    glDeleteTextures(1, &rightEyeDesc.resolveTextureId);
    glDeleteFramebuffers(1, &rightEyeDesc.resolveFramebufferId);

    if (companionWindowVAO != 0)
    {
        glDeleteVertexArrays(1, &companionWindowVAO);
    }

}

IGL_INLINE int VRApplication::getHmdWidth() {
    return hmdWidth;
}

IGL_INLINE int VRApplication::getHmdHeight() {
    return hmdHeight;
}

IGL_INLINE void VRApplication::updateCompanionWindow(Eigen::Vector4f viewport) {
    //companion window
    glDisable(GL_DEPTH_TEST);
    glViewport(viewport(0), viewport(1), viewport(2), viewport(3));

    glBindVertexArray(companionWindowVAO);
    glUseProgram(companionWindowProgramID);

    // render left eye (first half of index array )
    glBindTexture(GL_TEXTURE_2D, leftEyeDesc.resolveTextureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glDrawElements(GL_TRIANGLES, companionWindowIndexSize / 2, GL_UNSIGNED_SHORT, 0);

    // render right eye (second half of index array )
    glBindTexture(GL_TEXTURE_2D, rightEyeDesc.resolveTextureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glDrawElements(GL_TRIANGLES, companionWindowIndexSize / 2, GL_UNSIGNED_SHORT, (const void*)(uintptr_t)(companionWindowIndexSize));

    glBindVertexArray(0);
    glUseProgram(0);
}

IGL_INLINE void VRApplication::predraw(vr::EVREye eye) {

    glEnable(GL_MULTISAMPLE);

    if(eye == vr::EVREye::Eye_Left)
        glBindFramebuffer(GL_FRAMEBUFFER, leftEyeDesc.renderFramebufferId);
    else
        glBindFramebuffer(GL_FRAMEBUFFER, rightEyeDesc.renderFramebufferId);
    glClearColor(0.3,
        0.3,
        0.5,
        1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

IGL_INLINE void VRApplication::postdraw(vr::EVREye eye) {

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glDisable(GL_MULTISAMPLE);

    if (eye == vr::EVREye::Eye_Left) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, leftEyeDesc.renderFramebufferId);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, leftEyeDesc.resolveFramebufferId);
    }
    else {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, rightEyeDesc.renderFramebufferId);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, rightEyeDesc.resolveFramebufferId);
    }

    glBlitFramebuffer(0, 0, hmdWidth, hmdHeight, 0, 0, hmdWidth, hmdHeight,
        GL_COLOR_BUFFER_BIT,
        GL_LINEAR);

    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}

IGL_INLINE void VRApplication::printstuff() {
    printf("\naddres: %x\n", &leftEyeDesc);

    //if(leftEyeDesc != NULL)
        //printf("after init: %u\n", leftEyeDesc.renderFramebufferId);
}

IGL_INLINE void VRApplication::initGl() {
    if (!createFrameBuffer(leftEyeDesc)) {
        printf("Error creating frame buffers for left eye");
    }
    else {
        printf("Success creating frame buffers for left eye");
    }


    if (!createFrameBuffer(rightEyeDesc)) {
        printf("Error creating frame buffers for right eye");
    }
    else {
        printf("Success creating frame buffers for right eye");
    }
    setupCompanionWindow();

    create_shader_program(
        // vertex shader
        "#version 410\n"
        "uniform mat4 matrix;\n"
        "layout(location = 0) in vec4 position;\n"
        "layout(location = 1) in vec3 v3ColorIn;\n"
        "out vec4 v4Color;\n"
        "void main()\n"
        "{\n"
        "	v4Color.xyz = v3ColorIn; v4Color.a = 1.0;\n"
        "	gl_Position = matrix * position;\n"
        "}\n",

        // fragment shader
        "#version 410\n"
        "in vec4 v4Color;\n"
        "out vec4 outputColor;\n"
        "void main()\n"
        "{\n"
        "   outputColor = v4Color;\n"
        "}\n", {},
        controllerTransformProgramID
    );

    controllerMatrixLocation = glGetUniformLocation(controllerTransformProgramID, "matrix");
    if (controllerMatrixLocation == -1)
    {
        printf("Unable to find matrix uniform in controller shader\n");
    }
}

IGL_INLINE bool VRApplication::createFrameBuffer(FramebufferDesc& framebufferDesc) {
    printf("Creating framebuffers\n");


    glGenFramebuffers(1, &framebufferDesc.renderFramebufferId);
    glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.renderFramebufferId);

    // create a multisampled color attachment texture
    glGenTextures(1, &framebufferDesc.renderTextureId);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.renderTextureId);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA, hmdWidth, hmdHeight, GL_TRUE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.renderTextureId, 0);

    // create a (also multisampled) renderbuffer object for depth and stencil attachments
    glGenRenderbuffers(1, &framebufferDesc.depthBufferId);
    glBindRenderbuffer(GL_RENDERBUFFER, framebufferDesc.depthBufferId);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH24_STENCIL8, hmdWidth, hmdHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, framebufferDesc.depthBufferId);

    glGenFramebuffers(1, &framebufferDesc.resolveFramebufferId);
    glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.resolveFramebufferId);

    glGenTextures(1, &framebufferDesc.resolveTextureId);
    glBindTexture(GL_TEXTURE_2D, framebufferDesc.resolveTextureId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, hmdWidth, hmdHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, framebufferDesc.resolveTextureId, 0);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    //glBindRenderbuffer(GL_RENDERBUFFER, 0);
    //glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);

    return true;
}

IGL_INLINE void VRApplication::setupCompanionWindow()
{
    std::vector<VertexDataWindow> vVerts;

    // left eye verts
    vVerts.push_back(VertexDataWindow(Vector2(-1, -1), Vector2(0, 0)));
    vVerts.push_back(VertexDataWindow(Vector2(0, -1), Vector2(1, 0)));
    vVerts.push_back(VertexDataWindow(Vector2(-1, 1), Vector2(0, 1)));
    vVerts.push_back(VertexDataWindow(Vector2(0, 1), Vector2(1, 1)));

    // right eye verts
    vVerts.push_back(VertexDataWindow(Vector2(0, -1), Vector2(0, 0)));
    vVerts.push_back(VertexDataWindow(Vector2(1, -1), Vector2(1, 0)));
    vVerts.push_back(VertexDataWindow(Vector2(0, 1), Vector2(0, 1)));
    vVerts.push_back(VertexDataWindow(Vector2(1, 1), Vector2(1, 1)));

    GLushort vIndices[] = { 0, 1, 3,   0, 3, 2,   4, 5, 7,   4, 7, 6 };
    companionWindowIndexSize = _countof(vIndices);

    glGenVertexArrays(1, &companionWindowVAO);
    glBindVertexArray(companionWindowVAO);

    glGenBuffers(1, &companionWindowIDVertBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, companionWindowIDVertBuffer);
    glBufferData(GL_ARRAY_BUFFER, vVerts.size() * sizeof(VertexDataWindow), &vVerts[0], GL_STATIC_DRAW);

    glGenBuffers(1, &companionWindowIDIndexBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, companionWindowIDIndexBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, companionWindowIndexSize * sizeof(GLushort), &vIndices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_TRUE, sizeof(VertexDataWindow), (void*)offsetof(VertexDataWindow, position));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_TRUE, sizeof(VertexDataWindow), (void*)offsetof(VertexDataWindow, texCoord));

    glBindVertexArray(0);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    create_shader_program(
        // vertex shader
        "#version 410 core\n"
        "layout(location = 0) in vec4 position;\n"
        "layout(location = 1) in vec2 v2UVIn;\n"
        "noperspective out vec2 v2UV;\n"
        "void main()\n"
        "{\n"
        "	v2UV = v2UVIn;\n"
        "	gl_Position = position;\n"
        "}\n",

        // fragment shader
        "#version 410 core\n"
        "uniform sampler2D mytexture;\n"
        "noperspective in vec2 v2UV;\n"
        "out vec4 outputColor;\n"
        "void main()\n"
        "{\n"
        "		outputColor = texture(mytexture, v2UV);\n"
        "}\n"
        , {},
        companionWindowProgramID
    );
}

IGL_INLINE void VRApplication::drawControllerAxes(Eigen::Matrix4f viewProjectionMatrix) {
    if (!hmd->IsInputAvailable())
        return;
    // draw the controller axis lines
    glUseProgram(controllerTransformProgramID);
    glUniformMatrix4fv(controllerMatrixLocation, 1, GL_FALSE, viewProjectionMatrix.data());
    glBindVertexArray(controllerVAO);
    glDrawArrays(GL_LINES, 0, controllerVertCount);
    glBindVertexArray(0);
}

IGL_INLINE void VRApplication::renderControllerAxes() {
    if (!hmd->IsInputAvailable())
        return;

    std::vector<float> vertdataarray;

    controllerVertCount = 0;
    trackedControllerCount = 0;

    for (EHand eHand = Left; eHand <= Right; ((int&)eHand)++)
    {
        if (!m_rHand[eHand].m_bShowController)
            continue;

        Eigen::Matrix4f & mat = m_rHand[eHand].m_rmat4Pose;
        Eigen::Vector4f center = mat * Eigen::Vector4f(0, 0, 0, 1);
        
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3f color(0, 0, 0);
            Eigen::Vector4f point(0, 0, 0, 1);
            point[i] += 0.05f;
            color[i] = 1.0;
            point = mat * point;
            vertdataarray.push_back(center(0));
            vertdataarray.push_back(center(1));
            vertdataarray.push_back(center(2));

            vertdataarray.push_back(color(0));
            vertdataarray.push_back(color(1));
            vertdataarray.push_back(color(2));

            vertdataarray.push_back(point(0));
            vertdataarray.push_back(point(1));
            vertdataarray.push_back(point(2));

            vertdataarray.push_back(color(0));
            vertdataarray.push_back(color(1));
            vertdataarray.push_back(color(2));

            controllerVertCount += 2;
            
        }
        Eigen::Vector4f start = mat * Eigen::Vector4f(0, 0, -0.02f, 1);
        Eigen::Vector4f end = mat * Eigen::Vector4f(0, 0, -39.f, 1);
        Eigen::Vector3f color(.92f, .92f, .71f);
        vertdataarray.push_back(start(0)); vertdataarray.push_back(start(1)); vertdataarray.push_back(start(2));
        vertdataarray.push_back(color(0)); vertdataarray.push_back(color(1)); vertdataarray.push_back(color(2));

        vertdataarray.push_back(end(0)); vertdataarray.push_back(end(1)); vertdataarray.push_back(end(2));
        vertdataarray.push_back(color(0)); vertdataarray.push_back(color(1)); vertdataarray.push_back(color(2));
        controllerVertCount += 2;
    }
    // Setup the VAO the first time through.
    if (controllerVAO == 0)
    {
        glGenVertexArrays(1, &controllerVAO);
        glBindVertexArray(controllerVAO);

        glGenBuffers(1, &controllerVertBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, controllerVertBuffer);

        GLuint stride = 2 * 3 * sizeof(float);
        uintptr_t offset = 0;

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (const void*)offset);

        offset += sizeof(Eigen::Vector3f);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (const void*)offset);

        glBindVertexArray(0);
    }

    glBindBuffer(GL_ARRAY_BUFFER, controllerVertBuffer);

    // set vertex data if we have some
    if (vertdataarray.size() > 0)
    {
        //$ TODO: Use glBufferSubData for this...
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STREAM_DRAW);
    }

}

IGL_INLINE void VRApplication::handleInput() {
    // Process SteamVR events
    vr::VREvent_t event;
    while (hmd->PollNextEvent(&event, sizeof(event)))
    {
        //ProcessVREvent(event);
    }

    // Process SteamVR action state
    // UpdateActionState is called each frame to update the state of the actions themselves. The application
    // controls which action sets are active with the provided array of VRActiveActionSet_t structs.
    vr::VRActiveActionSet_t actionSet = { 0 };
    actionSet.ulActionSet = m_actionsetDemo;
    vr::VRInput()->UpdateActionState(&actionSet, sizeof(actionSet), 1);

    m_rHand[Left].m_bShowController = true;
    m_rHand[Right].m_bShowController = true;

    for (EHand eHand = Left; eHand <= Right; ((int&)eHand)++)
    {
        vr::InputPoseActionData_t poseData;

        if (vr::VRInput()->GetPoseActionDataForNextFrame(m_rHand[eHand].m_actionPose, vr::TrackingUniverseStanding, &poseData, sizeof(poseData), vr::k_ulInvalidInputValueHandle) != vr::VRInputError_None
            || !poseData.bActive || !poseData.pose.bPoseIsValid)
        {
            m_rHand[eHand].m_bShowController = false;
        }
        else
        {
            m_rHand[eHand].m_rmat4Pose = convertMatrix(poseData.pose.mDeviceToAbsoluteTracking);

            vr::InputOriginInfo_t originInfo;
            if (vr::VRInput()->GetOriginTrackedDeviceInfo(poseData.activeOrigin, &originInfo, sizeof(originInfo)) == vr::VRInputError_None
                && originInfo.trackedDeviceIndex != vr::k_unTrackedDeviceIndexInvalid)
            {
                std::string sRenderModelName = GetTrackedDeviceString(hmd, originInfo.trackedDeviceIndex, vr::Prop_RenderModelName_String);
                if (sRenderModelName != m_rHand[eHand].m_sRenderModelName)
                {
                    m_rHand[eHand].m_sRenderModelName = sRenderModelName;
                }
            }
        }
    }

}

} // end namespace

}
