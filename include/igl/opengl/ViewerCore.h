// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_OPENGL_VIEWERCORE_H
#define IGL_OPENGL_VIEWERCORE_H

#include <igl/opengl/MeshGL.h>

#include <igl/igl_inline.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "glfw/Viewer.h"

#include <openvr.h>

namespace igl
{
namespace opengl
{


    class VRApplication {

        struct FramebufferDesc
        {
            unsigned int depthBufferId;
            unsigned int renderTextureId;
            unsigned int renderFramebufferId;
            unsigned int resolveTextureId;
            unsigned int resolveFramebufferId;
        };

        struct Vector2 {
            float x;
            float y;

            // ctors
            Vector2() : x(0), y(0) {};
            Vector2(float x, float y) : x(x), y(y) {};

            // utils functions
            void        set(float x, float y);
            float       length() const;                         //
            float       distance(const Vector2& vec) const;     // distance between two vectors
            Vector2& normalize();                            //
            float       dot(const Vector2& vec) const;          // dot product
            bool        equal(const Vector2& vec, float e) const; // compare with epsilon

            // operators
            Vector2     operator-() const;                      // unary operator (negate)
            Vector2     operator+(const Vector2& rhs) const;    // add rhs
            Vector2     operator-(const Vector2& rhs) const;    // subtract rhs
            Vector2& operator+=(const Vector2& rhs);         // add rhs and update this object
            Vector2& operator-=(const Vector2& rhs);         // subtract rhs and update this object
            Vector2     operator*(const float scale) const;     // scale
            Vector2     operator*(const Vector2& rhs) const;    // multiply each element
            Vector2& operator*=(const float scale);          // scale and update this object
            Vector2& operator*=(const Vector2& rhs);         // multiply each element and update this object
            Vector2     operator/(const float scale) const;     // inverse scale
            Vector2& operator/=(const float scale);          // scale and update this object
            bool        operator==(const Vector2& rhs) const;   // exact compare, no epsilon
            bool        operator!=(const Vector2& rhs) const;   // exact compare, no epsilon
            bool        operator<(const Vector2& rhs) const;    // comparison for sort
            float       operator[](int index) const;            // subscript operator v[0], v[1]
            float& operator[](int index);                  // subscript operator v[0], v[1]

            friend Vector2 operator*(const float a, const Vector2 vec);
            friend std::ostream& operator<<(std::ostream& os, const Vector2& vec);
        };

        struct VertexDataWindow
        {
            Vector2 position;
            Vector2 texCoord;

            VertexDataWindow(const Vector2& pos, const Vector2 tex) : position(pos), texCoord(tex) {	}
        };

        vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];
        Eigen::Matrix4f mat4DevicePose[vr::k_unMaxTrackedDeviceCount];
        char m_rDevClassChar[vr::k_unMaxTrackedDeviceCount];   // for each device, a character representing its class
        int validPoseCount;

        Eigen::Matrix4f hmdPose, lEyeMat, rEyeMat, lProjectionMat, rProjectionMat;


        Eigen::Matrix4f convertMatrix(vr::HmdMatrix34_t);
        Eigen::Matrix4f convertMatrix(vr::HmdMatrix44_t);


        std::string GetTrackedDeviceString(vr::IVRSystem*, vr::TrackedDeviceIndex_t, vr::TrackedDeviceProperty, vr::TrackedPropertyError* peError = NULL);
        std::string GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class);

        std::string getHMDString(vr::IVRSystem* pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError* peError = nullptr);
        vr::IVRSystem* hmd = nullptr;
        void handleVRError(vr::EVRInitError);
        void initOpenVR();



        int companionWindowIndexSize;
        GLuint companionWindowVAO, companionWindowIDVertBuffer, companionWindowIDIndexBuffer, companionWindowProgramID;

        IGL_INLINE bool createFrameBuffer(FramebufferDesc& framebufferDesc);
        IGL_INLINE void setupCompanionWindow();

        float nearPlaneZ = 0.05f;
        float farPlaneZ = 100.0f;
        uint32_t hmdWidth = 1280, hmdHeight = 720;
        GLuint lTexture, rTexture;

        FramebufferDesc leftEyeDesc;
        FramebufferDesc rightEyeDesc;
    public:
        IGL_INLINE Eigen::Matrix4f getMatrixPoseEye(vr::EVREye);
        IGL_INLINE Eigen::Matrix4f getMatrixProjectionEye(vr::EVREye);
        IGL_INLINE Eigen::Matrix4f getMatrixPoseHmd();

        IGL_INLINE Eigen::Vector3f GetPosition(Eigen::Matrix4f);
        IGL_INLINE Eigen::Quaternionf EigenGetRotation(Eigen::Matrix4f);

        IGL_INLINE void printstuff();
        IGL_INLINE void predraw(vr::EVREye);
        IGL_INLINE void postdraw(vr::EVREye);
        IGL_INLINE void updatePose();
        IGL_INLINE void submitToHMD();
        IGL_INLINE int getHmdWidth();
        IGL_INLINE int getHmdHeight();
        IGL_INLINE VRApplication();
        IGL_INLINE void initGl();
        IGL_INLINE void updateCompanionWindow(Eigen::Vector4f);
        IGL_INLINE void shut();
    };

// Forward declaration
class ViewerData;

// Basic class of the 3D mesh viewer
// TODO: write documentation
class ViewerCore
{
public:
    bool vr = false;

    VRApplication *VRapp;

    IGL_INLINE ViewerCore(VRApplication*);

  IGL_INLINE ViewerCore();

  // Initialization
  IGL_INLINE void init();

  // Shutdown
  IGL_INLINE void shut();

  // Serialization code
  IGL_INLINE void InitSerialization();

  // ------------------- Camera control functions

  // Adjust the view to see the entire model
  IGL_INLINE void align_camera_center(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F);

  // Determines how much to zoom and shift such that the mesh fills the unit
  // box (centered at the origin)
  IGL_INLINE void get_scale_and_shift_to_fit_mesh(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    float & zoom,
    Eigen::Vector3f& shift);

    // Adjust the view to see the entire model
    IGL_INLINE void align_camera_center(
      const Eigen::MatrixXd& V);

    // Determines how much to zoom and shift such that the mesh fills the unit
    // box (centered at the origin)
    IGL_INLINE void get_scale_and_shift_to_fit_mesh(
      const Eigen::MatrixXd& V,
      float & zoom,
      Eigen::Vector3f& shift);

  // ------------------- Drawing functions

  // Clear the frame buffers
  IGL_INLINE void clear_framebuffers();

  // Draw everything
  //
  // data cannot be const because it is being set to "clean"
  IGL_INLINE void drawVR(ViewerData& data);
  IGL_INLINE void draw(ViewerData& data, bool update_matrices = true);
  IGL_INLINE void draw_buffer(
    ViewerData& data,
    bool update_matrices,
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& R,
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& G,
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& B,
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& A);

  // Trackball angle (quaternion)
  enum RotationType
  {
    ROTATION_TYPE_TRACKBALL = 0,
    ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP = 1,
    ROTATION_TYPE_NO_ROTATION = 2,
    NUM_ROTATION_TYPES = 3
  };
  IGL_INLINE void set_rotation_type(const RotationType & value);

  // ------------------- Option helpers

  // Set a ViewerData visualization option for this viewport
  IGL_INLINE void set(unsigned int &property_mask, bool value = true) const;

  // Unset a ViewerData visualization option for this viewport
  IGL_INLINE void unset(unsigned int &property_mask) const;

  // Toggle a ViewerData visualization option for this viewport
  IGL_INLINE void toggle(unsigned int &property_mask) const;

  // Check whether a ViewerData visualization option is set for this viewport
  IGL_INLINE bool is_set(unsigned int property_mask) const;

  // ------------------- Properties

  // Unique identifier
  unsigned int id = 1u;

  // Colors
  Eigen::Vector4f background_color;

  // Lighting
  Eigen::Vector3f light_position;
  float lighting_factor;

  RotationType rotation_type;
  Eigen::Quaternionf trackball_angle;

  // Camera parameters
  float camera_base_zoom;
  float camera_zoom;
  bool orthographic;
  Eigen::Vector3f camera_base_translation;
  Eigen::Vector3f camera_translation;
  Eigen::Vector3f camera_eye;
  Eigen::Vector3f camera_up;
  Eigen::Vector3f camera_center;
  float camera_view_angle;
  float camera_dnear;
  float camera_dfar;

  bool depth_test;

  // Animation
  bool is_animating;
  double animation_max_fps;

  // Caches the two-norm between the min/max point of the bounding box
  float object_scale;

  // Viewport size
  Eigen::Vector4f viewport;

  // Save the OpenGL transformation matrices used for the previous rendering pass
  Eigen::Matrix4f view;
  Eigen::Matrix4f proj;
  Eigen::Matrix4f norm;
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}
}

#include <igl/serialize.h>
namespace igl {
  namespace serialization {

    inline void serialization(bool s, igl::opengl::ViewerCore& obj, std::vector<char>& buffer)
    {

      SERIALIZE_MEMBER(background_color);

      SERIALIZE_MEMBER(light_position);
      SERIALIZE_MEMBER(lighting_factor);

      SERIALIZE_MEMBER(trackball_angle);
      SERIALIZE_MEMBER(rotation_type);

      SERIALIZE_MEMBER(camera_base_zoom);
      SERIALIZE_MEMBER(camera_zoom);
      SERIALIZE_MEMBER(orthographic);
      SERIALIZE_MEMBER(camera_base_translation);
      SERIALIZE_MEMBER(camera_translation);
      SERIALIZE_MEMBER(camera_view_angle);
      SERIALIZE_MEMBER(camera_dnear);
      SERIALIZE_MEMBER(camera_dfar);
      SERIALIZE_MEMBER(camera_eye);
      SERIALIZE_MEMBER(camera_center);
      SERIALIZE_MEMBER(camera_up);

      SERIALIZE_MEMBER(depth_test);
      SERIALIZE_MEMBER(is_animating);
      SERIALIZE_MEMBER(animation_max_fps);

      SERIALIZE_MEMBER(object_scale);

      SERIALIZE_MEMBER(viewport);
      SERIALIZE_MEMBER(view);
      SERIALIZE_MEMBER(proj);
      SERIALIZE_MEMBER(norm);
    }

    template<>
    inline void serialize(const igl::opengl::ViewerCore& obj, std::vector<char>& buffer)
    {
      serialization(true, const_cast<igl::opengl::ViewerCore&>(obj), buffer);
    }

    template<>
    inline void deserialize(igl::opengl::ViewerCore& obj, const std::vector<char>& buffer)
    {
      serialization(false, obj, const_cast<std::vector<char>&>(buffer));
    }
  }
}

#ifndef IGL_STATIC_LIBRARY
#  include "ViewerCore.cpp"
#endif

#endif
