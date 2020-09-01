
#ifndef IGL_OPENVR_VRAPPLICATION_H
#define IGL_OPENVR_VRAPPLICATION_H
#include <openvr.h>
#include "../igl_inline.h"
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace igl
{
  namespace openvr
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
        int validPoseCount; unsigned int controllerVertCount, trackedControllerCount;

        Eigen::Matrix4f hmdPose, lEyeMat, rEyeMat, lProjectionMat, rProjectionMat;


        Eigen::Matrix4f convertMatrix(vr::HmdMatrix34_t);
        Eigen::Matrix4f convertMatrix(vr::HmdMatrix44_t);


        std::string GetTrackedDeviceString(vr::IVRSystem*, vr::TrackedDeviceIndex_t, vr::TrackedDeviceProperty, vr::TrackedPropertyError* peError = NULL);
        std::string GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class);

        std::string getHMDString(vr::IVRSystem* pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError* peError = nullptr);
        vr::IVRSystem* hmd = nullptr;
        void handleVRError(vr::EVRInitError);
        void initOpenVR();

        unsigned int controllerVertBuffer;
        unsigned int controllerVAO = 0;
        struct ControllerInfo_t
        {
            vr::VRInputValueHandle_t m_source = vr::k_ulInvalidInputValueHandle;
            vr::VRActionHandle_t m_actionPose = vr::k_ulInvalidActionHandle;
            vr::VRActionHandle_t m_actionHaptic = vr::k_ulInvalidActionHandle;
            Eigen::Matrix4f m_rmat4Pose;
            //ViewerData m_pRenderModel;
            std::string m_sRenderModelName;
            bool m_bShowController;
        };
        enum EHand
        {
            Left = 0,
            Right = 1,
        };
        ControllerInfo_t m_rHand[2];
        unsigned int controllerTransformProgramID, controllerMatrixLocation;

        vr::VRActionSetHandle_t m_actionsetDemo = vr::k_ulInvalidActionSetHandle;//TODO: Delete me

        int companionWindowIndexSize;
        unsigned int companionWindowVAO, companionWindowIDVertBuffer, companionWindowIDIndexBuffer;
        unsigned int companionWindowProgramID;

        IGL_INLINE bool createFrameBuffer(FramebufferDesc& framebufferDesc);
        IGL_INLINE void setupCompanionWindow();

        float nearPlaneZ = 0.05f;
        float farPlaneZ = 100.0f;
        uint32_t hmdWidth = 1280, hmdHeight = 720;
        unsigned int lTexture, rTexture;

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
        IGL_INLINE void renderControllerAxes();
        IGL_INLINE void drawControllerAxes(Eigen::Matrix4f, Eigen::Matrix4f);
        IGL_INLINE void handleInput();
    };
  }
}

#ifndef IGL_STATIC_LIBRARY
#  include "VRApplication.cpp"
#endif

#endif