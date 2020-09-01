#include "VRApplication.h"
#include <string>

namespace igl
{
  namespace openvr
  {

    //-----------------------------------------------------------------------------
    // Purpose: helper to get a string from a tracked device property and turn it
    //			into a std::string
    //-----------------------------------------------------------------------------
    std::string VRApplication::GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError)
    {
      uint32_t requiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
      if (requiredBufferLen == 0)
        return "";

      char *pchBuffer = new char[requiredBufferLen];
      requiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, requiredBufferLen, peError);
      std::string sResult = pchBuffer;
      delete[] pchBuffer;

      return sResult;
    }

    //-----------------------------------------------------------------------------
    // Purpose: helper to get a string from a tracked device type class
    //-----------------------------------------------------------------------------
    std::string VRApplication::GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class)
    {

      std::string str_td_class = "Unknown class";

      switch (td_class)
      {
      case vr::TrackedDeviceClass_Invalid: // = 0, the ID was not valid.
        str_td_class = "invalid";
        break;
      case vr::TrackedDeviceClass_HMD: // = 1, Head-Mounted Displays
        str_td_class = "hmd";
        break;
      case vr::TrackedDeviceClass_Controller: // = 2, Tracked controllers
        str_td_class = "controller";
        break;
      case vr::TrackedDeviceClass_GenericTracker: // = 3, Generic trackers, similar to controllers
        str_td_class = "generic tracker";
        break;
      case vr::TrackedDeviceClass_TrackingReference: // = 4, Camera and base stations that serve as tracking reference points
        str_td_class = "base station";
        break;
      case vr::TrackedDeviceClass_DisplayRedirect: // = 5, Accessories that aren't necessarily tracked themselves, but may redirect video output from other tracked devices
        str_td_class = "display redirect";
        break;
      }

      return str_td_class;
    }

    VRApplication::VRApplication()
    {
      initOpenVR();
    }

    IGL_INLINE void VRApplication::initOpenVR()
    {
      vr::EVRInitError err = vr::VRInitError_None;
      hmd = vr::VR_Init(&err, vr::VRApplication_Scene);

      if (err != vr::VRInitError_None)
      {
        handleVRError(err);
      }

      std::clog << GetTrackedDeviceString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String) << std::endl;
      std::clog << GetTrackedDeviceString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String) << std::endl;

      const std::string &driver = GetTrackedDeviceString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
      const std::string &model = GetTrackedDeviceString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_ModelNumber_String);
      const std::string &serial = GetTrackedDeviceString(hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);
      const float freq = hmd->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);

      //get the proper resolution of the hmd
      hmd->GetRecommendedRenderTargetSize(&hmdWidth, &hmdHeight);

      fprintf(stderr, "HMD: %s '%s' #%s (%d x %d @ %g Hz)\n", driver.c_str(), model.c_str(), serial.c_str(), hmdWidth, hmdHeight, freq);

      const vr::HmdMatrix34_t &ltMatrix = hmd->GetEyeToHeadTransform(vr::Eye_Left);
      const vr::HmdMatrix34_t &rtMatrix = hmd->GetEyeToHeadTransform(vr::Eye_Right);

      lEyeMat = convertMatrix(ltMatrix);
      //lEyeMat = lEyeMat.reverse().eval();

      rEyeMat = convertMatrix(rtMatrix);
      //rEyeMat = rEyeMat.reverse().eval();

      const vr::HmdMatrix44_t &rtProj = hmd->GetProjectionMatrix(vr::Eye_Right, nearPlaneZ, farPlaneZ);
      const vr::HmdMatrix44_t &ltProj = hmd->GetProjectionMatrix(vr::Eye_Left, nearPlaneZ, farPlaneZ);

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
      vr::IVRCompositor *compositor = vr::VRCompositor();
      if (!compositor)
      {
        fprintf(stderr, "OpenVR Compositor initialization failed. See log file for details\n");
        vr::VR_Shutdown();
        assert("VR failed" && false);
      }
    }

    void VRApplication::handleVRError(vr::EVRInitError err)
    {
      throw std::runtime_error(vr::VR_GetVRInitErrorAsEnglishDescription(err));
    }

    Eigen::Matrix4f VRApplication::convertMatrix(vr::HmdMatrix34_t vrmat)
    {
      Eigen::Matrix4f mat;
      mat << vrmat.m[0][0], vrmat.m[0][1], vrmat.m[0][2], vrmat.m[0][3],
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

    Eigen::Matrix4f VRApplication::convertMatrix(vr::HmdMatrix44_t vrmat)
    {
      Eigen::Matrix4f mat;
      mat << vrmat.m[0][0], vrmat.m[0][1], vrmat.m[0][2], vrmat.m[0][3],
          vrmat.m[1][0], vrmat.m[1][1], vrmat.m[1][2], vrmat.m[1][3],
          vrmat.m[2][0], vrmat.m[2][1], vrmat.m[2][2], vrmat.m[2][3],
          vrmat.m[3][0], vrmat.m[3][1], vrmat.m[3][2], vrmat.m[3][3];

      return mat;
    }

    IGL_INLINE void VRApplication::updatePose()
    {
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
            case vr::TrackedDeviceClass_Controller:
              m_rDevClassChar[nDevice] = 'C';
              break;
            case vr::TrackedDeviceClass_HMD:
              m_rDevClassChar[nDevice] = 'H';
              break;
            case vr::TrackedDeviceClass_Invalid:
              m_rDevClassChar[nDevice] = 'I';
              break;
            case vr::TrackedDeviceClass_GenericTracker:
              m_rDevClassChar[nDevice] = 'G';
              break;
            case vr::TrackedDeviceClass_TrackingReference:
              m_rDevClassChar[nDevice] = 'T';
              break;
            default:
              m_rDevClassChar[nDevice] = '?';
              break;
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

    IGL_INLINE Eigen::Matrix4f VRApplication::getMatrixPoseHmd()
    {
      return hmdPose;
    }

    IGL_INLINE Eigen::Matrix4f VRApplication::getMatrixProjectionEye(vr::EVREye eye)
    {
      return convertMatrix(hmd->GetProjectionMatrix(eye, nearPlaneZ, farPlaneZ));
    }

    IGL_INLINE Eigen::Matrix4f VRApplication::getMatrixPoseEye(vr::EVREye eye)
    {
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

    IGL_INLINE Eigen::Quaternionf VRApplication::EigenGetRotation(Eigen::Matrix4f matrix)
    {
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

    IGL_INLINE Eigen::Vector3f VRApplication::GetPosition(Eigen::Matrix4f matrix)
    {
      Eigen::Vector3f vector;
      vector[0] = matrix(3, 0);
      vector[1] = (matrix(3, 1) - 1.2);
      vector[2] = matrix(3, 2);
      //printf("%.3f, %.3f, %.3f\n", vector[0], vector[1], vector[2]);
      //printf("%.3f, ", matrix.m[3][3]);

      return vector;
    }

    IGL_INLINE void VRApplication::submitToHMD()
    {

      //vr::EColorSpace colorSpace = vr::ColorSpace_Gamma;
      vr::Texture_t leftEyeTexture = {(void *)(uintptr_t)leftEyeDesc.resolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma};
      vr::Texture_t rightEyeTexture = {(void *)(uintptr_t)rightEyeDesc.resolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma};

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

    IGL_INLINE void VRApplication::shut()
    {
      if (hmd)
      {
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

    IGL_INLINE int VRApplication::getHmdWidth()
    {
      return hmdWidth;
    }

    IGL_INLINE int VRApplication::getHmdHeight()
    {
      return hmdHeight;
    }

    IGL_INLINE void VRApplication::updateCompanionWindow(Eigen::Vector4f viewport)
    {
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
      glDrawElements(GL_TRIANGLES, companionWindowIndexSize / 2, GL_UNSIGNED_SHORT, (const void *)(uintptr_t)(companionWindowIndexSize));

      glBindVertexArray(0);
      glUseProgram(0);
    }

    IGL_INLINE void VRApplication::predraw(vr::EVREye eye)
    {

      glEnable(GL_MULTISAMPLE);

      if (eye == vr::EVREye::Eye_Left)
        glBindFramebuffer(GL_FRAMEBUFFER, leftEyeDesc.renderFramebufferId);
      else
        glBindFramebuffer(GL_FRAMEBUFFER, rightEyeDesc.renderFramebufferId);
      glClearColor(0.3,
                   0.3,
                   0.5,
                   1.0);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    IGL_INLINE void VRApplication::postdraw(vr::EVREye eye)
    {

      glBindFramebuffer(GL_FRAMEBUFFER, 0);

      glDisable(GL_MULTISAMPLE);

      if (eye == vr::EVREye::Eye_Left)
      {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, leftEyeDesc.renderFramebufferId);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, leftEyeDesc.resolveFramebufferId);
      }
      else
      {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, rightEyeDesc.renderFramebufferId);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, rightEyeDesc.resolveFramebufferId);
      }

      glBlitFramebuffer(0, 0, hmdWidth, hmdHeight, 0, 0, hmdWidth, hmdHeight,
                        GL_COLOR_BUFFER_BIT,
                        GL_LINEAR);

      glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    }

    IGL_INLINE void VRApplication::printstuff()
    {
      printf("\naddres: %x\n", &leftEyeDesc);

      //if(leftEyeDesc != NULL)
      //printf("after init: %u\n", leftEyeDesc.renderFramebufferId);
    }

    IGL_INLINE void VRApplication::initGl()
    {
      if (!createFrameBuffer(leftEyeDesc))
      {
        printf("Error creating frame buffers for left eye");
      }
      else
      {
        printf("Success creating frame buffers for left eye");
      }

      if (!createFrameBuffer(rightEyeDesc))
      {
        printf("Error creating frame buffers for right eye");
      }
      else
      {
        printf("Success creating frame buffers for right eye");
      }
      setupCompanionWindow();

      create_shader_program(
          // vertex shader
          "#version 410\n"
          "uniform mat4 view;\n"
          "uniform mat4 proj;\n"
          "layout(location = 0) in vec4 position;\n"
          "layout(location = 1) in vec3 v3ColorIn;\n"
          "out vec3 position_eye;\n"
          "out vec4 v4Color;\n"
          "void main()\n"
          "{\n"
          "   position_eye = vec3 (view * position);\n"
          "	gl_Position = proj * vec4(position_eye, 1.0);\n"
          "	v4Color.xyz = v3ColorIn;\n"
          "	v4Color.a = 1.0;\n"
          "}\n",

          // fragment shader
          "#version 410\n"
          "in vec4 v4Color;\n"
          "out vec4 outputColor;\n"
          "void main()\n"
          "{\n"
          "   outputColor = v4Color;\n"
          "}\n",
          {},
          controllerTransformProgramID);

      //controllerMatrixLocation = glGetUniformLocation(controllerTransformProgramID, "matrix");
      //if (controllerMatrixLocation == -1)
      //{
      //    printf("Unable to find matrix uniform in controller shader\n");
      //}
    }

    IGL_INLINE bool VRApplication::createFrameBuffer(FramebufferDesc &framebufferDesc)
    {
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

      GLushort vIndices[] = {0, 1, 3, 0, 3, 2, 4, 5, 7, 4, 7, 6};
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
      glVertexAttribPointer(0, 2, GL_FLOAT, GL_TRUE, sizeof(VertexDataWindow), (void *)offsetof(VertexDataWindow, position));

      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 2, GL_FLOAT, GL_TRUE, sizeof(VertexDataWindow), (void *)offsetof(VertexDataWindow, texCoord));

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
          "}\n",
          {},
          companionWindowProgramID);
    }

    IGL_INLINE void VRApplication::drawControllerAxes(Eigen::Matrix4f view, Eigen::Matrix4f proj)
    {
      if (!hmd->IsInputAvailable())
        return;

      glEnable(GL_DEPTH_TEST);

      // draw the controller axis lines
      glUseProgram(controllerTransformProgramID);

      // Send transformations to the GPU
      GLint viewi = glGetUniformLocation(controllerTransformProgramID, "view");
      GLint proji = glGetUniformLocation(controllerTransformProgramID, "proj");
      glUniformMatrix4fv(viewi, 1, GL_FALSE, view.data());
      glUniformMatrix4fv(proji, 1, GL_FALSE, proj.data());

      //glUniformMatrix4fv(controllerMatrixLocation, 1, GL_FALSE, viewProjectionMatrix.data());
      glBindVertexArray(controllerVAO);
      glDrawArrays(GL_LINES, 0, controllerVertCount);
      glBindVertexArray(0);

      glUseProgram(0);
    }

    IGL_INLINE void VRApplication::renderControllerAxes()
    {
      if (!hmd->IsInputAvailable())
        return;

      std::vector<float> vertdataarray;

      controllerVertCount = 0;
      trackedControllerCount = 0;

      for (EHand eHand = Left; eHand <= Right; ((int &)eHand)++)
      {
        if (!m_rHand[eHand].m_bShowController)
          continue;

        Eigen::Matrix4f &mat = m_rHand[eHand].m_rmat4Pose;
        Eigen::Vector4f center = mat * Eigen::Vector4f(0, 0, 0, 1);

        for (int i = 0; i < 3; ++i)
        {
          Eigen::Vector3f color(0, 0, 0);
          Eigen::Vector4f point(0, 0, 0, 1);
          point(i) += 0.05f;
          color(i) = 1.0;
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
        vertdataarray.push_back(start(0));
        vertdataarray.push_back(start(1));
        vertdataarray.push_back(start(2));
        vertdataarray.push_back(color(0));
        vertdataarray.push_back(color(1));
        vertdataarray.push_back(color(2));

        vertdataarray.push_back(end(0));
        vertdataarray.push_back(end(1));
        vertdataarray.push_back(end(2));
        vertdataarray.push_back(color(0));
        vertdataarray.push_back(color(1));
        vertdataarray.push_back(color(2));
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
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

        offset += sizeof(Eigen::Vector3f);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

        glBindVertexArray(0);
      }

      glBindBuffer(GL_ARRAY_BUFFER, controllerVertBuffer);

      // set vertex data if we have some
      if (vertdataarray.size() > 0)
      {
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STREAM_DRAW);
      }
    }

    IGL_INLINE void VRApplication::handleInput()
    {
      // Process SteamVR events
      vr::VREvent_t event;
      while (hmd->PollNextEvent(&event, sizeof(event)))
      {
        //ProcessVREvent(event);
      }

      // Process SteamVR action state
      // UpdateActionState is called each frame to update the state of the actions themselves. The application
      // controls which action sets are active with the provided array of VRActiveActionSet_t structs.
      vr::VRActiveActionSet_t actionSet = {0};
      actionSet.ulActionSet = m_actionsetDemo;
      vr::VRInput()->UpdateActionState(&actionSet, sizeof(actionSet), 1);

      m_rHand[Left].m_bShowController = true;
      m_rHand[Right].m_bShowController = true;

      for (EHand eHand = Left; eHand <= Right; ((int &)eHand)++)
      {
        vr::InputPoseActionData_t poseData;

        if (vr::VRInput()->GetPoseActionDataForNextFrame(m_rHand[eHand].m_actionPose, vr::TrackingUniverseStanding, &poseData, sizeof(poseData), vr::k_ulInvalidInputValueHandle) != vr::VRInputError_None || !poseData.bActive || !poseData.pose.bPoseIsValid)
        {
          m_rHand[eHand].m_bShowController = false;
        }
        else
        {
          m_rHand[eHand].m_rmat4Pose = convertMatrix(poseData.pose.mDeviceToAbsoluteTracking);

          vr::InputOriginInfo_t originInfo;
          if (vr::VRInput()->GetOriginTrackedDeviceInfo(poseData.activeOrigin, &originInfo, sizeof(originInfo)) == vr::VRInputError_None && originInfo.trackedDeviceIndex != vr::k_unTrackedDeviceIndexInvalid)
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

  } // namespace openvr
} // namespace igl