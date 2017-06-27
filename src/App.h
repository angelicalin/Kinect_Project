#ifndef App_h
#define App_h

#include "BaseApp.h"
#include"Texture.h";
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Kinect.h"
#include <NuiKinectFusionApi.h> 
#include <locale>
#include <codecvt>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h> 
#include <assimp/DefaultLogger.hpp>
#include <assimp/LogStream.hpp>
#include "ppl.h"
#include <assimp/IOSystem.hpp>


namespace basicgraphics {

	// Safe release for interfaces
	template<class Interface>
	inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}

#ifndef SAFE_DELETE
#define SAFE_DELETE(p) { if (p) { delete (p); (p)=NULL; } }
#endif

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(p) { if (p) { delete[] (p); (p)=NULL; } }
#endif

#ifndef SAFE_FUSION_RELEASE_IMAGE_FRAME
#define SAFE_FUSION_RELEASE_IMAGE_FRAME(p) { if (p) { static_cast<void>(NuiFusionReleaseImageFrame(p)); (p)=NULL; } }
#endif

	class App : public BaseApp {

		static const int            cBytesPerPixel = 4; // for depth float and int-per-pixel raycast images
		static const int            cResetOnTimeStampSkippedMilliseconds = 1000;  // ms
		static const int            cResetOnNumberOfLostFrames = 100;
		static const int            cStatusMessageMaxLen = MAX_PATH * 2;
		static const int            cTimeDisplayInterval = 10;
		
		//added color portion
		static const int cColorWidth = 1920;
		static const int cColorHeight = 1080;

		static const int            cVisibilityTestQuantShift = 2; // shift by 2 == divide by 4
		static const UINT16         cDepthVisibilityTestThreshold = 50; //50 mm



	public:

		App(int argc, char** argv, std::string windowName, int windowWidth, int windowHeight);
		~App();

		void onRenderGraphics() override;
		void onEvent(std::shared_ptr<Event> event) override;


	protected:


		std::vector< std::shared_ptr<Texture> > textures;
		std::shared_ptr<Texture> kinectVision;
		std::unique_ptr<Mesh> _mesh;
		GLSLProgram shader;

		// Current Kinect
		IKinectSensor*              m_pNuiSensor;

		// Depth reader
		IDepthFrameReader*          m_pDepthFrameReader;
		//Color reader
		IColorFrameReader*			 m_pColorFrameReader;
		UINT                         m_cDepthWidth;
		UINT                         m_cDepthHeight;
		UINT                         m_cDepthImagePixels;

		INT64                       m_lastFrameTimeStamp;
		bool                        m_bResetReconstruction;

		BYTE*                       m_pDepthRGBX;

		/// <summary>
		/// Main processing function
		/// </summary>
		void                        Update();

		/// <summary>
		/// Create the first connected Kinect found 
		/// </summary>
		/// <returns>S_OK on success, otherwise failure code</returns>
		HRESULT                     CreateFirstConnected();

		/// <summary>
		/// Setup or update the Undistortion calculation for the connected camera
		/// </summary>
		HRESULT       SetupUndistortion();

	
		/// <summary>
		/// Handle Coordinate Mapping changed event.
		/// Note, this happens after sensor connect, or when Kinect Studio connects
		/// </summary>
		HRESULT                     OnCoordinateMappingChanged();

		/// <summary>
		/// Initialize Kinect Fusion volume and images for processing
		/// </summary>
		/// <returns>S_OK on success, otherwise failure code</returns>
		HRESULT                     InitializeKinectFusion();

		/// <summary>
		/// Handle new depth data
		/// </summary>
		void                        ProcessDepth();


		//HRESULT	 WriteAsciiObjMeshFile(INuiFusionMesh *mesh, std::string filename, bool flipYZ);
		HRESULT	 WriteAsciiObjMeshFile(INuiFusionColorMesh *mesh, std::string filename, bool flipYZ);
		HRESULT WriteAsciiPlyMeshFile(INuiFusionColorMesh *mesh, std::string lpOleFileName, bool flipYZ, bool outputColor);

		/// <summary>
		/// Reset the reconstruction camera pose and clear the volume.
		/// </summary>
		/// <returns>S_OK on success, otherwise failure code</returns>
		HRESULT                     ResetReconstruction();

		/// <summary>
		/// The Kinect Fusion Reconstruction Volume
		/// </summary>
	//			INuiFusionReconstruction*   m_pVolume;
		///if use with color
		INuiFusionColorReconstruction* m_pVolume;

		/// <summary>
		/// The Kinect Fusion Volume Parameters
		/// </summary>
		NUI_FUSION_RECONSTRUCTION_PARAMETERS m_reconstructionParams;

		/// <summary>
		// The Kinect Fusion Camera Transform
		/// </summary>
		Matrix4                     m_worldToCameraTransform;

		/// <summary>
		// The default Kinect Fusion World to Volume Transform
		/// </summary>
		Matrix4                     m_defaultWorldToVolumeTransform;

		Matrix4                     m_worldToBGRTransform;
		/// <summary>
		/// Frames from the depth input
		/// </summary>
		UINT16*                     m_pDepthImagePixelBuffer;
		NUI_FUSION_IMAGE_FRAME*     m_pDepthFloatImage;
		UINT16* m_pDepthRawPixelBuffer;

		/// <summary>
		/// For depth distortion correction
		/// </summary>
		ICoordinateMapper*          m_pMapper;
		DepthSpacePoint*            m_pDepthDistortionMap;
		UINT*                       m_pDepthDistortionLT;
		WAITABLE_HANDLE             m_coordinateMappingChangedEvent;

		/// <summary>
		/// Kinect camera parameters.
		/// </summary>
		NUI_FUSION_CAMERA_PARAMETERS m_cameraParameters;
		bool                        m_bHaveValidCameraParameters;

		/// <summary>
		/// Frames generated from ray-casting the Reconstruction Volume
		/// </summary>
		NUI_FUSION_IMAGE_FRAME*     m_pPointCloud;

		NUI_FUSION_IMAGE_FRAME*     m_pSmoothDepthFloatImage;
		/// <summary>
		/// Images for display
		/// </summary>
		NUI_FUSION_IMAGE_FRAME*     m_pShadedSurface;

		/// <summary>
		/// Camera Tracking parameters
		/// </summary>
		int                         m_cLostFrameCounter;
		bool                        m_bTrackingFailed;
		bool                        m_bSavingMesh;
		/// <summary>
		/// Parameter to turn automatic reset of the reconstruction when camera tracking is lost on or off.
		/// Set to true in the constructor to enable auto reset on cResetOnNumberOfLostFrames lost frames,
		/// or set false to never automatically reset.
		/// </summary>
		bool                        m_bAutoResetReconstructionWhenLost;





		/// <summary>
		/// Parameter to enable automatic reset of the reconstruction when there is a large
		/// difference in timestamp between subsequent frames. This should usually be set true as 
		/// default to enable recorded .xef files to generate a reconstruction reset on looping of
		/// the playback or scrubbing, however, for debug purposes, it can be set false to prevent
		/// automatic reset on timeouts.
		/// </summary>
		bool                        m_bAutoResetReconstructionOnTimeout;

		/// <summary>
		/// Processing parameters
		/// </summary>
		int                         m_deviceIndex;
		NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE m_processorType;
		bool                        m_bInitializeError;
		float                       m_fMinDepthThreshold;
		float                       m_fMaxDepthThreshold;
		bool                        m_bMirrorDepthFrame;
		unsigned short              m_cMaxIntegrationWeight;
		int                         m_cFrameCounter;


		/// <summary>
		/// Parameter to translate the reconstruction based on the minimum depth setting. When set to
		/// false, the reconstruction volume +Z axis starts at the camera lens and extends into the scene.
		/// Setting this true in the constructor will move the volume forward along +Z away from the
		/// camera by the minimum depth threshold to enable capture of very small reconstruction volumes
		/// by setting a non-identity camera transformation in the ResetReconstruction call.
		/// Small volumes should be shifted, as the Kinect hardware has a minimum sensing limit of ~0.35m,
		/// inside which no valid depth is returned, hence it is difficult to initialize and track robustly  
		/// when the majority of a small volume is inside this distance.
		/// </summary>
		bool                        m_bTranslateResetPoseByMinDepthThreshold;

		//HRESULT exportMesh(const std::string &filename, INuiFusionMesh* mesh);
		//void appendChildToParentNode(aiNode *pParent, aiNode *pChild);
		//std::shared_ptr <Assimp::Exporter> _exporter;
		//color portion
		NUI_FUSION_IMAGE_FRAME*     m_pColorImage;
		NUI_FUSION_IMAGE_FRAME*     m_pResampledColorImageDepthAligned;
		ColorSpacePoint*            m_pColorCoordinates;
		NUI_FUSION_IMAGE_FRAME*     m_pResampledColorImage;
		NUI_FUSION_IMAGE_FRAME*     m_pCapturedSurfaceColor;
		UINT16* m_pDepthVisibilityTestMap;
		HRESULT DownsampleColorFrameToDepthResolution(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest);

		HRESULT CopyColor(IColorFrame* pColorFrame);
		HRESULT MapColorToDepth();
		bool m_bCaptureColor;
	};
}
#endif