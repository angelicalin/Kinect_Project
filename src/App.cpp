#include "App.h"
#include <iostream>
#include <fstream>


namespace basicgraphics {


	using namespace std;
	using namespace glm;

	/// <summary>
	/// Set Identity in a Matrix4
	/// </summary>
	/// <param name="mat">The matrix to set to identity</param>
	void SetIdentityMatrix(Matrix4 &mat)
	{
		mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
		mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
		mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
		mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
	}

	App::App(int argc, char** argv, std::string windowName, int windowWidth, int windowHeight) : BaseApp(argc, argv, windowName, windowWidth, windowHeight),
		m_pVolume(nullptr),
		m_pNuiSensor(nullptr),
		m_cDepthImagePixels(0),
		m_bMirrorDepthFrame(false),
		m_bTranslateResetPoseByMinDepthThreshold(true),
		m_bAutoResetReconstructionWhenLost(false),
		m_bAutoResetReconstructionOnTimeout(true),
		m_lastFrameTimeStamp(0),
		m_bResetReconstruction(false),
		m_cLostFrameCounter(0),
		m_bTrackingFailed(false),
		m_cFrameCounter(0),
		m_pMapper(nullptr),
		m_pDepthImagePixelBuffer(nullptr),
		m_pDepthDistortionMap(nullptr),
		m_pDepthDistortionLT(nullptr),
		m_pDepthFloatImage(nullptr),
		m_pPointCloud(nullptr),
		m_pShadedSurface(nullptr),
		m_bInitializeError(false),
		m_pDepthFrameReader(NULL),
		m_coordinateMappingChangedEvent(NULL),
		m_bHaveValidCameraParameters(false) {

		// Get the depth frame size from the NUI_IMAGE_RESOLUTION enum
		// You can use NUI_IMAGE_RESOLUTION_640x480 or NUI_IMAGE_RESOLUTION_320x240 in this sample
		// Smaller resolutions will be faster in per-frame computations, but show less detail in reconstructions.
		m_cDepthWidth = NUI_DEPTH_RAW_WIDTH;
		m_cDepthHeight = NUI_DEPTH_RAW_HEIGHT;
		m_cDepthImagePixels = m_cDepthWidth * m_cDepthHeight;

		// create heap storage for depth pixel data in RGBX format
		m_pDepthRGBX = new BYTE[m_cDepthImagePixels * cBytesPerPixel];

		// Define a cubic Kinect Fusion reconstruction volume,
		// with the Kinect at the center of the front face and the volume directly in front of Kinect.
		m_reconstructionParams.voxelsPerMeter = 256;// 1000mm / 256vpm = ~3.9mm/voxel    
		m_reconstructionParams.voxelCountX = 384;   // 384 / 256vpm = 1.5m wide reconstruction
		m_reconstructionParams.voxelCountY = 384;   // Memory = 384*384*384 * 4bytes per voxel
		m_reconstructionParams.voxelCountZ = 384;   // This will require a GPU with at least 256MB

													// These parameters are for optionally clipping the input depth image 
		m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;   // min depth in meters
		m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;    // max depth in meters

																	// This parameter is the temporal averaging parameter for depth integration into the reconstruction
		m_cMaxIntegrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;	// Reasonable for static scenes

																			// This parameter sets whether GPU or CPU processing is used. Note that the CPU will likely be 
																			// too slow for real-time processing.
		m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP;

		// If GPU processing is selected, we can choose the index of the device we would like to
		// use for processing by setting this zero-based index parameter. Note that setting -1 will cause
		// automatic selection of the most suitable device (specifically the DirectX11 compatible device 
		// with largest memory), which is useful in systems with multiple GPUs when only one reconstruction
		// volume is required. Note that the automatic choice will not load balance across multiple 
		// GPUs, hence users should manually select GPU indices when multiple reconstruction volumes 
		// are required, each on a separate device.
		m_deviceIndex = -1;    // automatically choose device index for processing

		SetIdentityMatrix(m_worldToCameraTransform);
		SetIdentityMatrix(m_defaultWorldToVolumeTransform);

		// We don't know these at object creation time, so we use nominal values.
		// These will later be updated in response to the CoordinateMappingChanged event.
		m_cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
		m_cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
		m_cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
		m_cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;

		// Look for a connected Kinect, and create it if found
		HRESULT hr = CreateFirstConnected();
		if (FAILED(hr))
		{
			m_bInitializeError = true;
		}

		if (!m_bInitializeError)
		{
			hr = InitializeKinectFusion();
			if (FAILED(hr))
			{
				m_bInitializeError = true;
			}
		}


		glClearColor(0.2f, 0.2f, 0.2f, 1.0f);

		// This load shaders from disk, we do it once when the program starts
		// up, but since you don't need to recompile to reload shaders, you can
		// even do this inteactively as you debug your shaders!  Press the R
		// key to reload them while your program is running!


		// This loads the model from a file and initializes an instance of the model class to store it



		kinectVision = Texture::createEmpty("from kinect", m_cDepthWidth, m_cDepthHeight, 0, GL_RGBA, true, GL_TEXTURE_2D, GL_RGBA);
		glfwSetWindowSize(_window, m_cDepthWidth*2, m_cDepthHeight*2);
		//std::shared_ptr<Texture> Texture::createEmpty(const std::string &name, int width, int height, int depth, int numMipMapLevels, bool autoMipMap, GLenum target, GLenum internalFormat)

		Mesh::Vertex vertex;
	std:vector<Mesh::Vertex> cpuVertexArray;
		std::vector<int> cpuIndexArray;
		int index = 0;
		vertex.position = glm::vec3(-2,-2, 0);
		vertex.normal = glm::vec3(0, 0, 1);
		vertex.texCoord0 = glm::vec2(1,1);
		cpuVertexArray.push_back(vertex);
		cpuIndexArray.push_back(index);
		index++;
		vertex.position = glm::vec3(2, -2, 0);
		vertex.texCoord0 = glm::vec2(0, 1);
		cpuVertexArray.push_back(vertex);
		cpuIndexArray.push_back(index);
		index++;
		vertex.position = glm::vec3(2, 2, 0);
		vertex.texCoord0 = glm::vec2(0, 0);
		cpuVertexArray.push_back(vertex);
		cpuIndexArray.push_back(index);
		index++;
		vertex.position = glm::vec3(-2, 2, 0);
		vertex.texCoord0 = glm::vec2(1, 0);
		cpuVertexArray.push_back(vertex);
		cpuIndexArray.push_back(index);
		const int numVertices = cpuVertexArray.size();
		const int cpuVertexByteSize = sizeof(Mesh::Vertex)* numVertices;
		const int cpuIndexByteSize = sizeof(int) * cpuIndexArray.size();
		textures.push_back(kinectVision);
		_mesh.reset(new Mesh(textures, GL_TRIANGLE_FAN, GL_STATIC_DRAW, cpuVertexByteSize, cpuIndexByteSize, 0, cpuVertexArray, cpuIndexArray.size(), cpuIndexByteSize, &cpuIndexArray[0]));

	}

	App::~App() {
		// Clean up Kinect Fusion
		SafeRelease(m_pVolume);
		SafeRelease(m_pMapper);

		if (nullptr != m_pMapper)
			m_pMapper->UnsubscribeCoordinateMappingChanged(m_coordinateMappingChangedEvent);

		SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pShadedSurface);
		SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pPointCloud);
		SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDepthFloatImage);

		// done with depth frame reader
		SafeRelease(m_pDepthFrameReader);

		// Clean up Kinect
		if (m_pNuiSensor)
		{
			m_pNuiSensor->Close();
			m_pNuiSensor->Release();
		}

		// clean up the depth pixel array
		SAFE_DELETE_ARRAY(m_pDepthImagePixelBuffer);

		SAFE_DELETE_ARRAY(m_pDepthDistortionMap);
		SAFE_DELETE_ARRAY(m_pDepthDistortionLT);

		// done with depth pixel data
		SAFE_DELETE_ARRAY(m_pDepthRGBX);
	}

	/// <summary>
	/// Main processing function
	/// </summary>
	void App::Update()
	{
		if (nullptr == m_pNuiSensor)
		{
			return;
		}

		if (m_coordinateMappingChangedEvent != NULL &&
			WAIT_OBJECT_0 == WaitForSingleObject((HANDLE)m_coordinateMappingChangedEvent, 0))
		{
			OnCoordinateMappingChanged();
			ResetEvent((HANDLE)m_coordinateMappingChangedEvent);
		}

		if (!m_bHaveValidCameraParameters)
		{
			return;
		}

		m_bResetReconstruction = false;

		if (!m_pDepthFrameReader)
		{
			return;
		}

		IDepthFrame* pDepthFrame = NULL;

		HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

		if (SUCCEEDED(hr))
		{
			UINT nBufferSize = 0;
			UINT16 *pBuffer = NULL;
			INT64 currentTimestamp = 0;

			hr = pDepthFrame->get_RelativeTime(&currentTimestamp);
			if (SUCCEEDED(hr) && currentTimestamp - m_lastFrameTimeStamp > cResetOnTimeStampSkippedMilliseconds * 10000
				&& 0 != m_lastFrameTimeStamp)
			{
				m_bResetReconstruction = true;
			}
			m_lastFrameTimeStamp = currentTimestamp;

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			}

			if (SUCCEEDED(hr))
			{
				//copy and remap depth
				const UINT bufferLength = m_cDepthImagePixels;
				UINT16 * pDepth = m_pDepthImagePixelBuffer;
				for (UINT i = 0; i < bufferLength; i++, pDepth++)
				{
					const UINT id = m_pDepthDistortionLT[i];
					*pDepth = id < bufferLength ? pBuffer[id] : 0;
				}

				ProcessDepth();
			}
		}

		SafeRelease(pDepthFrame);
	}

	/// <summary>
	/// Create the first connected Kinect found 
	/// </summary>
	/// <returns>indicates success or failure</returns>
	HRESULT App::CreateFirstConnected()
	{
		HRESULT hr;

		hr = GetDefaultKinectSensor(&m_pNuiSensor);
		if (FAILED(hr))
		{
			return hr;
		}

		if (m_pNuiSensor)
		{
			// Initialize the Kinect and get the depth reader
			IDepthFrameSource* pDepthFrameSource = NULL;

			hr = m_pNuiSensor->Open();

			if (SUCCEEDED(hr))
			{
				hr = m_pNuiSensor->get_CoordinateMapper(&m_pMapper);
			}

			if (SUCCEEDED(hr))
			{
				hr = m_pNuiSensor->get_DepthFrameSource(&pDepthFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = m_pMapper->SubscribeCoordinateMappingChanged(&m_coordinateMappingChangedEvent);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
			}

			SafeRelease(pDepthFrameSource);
		}

		if (nullptr == m_pNuiSensor || FAILED(hr))
		{
			cout << ("No ready Kinect found!") << endl;
			return E_FAIL;
		}

		return hr;
	}

	void UpdateIntrinsics(NUI_FUSION_IMAGE_FRAME * pImageFrame, NUI_FUSION_CAMERA_PARAMETERS * params)
	{
		if (pImageFrame != nullptr && pImageFrame->pCameraParameters != nullptr && params != nullptr)
		{
			pImageFrame->pCameraParameters->focalLengthX = params->focalLengthX;
			pImageFrame->pCameraParameters->focalLengthY = params->focalLengthY;
			pImageFrame->pCameraParameters->principalPointX = params->principalPointX;
			pImageFrame->pCameraParameters->principalPointY = params->principalPointY;
		}

		// Confirm we are called correctly
		_ASSERT(pImageFrame != nullptr && pImageFrame->pCameraParameters != nullptr && params != nullptr);
	}

	HRESULT App::SetupUndistortion()
	{
		HRESULT hr = E_UNEXPECTED;

		if (m_cameraParameters.principalPointX != 0)
		{

			CameraSpacePoint cameraFrameCorners[4] = //at 1 meter distance. Take into account that depth frame is mirrored
			{
				/*LT*/{ -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f },
				/*RT*/{ (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f },
				/*LB*/{ -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f },
				/*RB*/{ (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f }
			};

			for (UINT rowID = 0; rowID < m_cDepthHeight; rowID++)
			{
				const float rowFactor = float(rowID) / float(m_cDepthHeight - 1);
				const CameraSpacePoint rowStart =
				{
					cameraFrameCorners[0].X + (cameraFrameCorners[2].X - cameraFrameCorners[0].X) * rowFactor,
					cameraFrameCorners[0].Y + (cameraFrameCorners[2].Y - cameraFrameCorners[0].Y) * rowFactor,
					1.f
				};

				const CameraSpacePoint rowEnd =
				{
					cameraFrameCorners[1].X + (cameraFrameCorners[3].X - cameraFrameCorners[1].X) * rowFactor,
					cameraFrameCorners[1].Y + (cameraFrameCorners[3].Y - cameraFrameCorners[1].Y) * rowFactor,
					1.f
				};

				const float stepFactor = 1.f / float(m_cDepthWidth - 1);
				const CameraSpacePoint rowDelta =
				{
					(rowEnd.X - rowStart.X) * stepFactor,
					(rowEnd.Y - rowStart.Y) * stepFactor,
					0
				};

				_ASSERT(m_cDepthWidth == NUI_DEPTH_RAW_WIDTH);
				CameraSpacePoint cameraCoordsRow[NUI_DEPTH_RAW_WIDTH];

				CameraSpacePoint currentPoint = rowStart;
				for (UINT i = 0; i < m_cDepthWidth; i++)
				{
					cameraCoordsRow[i] = currentPoint;
					currentPoint.X += rowDelta.X;
					currentPoint.Y += rowDelta.Y;
				}

				hr = m_pMapper->MapCameraPointsToDepthSpace(m_cDepthWidth, cameraCoordsRow, m_cDepthWidth, &m_pDepthDistortionMap[rowID * m_cDepthWidth]);
				if (FAILED(hr))
				{
					cout << "Failed to initialize Kinect Coordinate Mapper." << endl;
					return hr;
				}
			}

			if (nullptr == m_pDepthDistortionLT)
			{
				cout << "Failed to initialize Kinect Fusion depth image distortion Lookup Table." << endl;
				return E_OUTOFMEMORY;
			}

			UINT* pLT = m_pDepthDistortionLT;
			for (UINT i = 0; i < m_cDepthImagePixels; i++, pLT++)
			{
				//nearest neighbor depth lookup table 
				UINT x = UINT(m_pDepthDistortionMap[i].X + 0.5f);
				UINT y = UINT(m_pDepthDistortionMap[i].Y + 0.5f);

				*pLT = (x < m_cDepthWidth && y < m_cDepthHeight) ? x + y * m_cDepthWidth : UINT_MAX;
			}
			m_bHaveValidCameraParameters = true;
		}
		else
		{
			m_bHaveValidCameraParameters = false;
		}
		return S_OK;
	}
	///////////////////////////////////////////////////////////////////////////////////////////
	/// <summary>
	/// Initialize Kinect Fusion volume and images for processing
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT App::OnCoordinateMappingChanged()
	{
		HRESULT hr = E_UNEXPECTED;

		// Calculate the down sampled image sizes, which are used for the AlignPointClouds calculation frames
		CameraIntrinsics intrinsics = {};

		m_pMapper->GetDepthCameraIntrinsics(&intrinsics);

		float focalLengthX = intrinsics.FocalLengthX / NUI_DEPTH_RAW_WIDTH;
		float focalLengthY = intrinsics.FocalLengthY / NUI_DEPTH_RAW_HEIGHT;
		float principalPointX = intrinsics.PrincipalPointX / NUI_DEPTH_RAW_WIDTH;
		float principalPointY = intrinsics.PrincipalPointY / NUI_DEPTH_RAW_HEIGHT;

		if (m_cameraParameters.focalLengthX == focalLengthX && m_cameraParameters.focalLengthY == focalLengthY &&
			m_cameraParameters.principalPointX == principalPointX && m_cameraParameters.principalPointY == principalPointY)
			return S_OK;

		m_cameraParameters.focalLengthX = focalLengthX;
		m_cameraParameters.focalLengthY = focalLengthY;
		m_cameraParameters.principalPointX = principalPointX;
		m_cameraParameters.principalPointY = principalPointY;

		_ASSERT(m_cameraParameters.focalLengthX != 0);

		UpdateIntrinsics(m_pDepthFloatImage, &m_cameraParameters);
		UpdateIntrinsics(m_pPointCloud, &m_cameraParameters);
		UpdateIntrinsics(m_pShadedSurface, &m_cameraParameters);

		if (nullptr == m_pDepthDistortionMap)
		{
			cout << "Failed to initialize Kinect Fusion depth image distortion buffer." << endl;
			return E_OUTOFMEMORY;
		}

		hr = SetupUndistortion();
		return hr;
	}

	/// <summary>
	/// Initialize Kinect Fusion volume and images for processing
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT App::InitializeKinectFusion()
	{
		HRESULT hr = S_OK;

		// Check to ensure suitable DirectX11 compatible hardware exists before initializing Kinect Fusion
		WCHAR description[MAX_PATH];
		WCHAR instancePath[MAX_PATH];
		UINT memorySize = 0;

		if (FAILED(hr = NuiFusionGetDeviceInfo(
			m_processorType,
			m_deviceIndex,
			&description[0],
			ARRAYSIZE(description),
			&instancePath[0],
			ARRAYSIZE(instancePath),
			&memorySize)))
		{
			if (hr == E_NUI_BADINDEX)
			{
				// This error code is returned either when the device index is out of range for the processor 
				// type or there is no DirectX11 capable device installed. As we set -1 (auto-select default) 
				// for the device index in the parameters, this indicates that there is no DirectX11 capable 
				// device. The options for users in this case are to either install a DirectX11 capable device
				// (see documentation for recommended GPUs) or to switch to non-real-time CPU based 
				// reconstruction by changing the processor type to NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU.
				cout << "No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction." << endl;
			}
			else
			{
				cout << "Failed in call to NuiFusionGetDeviceInfo." << endl;
			}
			return hr;
		}

		// Create the Kinect Fusion Reconstruction Volume
		hr = NuiFusionCreateReconstruction(
			&m_reconstructionParams,
			m_processorType, m_deviceIndex,
			&m_worldToCameraTransform,
			&m_pVolume);

		if (FAILED(hr))
		{
			if (E_NUI_GPU_FAIL == hr)
			{
				WCHAR buf[MAX_PATH];
				swprintf_s(buf, ARRAYSIZE(buf), L"Device %d not able to run Kinect Fusion, or error initializing.", m_deviceIndex);
				cout << buf << endl;
			}
			else if (E_NUI_GPU_OUTOFMEMORY == hr)
			{
				WCHAR buf[MAX_PATH];
				swprintf_s(buf, ARRAYSIZE(buf), L"Device %d out of memory error initializing reconstruction - try a smaller reconstruction volume.", m_deviceIndex);
				cout << buf << endl;
			}
			else if (NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU != m_processorType)
			{
				WCHAR buf[MAX_PATH];
				swprintf_s(buf, ARRAYSIZE(buf), L"Failed to initialize Kinect Fusion reconstruction volume on device %d.", m_deviceIndex);
				cout << buf << endl;
			}
			else
			{
				cout << "Failed to initialize Kinect Fusion reconstruction volume on CPU." << endl;
			}

			return hr;
		}

		// Save the default world to volume transformation to be optionally used in ResetReconstruction
		hr = m_pVolume->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
		if (FAILED(hr))
		{
			cout << "Failed in call to GetCurrentWorldToVolumeTransform." << endl;
			return hr;
		}

		if (m_bTranslateResetPoseByMinDepthThreshold)
		{
			// This call will set the world-volume transformation
			hr = ResetReconstruction();
			if (FAILED(hr))
			{
				return hr;
			}
		}

		// Frames generated from the depth input
		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pDepthFloatImage);
		if (FAILED(hr))
		{
			cout << "Failed to initialize Kinect Fusion image." << endl;
			return hr;
		}

		// Create images to raycast the Reconstruction Volume
		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pPointCloud);
		if (FAILED(hr))
		{
			cout << "Failed to initialize Kinect Fusion image." << endl;
			return hr;
		}

		// Create images to raycast the Reconstruction Volume
		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pShadedSurface);
		if (FAILED(hr))
		{
			cout << "Failed to initialize Kinect Fusion image." << endl;
			return hr;
		}

		_ASSERT(m_pDepthImagePixelBuffer == nullptr);
		m_pDepthImagePixelBuffer = new(std::nothrow) UINT16[m_cDepthImagePixels];
		if (nullptr == m_pDepthImagePixelBuffer)
		{
			cout << "Failed to initialize Kinect Fusion depth image pixel buffer." << endl;
			return hr;
		}

		_ASSERT(m_pDepthDistortionMap == nullptr);
		m_pDepthDistortionMap = new(std::nothrow) DepthSpacePoint[m_cDepthImagePixels];
		if (nullptr == m_pDepthDistortionMap)
		{
			cout << "Failed to initialize Kinect Fusion depth image pixel buffer." << endl;
			return E_OUTOFMEMORY;
		}

		SAFE_DELETE_ARRAY(m_pDepthDistortionLT);
		m_pDepthDistortionLT = new(std::nothrow) UINT[m_cDepthImagePixels];

		if (nullptr == m_pDepthDistortionLT)
		{
			cout << "Failed to initialize Kinect Fusion depth image distortion Lookup Table." << endl;
			return E_OUTOFMEMORY;
		}

		// If we have valid parameters, let's go ahead and use them.
		if (m_cameraParameters.focalLengthX != 0)
		{
			SetupUndistortion();
		}


		// Set an introductory message
		cout << "Click ‘Reset Reconstruction' to clear!" << endl;

		return hr;
	}

	/// <summary>
	/// Handle new depth data and perform Kinect Fusion processing
	/// </summary>
	void App::ProcessDepth()
	{
		if (m_bInitializeError)
		{
			return;
		}

		HRESULT hr = S_OK;

		// To enable playback of a .xef file through Kinect Studio and reset of the reconstruction
		// if the .xef loops, we test for when the frame timestamp has skipped a large number. 
		// Note: this will potentially continually reset live reconstructions on slow machines which
		// cannot process a live frame in less time than the reset threshold. Increase the number of
		// milliseconds in cResetOnTimeStampSkippedMilliseconds if this is a problem.
		if (m_bAutoResetReconstructionOnTimeout && m_cFrameCounter != 0 && m_bResetReconstruction)
		{
			ResetReconstruction();

			if (FAILED(hr))
			{
				return;
			}
		}

		// Return if the volume is not initialized
		if (nullptr == m_pVolume)
		{
			cout << "Kinect Fusion reconstruction volume not initialized. Please try reducing volume size or restarting." << endl;
			return;
		}

		////////////////////////////////////////////////////////
		// Depth to DepthFloat

		// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
		// as floating point type in meters.
		hr = m_pVolume->DepthToDepthFloatFrame(m_pDepthImagePixelBuffer, m_cDepthImagePixels * sizeof(UINT16), m_pDepthFloatImage, m_fMinDepthThreshold, m_fMaxDepthThreshold, m_bMirrorDepthFrame);

		if (FAILED(hr))
		{
			cout << "Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed." << endl;
			return;
		}

		////////////////////////////////////////////////////////
		// ProcessFrame

		// Perform the camera tracking and update the Kinect Fusion Volume
		// This will create memory on the GPU, upload the image, run camera tracking and integrate the
		// data into the Reconstruction Volume if successful. Note that passing nullptr as the final 
		// parameter will use and update the internal camera pose.
		hr = m_pVolume->ProcessFrame(m_pDepthFloatImage, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, m_cMaxIntegrationWeight, nullptr, &m_worldToCameraTransform);

		// Test to see if camera tracking failed. 
		// If it did fail, no data integration or raycast for reference points and normals will have taken 
		//  place, and the internal camera pose will be unchanged.
		if (FAILED(hr))
		{
			if (hr == E_NUI_FUSION_TRACKING_ERROR)
			{
				m_cLostFrameCounter++;
				m_bTrackingFailed = true;
				cout << "Kinect Fusion camera tracking failed! Align the camera to the last tracked position. " << endl;
			}
			else
			{
				cout << "Kinect Fusion ProcessFrame call failed!" << endl;
				return;
			}
		}
		else
		{
			Matrix4 calculatedCameraPose;
			hr = m_pVolume->GetCurrentWorldToCameraTransform(&calculatedCameraPose);

			if (SUCCEEDED(hr))
			{
				// Set the pose
				m_worldToCameraTransform = calculatedCameraPose;
				m_cLostFrameCounter = 0;
				m_bTrackingFailed = false;
			}
		}

		if (m_bAutoResetReconstructionWhenLost && m_bTrackingFailed && m_cLostFrameCounter >= cResetOnNumberOfLostFrames)
		{
			// Automatically clear volume and reset tracking if tracking fails
			hr = ResetReconstruction();

			if (FAILED(hr))
			{
				return;
			}

			// Set bad tracking message
			cout << "Kinect Fusion camera tracking failed, automatically reset volume." << endl;
		}

		////////////////////////////////////////////////////////
		// CalculatePointCloud

		// Raycast all the time, even if we camera tracking failed, to enable us to visualize what is happening with the system
		hr = m_pVolume->CalculatePointCloud(m_pPointCloud, &m_worldToCameraTransform);

		if (FAILED(hr))
		{
			cout << "Kinect Fusion CalculatePointCloud call failed." << endl;
			return;
		}

		////////////////////////////////////////////////////////
		// ShadePointCloud and render

		hr = NuiFusionShadePointCloud(m_pPointCloud, &m_worldToCameraTransform, nullptr, m_pShadedSurface, nullptr);

		if (FAILED(hr))
		{
			cout << "Kinect Fusion NuiFusionShadePointCloud call failed." << endl;
			return;
		}

		// Draw the shaded raycast volume image
		BYTE * pBuffer = m_pShadedSurface->pFrameBuffer->pBits;

	//	cout << m_pShadedSurface->width << ", " << m_pShadedSurface->height<<", "<<m_cDepthWidth<<", "<< m_cDepthHeight << endl;

		// Draw the data
		//TODO: update your texture here
		//m_pDrawDepth->Draw(pBuffer, m_cDepthWidth * m_cDepthHeight * cBytesPerPixel);

		////////////////////////////////////////////////////////
		// Periodically Display Fps

		// Update frame counter
		m_cFrameCounter++;
	}


	/// <summary>
	/// Reset the reconstruction camera pose and clear the volume.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT App::ResetReconstruction()
	{
		if (nullptr == m_pVolume)
		{
			return E_FAIL;
		}

		HRESULT hr = S_OK;

		SetIdentityMatrix(m_worldToCameraTransform);

		// Translate the reconstruction volume location away from the world origin by an amount equal
		// to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
		// If set false, the default world origin is set to the center of the front face of the 
		// volume, which has the effect of locating the volume directly in front of the initial camera
		// position with the +Z axis into the volume along the initial camera direction of view.
		if (m_bTranslateResetPoseByMinDepthThreshold)
		{
			Matrix4 worldToVolumeTransform = m_defaultWorldToVolumeTransform;

			// Translate the volume in the Z axis by the minDepthThreshold distance
			float minDist = (m_fMinDepthThreshold < m_fMaxDepthThreshold) ? m_fMinDepthThreshold : m_fMaxDepthThreshold;
			worldToVolumeTransform.M43 -= (minDist * m_reconstructionParams.voxelsPerMeter);

			hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, &worldToVolumeTransform);
		}
		else
		{
			hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, nullptr);
		}

		m_cLostFrameCounter = 0;
		m_cFrameCounter = 0;

		if (SUCCEEDED(hr))
		{
			m_bTrackingFailed = false;

			cout << "Reconstruction has been reset." << endl;
		}
		else
		{
			cout << "Failed to reset reconstruction." << endl;
		}

		return hr;
	}

	void App::onRenderGraphics() {

		Update();

		glm::mat4 view = glm::lookAt(glm::vec3(0, 0, 4), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));

		// Setup the projection matrix so that things are rendered in perspective
		glm::vec3 eye_world = glm::vec3(glm::column(glm::inverse(view), 3));
		glm::mat4 projection = glm::perspective(glm::radians(45.0f), (GLfloat)_windowWidth / (GLfloat)_windowHeight, 0.1f, 500.0f);
		shader.compileShader("texture.vert", GLSLShader::VERTEX);
		shader.compileShader("texture.frag", GLSLShader::FRAGMENT);
		// Update shader variables
		shader.link();
		shader.use();
		shader.setUniform("view_mat", view);
		shader.setUniform("projection_mat", projection);
		shader.setUniform("model_mat", mat4(1.0));
		shader.setUniform("eye_world", eye_world);

		kinectVision->update(m_pShadedSurface->pFrameBuffer->pBits, GL_RGBA, GL_UNSIGNED_BYTE);
		_mesh->draw(shader);
	}


	void App::onEvent(shared_ptr<Event> event)
	{
		string name = event->getName();

		if (name == "kbd_UP_down" || name == "kbd_UP_repeat") {
			ResetReconstruction();
		}
		if (name == "kbd_S_down") {
			INuiFusionMesh* mesh = nullptr;
			HRESULT hr = m_pVolume->CalculateMesh(1,&mesh);
			if (SUCCEEDED(hr)) {
				ofstream OutFile("H:\\UsingBasicGraphics\\test.stl");
				hr = WriteBinarySTLMeshFile(mesh, "H:\\UsingBasicGraphics\\test.stl",false);
				if (SUCCEEDED(hr)) {
					cout << "Done" << endl;
				}
				else {
					cout << "file saving failed" << endl;
				}
			}
			else {
				cout << "Mesh creation failed" << endl;
			}
		}
	}

	

	HRESULT App::WriteBinarySTLMeshFile(INuiFusionMesh *mesh, std::string filename, bool flipYZ)
	{
		HRESULT hr = S_OK;

		if (NULL == mesh)
		{
			cout << "Invalid mesh" << endl;
			return E_INVALIDARG;
		}


		unsigned int numVertices = mesh->VertexCount();
		unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
		unsigned int numTriangles = numVertices / 3;

		if (0 == numVertices || 0 == numTriangleIndices || 0 != numVertices % 3 || numVertices != numTriangleIndices)
		{
			cout << "Invalid mesh" << endl;
			return E_INVALIDARG;
		}
	

		const Vector3 *vertices = NULL;
		hr = mesh->GetVertices(&vertices);
		if (FAILED(hr))
		{
			cout << "Unable to get vertices" << endl;
			return hr;
		}

		const Vector3 *normals = NULL;
		hr = mesh->GetNormals(&normals);
		if (FAILED(hr))
		{
			cout << "Unable to get normals" << endl;
			return hr;
		}

		const int *triangleIndices = NULL;
		hr = mesh->GetTriangleIndices(&triangleIndices);
		if (FAILED(hr))
		{
			cout << "Unable to get triangleindices" << endl;
			return hr;
		}

		// Open File
		
		FILE *meshFile = NULL;
		errno_t err = fopen_s(&meshFile, filename.c_str(), "wb");

		// Could not open file for writing - return
		if (0 != err || NULL == meshFile)
		{
			cout << "cannot open the file" << endl;
			return E_ACCESSDENIED;
		}

		// Write the header line
		const unsigned char header[80] = { 0 };   // initialize all values to 0
		fwrite(&header, sizeof(unsigned char), ARRAYSIZE(header), meshFile);

		// Write number of triangles
		fwrite(&numTriangles, sizeof(int), 1, meshFile);

		// Sequentially write the normal, 3 vertices of the triangle and attribute, for each triangle
		for (unsigned int t = 0; t < numTriangles; ++t)
		{
			Vector3 normal = normals[t * 3];

			if (flipYZ)
			{
				normal.y = -normal.y;
				normal.z = -normal.z;
			}

			// Write normal
			fwrite(&normal, sizeof(float), 3, meshFile);

			// Write vertices
			for (unsigned int v = 0; v<3; v++)
			{
				Vector3 vertex = vertices[(t * 3) + v];

				if (flipYZ)
				{
					vertex.y = -vertex.y;
					vertex.z = -vertex.z;
				}

				fwrite(&vertex, sizeof(float), 3, meshFile);
			}

			unsigned short attribute = 0;
			fwrite(&attribute, sizeof(unsigned short), 1, meshFile);
		}

		fflush(meshFile);
		fclose(meshFile);

		return hr;
	}
}
//namespace




