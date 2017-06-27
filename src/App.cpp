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
		m_pSmoothDepthFloatImage(nullptr),
		m_pPointCloud(nullptr),
		m_pShadedSurface(nullptr),
		m_bInitializeError(false),
		m_pDepthFrameReader(NULL),
		m_coordinateMappingChangedEvent(NULL),
		m_bHaveValidCameraParameters(false),
		m_bSavingMesh(false),
		///added color portion
		m_pColorCoordinates(nullptr),
		m_pColorImage(nullptr),
		m_pDepthVisibilityTestMap(nullptr),
		m_pResampledColorImage(nullptr),
		m_pResampledColorImageDepthAligned(nullptr),
		m_pCapturedSurfaceColor(nullptr),
		m_pDepthRawPixelBuffer(nullptr),
		m_bCaptureColor(true)
	{

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
		m_reconstructionParams.voxelCountX = 256;   // 384 / 256vpm = 1.5m wide reconstruction
		m_reconstructionParams.voxelCountY = 256;   // Memory = 384*384*384 * 4bytes per voxel
		m_reconstructionParams.voxelCountZ = 256;   // This will require a GPU with at least 256MB

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
		SetIdentityMatrix(m_worldToBGRTransform);

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
		SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pSmoothDepthFloatImage);
		SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pColorImage);
		SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pResampledColorImage);
		SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pResampledColorImageDepthAligned);
		SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pCapturedSurfaceColor);

		// done with depth frame reader
		SafeRelease(m_pDepthFrameReader);
		//done with color frame reader
		SafeRelease(m_pColorFrameReader);

		// Clean up Kinect
		if (m_pNuiSensor)
		{
			m_pNuiSensor->Close();
			m_pNuiSensor->Release();
		}

		// clean up the depth pixel array
		SAFE_DELETE_ARRAY(m_pDepthImagePixelBuffer);
		SAFE_DELETE_ARRAY(m_pDepthRawPixelBuffer);
		SAFE_DELETE_ARRAY(m_pDepthDistortionMap);
		SAFE_DELETE_ARRAY(m_pDepthDistortionLT);

		// done with depth pixel data
		SAFE_DELETE_ARRAY(m_pDepthRGBX);
		

		//Clean up the color coordinate array
		SAFE_DELETE_ARRAY(m_pColorCoordinates);
		SAFE_DELETE_ARRAY(m_pDepthVisibilityTestMap);
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

		if (!m_bSavingMesh) {
			kinectVision->update(m_pShadedSurface->pFrameBuffer->pBits, GL_RGBA, GL_UNSIGNED_BYTE);
		}
		_mesh->draw(shader);
	}
	
	void App::onEvent(shared_ptr<Event> event)
	{
		string name = event->getName();

		if (name == "kbd_UP_down" || name == "kbd_UP_repeat") {
			ResetReconstruction();
		}
		if (name == "kbd_S_down") {
			m_bSavingMesh = true;
			INuiFusionColorMesh* mesh = nullptr;
			HRESULT hr = m_pVolume->CalculateMesh(1, &mesh);
			if (SUCCEEDED(hr)) {
//				hr = WriteAsciiObjMeshFile(mesh, "testaboop", false);
				hr = WriteAsciiPlyMeshFile(mesh, "testply", false, true);
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
			m_bSavingMesh = false;
		}
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
				//copy and remap 

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
		bool colorSynchronized = false;   // assume we are not synchronized to start with

		if (m_bCaptureColor)
		{

			IColorFrame* pColorFrame;
			hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

			if (FAILED(hr))
			{
				// Here we just do not integrate color rather than reporting an error
				colorSynchronized = false;
			}
			else
			{

				if (SUCCEEDED(hr))
				{
					IFrameDescription *desp = nullptr;
					pColorFrame->get_FrameDescription(&desp);
					int widthx = 0;
					int heightx = 0;
					uint length = 0;
					desp->get_Width(&widthx);
					desp->get_Height(&heightx);
					desp->get_LengthInPixels(&length);
					CopyColor(pColorFrame);
				}
				SafeRelease(pColorFrame);
			}
		}
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
			IColorFrameSource* pColorFrameSource = NULL;

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
			if (SUCCEEDED(hr))
			{
				hr = m_pNuiSensor->get_ColorFrameSource(&pColorFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
			}

			SafeRelease(pDepthFrameSource);
			SafeRelease(pColorFrameSource);
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
		UpdateIntrinsics(m_pSmoothDepthFloatImage, &m_cameraParameters);

		//COLOR portion
		UpdateIntrinsics(m_pColorImage, &m_cameraParameters);
		UpdateIntrinsics(m_pResampledColorImage, &m_cameraParameters);
		UpdateIntrinsics(m_pResampledColorImageDepthAligned, &m_cameraParameters);
		UpdateIntrinsics(m_pCapturedSurfaceColor, &m_cameraParameters);

		if (nullptr == m_pDepthDistortionMap)
		{
			cout << "Failed to initialize Kinect Fusion depth image distortion buffer." << endl;
			return E_OUTOFMEMORY;
		}

		if (nullptr == m_pColorCoordinates){
			cout << "Failed to initialize Kinect Fusion color image coordinate buffer" << endl;
		}
		if (nullptr == m_pDepthVisibilityTestMap)
		{
			cout << "Failed to initialize Kinect Fusion depth points visibility test buffer." << endl;
			return E_OUTOFMEMORY;
		}
		if (nullptr == m_pDepthRawPixelBuffer)
		{
			cout << "Failed to initialize Kinect Fusion raw depth image pixel buffer."<<endl;
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
		//hr = NuiFusionCreateReconstruction(
		//	&m_reconstructionParams,
		//	m_processorType, m_deviceIndex,
		//	&m_worldToCameraTransform,
		//	&m_pVolume);

		//Create a color volume
		hr = NuiFusionCreateColorReconstruction(
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
		if (FAILED(hr)){
			cout << "Failed to initialize Kinect Fusion image." << endl;
			return hr;
		}

		//Frame generated from the raw color input of Kinect
		if (FAILED(hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, cColorWidth, cColorHeight, &m_cameraParameters, &m_pColorImage))){
			cout << "Failed to initialize Kinect Fusion image." << endl;
			return hr;
		}

		//Frame re-sampled from the color input of Kinect
		if (FAILED(hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pResampledColorImageDepthAligned))) {
			cout << "Failed to initialize Kinect Fusion image." << endl;
			return hr;
		}

		// Frame generated from the raw color input of Kinect for use in the camera pose finder.
		// Note color will be down-sampled to the depth size if depth and color capture resolutions differ.
		if (FAILED(hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pResampledColorImage)))
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
		// Image of the raycast Volume with the captured color to display
		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pCapturedSurfaceColor);
		if (FAILED(hr))
		{
			return hr;
		}

		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pSmoothDepthFloatImage);
		if (FAILED(hr))
		{
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

		SAFE_DELETE_ARRAY(m_pColorCoordinates);
		m_pColorCoordinates = new(std::nothrow)ColorSpacePoint[m_cDepthHeight*m_cDepthWidth];
		if (nullptr == m_pColorCoordinates){
			cout<<"Failed to initialize Kinect Fusion color image coordinate buffer."<<endl;
			return E_OUTOFMEMORY;
		}

		SAFE_DELETE_ARRAY(m_pDepthRawPixelBuffer);
		m_pDepthRawPixelBuffer = new(std::nothrow) UINT16[m_cDepthHeight*m_cDepthWidth];

		if (nullptr == m_pDepthRawPixelBuffer)
		{
			cout<<"Failed to initialize Kinect Fusion raw depth image pixel buffer."<<endl;
			return E_OUTOFMEMORY;
		}

		SAFE_DELETE_ARRAY(m_pDepthVisibilityTestMap);
		m_pDepthVisibilityTestMap = new(std::nothrow) UINT16[(cColorWidth >> cVisibilityTestQuantShift) * (cColorHeight >> cVisibilityTestQuantShift)];

		if (nullptr == m_pDepthVisibilityTestMap)
		{
			cout<<"Failed to initialize Kinect Fusion depth points visibility test buffer."<<endl;
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

		// Convert the pixels describing extended depth as unsigned short type in millimeters to depth as floating point type in meters.
		/*Converts Kinect depth frames in unsigned short format to depth frames in float format representing distance from the camera in meters (parallel to the optical center axis).
		Note: depthImageData and depthFloatFrame must be the same pixel resolution and equal to depthImageDataWidth by depthImageDataHeight.
		The min and max depth clip values enable clipping of the input data, for example, to help isolate particular objects or surfaces to be reconstructed. 
		Note that the thresholds return different values when a depth pixel is outside the threshold - pixels inside minDepthClip will will be returned as 0 and ignored in processing, 
		whereas pixels beyond maxDepthClip will be set to 1000 to signify a valid depth ray with depth beyond the set threshold. 
		Setting this far- distance flag is important for reconstruction integration in situations where the camera is static or does not move significantly, 
		as it enables any voxels closer to the camera along this ray to be culled instead of persisting (as would happen if the pixels were simply set to 0 and ignored in processing). 
		Note that when reconstructing large real-world size volumes, be sure to set large maxDepthClip distances, as when the camera moves around, 
		any voxels in view which go beyond this threshold distance from the camera will be removed.*/

		hr = m_pVolume->DepthToDepthFloatFrame(m_pDepthImagePixelBuffer, 
				m_cDepthImagePixels * sizeof(UINT16), 
				m_pDepthFloatImage, m_fMinDepthThreshold,
				m_fMaxDepthThreshold, 
				m_bMirrorDepthFrame);

		if (FAILED(hr))
		{
			cout << "Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed." << endl;
			return;
		}

		hr = m_pVolume->SmoothDepthFloatFrame(m_pDepthFloatImage, m_pSmoothDepthFloatImage, 1, 0.04f);
		if (FAILED(hr))
		{
			cout << "Kinect Fusion SmoothDepth call failed." << endl;
			return;
		}
		
		hr = NuiFusionDepthFloatFrameToPointCloud(
			m_pSmoothDepthFloatImage,
			m_pPointCloud);

		////////////////////////////////////////////////////////
		// ProcessFrame

		// Perform the camera tracking and update the Kinect Fusion Volume
		// This will create memory on the GPU, upload the image, run camera tracking and integrate the
		// data into the Reconstruction Volume if successful. Note that passing nullptr as the final 
		// parameter will use and update the internal camera pose.

	/*A high-level function to process a depth frame through the Kinect Fusion pipeline.
	Specifically, this performs processing equivalent to the following functions for each frame:
	1. AlignDepthFloatToReconstruction
	2. IntegrateFrame
	After this call completes, if a visible output image of the reconstruction is required, the user can call CalculatePointCloud and then ShadePointCloud. 
	The maximum image resolution supported in this function is 640x480.
	If there is a tracking error in the AlignDepthFloatToReconstruction stage, no depth data integration will be performed, and the camera pose will remain unchanged.*/
		
		
		//hr = m_pVolume->ProcessFrame(m_pDepthFloatImage, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, m_cMaxIntegrationWeight, nullptr, &m_worldToCameraTransform);
		FLOAT alignmentEnergy = 0;
		DownsampleColorFrameToDepthResolution(m_pColorImage, m_pResampledColorImageDepthAligned);
		//m_pVolume->AlignDepthFloatToReconstruction(m_pDepthFloatImage,MAX_NATURAL_ALIGNMENT,);
		// Map the color frame to the depth - this fills m_pResampledColorImageDepthAligned

		MapColorToDepth();
		hr = m_pVolume->ProcessFrame(m_pSmoothDepthFloatImage, m_pResampledColorImageDepthAligned,NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, m_cMaxIntegrationWeight, 1.0f, &alignmentEnergy,
			&m_worldToCameraTransform);
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
		/*Calculates a point cloud by raycasting into the reconstruction volume, 
		returning the point cloud containing 3D points and normals of the zero-crossing dense surface at every visible pixel in the image 
		from the specified camera pose and color visualization image.*/
		//hr = m_pVolume->CalculatePointCloud(m_pPointCloud, &m_worldToCameraTransform);
		hr = m_pVolume->CalculatePointCloud(m_pPointCloud, m_pCapturedSurfaceColor, &m_worldToCameraTransform);
		if (FAILED(hr))
		{
			cout << "Kinect Fusion CalculatePointCloud call failed." << endl;
			return;
		}

		////////////////////////////////////////////////////////
		// ShadePointCloud and render

		/*
		Produces two shaded images from the point cloud frame, based on point position and surface normal.
		
		NUI_FUSION_IMAGE_FRAME pPointCloudFrame : The point cloud frame to shade.
		
		Matrix4 pWorldToCameraTransform : The transform used to determine the perspective of the shaded point cloud relative to the world. 
		This affects the application of the worldToRGBTramsform to the point cloud. 
		To achieve consistent results, this should match the world-to-camera transform that was used to create the point cloud frame.
		
		Matrix4 pWorldToBGRTransform : The transform that defines a mapping from position to RGB color space. 
		The X, Y, Z components of the point positions transformed by this matrix are used as the R, G, B values in the rendered frame.
		
		NUI_FUSION_IMAGE_FRAME pShadedSurfaceFrame : A color image frame that receives the shaded frame which includes lighting based on the surface normal.
		
		NUI_FUSION_IMAGE_FRAME pShadedSurfaceNormalsFrame :  A color image frame that receives the shaded frame.
		*/
		hr = NuiFusionShadePointCloud(m_pPointCloud, &m_worldToCameraTransform, &m_worldToBGRTransform, m_pShadedSurface, nullptr);

		if (FAILED(hr))
		{
			cout << "Kinect Fusion NuiFusionShadePointCloud call failed." << endl;
			return;
		}

		// Draw the shaded raycast volume image
		//BYTE * pBuffer = m_pShadedSurface->pFrameBuffer->pBits;

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
		SetIdentityMatrix(m_worldToBGRTransform);
		m_worldToBGRTransform.M11 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountX;
		m_worldToBGRTransform.M22 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountY;
		m_worldToBGRTransform.M33 = m_reconstructionParams.voxelsPerMeter /m_reconstructionParams.voxelCountZ;
		m_worldToBGRTransform.M41 = 0.5f;
		m_worldToBGRTransform.M42 = 0.5f;
		m_worldToBGRTransform.M44 = 1.0f;
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

	/*HRESULT App::exportMesh(const std::string &filename, INuiFusionMesh *mesh) {
		
		aiScene* scene = new aiScene();
		scene->mRootNode = new aiNode;
		HRESULT hr = S_OK;
		std::vector<aiMesh*> MeshArray;
		//assert(NULL != )
		//const size_t oldMeshSize = MeshArray.size();
		aiNode *pNode = new aiNode;
		pNode->mName = "model";
		aiNode *pParent = scene->mRootNode;
		aiMesh *pMesh = new aiMesh;
		if (pParent != NULL) {
			appendChildToParentNode(pParent, pNode);
		}

		const Vector3 *vertices = NULL;
		hr = mesh->GetVertices(&vertices);
		if (FAILED(hr))
		{
			return hr;
		}

		const Vector3 *normals = NULL;
		hr = mesh->GetNormals(&normals);
		if (FAILED(hr))
		{
			return hr;
		}

		const int *triangleIndices = NULL;
		hr = mesh->GetTriangleIndices(&triangleIndices);
		if (FAILED(hr))
		{
			return hr;
		}
		unsigned int numVertices = mesh->VertexCount();
		unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
		unsigned int numTriangles = numVertices / 3;

		pMesh->mNumFaces = numTriangles;
		if (pMesh->mNumFaces > 0) {
			pMesh->mFaces = new aiFace[pMesh->mNumFaces];
			
			for (int i = 0; i < numTriangles; i++) {
				aiFace *pFace = &pMesh->mFaces[i];
				pFace->mNumIndices = 3;
				pFace->mIndices = new unsigned int[3];
			}
		}
		pMesh->mNumVertices = numTriangles * 3;
		
		if (pMesh->mNumVertices > 0) {
			pMesh->mVertices = new aiVector3D[pMesh->mNumVertices];
			pMesh->mNormals = new aiVector3D[pMesh->mNumVertices];
			pMesh->mNumUVComponents[0] = 2;
			pMesh->mTextureCoords[0] = new aiVector3D[pMesh->mNumVertices];

			for (int i = 0; i < numVertices; i++) {
				glm::dvec3 pt = dvec3(vertices[i].x, vertices[i].y, vertices[i].z);
				pMesh->mVertices[i].x = pt.x;
				pMesh->mVertices[i].y = pt.y;
				pMesh->mVertices[i].z = pt.z;
			//	cout << pMesh -> mVertices[i].x <<","<< pMesh->mVertices[i].y <<","<< pMesh->mVertices[i].z << endl;
				glm::dvec3 normal = dvec3(normals[i].x, normals[i].y, normals[i].z);
				pMesh->mNormals[i].x = normal.x;
				pMesh->mNormals[i].y = normal.y;
				pMesh->mNormals[i].z = normal.z;
				pMesh->mTextureCoords[0][i].x = 0.0;
				pMesh->mTextureCoords[0][i].y = 0.0;
				pMesh->mTextureCoords[0][i].z = 0.0;
			}
			for (int i = 0; i < numTriangles; i++) {
				aiFace *pDestFace = &pMesh->mFaces[i];
				for (int j = 0; j < 3; j++) {
					pDestFace->mIndices[j] = triangleIndices[3*i+j];
				}
			}
			MeshArray.push_back(pMesh);
		}
		else {
			delete pMesh;
		}
		pNode->mMeshes = new unsigned int[1];
		pNode->mNumMeshes = 1;
		scene->mNumMeshes = 1;
		scene->mMeshes = new aiMesh*[1];
		scene->mMeshes[0] = pMesh;
		
		scene->mNumMaterials = 1;
		scene->mMaterials = new aiMaterial*[1];
		aiMaterial* mat = new aiMaterial;
		aiString matName("defaultMaterial");
		mat->AddProperty(&matName, AI_MATKEY_NAME);
		int sm = aiShadingMode_Phong;
		mat->AddProperty<int>(&sm, 1, AI_MATKEY_SHADING_MODEL);

		aiColor3D diffuse(0.7, 0.7, 0.7);
		aiColor3D ambient(0.2, 0.2, 0.2);
		aiColor3D specular(1.0, 1.0, 1.0);
		float shineness = 0.5;
		float alpha = 1.0;
		mat->AddProperty(&ambient, 1, AI_MATKEY_COLOR_AMBIENT);
		mat->AddProperty(&diffuse, 1, AI_MATKEY_COLOR_DIFFUSE);
		mat->AddProperty(&specular, 1, AI_MATKEY_COLOR_SPECULAR);
		mat->AddProperty(&shineness, 1, AI_MATKEY_SHININESS);
		mat->AddProperty(&alpha, 1, AI_MATKEY_OPACITY);
		scene->mMaterials[0] = mat;


		
		if (_exporter.get() == nullptr) {
			_exporter.reset(new Assimp::Exporter());
		}
		aiReturn airesult=_exporter->Export(scene, "obj", filename, 0);
		if (FAILED(airesult)) {
			cout << "failed to save" << endl;
			return !hr;
		}
		delete scene;
		return hr;
	}

	void App::appendChildToParentNode(aiNode *pParent, aiNode *pChild)
	{
		// Checking preconditions
		assert(NULL != pParent);
		assert(NULL != pChild);

		// Assign parent to child
		pChild->mParent = pParent;
		size_t sNumChildren = 0;
		(void)sNumChildren; // remove warning on release build

							// If already children was assigned to the parent node, store them in a 
		std::vector<aiNode*> temp;
		if (pParent->mChildren != NULL)
		{
			sNumChildren = pParent->mNumChildren;
			assert(0 != sNumChildren);
			for (size_t index = 0; index < pParent->mNumChildren; index++)
			{
				temp.push_back(pParent->mChildren[index]);
			}
			delete[] pParent->mChildren;
		}

		// Copy node instances into parent node
		pParent->mNumChildren++;
		pParent->mChildren = new aiNode*[pParent->mNumChildren];
		for (size_t index = 0; index < pParent->mNumChildren - 1; index++)
		{
			pParent->mChildren[index] = temp[index];
		}
		pParent->mChildren[pParent->mNumChildren - 1] = pChild;
	}*/

	/// <summary>
	/// Get Color data
	/// </summary>
	/// <param name="imageFrame">The color image frame to copy.</param>
	/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT App::CopyColor(IColorFrame* pColorFrame)
{
	HRESULT hr = S_OK;

	if (nullptr == m_pColorImage)
	{
		cout<<"Error copying color texture pixels."<< endl;
		return E_FAIL;
	}

	NUI_FUSION_BUFFER *destColorBuffer = m_pColorImage->pFrameBuffer;

	if (nullptr == pColorFrame || nullptr == destColorBuffer)
	{
		return E_NOINTERFACE;
	}

	int width = m_pColorImage->width;
	int height = m_pColorImage->height;

	// Copy the color pixels so we can return the image frame
	hr = pColorFrame->CopyConvertedFrameDataToArray( cColorWidth * cColorHeight * sizeof(RGBQUAD), destColorBuffer->pBits, ColorImageFormat_Bgra);

	if (FAILED(hr))
	{
		cout<<"Error copying color texture pixels."<<endl;
		hr = E_FAIL;
	}

	return hr;
}

/// <summary>
/// Adjust color to the same space as depth
/// </summary>
/// <returns>S_OK for success, or failure code</returns>
HRESULT App::MapColorToDepth()
{
	HRESULT hr = S_OK;

	if (nullptr == m_pColorImage || nullptr == m_pResampledColorImageDepthAligned
		|| nullptr == m_pColorCoordinates || nullptr == m_pDepthVisibilityTestMap)
	{
		return E_FAIL;
	}

	NUI_FUSION_BUFFER *srcColorBuffer = m_pColorImage->pFrameBuffer;
	NUI_FUSION_BUFFER *destColorBuffer = m_pResampledColorImageDepthAligned->pFrameBuffer;

	if (nullptr == srcColorBuffer || nullptr == destColorBuffer)
	{
		cout << "Error accessing color textures." << endl;
		return E_NOINTERFACE;
	}

	if (FAILED(hr) || srcColorBuffer->Pitch == 0)
	{
		cout<<"Error accessing color texture pixels."<<endl;
		return  E_FAIL;
	}

	if (FAILED(hr) || destColorBuffer->Pitch == 0)
	{
		cout << "Error accessing color texture pixels."<<endl;
		return  E_FAIL;
	}

	int *rawColorData = reinterpret_cast<int*>(srcColorBuffer->pBits);
	int *colorDataInDepthFrame = reinterpret_cast<int*>(destColorBuffer->pBits);

	// Get the coordinates to convert color to depth space
	hr = m_pMapper->MapDepthFrameToColorSpace(NUI_DEPTH_RAW_WIDTH * NUI_DEPTH_RAW_HEIGHT, m_pDepthRawPixelBuffer,
		NUI_DEPTH_RAW_WIDTH * NUI_DEPTH_RAW_HEIGHT, m_pColorCoordinates);

	if (FAILED(hr))
	{
		return hr;
	}

	// construct dense depth points visibility test map so we can test for depth points that are invisible in color space
	const UINT16* const pDepthEnd = m_pDepthRawPixelBuffer + NUI_DEPTH_RAW_WIDTH * NUI_DEPTH_RAW_HEIGHT;
	const ColorSpacePoint* pColorPoint = m_pColorCoordinates;
	const UINT testMapWidth = UINT(cColorWidth >> cVisibilityTestQuantShift);
	const UINT testMapHeight = UINT(cColorHeight >> cVisibilityTestQuantShift);
	ZeroMemory(m_pDepthVisibilityTestMap, testMapWidth * testMapHeight * sizeof(UINT16));
	for (const UINT16* pDepth = m_pDepthRawPixelBuffer; pDepth < pDepthEnd; pDepth++, pColorPoint++)
	{
		const UINT x = UINT(pColorPoint->X + 0.5f) >> cVisibilityTestQuantShift;
		const UINT y = UINT(pColorPoint->Y + 0.5f) >> cVisibilityTestQuantShift;
		if (x < testMapWidth && y < testMapHeight)
		{
			const UINT idx = y * testMapWidth + x;
			const UINT16 oldDepth = m_pDepthVisibilityTestMap[idx];
			const UINT16 newDepth = *pDepth;
			if (!oldDepth || oldDepth > newDepth)
			{
				m_pDepthVisibilityTestMap[idx] = newDepth;
			}
		}
	}


	// Loop over each row and column of the destination color image and copy from the source image
	// Note that we could also do this the other way, and convert the depth pixels into the color space, 
	// avoiding black areas in the converted color image and repeated color images in the background
	// However, then the depth would have radial and tangential distortion like the color camera image,
	// which is not ideal for Kinect Fusion reconstruction.

	if (m_bMirrorDepthFrame)
	{
		Concurrency::parallel_for(0u, m_cDepthHeight, [&](UINT y)
		{
			const UINT depthWidth = m_cDepthWidth;
			const UINT depthImagePixels = m_cDepthImagePixels;
			const UINT colorHeight = cColorHeight;
			const UINT colorWidth = cColorWidth;
			const UINT testMapWidth = UINT(colorWidth >> cVisibilityTestQuantShift);

			UINT destIndex = y * depthWidth;
			for (UINT x = 0; x < depthWidth; ++x, ++destIndex)
			{
				int pixelColor = 0;
				const UINT mappedIndex = m_pDepthDistortionLT[destIndex];
				if (mappedIndex < depthImagePixels)
				{
					// retrieve the depth to color mapping for the current depth pixel
					const ColorSpacePoint colorPoint = m_pColorCoordinates[mappedIndex];

					// make sure the depth pixel maps to a valid point in color space
					const UINT colorX = (UINT)(colorPoint.X + 0.5f);
					const UINT colorY = (UINT)(colorPoint.Y + 0.5f);
					if (colorX < colorWidth && colorY < colorHeight)
					{
						const UINT16 depthValue = m_pDepthRawPixelBuffer[mappedIndex];
						const UINT testX = colorX >> cVisibilityTestQuantShift;
						const UINT testY = colorY >> cVisibilityTestQuantShift;
						const UINT testIdx = testY * testMapWidth + testX;
						const UINT16 depthTestValue = m_pDepthVisibilityTestMap[testIdx];
						_ASSERT(depthValue >= depthTestValue);
						if (depthValue - depthTestValue < cDepthVisibilityTestThreshold)
						{
							// calculate index into color array
							const UINT colorIndex = colorX + (colorY * colorWidth);
							pixelColor = rawColorData[colorIndex];
						}
					}
				}
				colorDataInDepthFrame[destIndex] = pixelColor;
			}
		});
	}
	else
	{
		Concurrency::parallel_for(0u, m_cDepthHeight, [&](UINT y)
		{
			const UINT depthWidth = m_cDepthWidth;
			const UINT depthImagePixels = m_cDepthImagePixels;
			const UINT colorHeight = cColorHeight;
			const UINT colorWidth = cColorWidth;
			const UINT testMapWidth = UINT(colorWidth >> cVisibilityTestQuantShift);

			// Horizontal flip the color image as the standard depth image is flipped internally in Kinect Fusion
			// to give a viewpoint as though from behind the Kinect looking forward by default.
			UINT destIndex = y * depthWidth;
			UINT flipIndex = destIndex + depthWidth - 1;
			for (UINT x = 0; x < depthWidth; ++x, ++destIndex, --flipIndex)
			{
				int pixelColor = 0;
				const UINT mappedIndex = m_pDepthDistortionLT[destIndex];
				if (mappedIndex < depthImagePixels)
				{
					// retrieve the depth to color mapping for the current depth pixel
					const ColorSpacePoint colorPoint = m_pColorCoordinates[mappedIndex];

					// make sure the depth pixel maps to a valid point in color space
					const UINT colorX = (UINT)(colorPoint.X + 0.5f);
					const UINT colorY = (UINT)(colorPoint.Y + 0.5f);
					if (colorX < colorWidth && colorY < colorHeight)
					{
						const UINT16 depthValue = m_pDepthRawPixelBuffer[mappedIndex];
						const UINT testX = colorX >> cVisibilityTestQuantShift;
						const UINT testY = colorY >> cVisibilityTestQuantShift;
						const UINT testIdx = testY * testMapWidth + testX;
						const UINT16 depthTestValue = m_pDepthVisibilityTestMap[testIdx];
						_ASSERT(depthValue >= depthTestValue);
						if (depthValue - depthTestValue < cDepthVisibilityTestThreshold)
						{
							// calculate index into color array
							const UINT colorIndex = colorX + (colorY * colorWidth);
							pixelColor = rawColorData[colorIndex];
						}
					}
				}
				colorDataInDepthFrame[flipIndex] = pixelColor;
			}
		});
	}

	return hr;
	}

	/// <summary>
	/// Down sample color frame with nearest neighbor to the depth frame resolution
	/// </summary>
	/// <param name="src">The source color image.</param>
	/// <param name="dest">The destination down sampled  image.</param>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT App:: DownsampleColorFrameToDepthResolution(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest)
	{
		if (nullptr == src || nullptr == dest)
		{
			return E_INVALIDARG;
		}

		if (src->imageType != NUI_FUSION_IMAGE_TYPE_COLOR || src->imageType != dest->imageType
			|| src->width != 1920 || src->height != 1080 || dest->width != NUI_DEPTH_RAW_WIDTH || dest->height != NUI_DEPTH_RAW_HEIGHT)
		{
			return E_INVALIDARG;
		}

		NUI_FUSION_BUFFER *srcFrameBuffer = src->pFrameBuffer;
		NUI_FUSION_BUFFER *downsampledFloatFrameBuffer = dest->pFrameBuffer;

		float factor = 1080.0f / NUI_DEPTH_RAW_HEIGHT;

		// Make sure we've received valid data
		if (srcFrameBuffer->Pitch == 0 || downsampledFloatFrameBuffer->Pitch == 0)
		{
			return E_NOINTERFACE;
		}

		HRESULT hr = S_OK;
		float *srcValues = (float *)srcFrameBuffer->pBits;
		float *downsampledDestValues = (float *)downsampledFloatFrameBuffer->pBits;

		const unsigned int filledZeroMargin = 0;
		const unsigned int downsampledWidth = dest->width;
		const unsigned int srcImageWidth = src->width;

		ZeroMemory(downsampledDestValues, downsampledFloatFrameBuffer->Pitch * dest->height);
		Concurrency::parallel_for(filledZeroMargin, dest->height - filledZeroMargin, [=, &downsampledDestValues, &srcValues](unsigned int y)
		{
			unsigned int index = dest->width * y;
			for (unsigned int x = 0; x < downsampledWidth; ++x, ++index)
			{
				int srcX = (int)(x * factor);
				int srcY = (int)(y * factor);
				int srcIndex = srcY * srcImageWidth + srcX;
				downsampledDestValues[index] = srcValues[srcIndex];
			}
		});

		return hr;
	}

	
	/// <summary>
	/// Write ASCII Wavefront .OBJ file
	/// See http://en.wikipedia.org/wiki/Wavefront_.obj_file for .OBJ format
	/// </summary>
	/// <param name="mesh">The Kinect Fusion mesh object.</param>
	/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
	/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
	/// <returns>indicates success or failure</returns>
	HRESULT App::WriteAsciiObjMeshFile(INuiFusionColorMesh *mesh, std::string filename, bool flipYZ)
	{
		HRESULT hr = S_OK;

		if (NULL == mesh)
		{
			return E_INVALIDARG;
		}

		unsigned int numVertices = mesh->VertexCount();
		unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
		unsigned int numTriangles = numVertices / 3;
		unsigned int numColors = mesh->ColorCount();

		if (0 == numVertices || 0 == numTriangleIndices || 0 != numVertices % 3 || numVertices != numTriangleIndices || ( numVertices != numColors))
		{
			return E_INVALIDARG;
		}

		const Vector3 *vertices = NULL;
		hr = mesh->GetVertices(&vertices);
		if (FAILED(hr))
		{
			return hr;
		}

		const Vector3 *normals = NULL;
		hr = mesh->GetNormals(&normals);
		if (FAILED(hr))
		{
			return hr;
		}

		const int *triangleIndices = NULL;
		hr = mesh->GetTriangleIndices(&triangleIndices);
		if (FAILED(hr))
		{
			return hr;
		}
		
		const int *colors = NULL;
		hr = mesh->GetColors(&colors);
		if (FAILED(hr)){
			return hr;
		}
		

		FILE *meshFile = NULL;
		errno_t err = fopen_s(&meshFile, filename.c_str(), "wt");

		// Could not open file for writing - return
		if (0 != err || NULL == meshFile)
		{
			return E_ACCESSDENIED;
		}

		// Write the header line
		std::string header = "#\n# OBJ file created by Microsoft Kinect Fusion\n#\n";
		fwrite(header.c_str(), sizeof(char), header.length(), meshFile);

		const unsigned int bufSize = MAX_PATH * 3;
		char outStr[bufSize];
		int written = 0;


			// Sequentially write the 3 vertices of the triangle, for each triangle
		for (unsigned int t = 0, vertexIndex = 0; t < numTriangles; ++t, vertexIndex += 3)
			{
				unsigned int color0 = colors[vertexIndex];
				unsigned int color1 = colors[vertexIndex + 1];
				unsigned int color2 = colors[vertexIndex + 2];

				written = sprintf_s(outStr, bufSize, "v %f %f %f \nv %f %f %f \nv %f %f %f\n",
					vertices[vertexIndex].x, vertices[vertexIndex].y, vertices[vertexIndex].z,
					//	(float)((color0>>16)&255)/255.0, (float)((color0 >> 8) & 255) / 255.0, (float)((color0) & 255) / 255.0,
					vertices[vertexIndex + 1].x, vertices[vertexIndex + 1].y, vertices[vertexIndex + 1].z,
					//	(float)((color1 >> 16) & 255) / 255.0, (float)((color1 >> 8) & 255) / 255.0, (float)((color1) & 255) / 255.0,
					vertices[vertexIndex + 2].x, vertices[vertexIndex + 2].y, vertices[vertexIndex + 2].z);
				//	(float)((color2 >> 16) & 255) / 255.0, (float)((color2 >> 8) & 255) / 255.0, (float)((color2) & 255) / 255.0;
				fwrite(outStr, sizeof(char), written, meshFile);
			}

			// Sequentially write the 3 normals of the triangle, for each triangle
			for (unsigned int t = 0, normalIndex = 0; t < numTriangles; ++t, normalIndex += 3)
			{
				written = sprintf_s(outStr, bufSize, "n %f %f %f\nn %f %f %f\nn %f %f %f\n",
					normals[normalIndex].x, normals[normalIndex].y, normals[normalIndex].z,
					normals[normalIndex + 1].x, normals[normalIndex + 1].y, normals[normalIndex + 1].z,
					normals[normalIndex + 2].x, normals[normalIndex + 2].y, normals[normalIndex + 2].z);
				fwrite(outStr, sizeof(char), written, meshFile);
			}
		
		
		// Sequentially write the 3 vertex indices of the triangle face, for each triangle
		// Note this is typically 1-indexed in an OBJ file when using absolute referencing!
		for (unsigned int t = 0, baseIndex = 1; t < numTriangles; ++t, baseIndex += 3) // Start at baseIndex=1 for the 1-based indexing.
		{
			written = sprintf_s(outStr, bufSize, "f %u//%u %u//%u %u//%u\n",
				baseIndex, baseIndex, baseIndex + 1, baseIndex + 1, baseIndex + 2, baseIndex + 2);
			fwrite(outStr, sizeof(char), written, meshFile);
		}

		// Note: we do not have texcoords to store, if we did, we would put the index of the texcoords between the vertex and normal indices (i.e. between the two slashes //) in the string above
		fflush(meshFile);
		fclose(meshFile);

		return hr;
	}
	
	/// <summary>
	/// Write ASCII .PLY file
	/// See http://paulbourke.net/dataformats/ply/ for .PLY format
	/// </summary>
	/// <param name="mesh">The Kinect Fusion mesh object.</param>
	/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
	/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
	/// <param name="outputColor">Set this true to write out the surface color to the file when it has been captured.</param>
	/// <returns>indicates success or failure</returns>
	HRESULT App::WriteAsciiPlyMeshFile(INuiFusionColorMesh *mesh, std::string filename, bool flipYZ, bool outputColor)
	{
		HRESULT hr = S_OK;

		if (NULL == mesh)
		{
			return E_INVALIDARG;
		}

		unsigned int numVertices = mesh->VertexCount();
		unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
		unsigned int numTriangles = numVertices / 3;
		unsigned int numColors = mesh->ColorCount();

		if (0 == numVertices || 0 == numTriangleIndices || 0 != numVertices % 3
			|| numVertices != numTriangleIndices || (outputColor && numVertices != numColors))
		{
			return E_INVALIDARG;
		}

		const Vector3 *vertices = NULL;
		hr = mesh->GetVertices(&vertices);
		if (FAILED(hr))
		{
			return hr;
		}

		const int *triangleIndices = NULL;
		hr = mesh->GetTriangleIndices(&triangleIndices);
		if (FAILED(hr))
		{
			return hr;
		}

		const int *colors = NULL;
		if (outputColor)
		{
			hr = mesh->GetColors(&colors);
			if (FAILED(hr))
			{
				return hr;
			}
		}

		// Open File
		FILE *meshFile = NULL;
		errno_t err = fopen_s(&meshFile, filename.c_str(), "wt");

		// Could not open file for writing - return
		if (0 != err || NULL == meshFile)
		{
			return E_ACCESSDENIED;
		}

		// Write the header line
		std::string header = "ply\nformat ascii 1.0\ncomment file created by Microsoft Kinect Fusion\n";
		fwrite(header.c_str(), sizeof(char), header.length(), meshFile);

		const unsigned int bufSize = MAX_PATH * 3;
		char outStr[bufSize];
		int written = 0;

		if (outputColor)
		{
			// Elements are: x,y,z, r,g,b
			written = sprintf_s(outStr, bufSize, "element vertex %u\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n", numVertices);
			fwrite(outStr, sizeof(char), written, meshFile);
		}
		else
		{
			// Elements are: x,y,z
			written = sprintf_s(outStr, bufSize, "element vertex %u\nproperty float x\nproperty float y\nproperty float z\n", numVertices);
			fwrite(outStr, sizeof(char), written, meshFile);
		}

		written = sprintf_s(outStr, bufSize, "element face %u\nproperty list uchar int vertex_index\nend_header\n", numTriangles);
		fwrite(outStr, sizeof(char), written, meshFile);

		if (flipYZ)
		{
			if (outputColor)
			{
				// Sequentially write the 3 vertices of the triangle, for each triangle
				for (unsigned int t = 0, vertexIndex = 0; t < numTriangles; ++t, vertexIndex += 3)
				{
					unsigned int color0 = colors[vertexIndex];
					unsigned int color1 = colors[vertexIndex + 1];
					unsigned int color2 = colors[vertexIndex + 2];

					written = sprintf_s(outStr, bufSize, "%f %f %f %u %u %u\n%f %f %f %u %u %u\n%f %f %f %u %u %u\n",
						vertices[vertexIndex].x, -vertices[vertexIndex].y, -vertices[vertexIndex].z,
						((color0 >> 16) & 255), ((color0 >> 8) & 255), (color0 & 255),
						vertices[vertexIndex + 1].x, -vertices[vertexIndex + 1].y, -vertices[vertexIndex + 1].z,
						((color1 >> 16) & 255), ((color1 >> 8) & 255), (color1 & 255),
						vertices[vertexIndex + 2].x, -vertices[vertexIndex + 2].y, -vertices[vertexIndex + 2].z,
						((color2 >> 16) & 255), ((color2 >> 8) & 255), (color2 & 255));

					fwrite(outStr, sizeof(char), written, meshFile);
				}
			}
			else
			{
				// Sequentially write the 3 vertices of the triangle, for each triangle
				for (unsigned int t = 0, vertexIndex = 0; t < numTriangles; ++t, vertexIndex += 3)
				{
					written = sprintf_s(outStr, bufSize, "%f %f %f\n%f %f %f\n%f %f %f\n",
						vertices[vertexIndex].x, -vertices[vertexIndex].y, -vertices[vertexIndex].z,
						vertices[vertexIndex + 1].x, -vertices[vertexIndex + 1].y, -vertices[vertexIndex + 1].z,
						vertices[vertexIndex + 2].x, -vertices[vertexIndex + 2].y, -vertices[vertexIndex + 2].z);
					fwrite(outStr, sizeof(char), written, meshFile);
				}
			}
		}
		else
		{
			if (outputColor)
			{
				// Sequentially write the 3 vertices of the triangle, for each triangle
				for (unsigned int t = 0, vertexIndex = 0; t < numTriangles; ++t, vertexIndex += 3)
				{
					unsigned int color0 = colors[vertexIndex];
					unsigned int color1 = colors[vertexIndex + 1];
					unsigned int color2 = colors[vertexIndex + 2];

					written = sprintf_s(outStr, bufSize, "%f %f %f %u %u %u\n%f %f %f %u %u %u\n%f %f %f %u %u %u\n",
						vertices[vertexIndex].x, vertices[vertexIndex].y, vertices[vertexIndex].z,
						((color0 >> 16) & 255), ((color0 >> 8) & 255), (color0 & 255),
						vertices[vertexIndex + 1].x, vertices[vertexIndex + 1].y, vertices[vertexIndex + 1].z,
						((color1 >> 16) & 255), ((color1 >> 8) & 255), (color1 & 255),
						vertices[vertexIndex + 2].x, vertices[vertexIndex + 2].y, vertices[vertexIndex + 2].z,
						((color2 >> 16) & 255), ((color2 >> 8) & 255), (color2 & 255));

					fwrite(outStr, sizeof(char), written, meshFile);
				}
			}
			else
			{
				// Sequentially write the 3 vertices of the triangle, for each triangle
				for (unsigned int t = 0, vertexIndex = 0; t < numTriangles; ++t, vertexIndex += 3)
				{
					written = sprintf_s(outStr, bufSize, "%f %f %f\n%f %f %f\n%f %f %f\n",
						vertices[vertexIndex].x, vertices[vertexIndex].y, vertices[vertexIndex].z,
						vertices[vertexIndex + 1].x, vertices[vertexIndex + 1].y, vertices[vertexIndex + 1].z,
						vertices[vertexIndex + 2].x, vertices[vertexIndex + 2].y, vertices[vertexIndex + 2].z);
					fwrite(outStr, sizeof(char), written, meshFile);
				}
			}
		}

		// Sequentially write the 3 vertex indices of the triangle face, for each triangle (0-referenced in PLY)
		for (unsigned int t = 0, baseIndex = 0; t < numTriangles; ++t, baseIndex += 3)
		{
			written = sprintf_s(outStr, bufSize, "3 %u %u %u\n", baseIndex, baseIndex + 1, baseIndex + 2);
			fwrite(outStr, sizeof(char), written, meshFile);
		}

		fflush(meshFile);
		fclose(meshFile);

		return hr;
	}

}
//namespace




