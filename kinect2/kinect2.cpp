#include "stdafx.h"
#include "kinect2.h"
#include <iostream>

template<typename Interface>
inline void SafeRelease(Interface *pInterface){
	if(pInterface){
		pInterface->Release();
		pInterface = NULL;
	}
}

Kinect2::Kinect2(void) : imageWidth(1920), imageHeight(1080),
	depthWidth(512), depthHeight(424)
{
	imageData = new BYTE[imageWidth * imageHeight * 4];
	depthData = new UINT16[depthWidth * depthHeight];
	kinectSensor = NULL;
	colorFrameReader = NULL;
	depthFrameReader = NULL;
	bodyFrameReader = NULL;
	coordinateMapper = NULL;
	isColorFrameNew = false;
	isDepthFrameNew = false;
	isBodyFrameNew = false;
	cameraSpacePoints = new CameraSpacePoint[imageWidth * imageHeight];
	depthSpacePoints = new DepthSpacePoint[imageWidth * imageHeight];
}

Kinect2::~Kinect2(void)
{
	if(imageData){
		delete [] imageData;
		imageData = NULL;
	}

	if(depthData){
		delete [] depthData;
		depthData = NULL;
	}

	if(cameraSpacePoints){
		delete [] cameraSpacePoints;
		cameraSpacePoints = NULL;
	}

	if(depthSpacePoints){
		delete [] depthSpacePoints;
		depthSpacePoints = NULL;
	}

	if(kinectSensor){
		kinectSensor->Close();
		SafeRelease(kinectSensor);
	}
	SafeRelease(colorFrameReader);
}

BYTE *Kinect2::getImageData(){
	isColorFrameNew = false;
	return imageData;
}

UINT16 *Kinect2::getDepthData(){
	isDepthFrameNew = false;
	return depthData;
}


int Kinect2::getNumOfJoints(){
	return JointType::JointType_Count;
}
	
Joint *Kinect2::getJoints(){
	isBodyFrameNew = false;
	return joints;
}
	
ICoordinateMapper *Kinect2::getCoordinateMapper(){
	return coordinateMapper;
}

int Kinect2::open(){
	HRESULT hr = GetDefaultKinectSensor(&kinectSensor);
	if(SUCCEEDED(hr)){
		// Open kinect
		hr = kinectSensor->Open();
		if(FAILED(hr)){
			return -1;
		}
		
		// Get color frame reader
		IColorFrameSource *colorFrameSource;
		hr = kinectSensor->get_ColorFrameSource(&colorFrameSource);
		if(FAILED(hr)){
			return -1;
		}
		hr = colorFrameSource->OpenReader(&colorFrameReader);
		if(FAILED(hr)){
			return -1;
		}
		SafeRelease(colorFrameSource);

		// Get depth frame reader
		IDepthFrameSource *depthFrameSource;
		hr = kinectSensor->get_DepthFrameSource(&depthFrameSource);
		if(FAILED(hr)){
			return -1;
		}
		hr = depthFrameSource->OpenReader(&depthFrameReader);
		if(FAILED(hr)){
			return -1;
		}
		SafeRelease(depthFrameSource);

		// Get body frame reader
		IBodyFrameSource *bodyFrameSource;
		hr = kinectSensor->get_BodyFrameSource(&bodyFrameSource);
		if(FAILED(hr)){
			return -1;
		}
		hr = bodyFrameSource->OpenReader(&bodyFrameReader);
		if(FAILED(hr)){
			return -1;
		}
		SafeRelease(bodyFrameSource);

		// Get mapper
		hr = kinectSensor->get_CoordinateMapper(&coordinateMapper);
		if(FAILED(hr)){
			return -1;
		}

		return 0;
	}

	return -1;
}

void Kinect2::update(){
	// read color frame
	if(colorFrameReader){
		IColorFrame *colorFrame = NULL;
		HRESULT hr = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if(SUCCEEDED(hr)){
			colorFrame->CopyConvertedFrameDataToArray(imageWidth * imageHeight * 4, imageData, ColorImageFormat::ColorImageFormat_Bgra);
			isColorFrameNew = true;
		}
		SafeRelease(colorFrame);
	}
	// read depth frame
	if(depthFrameReader){
		IDepthFrame *depthFrame = NULL;
		HRESULT hr = depthFrameReader->AcquireLatestFrame(&depthFrame);
		if(SUCCEEDED(hr)){
			//UINT capacity = depthWidth * depthHeight * sizeof(UINT16);
			//depthFrame->AccessUnderlyingBuffer(&capacity, &depthData);
			depthFrame->CopyFrameDataToArray(depthWidth * depthHeight, depthData);
			isDepthFrameNew = true;
			// map color points to camera space
			if(coordinateMapper != NULL){
				coordinateMapper->MapColorFrameToCameraSpace(depthWidth * depthHeight, depthData, imageWidth * imageHeight, cameraSpacePoints);
				coordinateMapper->MapColorFrameToDepthSpace(depthWidth * depthHeight, depthData, imageWidth * imageHeight, depthSpacePoints);
			}
		}
		SafeRelease(depthFrame);
	}

	// read body frame
	if(bodyFrameReader){
		IBodyFrame *bodyFrame = NULL;
		HRESULT hr = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
		if(SUCCEEDED(hr)){
			IBody *pBody[BODY_COUNT] = {0};
			hr = bodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
			if(SUCCEEDED(hr)){
				// first person only
				for(int i = 0; i < BODY_COUNT; i++){
					BOOLEAN tracked = false;
					hr = pBody[i]->get_IsTracked(&tracked);
					if(SUCCEEDED(hr) && tracked){
						pBody[i]->GetJoints(JointType::JointType_Count, joints);
						isBodyFrameNew = true;
						break;
					}
				}
			}
		}
		SafeRelease(bodyFrame);
	}
}

bool Kinect2::getIsColorFrameNew(){
	return isColorFrameNew;
}

bool Kinect2::getIsDepthFrameNew(){
	return isDepthFrameNew;
}

bool Kinect2::getIsBodyFrameNew(){
	return isBodyFrameNew;
}

CameraSpacePoint Kinect2::colorPointToCameraPoint(int colorX, int colorY){
	return cameraSpacePoints[colorY * imageWidth + colorX];
}

DepthSpacePoint Kinect2::colorPointToDepthPoint(int colorX, int colorY){
	return depthSpacePoints[colorY * imageWidth + colorX];
}