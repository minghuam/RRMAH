/**
 * http://www.buildinsider.net/small/kinectv2cpp/02
 */


#pragma once
#include <Windows.h>
#include <Kinect.h>

class Kinect2
{
private:
	BYTE *imageData;
	UINT16 *depthData;
	IKinectSensor *kinectSensor;
	IColorFrameReader *colorFrameReader;
	IDepthFrameReader *depthFrameReader;
	IBodyFrameReader *bodyFrameReader;
	ICoordinateMapper *coordinateMapper;
	CameraSpacePoint *cameraSpacePoints;
	DepthSpacePoint *depthSpacePoints;
	
	bool isColorFrameNew;
	bool isDepthFrameNew;
	bool isBodyFrameNew;
					
	Joint joints[JointType::JointType_Count];

public:
	Kinect2(void);
	~Kinect2(void);

	const int imageWidth;
	const int imageHeight;
	const int depthWidth;
	const int depthHeight;

	BYTE *getImageData();
	UINT16 *getDepthData();

	int open();
	void update();
	bool getIsColorFrameNew();
	bool getIsDepthFrameNew();
	bool getIsBodyFrameNew();
	int getNumOfJoints();
	Joint *getJoints();
	ICoordinateMapper *getCoordinateMapper();

	CameraSpacePoint colorPointToCameraPoint(int colorX, int colorY);
	DepthSpacePoint colorPointToDepthPoint(int colorX, int colorY);
};

