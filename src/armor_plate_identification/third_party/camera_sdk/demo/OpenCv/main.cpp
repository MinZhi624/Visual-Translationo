#include "CameraApi.h"

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;

unsigned char* g_pRgbBuffer;   // 处理后数据缓存区

int main()
{
    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability;
    tSdkFrameHead sFrameInfo;
    BYTE* pbyBuffer;
    int iDisplayFrames = 10000;
    int channel = 3;

    CameraSdkInit(1);

    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);
    printf("count = %d\n", iCameraCounts);

    if (iCameraCounts == 0) {
        return -1;
    }

    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    printf("state = %d\n", iStatus);

    if (iStatus != CAMERA_STATUS_SUCCESS) {
        return -1;
    }

    CameraGetCapability(hCamera, &tCapability);

    g_pRgbBuffer = (unsigned char*)malloc(
        tCapability.sResolutionRange.iHeightMax *
        tCapability.sResolutionRange.iWidthMax * 3
    );

    CameraPlay(hCamera);

    if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    } else {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }

    while (iDisplayFrames--) {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

            cv::Mat matImage(
                sFrameInfo.iHeight,
                sFrameInfo.iWidth,
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer
            );

            imshow("Opencv Demo", matImage);
            waitKey(5);

            CameraReleaseImageBuffer(hCamera, pbyBuffer);
        }
    }

    CameraUnInit(hCamera);
    free(g_pRgbBuffer);

    return 0;
}