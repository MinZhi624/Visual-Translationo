/*
用来收集点测试相机视频
*/

#include "camera_capture/CameraDriver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  // 打开相机
  ArmorCameraCapture camera;
  VideoWriter writer;
  if (!camera.open()) {
      cout << "相机打开失败,请检查相机是否连接正确！" << endl;
      return -1;
  }
  
  // 设置为灯条识别优化的低曝光模式
  if (camera.setLowExposureForLightBar(100, 80)) {
    cout << "设置成功曝光时间为100us,增益为80" << endl;
  }
  Mat frame;
  // 视频保存路径（会在当前工作目录下生成）
  string video_path = "camera_capture_output.avi";
  double fps = 100.0;
  // 等待第一帧获取成功，以确定视频尺寸
  if (!camera.read(frame)) {
      cout << "无法读取第一帧" << endl;
      return -1;
  }
  if (frame.empty()) {
      cout << "第一帧为空" << endl;
      return -1;
  }
  
  // 确保尺寸是偶数（某些编码器要求）
  cv::Size frame_size(frame.cols & ~1, frame.rows & ~1);
  if (frame_size.width != frame.cols || frame_size.height != frame.rows) {
      cout << "调整帧尺寸为偶数: " << frame_size.width << "x" << frame_size.height << endl;
      cv::resize(frame, frame, frame_size);
  }
  
  // 定义视频编码格式 (XVID编码最稳定，兼容性好)
  int fourcc = VideoWriter::fourcc('X', 'V', 'I', 'D');
  
  // 创建VideoWriter
  writer.open(video_path, fourcc, fps, frame_size, true);
  
  if (!writer.isOpened()) {
      cout << "视频写入器打开失败！" << endl;
      return -1;
  }
  
  cout << "开始录制视频..." << endl;
  cout << "保存路径: " << video_path << endl;
  cout << "分辨率: " << frame.cols << "x" << frame.rows << endl;
  cout << "帧率: " << fps << " fps" << endl;
  cout << "按 ESC 键退出并保存视频" << endl;
  // ========== end ==================
  while (true)
  {
      if (!camera.read(frame)) break;
      // =============  添加读取视频存储 ===========
      // 确保帧尺寸匹配（如果需要则调整）
      if (frame.cols != frame_size.width || frame.rows != frame_size.height) {
          cv::resize(frame, frame, frame_size);
      }
      
      // 写入当前帧到视频文件
      writer.write(frame);
      
      // 显示实时预览（可选）
      imshow("Camera Recording", frame);
      
      // 按 ESC 键退出
      if (waitKey(1) == 27) {  // 27 = ESC键
          cout << "ESC 键被按下，正在保存视频..." << endl;
          break;
      }
      // ========== end ==================
  }
  camera.release();
  
  // 释放视频写入器，确保文件正确保存
  writer.release();
  
  cout << "视频已保存至: " << video_path << endl;
  
  cv::destroyAllWindows();
  return 0;
}