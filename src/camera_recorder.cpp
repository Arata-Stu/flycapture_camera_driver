#include "rclcpp/rclcpp.hpp"
#include <FlyCapture2.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "std_srvs/srv/set_bool.hpp"  // レコーディング開始/停止サービス
#include "std_msgs/msg/string.hpp"    // トピックをString型に変更

namespace fs = std::filesystem;

//--------------------------------------
// カメラごとに管理するためのハンドラクラス
//--------------------------------------
class CameraHandler
{
public:
  // 画像データ＆タイムスタンプをまとめる構造体
  struct ImageData
  {
    cv::Mat image;
    FlyCapture2::TimeStamp timestamp;
  };

  CameraHandler(unsigned int index,
                FlyCapture2::BusManager &bus_mgr,
                const std::string &base_save_dir,
                const std::string &video_mode_str,
                const std::string &frame_rate_str,
                int grab_timeout_ms)
      : camera_index(index),
        busMgr(bus_mgr),
        base_save_directory(base_save_dir),
        video_mode_str(video_mode_str),
        frame_rate_str(frame_rate_str),
        grab_timeout_ms(grab_timeout_ms),
        camera(nullptr),
        is_recording(false),
        stop_saving(false)
  {
    // カメラ初期化＆キャプチャ開始
    initialize_camera();
    start_capture();

    // 保存用スレッド起動
    save_thread = std::thread(&CameraHandler::save_images, this);
  }

  ~CameraHandler()
  {
    // スレッド終了指示
    stop_saving = true;
    condition.notify_all();
    if (save_thread.joinable())
    {
      save_thread.join();
    }

    // カメラの停止＆切断
    if (camera)
    {
      FlyCapture2::Error error = camera->StopCapture();
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
      }

      error = camera->Disconnect();
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
      }
      delete camera;
      camera = nullptr;
    }
  }

  // 画像取得関数（タイマーや別スレッドから呼ばれる想定）
  void grab_frame(cv::Mat &frame)
  {
      FlyCapture2::Image raw_image;
      FlyCapture2::Error error = camera->RetrieveBuffer(&raw_image);
      if (error != FlyCapture2::PGRERROR_OK)
      {
          frame = cv::Mat();  // 空の画像を返す
          return;
      }

      FlyCapture2::Image bgr_image;
      raw_image.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgr_image);

      // OpenCV形式のフレームを生成
      frame = cv::Mat(cv::Size(bgr_image.GetCols(), bgr_image.GetRows()),
                      CV_8UC3, bgr_image.GetData()).clone();

      // レコーディング中ならキューに追加
      if (is_recording)
      {
          ImageData data;
          data.image = frame.clone();  // データのコピーを保存
          data.timestamp = raw_image.GetTimeStamp();
          {
              std::lock_guard<std::mutex> lock(queue_mutex);
              image_queue.push(data);
          }
          condition.notify_one();
      }
  }

  // レコーディング開始
  void start_recording(const std::string &save_dir)
  {
    // 新しい保存先ディレクトリを設定
    current_save_directory = save_dir;
    fs::create_directories(current_save_directory);

    // フラグON
    is_recording = true;
  }

  // レコーディング停止
  void stop_recording()
  {
    is_recording = false;
  }

  // is_recording の状態を取得するためのメソッド
  bool isRecording() const
  {
    return is_recording;
  }

private:
  // カメラ初期化
  void initialize_camera()
  {
    FlyCapture2::PGRGuid guid;
    FlyCapture2::Error error = busMgr.GetCameraFromIndex(camera_index, &guid);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      throw std::runtime_error("Failed to get camera from index");
    }

    camera = new FlyCapture2::Camera();
    error = camera->Connect(&guid);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      throw std::runtime_error("Failed to connect to camera");
    }

    // Grabタイムアウト設定
    FlyCapture2::FC2Config config;
    error = camera->GetConfiguration(&config);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      throw std::runtime_error("GetConfiguration failed");
    }
    config.grabTimeout = grab_timeout_ms;
    error = camera->SetConfiguration(&config);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      throw std::runtime_error("SetConfiguration failed");
    }

    // VideoMode と FrameRate の設定
    FlyCapture2::VideoMode vm = parse_video_mode(video_mode_str);
    FlyCapture2::FrameRate fr = parse_frame_rate(frame_rate_str);
    error = camera->SetVideoModeAndFrameRate(vm, fr);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      throw std::runtime_error("Failed to set video mode or frame rate");
    }
  }

  // カメラキャプチャ開始
  void start_capture()
  {
    FlyCapture2::Error error = camera->StartCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      throw std::runtime_error("Failed to start capture");
    }
  }

  // 画像保存用スレッド
  void save_images()
  {
    while (!stop_saving)
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      condition.wait(lock, [this]() {
        return !image_queue.empty() || stop_saving;
      });

      while (!image_queue.empty())
      {
        ImageData data = image_queue.front();
        image_queue.pop();
        lock.unlock();

        // タイムスタンプからファイル名を作成
        unsigned long sec = data.timestamp.seconds;
        unsigned long usec = data.timestamp.microSeconds;
        std::time_t time_sec = static_cast<std::time_t>(sec);
        std::tm *tm = std::localtime(&time_sec);

        std::ostringstream filename;
        filename << current_save_directory << "/"
                 << std::put_time(tm, "%Y%m%d_%H%M%S") << "_"
                 << std::setw(6) << std::setfill('0') << usec << ".jpg";

        // JPEGで保存
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
        cv::imwrite(filename.str(), data.image, compression_params);

        lock.lock();
      }
    }
  }

  // VideoMode 文字列 → FlyCapture2::VideoMode へ変換
  FlyCapture2::VideoMode parse_video_mode(const std::string &mode_str)
  {
    // 必要なモードが増えた場合は随時追加
    if (mode_str == "VIDEOMODE_640x480Y8") return FlyCapture2::VIDEOMODE_640x480Y8;
    if (mode_str == "VIDEOMODE_800x600YUV422") return FlyCapture2::VIDEOMODE_800x600YUV422;
    if (mode_str == "VIDEOMODE_1600x1200YUV422") return FlyCapture2::VIDEOMODE_1600x1200YUV422;
    if (mode_str == "VIDEOMODE_FORMAT7") return FlyCapture2::VIDEOMODE_FORMAT7;

    // 例外
    throw std::invalid_argument("Unsupported VideoMode: " + mode_str);
  }

  // FrameRate 文字列 → FlyCapture2::FrameRate へ変換
  FlyCapture2::FrameRate parse_frame_rate(const std::string &rate_str)
  {
    if (rate_str == "FRAMERATE_15") return FlyCapture2::FRAMERATE_15;
    if (rate_str == "FRAMERATE_30") return FlyCapture2::FRAMERATE_30;

    // 例外
    throw std::invalid_argument("Unsupported FrameRate: " + rate_str);
  }

  //----------------------------------
  // メンバ変数
  //----------------------------------
  unsigned int camera_index;
  FlyCapture2::BusManager &busMgr;
  std::string base_save_directory;
  std::string video_mode_str;
  std::string frame_rate_str;
  int grab_timeout_ms;

  FlyCapture2::Camera *camera;

  bool is_recording;  
  bool stop_saving;   
  std::string current_save_directory;

  std::queue<ImageData> image_queue;
  std::mutex queue_mutex;
  std::condition_variable condition;
  std::thread save_thread;
};

//---------------------------------------
// 複数カメラを管理するノードクラス
//---------------------------------------
class Grasshopper3Viewer : public rclcpp::Node
{
public:
  Grasshopper3Viewer()
      : Node("grasshopper3_viewer"),
        stop_all(false)
  {
    // パラメータ宣言
    this->declare_parameter<std::string>("right_camera_serial", "");
    this->declare_parameter<std::string>("left_camera_serial", "");
    this->declare_parameter<int>("timeout", 1000);
    this->declare_parameter<int>("frame_rate_ms", 33);
    this->declare_parameter<std::string>("save_directory", "./images");
    this->declare_parameter<std::string>("video_mode", "VIDEOMODE_800x600YUV422");
    this->declare_parameter<std::string>("frame_rate", "FRAMERATE_30");
    this->declare_parameter<bool>("show_window", false);

    // ディレクトリを切り替える間隔（秒）を追加
    // 例：10秒ごとにディレクトリを切り替えたい場合は10を設定
    this->declare_parameter<int>("directory_switch_interval", 10); 
    this->declare_parameter<bool>("rotate_180", false);

    // パラメータ取得
    std::string right_serial = this->get_parameter("right_camera_serial").as_string();
    std::string left_serial = this->get_parameter("left_camera_serial").as_string();
    timeout_ms = this->get_parameter("timeout").as_int();
    frame_rate_ms = this->get_parameter("frame_rate_ms").as_int();
    base_save_directory = this->get_parameter("save_directory").as_string();
    video_mode_str = this->get_parameter("video_mode").as_string();
    frame_rate_str = this->get_parameter("frame_rate").as_string();
    show_window = this->get_parameter("show_window").as_bool();

    directory_switch_interval_ = this->get_parameter("directory_switch_interval").as_int();
    rotate_180 = this->get_parameter("rotate_180").as_bool();


    // 保存先ディレクトリ作成
    if (!fs::exists(base_save_directory))
    {
      fs::create_directories(base_save_directory);
    }

    // バスからカメラ台数を取得
    unsigned int num_cameras = get_num_cameras(&busMgr);
    RCLCPP_INFO(this->get_logger(), "Detected %u cameras.", num_cameras);

    // シリアル番号でカメラを割り当て
    std::unordered_map<unsigned int, std::string> camera_roles; // カメラの役割（右/左）
    if (!right_serial.empty())
    {
        camera_roles[std::stoul(right_serial)] = "right";
    }
    if (!left_serial.empty())
    {
        camera_roles[std::stoul(left_serial)] = "left";
    }

    // カメラを初期化
    for (unsigned int i = 0; i < num_cameras; ++i)
    {
        FlyCapture2::PGRGuid guid;
        FlyCapture2::Error error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            error.PrintErrorTrace();
            continue;
        }

        FlyCapture2::Camera camera;
        error = camera.Connect(&guid);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            error.PrintErrorTrace();
            continue;
        }

        FlyCapture2::CameraInfo camera_info;
        error = camera.GetCameraInfo(&camera_info);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            error.PrintErrorTrace();
            continue;
        }

        unsigned int serial_number = camera_info.serialNumber;
        std::string role = "random";

        // シリアル番号に基づく役割の割り当て
        if (camera_roles.find(serial_number) != camera_roles.end())
        {
            role = camera_roles[serial_number];
            camera_roles.erase(serial_number);
        }

        // カメラハンドラを作成
        auto handler = std::make_shared<CameraHandler>(
            i, busMgr, base_save_directory,
            video_mode_str, frame_rate_str,
            timeout_ms);

        camera_handlers.push_back(handler);

        if (role == "right")
        {
            right_camera = handler;
            RCLCPP_INFO(this->get_logger(), "Assigned camera %u as RIGHT", serial_number);
        }
        else if (role == "left")
        {
            left_camera = handler;
            RCLCPP_INFO(this->get_logger(), "Assigned camera %u as LEFT", serial_number);
        }
        else
        {
            random_cameras.push_back(handler);
            RCLCPP_INFO(this->get_logger(), "Assigned camera %u as RANDOM", serial_number);
        }
    }

    // ランダムカメラのログ出力
    if (!random_cameras.empty())
    {
        RCLCPP_INFO(this->get_logger(), "%lu cameras assigned as RANDOM.", random_cameras.size());
    }

    // フレーム取得用のタイマー設定
    capture_timer = this->create_wall_timer(
        std::chrono::milliseconds(frame_rate_ms),
        std::bind(&Grasshopper3Viewer::capture_callback, this));

    // レコーディング状態通知用のパブリッシャ（String型）
    recording_status_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "recording_status",  // トピック名
        10                   // キューサイズ
    );

    // レコーディングを開始/停止するサービス
    record_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_recording",
        std::bind(&Grasshopper3Viewer::set_recording_callback,
                    this, std::placeholders::_1, std::placeholders::_2));

    // ここでは directory_switch_timer_ は生成しない
    // -> レコーディング開始時に生成し、レコーディング停止時に破棄する
  }

  ~Grasshopper3Viewer()
  {
    // ノード終了時
    stop_all = true;
    // もしタイマーが生きていれば破棄
    stop_directory_switch_timer();
  }

private:
  //-----------------------------------------
  // カメラ数の取得
  //-----------------------------------------
  unsigned int get_num_cameras(FlyCapture2::BusManager *bus_manager)
  {
    unsigned int cameras;
    FlyCapture2::Error error = bus_manager->GetNumOfCameras(&cameras);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      std::exit(-1);
    }

    if (cameras < 1)
    {
      RCLCPP_ERROR(this->get_logger(), "Error: At least 1 camera is required.");
      std::exit(-1);
    }

    return cameras;
  }

  //-----------------------------------------
  // フレーム取得用のタイマーコールバック
  //-----------------------------------------
  void capture_callback()
  {
      std::map<std::shared_ptr<CameraHandler>, cv::Mat> camera_to_frame;
      for (auto &handler : camera_handlers)
      {
          cv::Mat frame;
          handler->grab_frame(frame);
          camera_to_frame[handler] = frame;
      }

      std::vector<cv::Mat> frames_in_order;

      if (left_camera)
      {
          auto it = camera_to_frame.find(left_camera);
          if (it != camera_to_frame.end() && !it->second.empty())
          {
              cv::Mat processed_frame = it->second.clone();
              if (rotate_180)
              {
                  cv::rotate(processed_frame, processed_frame, cv::ROTATE_180);
              }
              frames_in_order.push_back(processed_frame);
          }
      }

      if (right_camera)
      {
          auto it = camera_to_frame.find(right_camera);
          if (it != camera_to_frame.end() && !it->second.empty())
          {
              cv::Mat processed_frame = it->second.clone();
              if (rotate_180)
              {
                  cv::rotate(processed_frame, processed_frame, cv::ROTATE_180);
              }
              frames_in_order.push_back(processed_frame);
          }
      }

      for (auto &random_cam : random_cameras)
      {
          auto it = camera_to_frame.find(random_cam);
          if (it != camera_to_frame.end() && !it->second.empty())
          {
              cv::Mat processed_frame = it->second.clone();
              if (rotate_180)
              {
                  cv::rotate(processed_frame, processed_frame, cv::ROTATE_180);
              }
              frames_in_order.push_back(processed_frame);
          }
      }

      if (!frames_in_order.empty())
      {
          cv::Mat concatenated_frame;
          cv::hconcat(frames_in_order, concatenated_frame);
          cv::imshow("Camera View", concatenated_frame);
          cv::waitKey(1);
      }
  }



  //-----------------------------------------
  // レコーディング開始/停止サービスのコールバック
  //   true -> "start"
  //   false -> "stop"
  //-----------------------------------------
  void set_recording_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
      if (request->data)
      {
          // 親ディレクトリを作成
          auto now = std::chrono::system_clock::now();
          auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
          auto now_t = std::chrono::system_clock::to_time_t(now);
          auto us_part = now_us % 1000000;

          std::stringstream ss;
          ss << base_save_directory << "/"
             << std::put_time(std::localtime(&now_t), "%Y%m%d_%H%M%S")
             << "_" << std::setw(6) << std::setfill('0') << us_part;

          std::string recording_dir = ss.str();
          fs::create_directories(recording_dir);

          // カメラごとにサブディレクトリを作成＆start_recordingを呼ぶ
          for (size_t i = 0; i < camera_handlers.size(); ++i)
          {
              std::string camera_dir;
              if (camera_handlers[i] == right_camera)
              {
                  camera_dir = recording_dir + "/right_camera";
              }
              else if (camera_handlers[i] == left_camera)
              {
                  camera_dir = recording_dir + "/left_camera";
              }
              else
              {
                  camera_dir = recording_dir + "/camera_" + std::to_string(i);
              }

              fs::create_directories(camera_dir);
              camera_handlers[i]->start_recording(camera_dir);
          }

          response->success = true;
          response->message = "Recording started in: " + recording_dir;
          RCLCPP_INFO(this->get_logger(), "Recording started in: %s", recording_dir.c_str());

          // トピックで"start"を送信
          {
            std_msgs::msg::String msg;
            msg.data = "start";
            recording_status_publisher_->publish(msg);
          }

          // ディレクトリ切り替え用タイマーをスタート
          start_directory_switch_timer();
      }
      else
      {
          // レコーディング停止
          for (auto &handler : camera_handlers)
          {
              handler->stop_recording();
          }

          response->success = true;
          response->message = "Recording stopped.";
          RCLCPP_INFO(this->get_logger(), "Recording stopped.");

          // トピックで"stop"を送信
          {
            std_msgs::msg::String msg;
            msg.data = "stop";
            recording_status_publisher_->publish(msg);
          }

          // ディレクトリ切り替え用タイマーをストップ(破棄)
          stop_directory_switch_timer();
      }
  }

  //-----------------------------------------
  // ディレクトリ切り替え用タイマーのスタート
  // （すでにタイマーが生成されている場合は再生成しない）
  //-----------------------------------------
  void start_directory_switch_timer()
  {
    if (!directory_switch_timer_)
    {
      directory_switch_timer_ = this->create_wall_timer(
        std::chrono::seconds(directory_switch_interval_),
        std::bind(&Grasshopper3Viewer::switch_directory_timer_callback, this));
    }
  }

  //-----------------------------------------
  // ディレクトリ切り替え用タイマーのストップ(破棄)
  // -> reset() することでタイマーは無効になる
  //-----------------------------------------
  void stop_directory_switch_timer()
  {
    if (directory_switch_timer_)
    {
      directory_switch_timer_.reset();
    }
  }

  //-----------------------------------------
  // ディレクトリをスイッチするためのタイマーコールバック
  // 「録画中」のカメラに対してのみディレクトリを再設定し、
  // そのタイミングで"switch"をPublishする
  //-----------------------------------------
  void switch_directory_timer_callback()
  {
    // どれか一つでも録画中のカメラがあればディレクトリ切り替え
    bool any_recording = false;
    for (auto &handler : camera_handlers)
    {
      if (handler->isRecording())
      {
        any_recording = true;
        break;
      }
    }

    // 録画中でなければ何もしない
    if (!any_recording)
    {
      return;
    }

    // 新しいディレクトリを作って再度 start_recording
    auto now = std::chrono::system_clock::now();
    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    auto now_t = std::chrono::system_clock::to_time_t(now);
    auto us_part = now_us % 1000000;

    std::stringstream ss;
    ss << base_save_directory << "/"
       << std::put_time(std::localtime(&now_t), "%Y%m%d_%H%M%S")
       << "_" << std::setw(6) << std::setfill('0') << us_part;

    std::string new_dir = ss.str();
    fs::create_directories(new_dir);

    // 各カメラについて、録画中のもののみディレクトリを切り替える
    for (size_t i = 0; i < camera_handlers.size(); ++i)
    {
      if (camera_handlers[i]->isRecording())
      {
        std::string camera_dir;
        if (camera_handlers[i] == right_camera)
        {
            camera_dir = new_dir + "/right_camera";
        }
        else if (camera_handlers[i] == left_camera)
        {
            camera_dir = new_dir + "/left_camera";
        }
        else
        {
            camera_dir = new_dir + "/camera_" + std::to_string(i);
        }

        fs::create_directories(camera_dir);
        camera_handlers[i]->start_recording(camera_dir);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Switched recording directory to: %s", new_dir.c_str());

    // "switch" をトピックにpub
    std_msgs::msg::String msg;
    msg.data = "switch";
    recording_status_publisher_->publish(msg);
  }

  //-----------------------------------------
  // メンバ変数
  //-----------------------------------------
  std::shared_ptr<CameraHandler> right_camera = nullptr;
  std::shared_ptr<CameraHandler> left_camera = nullptr;
  std::vector<std::shared_ptr<CameraHandler>> random_cameras;

  FlyCapture2::BusManager busMgr;
  std::vector<std::shared_ptr<CameraHandler>> camera_handlers;

  // フレーム取得用のタイマー
  rclcpp::TimerBase::SharedPtr capture_timer;

  // ★ ディレクトリ切り替え用のタイマー(記録中のみ有効)
  rclcpp::TimerBase::SharedPtr directory_switch_timer_;  

  // レコーディング開始/停止サービス
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_service_;

  // パブリッシャー(String型)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recording_status_publisher_;

  // パラメータ
  std::string base_save_directory;
  std::string video_mode_str;
  std::string frame_rate_str;
  int timeout_ms;
  int frame_rate_ms;
  bool show_window;

  // ディレクトリ切り替え周期(秒)
  int directory_switch_interval_;
  bool rotate_180;

  bool stop_all;
};

//---------------------------------------
// メイン関数
//---------------------------------------
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Grasshopper3Viewer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
