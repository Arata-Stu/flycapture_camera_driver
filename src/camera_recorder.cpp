#include "rclcpp/rclcpp.hpp"
#include <FlyCapture2.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "std_srvs/srv/set_bool.hpp"

namespace fs = std::filesystem;

class Grasshopper3Viewer : public rclcpp::Node
{
public:
  Grasshopper3Viewer()
    : Node("grasshopper3_viewer"),
      is_recording(false),
      stop_saving(false)
  {
    // パラメータの宣言と取得
    declare_parameter("timeout", 1000);
    declare_parameter("frame_rate_ms", 33);            // フレーム周期 (ms)
    declare_parameter("save_directory", "./images");   // 保存先ディレクトリ
    // ★★★ VideoMode と FrameRate を文字列で宣言 ★★★
    declare_parameter("video_mode", "VIDEOMODE_800x600YUV422");
    declare_parameter("frame_rate", "FRAMERATE_30");

    timeout           = get_parameter("timeout").as_int();
    frame_rate_ms     = get_parameter("frame_rate_ms").as_int();
    base_save_directory = get_parameter("save_directory").as_string();
    video_mode_str    = get_parameter("video_mode").as_string();
    frame_rate_str    = get_parameter("frame_rate").as_string();

    // 保存ディレクトリの管理
    if (!fs::exists(base_save_directory))
    {
      fs::create_directories(base_save_directory);
    }

    // FlyCapture2 のカメラ初期化
    int camera_num = get_num_cameras(&busMgr);
    initialize_camera(&camera, &busMgr, camera_num, timeout);

    // カメラのキャプチャを開始
    start_capture(&camera);

    // 保存スレッドを開始
    save_thread = std::thread(&Grasshopper3Viewer::save_images, this);

    // ウィンドウ表示用のタイマー
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(frame_rate_ms),
      std::bind(&Grasshopper3Viewer::display_callback, this)
    );

    // レコーディングスタート/ストップ用のサービス
    record_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_recording",
        std::bind(&Grasshopper3Viewer::set_recording_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

  ~Grasshopper3Viewer()
  {
    stop_saving = true;
    condition.notify_all();
    if (save_thread.joinable())
    {
      save_thread.join();
    }
  }

private:

  // ◆◆◆ 画像とカメラタイムスタンプをペアで保持する構造体 ◆◆◆
  struct ImageData
  {
    cv::Mat image;
    FlyCapture2::TimeStamp timestamp;
  };

  // カメラ数の取得
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
      RCLCPP_ERROR(this->get_logger(), "Error: This program requires at least 1 camera.");
      std::exit(-1);
    }
    return cameras;
  }

  FlyCapture2::VideoMode parse_video_mode(const std::string& mode_str) {
      if (mode_str == "VIDEOMODE_160x120YUV444") return FlyCapture2::VIDEOMODE_160x120YUV444;
      if (mode_str == "VIDEOMODE_320x240YUV422") return FlyCapture2::VIDEOMODE_320x240YUV422;
      if (mode_str == "VIDEOMODE_640x480YUV411") return FlyCapture2::VIDEOMODE_640x480YUV411;
      if (mode_str == "VIDEOMODE_640x480YUV422") return FlyCapture2::VIDEOMODE_640x480YUV422;
      if (mode_str == "VIDEOMODE_640x480RGB") return FlyCapture2::VIDEOMODE_640x480RGB;
      if (mode_str == "VIDEOMODE_640x480Y8") return FlyCapture2::VIDEOMODE_640x480Y8;
      if (mode_str == "VIDEOMODE_640x480Y16") return FlyCapture2::VIDEOMODE_640x480Y16;
      if (mode_str == "VIDEOMODE_800x600YUV422") return FlyCapture2::VIDEOMODE_800x600YUV422;
      if (mode_str == "VIDEOMODE_800x600RGB") return FlyCapture2::VIDEOMODE_800x600RGB;
      if (mode_str == "VIDEOMODE_800x600Y8") return FlyCapture2::VIDEOMODE_800x600Y8;
      if (mode_str == "VIDEOMODE_800x600Y16") return FlyCapture2::VIDEOMODE_800x600Y16;
      if (mode_str == "VIDEOMODE_1024x768YUV422") return FlyCapture2::VIDEOMODE_1024x768YUV422;
      if (mode_str == "VIDEOMODE_1024x768RGB") return FlyCapture2::VIDEOMODE_1024x768RGB;
      if (mode_str == "VIDEOMODE_1024x768Y8") return FlyCapture2::VIDEOMODE_1024x768Y8;
      if (mode_str == "VIDEOMODE_1024x768Y16") return FlyCapture2::VIDEOMODE_1024x768Y16;
      if (mode_str == "VIDEOMODE_1280x960YUV422") return FlyCapture2::VIDEOMODE_1280x960YUV422;
      if (mode_str == "VIDEOMODE_1280x960RGB") return FlyCapture2::VIDEOMODE_1280x960RGB;
      if (mode_str == "VIDEOMODE_1280x960Y8") return FlyCapture2::VIDEOMODE_1280x960Y8;
      if (mode_str == "VIDEOMODE_1280x960Y16") return FlyCapture2::VIDEOMODE_1280x960Y16;
      if (mode_str == "VIDEOMODE_1600x1200YUV422") return FlyCapture2::VIDEOMODE_1600x1200YUV422;
      if (mode_str == "VIDEOMODE_1600x1200RGB") return FlyCapture2::VIDEOMODE_1600x1200RGB;
      if (mode_str == "VIDEOMODE_1600x1200Y8") return FlyCapture2::VIDEOMODE_1600x1200Y8;
      if (mode_str == "VIDEOMODE_1600x1200Y16") return FlyCapture2::VIDEOMODE_1600x1200Y16;
      if (mode_str == "VIDEOMODE_FORMAT7") return FlyCapture2::VIDEOMODE_FORMAT7;

      throw std::invalid_argument("Unsupported VideoMode: " + mode_str);
  }

  FlyCapture2::FrameRate parse_frame_rate(const std::string& rate_str) {
      if (rate_str == "FRAMERATE_1_875") return FlyCapture2::FRAMERATE_1_875;
      if (rate_str == "FRAMERATE_3_75") return FlyCapture2::FRAMERATE_3_75;
      if (rate_str == "FRAMERATE_7_5") return FlyCapture2::FRAMERATE_7_5;
      if (rate_str == "FRAMERATE_15") return FlyCapture2::FRAMERATE_15;
      if (rate_str == "FRAMERATE_30") return FlyCapture2::FRAMERATE_30;
      if (rate_str == "FRAMERATE_60") return FlyCapture2::FRAMERATE_60;
      if (rate_str == "FRAMERATE_120") return FlyCapture2::FRAMERATE_120;
      if (rate_str == "FRAMERATE_240") return FlyCapture2::FRAMERATE_240;

      throw std::invalid_argument("Unsupported FrameRate: " + rate_str);
  }


  // カメラ初期化
  void initialize_camera(FlyCapture2::Camera **camera,
                       FlyCapture2::BusManager *bus_manager,
                       int camera_num,
                       int timeout_ms)
  {
      FlyCapture2::PGRGuid guid;
      FlyCapture2::Error error = bus_manager->GetCameraFromIndex(0, &guid);
      if (error != FlyCapture2::PGRERROR_OK)
      {
          error.PrintErrorTrace();
          std::exit(-1);
      }

      *camera = new FlyCapture2::Camera();

      error = (*camera)->Connect(&guid);
      if (error != FlyCapture2::PGRERROR_OK)
      {
          error.PrintErrorTrace();
          std::exit(-1);
      }

      try
      {
          // VideoMode と FrameRate を設定
          FlyCapture2::VideoMode video_mode = parse_video_mode(video_mode_str);
          FlyCapture2::FrameRate frame_rate = parse_frame_rate(frame_rate_str);

          error = (*camera)->SetVideoModeAndFrameRate(video_mode, frame_rate);
          if (error != FlyCapture2::PGRERROR_OK)
          {
              error.PrintErrorTrace();
              std::exit(-1);
          }
      }
      catch (const std::exception& ex)
      {
          RCLCPP_ERROR(this->get_logger(), "Error setting VideoMode/FrameRate: %s", ex.what());
          std::exit(-1);
      }

      // タイムアウト設定
      FlyCapture2::FC2Config config;
      error = (*camera)->GetConfiguration(&config);
      if (error != FlyCapture2::PGRERROR_OK)
      {
          error.PrintErrorTrace();
          std::exit(-1);
      }
      config.grabTimeout = timeout_ms;
      error = (*camera)->SetConfiguration(&config);
      if (error != FlyCapture2::PGRERROR_OK)
      {
          error.PrintErrorTrace();
          std::exit(-1);
      }
  }


  // カメラのキャプチャ開始
  void start_capture(FlyCapture2::Camera **camera)
  {
    FlyCapture2::Error error = (*camera)->StartCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      std::exit(-1);
    }
  }

  // 画像取得と表示用コールバック
  void display_callback()
  {
    FlyCapture2::Image raw_image;
    FlyCapture2::Error error = camera->RetrieveBuffer(&raw_image);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      return;
    }

    // カメラタイムスタンプの取得
    FlyCapture2::TimeStamp cam_timestamp = raw_image.GetTimeStamp();

    // カメラ画像をBGRへ変換
    FlyCapture2::Image rgb_image;
    raw_image.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgb_image);

    cv::Mat image(cv::Size(rgb_image.GetCols(), rgb_image.GetRows()), CV_8UC3, rgb_image.GetData());

    // ウィンドウ表示
    cv::imshow("Grasshopper3 Viewer", image);
    cv::waitKey(1);

    // レコーディング中なら保存用キューへ
    if (is_recording)
    {
      ImageData data;
      data.image = image.clone();
      data.timestamp = cam_timestamp;

      std::lock_guard<std::mutex> lock(queue_mutex);
      image_queue.push(data);
      condition.notify_one();
    }
  }

  // 画像保存用スレッド
  void save_images()
  {
    while (!stop_saving)
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      condition.wait(lock, [this]() { return !image_queue.empty() || stop_saving; });

      while (!image_queue.empty())
      {
        ImageData data = image_queue.front();
        image_queue.pop();
        lock.unlock();

        // タイムスタンプ
        unsigned long sec = data.timestamp.seconds;
        unsigned long usec = data.timestamp.microSeconds;

        // ファイル名: frame_sec_usec.jpg
        std::ostringstream filename;
        filename << current_save_directory << "/frame_"
                 << sec << "_"
                 << std::setw(6) << std::setfill('0') << usec
                 << ".jpg";

        // JPEG で保存
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
        cv::imwrite(filename.str(), data.image, compression_params);

        lock.lock();
      }
    }
  }

  // サービスコールバック（レコーディングの開始／停止）
  void set_recording_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data)
    {
      // レコーディング開始
      is_recording = true;

      // ★★★ 新しいタイムスタンプ付きフォルダを作成 ★★★
      // 例: base_save_directory/20241229_153045 など
      auto now      = std::chrono::system_clock::now();
      auto now_t    = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << base_save_directory << "/" << std::put_time(std::localtime(&now_t), "%Y%m%d_%H%M%S");
      current_save_directory = ss.str();

      // ディレクトリ作成
      fs::create_directories(current_save_directory);

      response->success = true;
      response->message = "Recording started in directory " + current_save_directory;
      RCLCPP_INFO(this->get_logger(), "Recording started: %s", current_save_directory.c_str());
    }
    else
    {
      // レコーディング停止
      is_recording = false;
      response->success = true;
      response->message = "Recording stopped.";
      RCLCPP_INFO(this->get_logger(), "Recording stopped.");
    }
  }

  int timeout;
  int frame_rate_ms;

  std::string video_mode_str;
  std::string frame_rate_str;

  std::string current_save_directory;   
  std::string base_save_directory;

  FlyCapture2::BusManager busMgr;
  FlyCapture2::Camera *camera;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_service_;

  std::queue<ImageData> image_queue;
  std::mutex queue_mutex;
  std::condition_variable condition;
  std::thread save_thread;

  bool is_recording;
  bool stop_saving;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Grasshopper3Viewer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
