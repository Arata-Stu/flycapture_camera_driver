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
      stop_saving(false),
      record_number(0)
  {
    // パラメータの宣言と取得
    declare_parameter("timeout", 1000);
    declare_parameter("frame_rate_ms", 33);           // フレームレート周期（ミリ秒）
    declare_parameter("save_directory", "./images");  // 保存先ディレクトリ

    timeout = get_parameter("timeout").as_int();
    frame_rate_ms = get_parameter("frame_rate_ms").as_int();
    base_save_directory = get_parameter("save_directory").as_string();

    // 保存ディレクトリの管理
    if (!fs::exists(base_save_directory))
    {
      fs::create_directories(base_save_directory);
    }
    update_record_number();

    // FlyCapture2のカメラ初期化
    int camera_num = get_num_cameras(&busMgr);
    initialize_camera(&camera, &busMgr, camera_num, timeout);

    // カメラのキャプチャを開始
    start_capture(&camera);

    // 保存スレッドを開始
    save_thread = std::thread(&Grasshopper3Viewer::save_images, this);

    // ウィンドウを表示するタイマー
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

  // ディレクトリ番号の更新
  void update_record_number()
  {
    record_number = 0;
    for (const auto &entry : fs::directory_iterator(base_save_directory))
    {
      if (entry.is_directory() && entry.path().filename().string().find("record_") == 0)
      {
        try
        {
          int current_number = std::stoi(entry.path().filename().string().substr(7));
          record_number = std::max(record_number, current_number);
        }
        catch (...) {}
      }
    }
  }

  // 新しいレコード用ディレクトリの作成
  void start_new_record_directory()
  {
      // 現在時刻を取得
      auto now = std::chrono::system_clock::now();
      auto duration = now.time_since_epoch();
      auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
      auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count() % 1000000;

      // JSTに変換
      std::time_t current_time = std::chrono::system_clock::to_time_t(now + std::chrono::hours(9));
      std::tm *local_time = std::localtime(&current_time);

      // フォーマットされた時間文字列を生成
      char formatted_time[100];
      std::strftime(formatted_time, sizeof(formatted_time), "%Y%m%d_%H%M%S", local_time);

      // ディレクトリ名: YYYYMMDD_HHMMSS_マイクロ秒
      std::ostringstream directory_name;
      directory_name << formatted_time << "_"
                     << std::setw(6) << std::setfill('0') << microseconds;

      // 保存ディレクトリを作成
      current_save_directory = base_save_directory + "/" + directory_name.str();
      fs::create_directories(current_save_directory);

      RCLCPP_INFO(this->get_logger(), "New record directory created: %s", current_save_directory.c_str());
  }

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
      std::cerr << "Error: This program requires at least 1 camera." << std::endl;
      std::exit(-1);
    }
    return cameras;
  }

  // ◆◆◆ カメラの初期化 (VideoMode指定バージョン) ◆◆◆
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

    // ◆◆◆ ここで VideoMode と FrameRate を設定 ◆◆◆
    error = (*camera)->SetVideoModeAndFrameRate(
              FlyCapture2::VIDEOMODE_800x600YUV422,
              FlyCapture2::FRAMERATE_30
            );
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      std::exit(-1);
    }

    // タイムアウトを設定（イメージ取得時の待ち時間）
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

    // カメラタイムスタンプの取得（秒・マイクロ秒）
    FlyCapture2::TimeStamp cam_timestamp = raw_image.GetTimeStamp();

    // カメラ画像をBGRへ変換しOpenCVマットへ
    FlyCapture2::Image rgb_image;
    raw_image.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgb_image);

    cv::Mat image(cv::Size(rgb_image.GetCols(), rgb_image.GetRows()), CV_8UC3, rgb_image.GetData());

    // ウィンドウに表示
    cv::imshow("Grasshopper3 Viewer", image);
    cv::waitKey(1);

    // レコーディング中であれば保存用キューへ詰める
    if (is_recording)
    {
      ImageData data;
      data.image = image.clone();         // コピー
      data.timestamp = cam_timestamp;     // カメラから取得したタイムスタンプ

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

        // カメラのタイムスタンプ (sec, usec)
        unsigned long sec = data.timestamp.seconds;
        unsigned long usec = data.timestamp.microSeconds;

        // ファイル名: frame_sec_usec.jpg
        std::ostringstream filename;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
        filename << current_save_directory << "/frame_"
                 << sec << "_"
                 << std::setw(6) << std::setfill('0') << usec
                 << ".jpg";

        // 画像を保存
        cv::imwrite(filename.str(), data.image, compression_params);

        lock.lock();
      }
    }
  }

  // サービスコールバック（レコーディングの開始／停止）
  void set_recording_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data)
    {
      // 新しいディレクトリを作成してレコード開始
      start_new_record_directory();
      is_recording = true;
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
  std::string base_save_directory;
  std::string current_save_directory;
  int record_number;

  FlyCapture2::BusManager busMgr;
  FlyCapture2::Camera *camera; // VideoMode用なので、Format7は削除
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
