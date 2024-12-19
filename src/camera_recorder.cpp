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
  Grasshopper3Viewer() : Node("grasshopper3_viewer"), is_recording(false), stop_saving(false), record_number(0)
  {
    // パラメータの宣言と取得
    declare_parameter("mode", 2);
    declare_parameter("timeout", 1000);
    declare_parameter("frame_rate_ms", 33); // フレームレート周期（ミリ秒）を追加
    declare_parameter("save_directory", "./images"); // 保存先ディレクトリを追加

    mode = get_parameter("mode").as_int();
    timeout = get_parameter("timeout").as_int();
    frame_rate_ms = get_parameter("frame_rate_ms").as_int();
    base_save_directory = get_parameter("save_directory").as_string();

    desired_mode = static_cast<FlyCapture2::Mode>(mode);

    // 保存ディレクトリの管理
    if (!fs::exists(base_save_directory))
    {
      fs::create_directories(base_save_directory);
    }
    update_record_number();

    // FlyCapture2のカメラ初期化
    int camera_num = get_num_cameras(&busMgr);
    initialize_camera(&camera, &busMgr, camera_num, desired_mode, FlyCapture2::PIXEL_FORMAT_RAW8, timeout);

    // カメラのキャプチャを開始
    start_capture(&camera);

    // 保存スレッドを開始
    save_thread = std::thread(&Grasshopper3Viewer::save_images, this);

    // ウィンドウを表示するタイマー
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(frame_rate_ms), std::bind(&Grasshopper3Viewer::display_callback, this));

    // レコーディングスタート/ストップのサービスを追加
    record_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_recording", std::bind(&Grasshopper3Viewer::set_recording_callback, this, std::placeholders::_1, std::placeholders::_2));
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
  void update_record_number()
  {
    // 現在のディレクトリ内の最大番号を取得
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

      // フォーマットした時間にマイクロ秒を追加してディレクトリ名を作成
      std::ostringstream directory_name;
      directory_name << formatted_time << "_" << std::setw(6) << std::setfill('0') << microseconds;

      // 保存ディレクトリを作成
      current_save_directory = base_save_directory + "/" + directory_name.str();
      fs::create_directories(current_save_directory);

      RCLCPP_INFO(this->get_logger(), "New record directory created: %s", current_save_directory.c_str());
  }



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

  void initialize_camera(FlyCapture2::Camera **camera,
                         FlyCapture2::BusManager *bus_manager,
                         int camera_num,
                         FlyCapture2::Mode desired_mode,
                         FlyCapture2::PixelFormat desired_pixel_format,
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

    FlyCapture2::Format7ImageSettings image_settings;
    FlyCapture2::Format7PacketInfo packet_info;
    bool supported = false;

    FlyCapture2::Format7Info format7_info;
    format7_info.mode = desired_mode;

    error = (*camera)->GetFormat7Info(&format7_info, &supported);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      std::exit(-1);
    }

    if (supported)
    {
      image_settings.mode = desired_mode;
      image_settings.pixelFormat = desired_pixel_format;
      image_settings.offsetX = 0;
      image_settings.offsetY = 0;
      image_settings.width = format7_info.maxWidth;
      image_settings.height = format7_info.maxHeight;

      error = (*camera)->SetFormat7Configuration(&image_settings, packet_info.recommendedBytesPerPacket);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
        std::exit(-1);
      }
    }
  }

  void start_capture(FlyCapture2::Camera **camera)
  {
    FlyCapture2::Error error = (*camera)->StartCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      std::exit(-1);
    }
  }

  void display_callback()
  {
    FlyCapture2::Image raw_image;
    FlyCapture2::Error error = camera->RetrieveBuffer(&raw_image);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      return;
    }

    FlyCapture2::Image rgb_image;
    raw_image.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgb_image);

    cv::Mat image(cv::Size(rgb_image.GetCols(), rgb_image.GetRows()), CV_8UC3, rgb_image.GetData());

    // ウィンドウに表示
    cv::imshow("Grasshopper3 Viewer", image);
    cv::waitKey(1);

    // 画像をバッファに追加
    if (is_recording)
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      image_queue.push(image.clone());
      condition.notify_one();
    }
  }

  void save_images()
  {
      while (!stop_saving)
      {
          std::unique_lock<std::mutex> lock(queue_mutex);
          condition.wait(lock, [this]() { return !image_queue.empty() || stop_saving; });

          while (!image_queue.empty())
          {
              cv::Mat image = image_queue.front();
              image_queue.pop();
              lock.unlock();

              // 現在時刻を取得
              auto now = std::chrono::system_clock::now();
              auto duration = now.time_since_epoch();

              // JSTに変換
              std::time_t current_time = std::chrono::system_clock::to_time_t(now + std::chrono::hours(9));
              std::tm *local_time = std::localtime(&current_time);

              // フォーマットされた時間文字列を生成
              char formatted_time[100];
              std::strftime(formatted_time, sizeof(formatted_time), "%Y%m%d_%H%M%S", local_time);

              // マイクロ秒を追加
              auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count() % 1000000;
              std::ostringstream filename;
              filename << current_save_directory << "/frame_" << formatted_time << "_" << std::setw(6) << std::setfill('0') << microseconds << ".png";
              // 画像を保存
              cv::imwrite(filename.str(), image);

              lock.lock();
          }
      }
  }



  void set_recording_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data)
    {
      start_new_record_directory();
      is_recording = true;
      response->success = true;
      response->message = "Recording started in directory " + current_save_directory;
      RCLCPP_INFO(this->get_logger(), "Recording started: %s", current_save_directory.c_str());
    }
    else
    {
      is_recording = false;
      response->success = true;
      response->message = "Recording stopped.";
      RCLCPP_INFO(this->get_logger(), "Recording stopped.");
    }
  }

  int mode;
  int timeout;
  int frame_rate_ms;
  std::string base_save_directory;
  std::string current_save_directory;
  int record_number;

  FlyCapture2::BusManager busMgr;
  FlyCapture2::Camera *camera;
  FlyCapture2::Mode desired_mode;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_service_;

  std::queue<cv::Mat> image_queue;
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
