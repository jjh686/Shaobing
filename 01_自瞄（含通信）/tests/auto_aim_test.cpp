#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
// [USB_SP 新增 开始] 可选USB联动，未指定--usb时保留离线预览。
#include "io/gimbal/sp_link.hpp"
// [USB_SP 新增 结束]
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/runtime_stats.hpp"
#include "tools/vision_status.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/demo.yaml | yaml配置文件的路径}"
  "{camera         | false             | 使用配置中的实时相机输入}"
  "{decision-status| false             | 向本机决策发布实时跟踪状态（需要相机及USB回传）}"
  "{enemy-color    |                   | 覆盖跟踪颜色red或blue，留空使用配置；窗口按R/B切换}"
  "{headless       | false             | 不打开图形窗口，用于回归验证}"
  // [USB_SP 修改 开始] --usb显式开启收发；--usb-check只收不发且不启动相机。
  "{show-control   | false             | 显示MPC面板；是否发送由usb参数决定}"
  "{usb            | false             | 接入C板USB SP姿态与目标控制，不发送开火请求}"
  "{usb-check      | 0                 | 只接收SP状态的秒数，不启动相机或发送数据}"
  // [USB_SP 修改 结束]
  "{bullet-speed   | 22                | MPC预览使用的假定弹速(m/s)，范围10到25}"
  "{snapshot-dir   | logs/live_auto_aim | 输入图像、重投影和元数据保存目录}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";

cv::Mat draw_control_preview(
  const auto_aim::Plan & plan, bool fixed_camera, double bullet_speed, std::size_t detected,
  std::size_t tracked, const std::string & tracking_color)
{
  cv::Mat panel(450, 920, CV_8UC3, cv::Scalar(28, 28, 28));
  const cv::Scalar text(230, 230, 230), muted(170, 170, 170), accent(80, 220, 255);
  auto draw = [&](const std::string & line, int x, int y, const cv::Scalar & color,
                  double scale = 0.58) {
    cv::putText(panel, line, {x, y}, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
  };
  draw("MPC CONTROL PREVIEW", 24, 32, text, 0.75);
  draw("USB TX: OFF - this test sends NO hardware commands", 24, 62, accent);
  draw(
    fmt::format("{} | track: {} [R/B]",
                fixed_camera ? "Pose: fixed camera, no IMU feedback" : "Pose: recorded quaternion",
                tracking_color),
    24, 88, muted);
  draw(
    fmt::format("Assumed bullet speed: {:.1f} m/s | detected: {} | tracked: {}",
                bullet_speed, detected, tracked),
    24, 114, text);
  const int mode = plan.control ? (plan.fire ? 2 : 1) : 0;
  draw(
    fmt::format("Preview mode: {} | control: {} | fire request: {}", mode, plan.control,
                plan.fire),
    24, 140, accent);
  draw("Field", 24, 178, muted);
  draw("Protocol units", 290, 178, muted);
  draw("Degree conversion", 590, 178, muted);
  cv::line(panel, {24, 188}, {896, 188}, cv::Scalar(80, 80, 80));
  const char * names[] = {"yaw", "yaw_vel", "yaw_acc", "pitch", "pitch_vel", "pitch_acc"};
  const char * units[] = {"rad", "rad/s", "rad/s^2", "rad", "rad/s", "rad/s^2"};
  const char * degree_units[] = {"deg", "deg/s", "deg/s^2", "deg", "deg/s", "deg/s^2"};
  const double values[] = {plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
                           plan.pitch_acc};
  for (int i = 0; i < 6; ++i) {
    const int y = 216 + i * 32;
    draw(names[i], 24, y, text);
    draw(fmt::format("{:+.5f} {}", values[i], units[i]), 290, y, text);
    draw(fmt::format("{:+.3f} {}", values[i] * 180.0 / CV_PI, degree_units[i]), 590, y, text);
  }
  draw(plan.control ? "Computed motion targets; no measured gimbal state."
                    : "No valid control plan: zero fields are inactive, NOT a home command.",
       24, 416, muted, 0.52);
  draw("USB delivery not verified. Window keys: R=red B=blue Q=quit S=snapshot.",
       24, 440, muted, 0.48);
  return panel;
}

int main(int argc, char * argv[]) try
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");
  auto camera_mode = cli.get<bool>("camera");
  auto tracking_color = cli.get<std::string>("enemy-color");
  auto headless = cli.get<bool>("headless");
  auto show_control = cli.get<bool>("show-control");
  // [USB_SP 新增 开始]
  const auto usb_mode = cli.get<bool>("usb");
  const auto usb_check = cli.get<int>("usb-check");
  const bool decision_status_enabled =
    cli.get<bool>("decision-status") && camera_mode && usb_mode && !usb_check;
  if (usb_mode) show_control = true;
  if (usb_check < 0 || usb_check > 60 || (usb_mode && !camera_mode && !usb_check))
    throw std::invalid_argument("USB requires --camera=true, or --usb-check=1..60");
  // [USB_SP 新增 结束]
  auto preview_bullet_speed = cli.get<double>("bullet-speed");
  auto snapshot_dir = cli.get<std::string>("snapshot-dir");
  if (!cli.check()) {
    cli.printErrors();
    return 2;
  }
  if (start_index < 0 || end_index < 0 || (!camera_mode && end_index > 0 && end_index < start_index)) {
    tools::logger()->error("Invalid start/end frame indices.");
    return 2;
  }
  if (show_control && (!std::isfinite(preview_bullet_speed) || preview_bullet_speed < 10 ||
                       preview_bullet_speed > 25)) {
    tools::logger()->error("MPC preview bullet speed must be between 10 and 25 m/s.");
    return 2;
  }

  tools::Plotter plotter;
  tools::VisionStatus decision_status(decision_status_enabled);
  tools::Exiter exiter;

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video;
  std::ifstream text;
  if (!camera_mode && !usb_check) {  // [USB_SP 修改] 接收检测不加载录像
    video.open(video_path);
    text.open(text_path);
    if (!video.isOpened() || !text.is_open()) {
      tools::logger()->error("Cannot open demo inputs: video={}, timestamps={}", video_path, text_path);
      return 3;
    }
  }

  auto yaml = YAML::LoadFile(config_path);
  // [USB_SP 新增 开始] 先识别有效SP回传，之后才启动相机/模型。
  std::unique_ptr<io::SpLink> usb;
  const auto rx_timeout = yaml["usb_rx_timeout_ms"].as<int>(100);
  const auto target_timeout = yaml["usb_target_timeout_ms"].as<int>(100);
  if (usb_mode || usb_check) {
    tools::logger()->info("[USB SP] port={}, TX={}, waiting for valid 43-byte feedback",
      yaml["com_port"].as<std::string>(), usb_check ? "OFF (receive only)" : "ON after SP RX");
    usb = std::make_unique<io::SpLink>(yaml["com_port"].as<std::string>(),
                                     rx_timeout, target_timeout, !usb_check);
    const auto start = std::chrono::steady_clock::now();
    auto last_log = start - std::chrono::seconds(1);
    const auto timeout = std::chrono::seconds(usb_check ? usb_check : 8);
    while (!exiter.exit() && std::chrono::steady_clock::now() - start < timeout) {
      auto feedback = usb->feedback();
      if (!usb_check && feedback) break;
      const auto now = std::chrono::steady_clock::now();
      if (now - last_log >= std::chrono::seconds(1)) {
        auto s = usb->stats();
        tools::logger()->info("[USB SP] bytes={} RX={} TX={} crc_errors={} invalid={} error={}",
          s.rx_bytes, s.rx_frames, s.tx_frames, s.crc_errors, s.invalid_frames, s.error);
        if (feedback) tools::logger()->info("[USB SP] mode={} q(wxyz)={:.4f},{:.4f},{:.4f},{:.4f} yaw={:.4f} pitch={:.4f} rad",
          feedback->mode, feedback->q.w(), feedback->q.x(), feedback->q.y(), feedback->q.z(),
          feedback->state.yaw, feedback->state.pitch);
        last_log = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto s = usb->stats();
    // [USB_SP 修改 开始] 根据实际失败阶段报错；没打开端口时不能判断固件协议。
    const auto feedback_failure = [&](const io::SpStats & stats) -> std::string {
      const auto port = yaml["com_port"].as<std::string>();
      if (!stats.connected) {
        if (stats.error == "Permission denied" || stats.error == "Operation not permitted")
          return fmt::format("[USB_PERMISSION] 没有串口读写权限：{}。烧录/拔插后临时权限可能失效；请重新授权设备节点。尚未读取数据，不能判断固件协议。", port);
        return fmt::format("[USB_OPEN] 串口未打开或已断开：{}；系统原因：{}。检查设备路径、连接和串口占用。",
          port, stats.error.empty() ? "未知" : stats.error);
      }
      if (!stats.rx_bytes)
        return "[USB_NO_DATA] 串口已打开，但未收到任何字节。检查C板USB发送任务、IMU初始化和供电；此时不能判定协议格式。";
      if (stats.rx_frames)
        return fmt::format("[USB_STALE] 曾收到{}帧有效SP状态，但最近回传已超时。检查C板是否持续发送、IMU和USB任务。", stats.rx_frames);
      return fmt::format("[USB_PROTOCOL] 已收到{}字节，但有效SP状态为0（CRC错误{}，字段错误{}）。需要检查实际回传格式、43字节字段和校验；这不单独证明烧录失败。",
        stats.rx_bytes, stats.crc_errors, stats.invalid_frames);
    };
    // [USB_SP 修改 结束]
    if (usb_check) {
      tools::logger()->info("[USB CHECK] bytes={} RX={} TX={} crc_errors={} invalid={}",
        s.rx_bytes, s.rx_frames, s.tx_frames, s.crc_errors, s.invalid_frames);
      if (!s.rx_frames) tools::logger()->error("{}", feedback_failure(s)); // [USB_SP 修改]
      return s.rx_frames ? 0 : 2;
    }
    if (exiter.exit()) return 0;
    if (!usb->feedback()) throw std::runtime_error(feedback_failure(s)); // [USB_SP 修改]
    tools::logger()->info("[USB SP] First valid feedback received. Commands use mode0/1 only; fire is disabled.");
  }
  // [USB_SP 新增 结束]
  if (tracking_color.empty()) tracking_color = yaml["enemy_color"].as<std::string>();
  if (tracking_color != "red" && tracking_color != "blue") {
    tools::logger()->error("Tracking color must be red or blue, got: {}", tracking_color);
    return 2;
  }
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  auto model_key = yolo_name + "_model_path";
  tools::logger()->info(
    "Auto aim source={}, config={}, model={} ({}), device={}, enemy_color={}, headless={}",
    camera_mode ? "camera" : video_path, config_path, yolo_name,
    yaml[model_key].as<std::string>(), yaml["device"].as<std::string>(),
    tracking_color, headless);
  tools::logger()->info("Tracking color: {}. Focus an image window: R=red, B=blue. "
                        "Runtime switches do not change the YAML file.", tracking_color);
  if (camera_mode && !usb) {  // [USB_SP 修改] 联动模式必须使用真实姿态
    tools::logger()->warn(
      "fixed-camera posture: no IMU input, using identity quaternion. "
      "Reprojection assumes the camera is fixed; this test sends no hardware commands.");
  }
  auto_aim::YOLO yolo(config_path, !headless);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  tracker.set_enemy_color(tracking_color == "red" ? auto_aim::Color::red : auto_aim::Color::blue);
  auto aimer = std::make_unique<auto_aim::Aimer>(config_path);
  std::unique_ptr<auto_aim::Planner> preview_planner;
  double preview_decision_speed = 0, preview_high_delay = 0, preview_low_delay = 0;
  if (show_control) {
    preview_planner = std::make_unique<auto_aim::Planner>(config_path);
    preview_decision_speed = yaml["decision_speed"].as<double>();
    preview_high_delay = yaml["high_speed_delay_time"].as<double>();
    preview_low_delay = yaml["low_speed_delay_time"].as<double>();
    // [USB_SP 修改 开始] 说明实际使用的弹速来源和发送状态。
    tools::logger()->info("MPC panel enabled: USB={}, fallback/preview speed={:.1f} m/s.",
                          usb ? "SP mode0/1" : "OFF", preview_bullet_speed);
    // [USB_SP 修改 结束]
  }
  std::unique_ptr<io::Camera> camera;
  if (camera_mode) camera = std::make_unique<io::Camera>(config_path);
  auto performance_counters = [&] {
    const auto cam = camera ? camera->stats() : io::CameraStats{};
    const auto link = usb ? usb->stats() : io::SpStats{};
    return tools::RuntimeStats::Counters{
      cam.frames, cam.sdk_frame_gaps, link.rx_frames, link.tx_frames,
      link.crc_errors, link.invalid_frames};
  };
  tools::RuntimeStats performance(performance_counters(), camera_mode);

  cv::Mat control_panel;
  std::filesystem::create_directories(snapshot_dir);
  auto save_snapshot = [&](int frame, const cv::Mat & raw, const cv::Mat & reprojection,
                           const nlohmann::json & metadata) {
    const auto base = std::filesystem::path(snapshot_dir) / fmt::format("frame_{:06d}", frame);
    const auto raw_path = base.string() + "_input.png";
    const auto reprojection_path = base.string() + "_reprojection.png";
    const auto json_path = base.string() + "_metadata.json";
    if (!cv::imwrite(raw_path, raw) || !cv::imwrite(reprojection_path, reprojection)) {
      throw std::runtime_error("Cannot save snapshot images to " + snapshot_dir);
    }
    if (!control_panel.empty() && !cv::imwrite(base.string() + "_control.png", control_panel)) {
      throw std::runtime_error("Cannot save control preview to " + snapshot_dir);
    }
    std::ofstream metadata_file(json_path);
    metadata_file << metadata.dump(2) << '\n';
    if (!metadata_file) throw std::runtime_error("Cannot save snapshot metadata to " + json_path);
    tools::logger()->info("Saved input, reprojection, and metadata: {}", base.string());
  };

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();
  auto last_preview_log = t0 - std::chrono::seconds(1);
  cv::Mat last_raw, last_reprojection;
  nlohmann::json last_metadata;
  int processed_frames = 0;
  std::size_t total_detected_armors = 0;
  int frames_with_targets = 0;
  int last_frame = -1;
  int last_saved_frame = -1;

  auto_aim::Target last_target;
  io::Command last_command{};
  double last_t = -1;
  // [USB_SP 新增 开始] 姿态丢失或模式变化后清除旧跟踪结果。
  bool previous_pose_ok = false;
  uint64_t previous_epoch = 0;
  auto reset_tracker = [&] {
    const auto selected = tracking_color == "red" ? auto_aim::Color::red : auto_aim::Color::blue;
    tracker.set_enemy_color(selected == auto_aim::Color::red ? auto_aim::Color::blue : auto_aim::Color::red);
    tracker.set_enemy_color(selected);
    last_command = {}; last_t = -1;
    if (usb) usb->clear_command();
  };
  // [USB_SP 新增 结束]

  if (!camera_mode) {
    video.set(cv::CAP_PROP_POS_FRAMES, start_index);
    for (int i = 0; i < start_index; i++) {
      double t, w, x, y, z;
      if (!(text >> t >> w >> x >> y >> z)) {
        tools::logger()->error("Timestamp data ended before start frame {}.", start_index);
        return 3;
      }
    }
  }

  for (int frame_count = camera_mode ? 0 : start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;
    const auto loop_start = std::chrono::steady_clock::now();

    double t = 0, w = 1, x = 0, y = 0, z = 0;
    std::chrono::steady_clock::time_point timestamp;
    if (camera_mode) {
      camera->read(img, timestamp);
      if (img.empty()) {
        tools::logger()->error("Camera returned an empty image at frame {}.", frame_count);
        return 3;
      }
      t = tools::delta_time(timestamp, t0);
    } else {
      video.read(img);
      if (img.empty()) break;
      if (!(text >> t >> w >> x >> y >> z) || !std::isfinite(t) || !std::isfinite(w) ||
          !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        tools::logger()->error("Invalid/missing timestamp or quaternion at frame {}.", frame_count);
        return 3;
      }
      timestamp = t0 + std::chrono::microseconds(static_cast<int64_t>(t * 1e6));
    }
    const auto image_received = std::chrono::steady_clock::now();
    auto raw_input = img.clone();
    // [USB_SP 新增 开始] 用接收时间对图像插值；SP没有板端时间戳，不宣称硬件同步。
    std::optional<io::SpFeedback> board_feedback;
    bool pose_ok = true;
    double effective_speed = preview_bullet_speed;
    if (usb) {
      board_feedback = usb->feedback_at(timestamp);
      pose_ok = board_feedback.has_value();
      if (pose_ok) {
        w = board_feedback->q.w(); x = board_feedback->q.x();
        y = board_feedback->q.y(); z = board_feedback->q.z();
        const auto speed = board_feedback->state.bullet_speed;
        if (speed >= 10 && speed <= 25) effective_speed = speed;
      }
      const auto epoch = pose_ok ? board_feedback->epoch : 0;
      if (pose_ok != previous_pose_ok || epoch != previous_epoch) reset_tracker();
      previous_pose_ok = pose_ok; previous_epoch = epoch;
      if (!pose_ok) usb->clear_command();
    }
    // [USB_SP 新增 结束]

    /// 自瞄核心逻辑

    if (pose_ok) solver.set_R_gimbal2world({w, x, y, z}); // [USB_SP 修改] 不伪造在线姿态

    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);
    const auto detect_finished = std::chrono::steady_clock::now();
    const auto detected_count = armors.size();
    total_detected_armors += detected_count;
    nlohmann::json detected_armors = nlohmann::json::array();
    for (const auto & armor : armors) {
      nlohmann::json points = nlohmann::json::array();
      for (const auto & point : armor.points) points.push_back({point.x, point.y});
      detected_armors.push_back({
        {"name", auto_aim::ARMOR_NAMES.at(armor.name)},
        {"label", auto_aim::ARMOR_NAMES.at(armor.name)},
        {"color", auto_aim::COLORS.at(armor.color)},
        {"type", auto_aim::ARMOR_TYPES.at(armor.type)},
        {"confidence", armor.confidence},
        {"points", points}});
    }

    auto tracker_start = std::chrono::steady_clock::now();
    // [USB_SP 修改 开始] 无有效姿态时仅显示检测，不生成世界系目标。
    auto targets = pose_ok ? tracker.track(armors, timestamp) : std::list<auto_aim::Target>{};
    // [USB_SP 修改 结束]
    if (decision_status_enabled) {
      const auto state = pose_ok ? tracker.state() : "unavailable";
      decision_status.publish(timestamp, state == "tracking" && targets.empty() ? "unavailable" : state);
    }
    if (!targets.empty()) ++frames_with_targets;

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer->aim(targets, timestamp, usb ? effective_speed : 27, false); // [USB_SP 修改]

    if (
      !targets.empty() && aimer->debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;

    if (command.control) last_command = command;

    auto_aim::Plan preview_plan{};
    nlohmann::json preview_data;
    if (preview_planner) {
      if (!targets.empty()) {
        auto preview_target = targets.front();
        const auto delay = std::abs(preview_target.ekf_x()[7]) > preview_decision_speed
                             ? preview_high_delay : preview_low_delay;
        // Keep recorded targets on their recorded timeline; live input uses wall-clock latency.
        const auto plan_time = camera_mode ? std::chrono::steady_clock::now() : timestamp;
        preview_target.predict(
          plan_time + std::chrono::microseconds(static_cast<int64_t>(delay * 1e6)));
        preview_plan = preview_planner->plan(preview_target, effective_speed); // [USB_SP 修改]
      }
      const int mode = preview_plan.control ? (preview_plan.fire ? 2 : 1) : 0;
      preview_data = {
        {"usb_enabled", usb != nullptr}, {"source", usb ? "MPC USB SP" : "MPC computation preview"}, // [USB_SP 修改]
        {"enemy_color", tracking_color},
        {"planning_bullet_speed_mps", effective_speed}, // [USB_SP 修改] 板端越界弹速回退22
        // [USB_SP 修改 开始] 计算结果与实际发出模式分开记，禁止把fire预览当作开火。
        {"computed_mode", mode}, {"control", preview_plan.control},
        {"computed_fire_request", preview_plan.fire},
        // [USB_SP 修改 结束]
        {"yaw_rad", preview_plan.yaw}, {"yaw_vel_rad_s", preview_plan.yaw_vel},
        {"yaw_acc_rad_s2", preview_plan.yaw_acc}, {"pitch_rad", preview_plan.pitch},
        {"pitch_vel_rad_s", preview_plan.pitch_vel}, {"pitch_acc_rad_s2", preview_plan.pitch_acc}};
      control_panel = draw_control_preview(
        preview_plan, camera_mode, preview_bullet_speed, detected_count, targets.size(), tracking_color);
      // [USB_SP 新增 开始] 每帧更新目标，独立串口线程100Hz发送并按图像时间超时。
      if (usb) {
        io::VisionToGimbal tx{};
        if (board_feedback && board_feedback->mode == 1 && preview_plan.control) {
          tx.mode = 1; tx.yaw = preview_plan.yaw; tx.yaw_vel = preview_plan.yaw_vel;
          tx.yaw_acc = preview_plan.yaw_acc; tx.pitch = preview_plan.pitch;
          tx.pitch_vel = preview_plan.pitch_vel; tx.pitch_acc = preview_plan.pitch_acc;
        }
        usb->submit(tx, timestamp, board_feedback ? board_feedback->epoch : 0);
        const auto stats = usb->stats();
        control_panel(cv::Rect(0, 0, 920, 185)).setTo(cv::Scalar(28,28,28));
        control_panel(cv::Rect(0, 398, 920, 52)).setTo(cv::Scalar(28,28,28));
        auto line = [&](const std::string & text, int row) {
          cv::putText(control_panel, text, {24,row}, cv::FONT_HERSHEY_SIMPLEX, .55,
                      cv::Scalar(80,220,255), 1, cv::LINE_AA);
        };
        line("USB SP LIVE - FIRE DISABLED",32);
        line(fmt::format("RX {} | TX {} | TX mode {} | CRC errors {}",
          stats.rx_frames, stats.tx_frames, stats.last_tx_mode, stats.crc_errors),62);
        line(fmt::format("Pose: {} | board mode: {} | track: {}",
          pose_ok ? "LIVE" : "STALE / NO CONTROL", board_feedback ? int(board_feedback->mode) : -1, tracking_color),88);
        line(fmt::format("Planning speed {:.1f} m/s | detected {} | tracked {}",
          effective_speed, detected_count, targets.size()),114);
        line("Numbers below = computed targets. Actual TX mode is shown above.",140);
        line("Field                    Protocol units                 Degree conversion",178);
        if (board_feedback) line(fmt::format("Measured yaw {:+.4f} rad | pitch {:+.4f} rad",
          board_feedback->state.yaw,board_feedback->state.pitch),416);
        else line("No valid pose: control disabled, waiting for SP feedback.",416);
        line("TX counts host writes; SP has no command ACK. R/B=color Q=quit S=snapshot",440);
        preview_data["rx_frames"] = stats.rx_frames;
        preview_data["tx_frames"] = stats.tx_frames;
        preview_data["tx_mode"] = stats.last_tx_mode;
        preview_data["pose_valid"] = pose_ok;
        preview_data["board_mode"] = board_feedback ? int(board_feedback->mode) : -1;
        preview_data["reported_bullet_speed_mps"] = board_feedback ?
          nlohmann::json(board_feedback->state.bullet_speed) : nlohmann::json(nullptr);
        preview_data["bullet_speed_fallback"] = !board_feedback ||
          board_feedback->state.bullet_speed < 10 || board_feedback->state.bullet_speed > 25;
      }
      // [USB_SP 新增 结束]
      const auto now = std::chrono::steady_clock::now();
      if (now - last_preview_log >= std::chrono::milliseconds(200)) {
        tools::logger()->info(
          "[MPC USB={}] computed_mode={} control={} computed_fire_request={} | " // [USB_SP 修改]
          "yaw={:+.5f}rad vel={:+.5f}rad/s acc={:+.5f}rad/s^2 | "
          "pitch={:+.5f}rad vel={:+.5f}rad/s acc={:+.5f}rad/s^2",
          usb ? "SP" : "OFF", mode, preview_plan.control, preview_plan.fire, preview_plan.yaw, preview_plan.yaw_vel, // [USB_SP 修改]
          preview_plan.yaw_acc, preview_plan.pitch, preview_plan.pitch_vel, preview_plan.pitch_acc);
        // [USB_SP 新增 开始] 日志中的TX为成功写入USB驱动的帧数，不是板端ACK。
        if (usb) {
          const auto s = usb->stats();
          tools::logger()->info("[USB SP] RX={} TX={} TX_mode={} board_mode={} pose={} speed={:.1f}m/s error={}",
            s.rx_frames, s.tx_frames, s.last_tx_mode, board_feedback ? int(board_feedback->mode) : -1,
            pose_ok, effective_speed, s.error);
        }
        // [USB_SP 新增 结束]
        last_preview_log = now;
      }
    }
    /// 调试输出

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] detected={}, after_color_filter={}, targets={}, tracker_state={}, "
      "yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      detected_count, armors.size(), targets.size(), tracker.state(),
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    // [USB_SP 修改 开始] 联动画面显示真正发送的MPC模式，旧Aimer只用于离线预览。
    const auto command_text = usb ?
      fmt::format("USB TX mode {} | MPC yaw {:.2f} pitch {:.2f} | FIRE OFF",
        usb->stats().last_tx_mode, preview_plan.yaw * 57.3, preview_plan.pitch * 57.3) :
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
        command.pitch * 57.3, command.shoot);
    tools::draw_text(img, command_text, {10, 60}, {154, 50, 205});
    // [USB_SP 修改 结束]

    Eigen::Quaternion gimbal_q = {w, x, y, z};
    tools::draw_text(
      img,
      // [USB_SP 修改 开始] 显示同安装矩阵后的角度；断线不显示假的零度。
      usb && !pose_ok ? "gimbal yaw: unavailable" : fmt::format(
        "gimbal yaw{:.2f}", usb ? board_feedback->state.yaw * 57.3 :
          (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      // [USB_SP 修改 结束]
      {10, 90}, {255, 255, 255});
    tools::draw_text(
      img, fmt::format("detected:{} tracked:{}", detected_count, targets.size()),
      {10, 30}, {0, 255, 255});
    if (camera_mode) {
      // [USB_SP 修改 开始] 识别画面标记真实姿态状态。
      tools::draw_text(img, usb ? (pose_ok ? "USB SP posture LIVE" : "USB SP STALE - control OFF")
                               : "fixed-camera posture (no IMU)", {10, 120}, {0, 255, 255});
      // [USB_SP 修改 结束]
    }
    tools::draw_text(
      img, fmt::format("tracking color: {} [R=red B=blue]", tracking_color),
      {10, 150}, {0, 255, 255});
    tools::draw_text(img, performance.overlay(), {10, 180}, {0, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty() && pose_ok) { // [USB_SP 修改] 无姿态时不记录未求解的世界坐标
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    Eigen::Quaternion q{w, x, y, z};
    auto yaw = tools::eulers(q, 2, 1, 0)[0];
    // [USB_SP 修改 开始]
    data["gimbal_yaw"] = usb ? (board_feedback ? nlohmann::json(board_feedback->state.yaw * 57.3)
                                                 : nlohmann::json(nullptr)) : nlohmann::json(yaw * 57.3);
    // [USB_SP 修改 结束]
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;
    if (preview_planner) {
      data["preview_mode"] = preview_data["computed_mode"]; // [USB_SP 修改]
      data["preview_control"] = preview_plan.control ? 1 : 0;
      data["preview_fire_request"] = preview_plan.fire ? 1 : 0;
      data["plan_yaw"] = preview_plan.yaw;
      data["plan_yaw_vel"] = preview_plan.yaw_vel;
      data["plan_yaw_acc"] = preview_plan.yaw_acc;
      data["plan_pitch"] = preview_plan.pitch;
      data["plan_pitch_vel"] = preview_plan.pitch_vel;
      data["plan_pitch_acc"] = preview_plan.pitch_acc;
    }

    if (!targets.empty()) {
      auto target = targets.front();

      if (last_t == -1) {
        last_target = target;
        last_t = t;
      }

      std::vector<Eigen::Vector4d> armor_xyza_list;

      // 当前帧target更新后
      armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer->debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    plotter.plot(data);

    nlohmann::json metadata = {
      {"source", camera_mode ? "camera" : "video"},
      {"input", camera_mode ? yaml["camera_name"].as<std::string>() : video_path},
      {"config", config_path},
      {"model", yaml[model_key].as<std::string>()},
      {"enemy_color", tracking_color},
      {"frame", frame_count},
      {"timestamp_seconds", t},
      {"image_width", raw_input.cols},
      {"image_height", raw_input.rows},
      {"fixed_camera_posture", camera_mode && !usb}, // [USB_SP 修改]
      {"usb_pose_valid", usb && pose_ok}, // [USB_SP 新增]
      {"quaternion_wxyz", pose_ok ? nlohmann::json({w, x, y, z}) : nlohmann::json(nullptr)}, // [USB_SP 修改]
      {"detected_count", detected_count},
      {"after_color_filter_count", armors.size()},
      {"tracked_count", targets.size()},
      {"tracker_state", tracker.state()},
      {"detections", detected_armors}};
    if (preview_planner) metadata["control_preview"] = preview_data;

    ++processed_frames;
    last_frame = frame_count;
    if (camera_mode && frame_count == 0) {
      save_snapshot(frame_count, raw_input, img, metadata);
      last_saved_frame = frame_count;
    }
    if (headless && !camera_mode) {
      last_raw = raw_input;
      last_reprojection = img.clone();
      last_metadata = metadata;
    }

    const auto gui_start = std::chrono::steady_clock::now();
    if (!headless) {
      cv::Mat drawing;
      cv::resize(img, drawing, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
      cv::imshow("reprojection", drawing);
      if (!control_panel.empty()) cv::imshow(usb ? "USB SP control" : "control preview (NO USB)", control_panel); // [USB_SP 修改]
      // [USB_SP 帧率优化] 处理窗口事件，缩短每轮显示等待以提高目标更新频率。
      auto key = cv::waitKey(1);
      if (key == 'r' || key == 'R' || key == 'b' || key == 'B') {
        const std::string next_color = (key == 'r' || key == 'R') ? "red" : "blue";
        if (next_color != tracking_color) {
          if (usb) usb->clear_command(); // [USB_SP 新增] 切色立即撤销上一目标
          tracker.set_enemy_color(next_color == "red" ? auto_aim::Color::red : auto_aim::Color::blue);
          aimer = std::make_unique<auto_aim::Aimer>(config_path);
          last_command = {};
          last_t = -1;
          tracking_color = next_color;
          tools::logger()->info("Switched tracking color to {}; previous target discarded.", tracking_color);
        }
      }
      if (key == 's' || key == 'S') {
        save_snapshot(frame_count, raw_input, img, metadata);
        last_saved_frame = frame_count;
      }
      if (key == 'q' || key == 'Q') break;
    }
    const auto loop_end = std::chrono::steady_clock::now();
    auto ms = [](auto a, auto b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    if (performance.add({{
          ms(loop_start, image_received), ms(yolo_start, detect_finished),
          ms(tracker_start, aimer_start), ms(aimer_start, finish), ms(gui_start, loop_end),
          ms(loop_start, loop_end), camera_mode ? ms(timestamp, loop_end) : 0.0},
          detected_count > 0, !targets.empty(), usb && pose_ok}, loop_end))
      performance.publish(performance_counters(), tracking_color, loop_end);
  }
  performance.publish(performance_counters(), tracking_color, std::chrono::steady_clock::now());

  if (processed_frames == 0) {
    tools::logger()->error("No frames were processed; check the input and frame range.");
    return 3;
  }
  if (headless && !camera_mode && last_saved_frame != last_frame) {
    save_snapshot(last_frame, last_raw, last_reprojection, last_metadata);
  }
  tools::logger()->info(
    "Summary: processed_frames={}, total_detected_armors={}, frames_with_targets={}, source={}",
    processed_frames, total_detected_armors, frames_with_targets, camera_mode ? "camera" : video_path);
  return 0;
}
catch (const std::exception & error)
{
  tools::logger()->error("Auto aim test failed: {}", error.what());
  return 1;
}
