#include "jaka_s5_controller/s5_joint_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "jaka_s5_interfaces/srv/jog_joint.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"

#include <QApplication>
#include <QButtonGroup>
#include <QCloseEvent>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QProgressBar>
#include <QPointer>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QStatusBar>
#include <QTableWidget>
#include <QTimer>
#include <QToolButton>
#include <QStyle>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace
{
using jaka_s5_controller::s5::kJointCount;

constexpr char kPlanningGroup[] = "jaka_s5";
constexpr auto kStateTimeout = std::chrono::milliseconds(1000);
constexpr double kCurrentStateTimeoutSeconds = 2.0;
constexpr double kS5JogMaximumVelocity = 1.57;
constexpr double kNamedPoseToleranceRadians = 0.2 * jaka_s5_controller::s5::kPi / 180.0;

class S5JointControllerWindow : public QMainWindow
{
public:
  S5JointControllerWindow(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group)
  : node_(node), move_group_(move_group)
  {
    setWindowTitle(QStringLiteral("JAKA S5 六关节控制器"));
    resize(1080, 570);
    buildUi();

    jog_client_ = node_->create_client<jaka_s5_interfaces::srv::JogJoint>(
      "/jaka_s5_controller/jog_joint");
    stop_jog_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/jaka_s5_controller/stop_jog");
    shutdown_robot_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/jaka_s5_controller/shutdown_robot");
    auto_power_off_on_exit_ =
      node_->declare_parameter<bool>("auto_power_off_on_exit", true);
    auto_move_to_initial_on_start_ =
      node_->declare_parameter<bool>("auto_move_to_initial_on_start", false);

    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::array<double, kJointCount> ordered{};
        if (!jaka_s5_controller::s5::reorderJointPositions(
            message->name, message->position, ordered))
        {
          return;
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_positions_ = ordered;
        last_state_received_ = std::chrono::steady_clock::now();
        has_valid_state_ = true;
      });

    refresh_timer_ = new QTimer(this);
    refresh_timer_->setInterval(100);
    connect(refresh_timer_, &QTimer::timeout, this, [this]() {refreshUi();});
    refresh_timer_->start();
  }

  ~S5JointControllerWindow() override
  {
    shutdown();
  }

  void shutdown()
  {
    if (shutdown_.exchange(true)) {
      return;
    }
    stop_requested_.store(true);
    if (jog_busy_.load() && rclcpp::ok() && stop_jog_client_->service_is_ready()) {
      stop_jog_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }
    if (busy_.load() && rclcpp::ok()) {
      move_group_->stop();
    }
    if (motion_thread_.joinable()) {
      motion_thread_.join();
    }
  }

protected:
  void closeEvent(QCloseEvent * event) override
  {
    if (allow_close_ || !rclcpp::ok()) {
      shutdown();
      event->accept();
      return;
    }
    event->ignore();
    requestRobotShutdown();
  }

private:
  void buildUi()
  {
    auto * central = new QWidget(this);
    auto * root = new QVBoxLayout(central);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(12);

    auto * top_row = new QHBoxLayout();
    connection_label_ = new QLabel(QStringLiteral("等待关节状态"), central);
    connection_label_->setMinimumWidth(180);
    top_row->addWidget(connection_label_);
    top_row->addStretch();

    single_mode_button_ = new QRadioButton(QStringLiteral("单关节"), central);
    all_mode_button_ = new QRadioButton(QStringLiteral("六关节"), central);
    single_mode_button_->setChecked(true);
    top_row->addWidget(single_mode_button_);
    top_row->addWidget(all_mode_button_);

    unit_combo_ = new QComboBox(central);
    unit_combo_->addItem(QStringLiteral("目标单位：度 (deg)"), false);
    unit_combo_->addItem(QStringLiteral("目标单位：弧度 (rad)"), true);
    top_row->addWidget(unit_combo_);
    root->addLayout(top_row);

    table_ = new QTableWidget(static_cast<int>(kJointCount), 6, central);
    table_->setHorizontalHeaderLabels({
      QStringLiteral("选择"), QStringLiteral("关节"), QStringLiteral("当前角度 (deg)"),
      QStringLiteral("当前弧度 (rad)"), QStringLiteral("目标"), QStringLiteral("单次点动")});
    table_->verticalHeader()->setVisible(false);
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
    table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Fixed);
    table_->horizontalHeader()->resizeSection(0, 58);
    table_->horizontalHeader()->resizeSection(1, 100);
    table_->horizontalHeader()->resizeSection(5, 140);
    table_->setSelectionMode(QAbstractItemView::NoSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setMinimumHeight(320);

    joint_selection_group_ = new QButtonGroup(this);
    joint_selection_group_->setExclusive(true);
    for (std::size_t index = 0; index < kJointCount; ++index) {
      auto * selector = new QRadioButton(table_);
      selector->setChecked(index == 0);
      joint_selection_group_->addButton(selector, static_cast<int>(index));
      table_->setCellWidget(static_cast<int>(index), 0, selector);
      table_->setItem(
        static_cast<int>(index), 1,
        new QTableWidgetItem(QString::fromStdString(jaka_s5_controller::s5::jointNames()[index])));

      degree_labels_[index] = new QLabel(QStringLiteral("--"), table_);
      radian_labels_[index] = new QLabel(QStringLiteral("--"), table_);
      degree_labels_[index]->setAlignment(Qt::AlignCenter);
      radian_labels_[index]->setAlignment(Qt::AlignCenter);
      table_->setCellWidget(static_cast<int>(index), 2, degree_labels_[index]);
      table_->setCellWidget(static_cast<int>(index), 3, radian_labels_[index]);

      target_inputs_[index] = new QDoubleSpinBox(table_);
      target_inputs_[index]->setDecimals(6);
      target_inputs_[index]->setRange(-100000.0, 100000.0);
      target_inputs_[index]->setSingleStep(1.0);
      target_inputs_[index]->setSuffix(QStringLiteral(" deg"));
      target_inputs_[index]->setAlignment(Qt::AlignRight);
      table_->setCellWidget(static_cast<int>(index), 4, target_inputs_[index]);

      auto * jog_widget = new QWidget(table_);
      jog_widget->setObjectName(QStringLiteral("jogControl"));
      jog_widget->setStyleSheet(QStringLiteral(
        "QWidget#jogControl { background: #f2f2f2; border: 1px solid #c9c9c9; "
        "border-radius: 17px; }"
        "QWidget#jogControl QToolButton { border: 0; background: transparent; }"
        "QWidget#jogControl QToolButton:hover { background: #e1e1e1; border-radius: 14px; }"
        "QWidget#jogControl QToolButton:disabled { background: transparent; }"));
      auto * jog_layout = new QHBoxLayout(jog_widget);
      jog_layout->setContentsMargins(3, 2, 3, 2);
      jog_layout->setSpacing(2);
      jog_negative_buttons_[index] = makeArrowButton(
        QStyle::SP_ArrowLeft, QStringLiteral("J%1 负向点动").arg(index + 1), jog_widget);
      auto * joint_marker = new QLabel(QStringLiteral("J%1").arg(index + 1), jog_widget);
      joint_marker->setAlignment(Qt::AlignCenter);
      joint_marker->setFixedSize(32, 32);
      joint_marker->setStyleSheet(QStringLiteral(
        "background: white; border: 3px solid #d71920; border-radius: 16px; font-weight: 600;"));
      jog_positive_buttons_[index] = makeArrowButton(
        QStyle::SP_ArrowRight, QStringLiteral("J%1 正向点动").arg(index + 1), jog_widget);
      jog_layout->addWidget(jog_negative_buttons_[index]);
      jog_layout->addWidget(joint_marker);
      jog_layout->addWidget(jog_positive_buttons_[index]);
      table_->setCellWidget(static_cast<int>(index), 5, jog_widget);
      table_->setRowHeight(static_cast<int>(index), 44);

      connect(jog_negative_buttons_[index], &QToolButton::clicked, this,
        [this, index]() {startJog(index, -1);});
      connect(jog_positive_buttons_[index], &QToolButton::clicked, this,
        [this, index]() {startJog(index, 1);});
    }
    root->addWidget(table_);

    auto * settings = new QGroupBox(QStringLiteral("运动参数"), central);
    auto * settings_layout = new QGridLayout(settings);
    velocity_slider_ = makeScalingSlider(settings);
    acceleration_slider_ = makeScalingSlider(settings);
    velocity_value_ = new QLabel(QStringLiteral("10%"), settings);
    acceleration_value_ = new QLabel(QStringLiteral("10%"), settings);
    settings_layout->addWidget(new QLabel(QStringLiteral("速度"), settings), 0, 0);
    settings_layout->addWidget(velocity_slider_, 0, 1);
    settings_layout->addWidget(velocity_value_, 0, 2);
    settings_layout->addWidget(new QLabel(QStringLiteral("加速度"), settings), 1, 0);
    settings_layout->addWidget(acceleration_slider_, 1, 1);
    settings_layout->addWidget(acceleration_value_, 1, 2);
    jog_step_input_ = new QDoubleSpinBox(settings);
    jog_step_input_->setRange(0.1, 10.0);
    jog_step_input_->setDecimals(2);
    jog_step_input_->setValue(1.0);
    jog_step_input_->setSuffix(QStringLiteral(" deg"));
    settings_layout->addWidget(new QLabel(QStringLiteral("点动单次步长"), settings), 2, 0);
    settings_layout->addWidget(jog_step_input_, 2, 1);
    settings_layout->addWidget(new QLabel(QStringLiteral("固定使用度"), settings), 2, 2);
    root->addWidget(settings);

    auto * command_row = new QHBoxLayout();
    sync_button_ = new QPushButton(QStringLiteral("同步当前姿态"), central);
    initial_pose_button_ = new QPushButton(QStringLiteral("初始位"), central);
    ready_pose_button_ = new QPushButton(QStringLiteral("预备位"), central);
    execute_button_ = new QPushButton(QStringLiteral("规划并执行"), central);
    stop_button_ = new QPushButton(QStringLiteral("停止运动"), central);
    stop_button_->setEnabled(false);
    command_row->addWidget(sync_button_);
    command_row->addWidget(initial_pose_button_);
    command_row->addWidget(ready_pose_button_);
    command_row->addStretch();
    command_row->addWidget(stop_button_);
    command_row->addWidget(execute_button_);
    root->addLayout(command_row);

    progress_ = new QProgressBar(central);
    progress_->setRange(0, 1);
    progress_->setValue(0);
    progress_->setTextVisible(false);
    progress_->setFixedHeight(5);
    root->addWidget(progress_);
    setCentralWidget(central);
    statusBar()->showMessage(QStringLiteral("等待 MoveIt 和机器人状态"));

    connect(single_mode_button_, &QRadioButton::toggled, this, [this]() {updateInputMode();});
    connect(joint_selection_group_, qOverload<int>(&QButtonGroup::idClicked), this,
      [this](int) {updateInputMode();});
    connect(unit_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this](int) {convertTargetUnits();});
    connect(sync_button_, &QPushButton::clicked, this, [this]() {syncTargets();});
    connect(initial_pose_button_, &QPushButton::clicked, this, [this]() {
      startNamedPose(
        QStringLiteral("初始位"), jaka_s5_controller::s5::initialPoseDegrees(), true);
    });
    connect(ready_pose_button_, &QPushButton::clicked, this, [this]() {
      startNamedPose(
        QStringLiteral("任务预备位"), jaka_s5_controller::s5::taskReadyPoseDegrees(), true);
    });
    connect(execute_button_, &QPushButton::clicked, this, [this]() {startMotion();});
    connect(stop_button_, &QPushButton::clicked, this, [this]() {requestStop();});
    connect(velocity_slider_, &QSlider::valueChanged, this,
      [this](int value) {velocity_value_->setText(QString::number(value) + "%");});
    connect(acceleration_slider_, &QSlider::valueChanged, this,
      [this](int value) {acceleration_value_->setText(QString::number(value) + "%");});
    updateInputMode();
  }

  QSlider * makeScalingSlider(QWidget * parent)
  {
    auto * slider = new QSlider(Qt::Horizontal, parent);
    slider->setRange(1, 100);
    slider->setValue(10);
    slider->setTickInterval(10);
    return slider;
  }

  QToolButton * makeArrowButton(
    QStyle::StandardPixmap icon, const QString & tooltip, QWidget * parent)
  {
    auto * button = new QToolButton(parent);
    button->setIcon(style()->standardIcon(icon));
    button->setToolTip(tooltip);
    button->setFixedSize(32, 30);
    button->setIconSize(QSize(18, 18));
    button->setEnabled(false);
    return button;
  }

  bool snapshotCurrent(std::array<double, kJointCount> & positions) const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_valid_state_ || !jaka_s5_controller::s5::isStateFresh(
        last_state_received_, std::chrono::steady_clock::now(), kStateTimeout))
    {
      return false;
    }
    positions = current_positions_;
    return true;
  }

  void refreshUi()
  {
    if (!rclcpp::ok()) {
      refresh_timer_->stop();
      QApplication::quit();
      return;
    }
    std::array<double, kJointCount> positions{};
    const bool connected = snapshotCurrent(positions);
    connection_label_->setText(
      connected ? QStringLiteral("关节状态：已连接") : QStringLiteral("关节状态：已超时"));
    if (connected) {
      for (std::size_t index = 0; index < kJointCount; ++index) {
        degree_labels_[index]->setText(
          QString::number(jaka_s5_controller::s5::radiansToDegrees(positions[index]), 'f', 3));
        radian_labels_[index]->setText(QString::number(positions[index], 'f', 6));
      }
    }
    const bool controls_idle =
      !busy_.load() && !jog_busy_.load() && !shutdown_busy_.load();
    execute_button_->setEnabled(connected && controls_idle);
    initial_pose_button_->setEnabled(connected && controls_idle);
    ready_pose_button_->setEnabled(connected && controls_idle);
    const bool jog_ready = connected && controls_idle && jog_client_->service_is_ready();
    for (std::size_t index = 0; index < kJointCount; ++index) {
      jog_negative_buttons_[index]->setEnabled(jog_ready);
      jog_positive_buttons_[index]->setEnabled(jog_ready);
    }
    if (jaka_s5_controller::s5::shouldStartAutomaticInitialPose(
        auto_move_to_initial_on_start_, connected, controls_idle,
        automatic_initial_attempted_))
    {
      startNamedPose(
        QStringLiteral("初始位"), jaka_s5_controller::s5::initialPoseDegrees(), false);
    }
  }

  void updateInputMode()
  {
    const bool all_joints = all_mode_button_->isChecked();
    const int selected = joint_selection_group_->checkedId();
    for (std::size_t index = 0; index < kJointCount; ++index) {
      target_inputs_[index]->setEnabled(
        !busy_.load() && !jog_busy_.load() && !shutdown_busy_.load() &&
        (all_joints || static_cast<int>(index) == selected));
      if (auto * selector = qobject_cast<QRadioButton *>(
          table_->cellWidget(static_cast<int>(index), 0)))
      {
        selector->setEnabled(
          !busy_.load() && !jog_busy_.load() && !shutdown_busy_.load() && !all_joints);
      }
    }
  }

  void convertTargetUnits()
  {
    const bool new_radians = unit_combo_->currentData().toBool();
    if (new_radians == targets_in_radians_) {
      return;
    }
    for (auto * input : target_inputs_) {
      const double converted = new_radians ?
        jaka_s5_controller::s5::degreesToRadians(input->value()) :
        jaka_s5_controller::s5::radiansToDegrees(input->value());
      input->setValue(converted);
      input->setSingleStep(new_radians ? 0.01 : 1.0);
      input->setSuffix(new_radians ? QStringLiteral(" rad") : QStringLiteral(" deg"));
    }
    targets_in_radians_ = new_radians;
  }

  void syncTargets()
  {
    std::array<double, kJointCount> positions{};
    if (!snapshotCurrent(positions)) {
      QMessageBox::warning(this, QStringLiteral("无法同步"), QStringLiteral("没有最新的六关节状态。"));
      return;
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
      target_inputs_[index]->setValue(
        targets_in_radians_ ? positions[index] :
        jaka_s5_controller::s5::radiansToDegrees(positions[index]));
    }
    statusBar()->showMessage(QStringLiteral("目标已同步为当前姿态"), 3000);
  }

  bool readTargets(
    const std::array<double, kJointCount> & current,
    std::array<double, kJointCount> & target)
  {
    const bool all_joints = all_mode_button_->isChecked();
    target = current;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (!all_joints && static_cast<int>(index) != joint_selection_group_->checkedId()) {
        continue;
      }
      if (!target_inputs_[index]->hasAcceptableInput()) {
        QMessageBox::warning(this, QStringLiteral("目标无效"), QStringLiteral("目标值为空或格式不正确。"));
        return false;
      }
      const double value = target_inputs_[index]->value();
      target[index] = targets_in_radians_ ? value : jaka_s5_controller::s5::degreesToRadians(value);
      if (!std::isfinite(target[index])) {
        QMessageBox::warning(this, QStringLiteral("目标无效"), QStringLiteral("目标值必须是有限数值。"));
        return false;
      }
    }
    return true;
  }

  bool targetWithinMoveItBounds(const std::array<double, kJointCount> & target, QString & error)
  {
    const auto model = move_group_->getRobotModel();
    const auto * group = model ? model->getJointModelGroup(kPlanningGroup) : nullptr;
    if (group == nullptr) {
      error = QStringLiteral("MoveIt 中未找到 jaka_s5 规划组。");
      return false;
    }
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    for (std::size_t index = 0; index < kJointCount; ++index) {
      state.setVariablePosition(jaka_s5_controller::s5::jointNames()[index], target[index]);
    }
    state.update();
    if (!state.satisfiesBounds(group)) {
      error = QStringLiteral("目标超过 S5 的 MoveIt 关节限位。");
      return false;
    }
    return true;
  }

  QString targetSummary(
    const std::array<double, kJointCount> & current,
    const std::array<double, kJointCount> & target) const
  {
    QString summary;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      summary += QStringLiteral("J%1: %2°  →  %3°\n")
        .arg(index + 1)
        .arg(jaka_s5_controller::s5::radiansToDegrees(current[index]), 0, 'f', 3)
        .arg(jaka_s5_controller::s5::radiansToDegrees(target[index]), 0, 'f', 3);
    }
    summary += QStringLiteral("\n速度 %1%，加速度 %2%")
      .arg(velocity_slider_->value()).arg(acceleration_slider_->value());
    return summary;
  }

  QString taskTargetSummary(
    const std::array<double, kJointCount> & current,
    const std::array<double, kJointCount> & target) const
  {
    const auto ready = jaka_s5_controller::s5::poseDegreesToRadians(
      jaka_s5_controller::s5::taskReadyPoseDegrees());
    QString summary = QStringLiteral(
      "六关节任务将先到任务预备位，再执行目标：\n\n");
    for (std::size_t index = 0; index < kJointCount; ++index) {
      summary += QStringLiteral("J%1: %2°  →  %3°  →  %4°\n")
        .arg(index + 1)
        .arg(jaka_s5_controller::s5::radiansToDegrees(current[index]), 0, 'f', 3)
        .arg(jaka_s5_controller::s5::radiansToDegrees(ready[index]), 0, 'f', 3)
        .arg(jaka_s5_controller::s5::radiansToDegrees(target[index]), 0, 'f', 3);
    }
    summary += QStringLiteral("\n速度 %1%，加速度 %2%")
      .arg(velocity_slider_->value()).arg(acceleration_slider_->value());
    return summary;
  }

  void startNamedPose(
    const QString & pose_name,
    const jaka_s5_controller::s5::JointPositions & pose_degrees,
    bool require_confirmation)
  {
    if (busy_.load() || jog_busy_.load() || shutdown_busy_.load()) {
      return;
    }
    std::array<double, kJointCount> current{};
    if (!snapshotCurrent(current)) {
      if (require_confirmation) {
        QMessageBox::warning(
          this, QStringLiteral("无法回位"), QStringLiteral("关节状态已超过 1 秒未更新。"));
      }
      return;
    }
    const auto target = jaka_s5_controller::s5::poseDegreesToRadians(pose_degrees);
    QString bounds_error;
    if (!targetWithinMoveItBounds(target, bounds_error)) {
      QMessageBox::warning(this, QStringLiteral("命名位姿无效"), bounds_error);
      return;
    }
    if (require_confirmation && QMessageBox::question(
        this, QStringLiteral("确认移动到") + pose_name, targetSummary(current, target),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Ok)
    {
      return;
    }

    if (motion_thread_.joinable()) {
      motion_thread_.join();
    }
    busy_.store(true);
    stop_requested_.store(false);
    setBusyUi(true);
    const double velocity = velocity_slider_->value() / 100.0;
    const double acceleration = acceleration_slider_->value() / 100.0;
    motion_thread_ = std::thread([this, target, velocity, acceleration, pose_name]() {
      runNamedPose(target, velocity, acceleration, pose_name);
    });
  }

  void startMotion()
  {
    std::array<double, kJointCount> current{};
    std::array<double, kJointCount> target{};
    if (!snapshotCurrent(current)) {
      QMessageBox::warning(this, QStringLiteral("无法执行"), QStringLiteral("关节状态已超过 1 秒未更新。"));
      return;
    }
    if (!readTargets(current, target)) {
      return;
    }
    QString bounds_error;
    if (!targetWithinMoveItBounds(target, bounds_error)) {
      QMessageBox::warning(this, QStringLiteral("目标无效"), bounds_error);
      return;
    }
    if (all_mode_button_->isChecked() && QMessageBox::question(
        this, QStringLiteral("确认六关节任务"), taskTargetSummary(current, target),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Ok)
    {
      return;
    }

    if (motion_thread_.joinable()) {
      motion_thread_.join();
    }
    busy_.store(true);
    stop_requested_.store(false);
    setBusyUi(true);
    const double velocity = velocity_slider_->value() / 100.0;
    const double acceleration = acceleration_slider_->value() / 100.0;
    const bool single_joint = single_mode_button_->isChecked();
    const std::size_t selected_joint = static_cast<std::size_t>(
      std::max(0, joint_selection_group_->checkedId()));
    motion_thread_ = std::thread(
      [this, target, velocity, acceleration, single_joint, selected_joint]() {
        runMotion(target, velocity, acceleration, single_joint, selected_joint);
    });
  }

  bool executePose(
    std::array<double, kJointCount> target,
    double velocity,
    double acceleration,
    const QString & stage,
    QString & error,
    int selected_joint = -1,
    bool skip_if_reached = false)
  {
    postStatus(stage + QStringLiteral("：正在获取最新状态..."));
    const auto current_state = move_group_->getCurrentState(kCurrentStateTimeoutSeconds);
    if (!current_state) {
      error = QStringLiteral("失败：") + stage + QStringLiteral("：无法获取 MoveIt 当前状态");
      return false;
    }
    if (stop_requested_.load()) {
      error = QStringLiteral("已停止");
      return false;
    }

    std::array<double, kJointCount> current{};
    for (std::size_t index = 0; index < kJointCount; ++index) {
      current[index] = current_state->getVariablePosition(
        jaka_s5_controller::s5::jointNames()[index]);
      if (selected_joint >= 0 && static_cast<int>(index) != selected_joint) {
        target[index] = current[index];
      }
    }
    if (skip_if_reached &&
      jaka_s5_controller::s5::positionsNear(current, target, kNamedPoseToleranceRadians))
    {
      postStatus(stage + QStringLiteral("：已在目标位置"));
      return true;
    }

    QString bounds_error;
    if (!targetWithinMoveItBounds(target, bounds_error)) {
      error = QStringLiteral("失败：") + stage + QStringLiteral("：") + bounds_error;
      return false;
    }
    std::map<std::string, double> targets;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      targets.emplace(jaka_s5_controller::s5::jointNames()[index], target[index]);
    }
    move_group_->setStartState(*current_state);
    move_group_->setMaxVelocityScalingFactor(velocity);
    move_group_->setMaxAccelerationScalingFactor(acceleration);
    if (!move_group_->setJointValueTarget(targets)) {
      error = QStringLiteral("失败：") + stage + QStringLiteral("：目标被 MoveIt 拒绝");
      return false;
    }

    postStatus(stage + QStringLiteral("：正在规划..."));
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planned = move_group_->plan(plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
      error = QStringLiteral("失败：") + stage + QStringLiteral("：未找到无碰撞轨迹");
      return false;
    }
    if (stop_requested_.load()) {
      error = QStringLiteral("已停止");
      return false;
    }

    postStatus(stage + QStringLiteral("：正在执行..."));
    const auto executed = move_group_->execute(plan);
    if (stop_requested_.load()) {
      error = QStringLiteral("已停止");
      return false;
    }
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      error = QStringLiteral("失败：") + stage + QStringLiteral("：轨迹执行未完成");
      return false;
    }
    return true;
  }

  void finishMotion(const QString & final_status, bool success)
  {
    QMetaObject::invokeMethod(this, [this, final_status, success]() {
      busy_.store(false);
      setBusyUi(false);
      statusBar()->showMessage(final_status);
      if (!success && final_status.startsWith(QStringLiteral("失败"))) {
        QMessageBox::warning(this, QStringLiteral("运动失败"), final_status);
      }
    }, Qt::QueuedConnection);
  }

  void runNamedPose(
    const std::array<double, kJointCount> & target,
    double velocity,
    double acceleration,
    const QString & pose_name)
  {
    bool success = false;
    QString final_status;
    try {
      success = executePose(target, velocity, acceleration, pose_name, final_status, -1, true);
      if (success) {
        final_status = pose_name + QStringLiteral("已到达");
      }
    } catch (const std::exception & exception) {
      final_status = QStringLiteral("失败：") + pose_name + QStringLiteral("：") +
        QString::fromUtf8(exception.what());
    }
    finishMotion(final_status, success);
  }

  void startJog(std::size_t joint_index, int direction)
  {
    std::array<double, kJointCount> current{};
    if (!snapshotCurrent(current)) {
      QMessageBox::warning(this, QStringLiteral("无法点动"), QStringLiteral("关节状态已超过 1 秒未更新。"));
      return;
    }
    if (!jog_client_->service_is_ready()) {
      QMessageBox::warning(this, QStringLiteral("无法点动"), QStringLiteral("JOG 服务尚未就绪。"));
      return;
    }
    if (joint_index >= kJointCount || (direction != -1 && direction != 1)) {
      return;
    }
    const double step_rad = jaka_s5_controller::s5::degreesToRadians(jog_step_input_->value());
    const auto proposed = jaka_s5_controller::s5::incrementalJointTarget(
      current, joint_index, direction, step_rad);
    QString bounds_error;
    if (!targetWithinMoveItBounds(proposed, bounds_error)) {
      QMessageBox::warning(this, QStringLiteral("点动越界"), bounds_error);
      return;
    }

    auto request = std::make_shared<jaka_s5_interfaces::srv::JogJoint::Request>();
    request->joint_index = static_cast<uint8_t>(joint_index);
    request->direction = static_cast<int8_t>(direction);
    request->speed_rad_s = kS5JogMaximumVelocity * velocity_slider_->value() / 100.0;
    request->step_rad = step_rad;
    jog_busy_.store(true);
    setBusyUi(true);
    statusBar()->showMessage(
      QStringLiteral("J%1 正在%2点动 %3°...")
      .arg(joint_index + 1).arg(direction > 0 ? QStringLiteral("正向") : QStringLiteral("负向"))
      .arg(jog_step_input_->value(), 0, 'f', 2));

    const QPointer<S5JointControllerWindow> window(this);
    jog_client_->async_send_request(request,
      [window](rclcpp::Client<jaka_s5_interfaces::srv::JogJoint>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!window) {
            return;
          }
          QMetaObject::invokeMethod(window, [window, response]() {
            if (!window) {
              return;
            }
            window->jog_busy_.store(false);
            window->setBusyUi(false);
            window->statusBar()->showMessage(QString::fromStdString(response->message));
            if (!response->success && response->error_code != 0) {
              QMessageBox::warning(
                window, QStringLiteral("点动失败"), QString::fromStdString(response->message));
            }
          }, Qt::QueuedConnection);
        } catch (const std::exception & exception) {
          if (window) {
            QMetaObject::invokeMethod(window, [window, message = QString::fromUtf8(exception.what())]() {
              if (window) {
                window->jog_busy_.store(false);
                window->setBusyUi(false);
                window->statusBar()->showMessage(QStringLiteral("点动服务失败：") + message);
              }
            }, Qt::QueuedConnection);
          }
        }
      });
  }

  void runMotion(
    std::array<double, kJointCount> target,
    double velocity,
    double acceleration,
    bool single_joint,
    std::size_t selected_joint)
  {
    bool success = false;
    QString final_status;
    try {
      if (single_joint) {
        success = executePose(
          target, velocity, acceleration, QStringLiteral("单关节运动"), final_status,
          static_cast<int>(selected_joint));
      } else {
        const auto ready = jaka_s5_controller::s5::poseDegreesToRadians(
          jaka_s5_controller::s5::taskReadyPoseDegrees());
        success = executePose(
          ready, velocity, acceleration, QStringLiteral("任务预备位"), final_status, -1, true);
        if (success) {
          success = executePose(
            target, velocity, acceleration, QStringLiteral("任务目标"), final_status);
        }
      }
      if (success) {
        final_status = single_joint ? QStringLiteral("单关节运动完成") :
          QStringLiteral("六关节任务完成");
      }
    } catch (const std::exception & exception) {
      final_status = QStringLiteral("失败：") + QString::fromUtf8(exception.what());
    }
    finishMotion(final_status, success);
  }

  void requestStop()
  {
    if (!busy_.load() && !jog_busy_.load()) {
      return;
    }
    statusBar()->showMessage(QStringLiteral("正在请求停止..."));
    if (jog_busy_.load()) {
      if (stop_jog_client_->service_is_ready()) {
        stop_jog_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      }
      return;
    }
    stop_requested_.store(true);
    move_group_->stop();
  }

  void requestRobotShutdown()
  {
    if (shutdown_busy_.exchange(true)) {
      return;
    }
    if (!shutdown_robot_client_->service_is_ready()) {
      shutdown_busy_.store(false);
      QMessageBox::critical(
        this, QStringLiteral("无法关闭"),
        QStringLiteral("机器人关机服务不可用，窗口将保持打开。"));
      return;
    }

    stop_requested_.store(true);
    if (busy_.load()) {
      move_group_->stop();
    }
    setBusyUi(true);
    stop_button_->setEnabled(false);
    statusBar()->showMessage(
      auto_power_off_on_exit_ ?
      QStringLiteral("正在停止机器人、下使能并断电...") :
      QStringLiteral("正在停止机器人并退出伺服模式..."));

    const std::size_t attempt = ++shutdown_attempt_;
    const QPointer<S5JointControllerWindow> window(this);
    shutdown_robot_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [window, attempt](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!window) {
            return;
          }
          QMetaObject::invokeMethod(window, [window, response, attempt]() {
            if (!window || attempt != window->shutdown_attempt_) {
              return;
            }
            if (response->success) {
              window->allow_close_ = true;
              window->statusBar()->showMessage(
                window->auto_power_off_on_exit_ ?
                QStringLiteral("机器人已下使能并断电") :
                QStringLiteral("机器人已停止并退出伺服模式"));
              window->close();
              return;
            }
            window->shutdown_busy_.store(false);
            window->setBusyUi(false);
            QMessageBox::critical(
              window, QStringLiteral("机器人关机失败"),
              QString::fromStdString(response->message) +
              QStringLiteral("\n窗口将保持打开，请检查机器人状态后重试。"));
          }, Qt::QueuedConnection);
        } catch (const std::exception & exception) {
          if (window) {
            QMetaObject::invokeMethod(window,
              [window, attempt, message = QString::fromUtf8(exception.what())]() {
                if (!window || attempt != window->shutdown_attempt_) {
                  return;
                }
                window->shutdown_busy_.store(false);
                window->setBusyUi(false);
                QMessageBox::critical(
                  window, QStringLiteral("机器人关机失败"), message);
              }, Qt::QueuedConnection);
          }
        }
      });

    QTimer::singleShot(10000, this, [this, attempt]() {
      if (attempt != shutdown_attempt_ || !shutdown_busy_.load() || allow_close_) {
        return;
      }
      ++shutdown_attempt_;
      shutdown_busy_.store(false);
      setBusyUi(false);
      QMessageBox::critical(
        this, QStringLiteral("机器人关机超时"),
        QStringLiteral("未收到机器人关机确认，窗口将保持打开。"));
    });
  }

  void postStatus(const QString & status)
  {
    QMetaObject::invokeMethod(this, [this, status]() {
      statusBar()->showMessage(status);
    }, Qt::QueuedConnection);
  }

  void setBusyUi(bool busy)
  {
    const bool controls_busy =
      busy_.load() || jog_busy_.load() || shutdown_busy_.load() || busy;
    execute_button_->setEnabled(!controls_busy);
    sync_button_->setEnabled(!controls_busy);
    initial_pose_button_->setEnabled(!controls_busy);
    ready_pose_button_->setEnabled(!controls_busy);
    single_mode_button_->setEnabled(!controls_busy);
    all_mode_button_->setEnabled(!controls_busy);
    unit_combo_->setEnabled(!controls_busy);
    velocity_slider_->setEnabled(!controls_busy);
    acceleration_slider_->setEnabled(!controls_busy);
    jog_step_input_->setEnabled(!controls_busy);
    for (std::size_t index = 0; index < kJointCount; ++index) {
      jog_negative_buttons_[index]->setEnabled(!controls_busy);
      jog_positive_buttons_[index]->setEnabled(!controls_busy);
    }
    stop_button_->setEnabled(controls_busy);
    progress_->setRange(controls_busy ? 0 : 0, controls_busy ? 0 : 1);
    if (!controls_busy) {
      progress_->setValue(0);
    }
    updateInputMode();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Client<jaka_s5_interfaces::srv::JogJoint>::SharedPtr jog_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_jog_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_robot_client_;
  mutable std::mutex state_mutex_;
  std::array<double, kJointCount> current_positions_{};
  std::chrono::steady_clock::time_point last_state_received_{};
  bool has_valid_state_{false};
  std::atomic<bool> busy_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> jog_busy_{false};
  std::atomic<bool> shutdown_busy_{false};
  std::atomic<bool> shutdown_{false};
  std::thread motion_thread_;

  QLabel * connection_label_{nullptr};
  QTableWidget * table_{nullptr};
  std::array<QLabel *, kJointCount> degree_labels_{};
  std::array<QLabel *, kJointCount> radian_labels_{};
  std::array<QDoubleSpinBox *, kJointCount> target_inputs_{};
  QButtonGroup * joint_selection_group_{nullptr};
  QRadioButton * single_mode_button_{nullptr};
  QRadioButton * all_mode_button_{nullptr};
  QComboBox * unit_combo_{nullptr};
  QSlider * velocity_slider_{nullptr};
  QSlider * acceleration_slider_{nullptr};
  QLabel * velocity_value_{nullptr};
  QLabel * acceleration_value_{nullptr};
  QPushButton * sync_button_{nullptr};
  QPushButton * initial_pose_button_{nullptr};
  QPushButton * ready_pose_button_{nullptr};
  QPushButton * execute_button_{nullptr};
  QPushButton * stop_button_{nullptr};
  QProgressBar * progress_{nullptr};
  QDoubleSpinBox * jog_step_input_{nullptr};
  std::array<QToolButton *, kJointCount> jog_negative_buttons_{};
  std::array<QToolButton *, kJointCount> jog_positive_buttons_{};
  QTimer * refresh_timer_{nullptr};
  bool targets_in_radians_{false};
  bool auto_power_off_on_exit_{true};
  bool auto_move_to_initial_on_start_{false};
  bool automatic_initial_attempted_{false};
  bool allow_close_{false};
  std::size_t shutdown_attempt_{0};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);
  auto node = rclcpp::Node::make_shared("s5_joint_controller");

  auto move_group =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, kPlanningGroup);
  S5JointControllerWindow window(node, move_group);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread ros_thread([&executor]() {executor.spin();});

  window.show();
  const int result = application.exec();
  window.shutdown();
  executor.cancel();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }
  rclcpp::shutdown();
  return result;
}
