from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory('vins_fusion_ros2_d435i'),
        'config',
        'project_f_d435i',
        'd435i_stereo_imu_config.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config_file,
            description='Absolute path to a VINS mono/stereo configuration file.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo /clock.'),
        DeclareLaunchArgument(
            'imu_best_effort', default_value='false',
            description='Accept a best-effort IMU publisher such as the D435i driver.'),
        DeclareLaunchArgument(
            'use_rotor_drag_dynamics_override', default_value='-1',
            description='Diagnostic override: -1 config, 0 off, 1 on.'),
        DeclareLaunchArgument(
            'use_source_neutral_motor_input', default_value='false',
            description='R4 replay-only effective motor state/quality input.'),
        DeclareLaunchArgument(
            'motor_input_mode', default_value='MOTOR_OFF',
            description='Frozen R4 motor source mode.'),
        DeclareLaunchArgument(
            'rotor_drag_dynamics_accel_noise_override',
            default_value='-1.0',
            description='Diagnostic override: -1 config, otherwise a '
                        'finite positive acceleration sigma in m/s^2.'),
        DeclareLaunchArgument(
            'use_gps_dropout_gyro_bias_prior_override',
            default_value='-1',
            description='Diagnostic override: -1 config, 0 off, 1 on.'),
        DeclareLaunchArgument(
            'gps_dropout_gyro_bias_sigma_override',
            default_value='-1.0',
            description='Diagnostic override: -1 config, otherwise a '
                        'finite positive gyro-bias sigma in rad/s.'),
        DeclareLaunchArgument(
            'use_pose_only_visual_override',
            default_value='-1',
            description='Diagnostic override: -1 config, 0 off, '
                        '1 replace feature factors, 2 augment them.'),
        DeclareLaunchArgument(
            'pose_only_visual_weight_scale_override',
            default_value='-1.0',
            description='Diagnostic override: -1 config, otherwise a '
                        'finite positive multiplier on pose-only residuals.'),
        DeclareLaunchArgument(
            'replay_reset_events', default_value='false',
            description='Apply recorded preflight reset events in-process.'),
        DeclareLaunchArgument(
            'input_downsample_phase', default_value='0',
            description='Estimator 2:1 input phase (0 live default, 1 replay alignment).'),
        DeclareLaunchArgument(
            'replay_skip_initial_images', default_value='0',
            description='Replay diagnostic only: discard this many leading camera images.'),
        DeclareLaunchArgument(
            'replay_position_jump_quarantine', default_value='false',
            description='Replay only: rollback one jump and observe three updates.'),
        DeclareLaunchArgument(
            'replay_jump_fallback_iterations', default_value='0',
            description='Replay only: re-solve a jump with this LM iteration cap.'),
        DeclareLaunchArgument(
            'replay_jump_transaction_reject', default_value='false',
            description='Replay only: reject a jump solve and keep IMU propagation.'),
        DeclareLaunchArgument(
            'replay_jump_prior_reset', default_value='false',
            description='Replay only: drop the prior and re-solve a jump.'),
        DeclareLaunchArgument(
            'replay_reject_persistent_innovation_timestamp',
            default_value='-1.0',
            description='Replay only: transaction-reject one persistent '
                        'innovation approval at this simulation timestamp.'),
        DeclareLaunchArgument(
            'replay_reject_accepted_update_timestamp',
            default_value='-1.0',
            description='Replay only: transaction-reject one accepted '
                        'non-suspect update at this simulation timestamp.'),
        DeclareLaunchArgument(
            'replay_velocity_trace_start_timestamp',
            default_value='-1.0',
            description='Replay only: first simulation timestamp for '
                        'accepted-update velocity/prior tracing.'),
        DeclareLaunchArgument(
            'replay_velocity_trace_end_timestamp',
            default_value='-1.0',
            description='Replay only: last simulation timestamp for '
                        'accepted-update velocity/prior tracing.'),
        DeclareLaunchArgument(
            'replay_backend_health_trace_start_timestamp',
            default_value='-1.0',
            description='Replay only: first simulation timestamp for '
                        'side-effect-free backend health metric tracing.'),
        DeclareLaunchArgument(
            'replay_backend_health_trace_end_timestamp',
            default_value='-1.0',
            description='Replay only: last simulation timestamp for '
                        'side-effect-free backend health metric tracing.'),
        DeclareLaunchArgument(
            'replay_dual_state_backend',
            default_value='false',
            description='Replay only: isolate nonlinear solves in a '
                        'committed/candidate dual-state backend.'),
        DeclareLaunchArgument(
            'replay_dual_state_required_healthy_updates',
            default_value='3',
            description='Replay only: consecutive healthy candidate '
                        'updates required before commit.'),
        DeclareLaunchArgument(
            'replay_multi_checkpoint_recovery',
            default_value='false',
            description='Replay only: retain synchronized backend/tracker '
                        'checkpoints and raw sensor ring.'),
        DeclareLaunchArgument(
            'replay_warm_reinitialization',
            default_value='false',
            description='Replay only: bridge an internal reset with the last '
                        'healthy metric IMU state for at most five seconds.'),
        DeclareLaunchArgument(
            'replay_motion_onset_warm_reinitialization',
            default_value='false',
            description='Replay causal A/B only: rebuild a stale window and '
                        'prior from continuous recent camera/IMU history at '
                        'the detected stationary-to-motion transition.'),
        DeclareLaunchArgument(
            'replay_prior_scale_projection',
            default_value='false',
            description='Replay only: remove the joint metric-scale direction '
                        'from every newly formed marginalization prior.'),
        DeclareLaunchArgument(
            'replay_prior_scale_projection_start_timestamp',
            default_value='-1.0',
            description='Replay causal A/B only: first simulation timestamp '
                        'eligible for prior scale projection.'),
        DeclareLaunchArgument(
            'replay_prior_scale_projection_minimum_directional_gradient',
            default_value='-1.7976931348623157e+308',
            description='Replay causal A/B only: minimum prior gradient along '
                        'the joint metric-scale direction.'),
        DeclareLaunchArgument(
            'replay_prior_scale_projection_maximum_count',
            default_value='0',
            description='Replay causal A/B only: maximum actual projections; '
                        'zero means unlimited.'),
        DeclareLaunchArgument(
            'replay_skip_marginalization_prior_timestamp',
            default_value='-1.0',
            description='Replay causal A/B only: omit the marginalization '
                        'prior from exactly one solve at this timestamp.'),
        DeclareLaunchArgument(
            'replay_include_long_imu_factors',
            default_value='false',
            description='Replay causal A/B only: keep preintegrated IMU '
                        'factors longer than the production 10 s cutoff.'),
        DeclareLaunchArgument(
            'replay_maximum_imu_window_interval',
            default_value='-1.0',
            description='Replay causal A/B only: force oldest-state '
                        'marginalization when a retained IMU interval exceeds '
                        'this many seconds.'),
        DeclareLaunchArgument(
            'replay_gt_accel_bias_zero_oracle',
            default_value='false',
            description='Replay oracle only: fix simulator accel-bias '
                        'coordinates to the known GT value zero.'),
        DeclareLaunchArgument(
            'replay_gt_initial_velocity_zero_oracle',
            default_value='false',
            description='Replay oracle only: fix initial stationary-window '
                        'velocity to known GT zero, then release it.'),
        Node(
            package='vins_fusion_ros2_d435i',
            executable='vins_fusion_ros2_node',
            name='vins_fusion_ros2_d435i_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'imu_best_effort': LaunchConfiguration('imu_best_effort')},
                        {'config_file': LaunchConfiguration('config_file')},
                        {'use_rotor_drag_dynamics_override':
                         LaunchConfiguration(
                             'use_rotor_drag_dynamics_override')},
                        {'use_source_neutral_motor_input':
                         LaunchConfiguration(
                             'use_source_neutral_motor_input')},
                        {'motor_input_mode':
                         LaunchConfiguration('motor_input_mode')},
                        {'rotor_drag_dynamics_accel_noise_override':
                         LaunchConfiguration(
                             'rotor_drag_dynamics_accel_noise_override')},
                        {'use_gps_dropout_gyro_bias_prior_override':
                         LaunchConfiguration(
                             'use_gps_dropout_gyro_bias_prior_override')},
                        {'gps_dropout_gyro_bias_sigma_override':
                         LaunchConfiguration(
                             'gps_dropout_gyro_bias_sigma_override')},
                        {'use_pose_only_visual_override':
                         LaunchConfiguration(
                             'use_pose_only_visual_override')},
                        {'pose_only_visual_weight_scale_override':
                         LaunchConfiguration(
                             'pose_only_visual_weight_scale_override')},
                        {'replay_reset_events':
                         LaunchConfiguration('replay_reset_events')},
                        {'input_downsample_phase':
                         LaunchConfiguration('input_downsample_phase')},
                        {'replay_skip_initial_images':
                         LaunchConfiguration('replay_skip_initial_images')},
                        {'replay_position_jump_quarantine':
                         LaunchConfiguration(
                             'replay_position_jump_quarantine')},
                        {'replay_jump_fallback_iterations':
                         LaunchConfiguration(
                             'replay_jump_fallback_iterations')},
                        {'replay_jump_transaction_reject':
                         LaunchConfiguration(
                             'replay_jump_transaction_reject')},
                        {'replay_jump_prior_reset':
                         LaunchConfiguration(
                             'replay_jump_prior_reset')},
                        {'replay_reject_persistent_innovation_timestamp':
                         LaunchConfiguration(
                             'replay_reject_persistent_innovation_timestamp')},
                        {'replay_reject_accepted_update_timestamp':
                         LaunchConfiguration(
                             'replay_reject_accepted_update_timestamp')},
                        {'replay_velocity_trace_start_timestamp':
                         LaunchConfiguration(
                             'replay_velocity_trace_start_timestamp')},
                        {'replay_velocity_trace_end_timestamp':
                         LaunchConfiguration(
                             'replay_velocity_trace_end_timestamp')},
                        {'replay_backend_health_trace_start_timestamp':
                         LaunchConfiguration(
                             'replay_backend_health_trace_start_timestamp')},
                        {'replay_backend_health_trace_end_timestamp':
                         LaunchConfiguration(
                             'replay_backend_health_trace_end_timestamp')},
                        {'replay_dual_state_backend':
                         LaunchConfiguration(
                             'replay_dual_state_backend')},
                        {'replay_dual_state_required_healthy_updates':
                         LaunchConfiguration(
                             'replay_dual_state_required_healthy_updates')},
                        {'replay_multi_checkpoint_recovery':
                         LaunchConfiguration(
                             'replay_multi_checkpoint_recovery')},
                        {'replay_warm_reinitialization':
                         LaunchConfiguration(
                             'replay_warm_reinitialization')},
                        {'replay_motion_onset_warm_reinitialization':
                         LaunchConfiguration(
                             'replay_motion_onset_warm_reinitialization')},
                        {'replay_prior_scale_projection':
                         LaunchConfiguration(
                             'replay_prior_scale_projection')},
                        {'replay_prior_scale_projection_start_timestamp':
                         LaunchConfiguration(
                             'replay_prior_scale_projection_start_timestamp')},
                        {'replay_prior_scale_projection_minimum_directional_gradient':
                         LaunchConfiguration(
                             'replay_prior_scale_projection_minimum_directional_gradient')},
                        {'replay_prior_scale_projection_maximum_count':
                         LaunchConfiguration(
                             'replay_prior_scale_projection_maximum_count')},
                        {'replay_skip_marginalization_prior_timestamp':
                         LaunchConfiguration(
                             'replay_skip_marginalization_prior_timestamp')},
                        {'replay_include_long_imu_factors':
                         LaunchConfiguration(
                             'replay_include_long_imu_factors')},
                        {'replay_maximum_imu_window_interval':
                         LaunchConfiguration(
                             'replay_maximum_imu_window_interval')},
                        {'replay_gt_accel_bias_zero_oracle':
                         LaunchConfiguration(
                             'replay_gt_accel_bias_zero_oracle')},
                        {'replay_gt_initial_velocity_zero_oracle':
                         LaunchConfiguration(
                             'replay_gt_initial_velocity_zero_oracle')}],
        )
    ])
