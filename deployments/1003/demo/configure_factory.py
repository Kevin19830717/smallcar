"""Derive the demo config from the unmodified vendor mini_4wd preset.

Run on the Ubuntu VM (PyYAML is supplied with ROS). Factory mode preserves the
vendor motion parameters. Precision mode records explicit experimental changes;
it is not an assertion that real physical accuracy has already been validated.
"""
from pathlib import Path
import copy
import hashlib
import json
import yaml
import argparse

parser=argparse.ArgumentParser()
parser.add_argument('--profile',choices=('factory','precision'),default='factory')
parser.add_argument('--controller',choices=('mppi','dwb'))
args=parser.parse_args()
args.controller=args.controller or ('dwb' if args.profile=='precision' else 'mppi')
if args.profile=='factory' and args.controller!='mppi':
    raise ValueError('factory profile uses its original MPPI preset')

here = Path(__file__).resolve().parent
source = here/'assets/factory_param_mini_4wd.yaml'
if hashlib.sha256(source.read_bytes()).hexdigest() != '4d326c7ed83b25db3abf00773abcc607320695ad200fca2a99263f1029140f7d':
    raise RuntimeError('厂家原始配置校验失败，请从指定源码压缩包重新提取')
original = yaml.safe_load(source.read_text())
config = copy.deepcopy(original)
# The operator must supply and verify the real starting pose.
config['amcl']['ros__parameters']['set_initial_pose'] = False
# This saved map covers only part of the lab. Do not take shortcuts in unknowns.
config['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = False
# The platform uses odom_combined, while the behavior plugin defaults to odom.
config['behavior_server']={'ros__parameters':{
    'use_sim_time':False,'global_frame':'odom_combined',
    'robot_base_frame':'base_footprint','transform_tolerance':.2}}
# The user explicitly requested avoiding the troublesome western threshold.
for name in ('local_costmap','global_costmap'):
    params = config[name][name]['ros__parameters']
    params['filters'] = ['threshold_keepout']
    params['threshold_keepout'] = {'plugin':'nav2_costmap_2d::KeepoutFilter',
                                 'enabled':True,'filter_info_topic':'/threshold_filter_info'}
if args.profile == 'precision':
    # Do not silently substitute a nearby goal when the requested cell is blocked.
    config['planner_server']['ros__parameters']['GridBased']['tolerance']=0.
    controller=config['controller_server']['ros__parameters']
    controller['goal_checker_plugins']=['goal_checker']
    controller['goal_checker'].update(plugin='nav2_controller::SimpleGoalChecker',
        xy_goal_tolerance=.03,yaw_goal_tolerance=.05235987756,stateful=False)
    # Preserve the native 20 Hz / 0.05 s model and 2.8 s prediction horizon.
    # Reduce candidate count to address the measured missed control deadlines.
    controller['FollowPath'].update(batch_size=500,vx_max=.20,vx_min=-.10)
    # Reach the position first; the vendor shim handles final goal orientation.
    controller['FollowPath']['GoalAngleCritic']['enabled']=False
    controller['FollowPath'].update(rotate_to_goal_heading=True,closed_loop=False,
        rotate_to_heading_angular_vel=.35,angular_disengage_threshold=.35)
    if args.controller == 'dwb':
        source_dwb=here/'assets/factory_param_mini_4wd_DWB.yaml'
        if hashlib.sha256(source_dwb.read_bytes()).hexdigest()!='ab0ffb3749808aa869a75eb9731e93dbcc9aee3b9197f9cd2602d13cc7fe310f':
            raise RuntimeError('厂家 DWB 配置校验失败')
        controller['FollowPath']=copy.deepcopy(yaml.safe_load(source_dwb.read_text())['controller_server']['ros__parameters']['FollowPath'])
        controller['FollowPath'].update(min_vel_x=-.06,max_vel_x=.20,max_speed_xy=.20,max_vel_theta=.22,
            min_speed_xy=.01,min_speed_theta=.10,acc_lim_x=.25,decel_lim_x=-.25,
            acc_lim_theta=.25,decel_lim_theta=-.25,
            xy_goal_tolerance=.03,trans_stopped_velocity=.02,
            vx_samples=20,vtheta_samples=40)
        controller['goal_checker'].update(plugin='nav2_controller::SimpleGoalChecker',
            xy_goal_tolerance=.025,yaw_goal_tolerance=.05235987756,stateful=True,
            trans_stopped_velocity=.015,rot_stopped_velocity=.03)
        controller['odom_topic']='/odom'
        controller['FollowPath']['decel_lim_theta']=-.25
        controller['FollowPath']['xy_goal_tolerance']=.025
        controller['FollowPath']['RotateToGoal.lookahead_time']=1.2
        # Waypoints have two separate native controller tasks. Transit/return
        # retain FollowPath and their own relaxed, native goal checker.
        controller['controller_plugins']=['FollowPath','PointPosition','PointRecovery','PointHeading']
        controller['goal_checker_plugins']=['goal_checker','point_position_checker','recovery_position_checker','point_heading_checker']
        controller['point_position_checker']={'plugin':'nav2_controller::PositionGoalChecker',
            'xy_goal_tolerance':.035,'stateful':True}
        controller['point_heading_checker']={'plugin':'nav2_controller::SimpleGoalChecker',
            'xy_goal_tolerance':.10,'yaw_goal_tolerance':.05235987756,'stateful':True}
        # The shipped RPP plugin follows continuous path geometry instead of
        # selecting near-goal velocities using quantized DWB grid scores.
        position={'plugin':'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            'desired_linear_vel':.20,'lookahead_dist':.25,'min_lookahead_dist':.18,
            'max_lookahead_dist':.40,'lookahead_time':1.5,'use_velocity_scaled_lookahead_dist':True,
            'rotate_to_heading_angular_vel':.20,'rotate_to_heading_min_angle':.45,
            'max_angular_accel':3.2,'use_rotate_to_heading':True,'allow_reversing':False,
            'min_approach_linear_velocity':.04,'approach_velocity_scaling_dist':.50,
            'use_regulated_linear_velocity_scaling':True,'use_cost_regulated_linear_velocity_scaling':True,
            'regulated_linear_scaling_min_radius':.50,'regulated_linear_scaling_min_speed':.04,
            'cost_scaling_dist':.15,'cost_scaling_gain':1.,'inflation_cost_scaling_factor':3.,
            'use_collision_detection':True,'max_allowed_time_to_collision_up_to_carrot':1.,
            'transform_tolerance':.20,'use_interpolation':True}
        controller['PointPosition']=position
        # Search alternative safe native velocities if RPP's single geometric
        # command is blocked. RPP still handles the final precise position.
        recovery=copy.deepcopy(controller['FollowPath'])
        recovery.update(min_vel_x=0.,max_vel_x=.10,max_speed_xy=.10,
            xy_goal_tolerance=.10,sim_time=1.2,
            critics=['ObstacleFootprint','Oscillation','PathAlign','PathDist','GoalDist'])
        recovery['ObstacleFootprint.scale']=.02
        controller['PointRecovery']=recovery
        controller['recovery_position_checker']={'plugin':'nav2_controller::PositionGoalChecker',
            'xy_goal_tolerance':.10,'stateful':True}
        config['local_costmap']['local_costmap']['ros__parameters']['resolution']=.02
        config['smoother_server']['ros__parameters']['simple_smoother'].update(
            tolerance=1e-6,max_its=3000,w_data=0.,w_smooth=.40,do_refinement=True)
        heading=copy.deepcopy(controller['FollowPath'])
        heading.update(min_vel_x=0.,max_vel_x=0.,min_speed_xy=.01,max_speed_xy=0.,
            vx_samples=1,vtheta_samples=81,xy_goal_tolerance=.10,sim_time=1.2,
            critics=['ObstacleFootprint','RotateToGoal'])
        heading['ObstacleFootprint.scale']=.02
        heading['RotateToGoal.lookahead_time']=1.0
        controller['PointHeading']=heading
        config['demo_arrival']={'ros__parameters':{'enabled':True,'nominal_speed':.20,
            'approach_speed':.10,'creep_speed':.05,'approach_distance':.65,'creep_distance':.20,
            'final_spin':False,'waypoint_position_then_heading':True,
            'heading_timeout':30.,'heading_retries':1,'final_position_tolerance':.05,
            'docking_timeout':90.,
            'allow_near_arrival':False,'near_position_tolerance':.08,'near_yaw_tolerance':.08726646260}}
        config['behavior_server']['ros__parameters'].update(cycle_frequency=20.,
            max_rotational_vel=.25,min_rotational_vel=.12,rotational_acc_lim=.12)
        ekf_source=here/'assets/factory_ekf_nav.yaml'
        if hashlib.sha256(ekf_source.read_bytes()).hexdigest()!='bb6710ddd46c505b6d7cda14eb1e08459ad619d6e8ca2e66c368ae44a88458ba':
            raise RuntimeError('厂家 EKF 配置校验失败')
        ekf=yaml.safe_load(ekf_source.read_text())
        original_ekf=copy.deepcopy(ekf)
        ekf['nav_ekf_filter_node']['ros__parameters'].update(frequency=20.,
            imu0_twist_rejection_threshold=5.,debug=False,predict_to_current_time=True)
        (here/'assets/demo_ekf_nav.yaml').write_text(yaml.safe_dump(ekf,sort_keys=False))
    controller['progress_checker'].update(required_movement_radius=.03,movement_time_allowance=20.)
    if args.controller == 'dwb':
        # Supplied native plugin counts turning as progress as well as translation.
        controller['progress_checker'].update(plugin='nav2_controller::PoseProgressChecker',
                                              required_movement_angle=.035)
    config['amcl']['ros__parameters'].update(update_min_d=.03,update_min_a=.05)
    # The bundled older MPPI implementation does not read acceleration limits;
    # use the supplied native smoother rather than assuming YAML keys suffice.
    config['velocity_smoother']={'ros__parameters':{
        'use_sim_time':False,'smoothing_frequency':20.,'scale_velocities':False,
        'feedback':'OPEN_LOOP','max_velocity':[.20,0.,.50],
        'min_velocity':[-.10,0.,-.50],'max_accel':[.50,0.,3.2],
        'max_decel':[-1.,0.,-3.2],'deadband_velocity':[0.,0.,0.],
        'velocity_timeout':.3,'odom_topic':'/odom_combined','odom_duration':.1}}
(here/'navigation.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
def differences(a,b,path=''):
    result=[]
    if isinstance(a,dict) and isinstance(b,dict):
        for key in a.keys()|b.keys():
            p=f'{path}.{key}'.lstrip('.')
            if key not in a or key not in b:
                result.append({'path':p,'factory':a.get(key),'demo':b.get(key)})
            else: result.extend(differences(a[key],b[key],p))
    elif a!=b:
        result.append({'path':path,'factory':a,'demo':b})
    return result
report={'source_archive':'X5_wheeltec_ros2_src_20260903.zip',
        'profile':args.profile,
        'controller':args.controller,
        'entry':'src/wheeltec_robot_nav2/param/wheeltec_params/param_mini_4wd.yaml',
        'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'changes':differences(original,config)}
if args.profile=='precision' and args.controller=='dwb':
    report['ekf']={'source_entry':'src/turn_on_wheeltec_robot/config/ekf_nav.yaml',
                   'source_sha256':hashlib.sha256(ekf_source.read_bytes()).hexdigest(),
                   'changes':differences(original_ekf,ekf)}
(here/'factory_config_diff.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False,indent=2))
