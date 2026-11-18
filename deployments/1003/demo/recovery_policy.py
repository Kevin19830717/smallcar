"""Arrival and conservative rear laser coverage checks, independent of ROS."""
import math


def position_tolerance(op, arrival):
    if op.get('kind') == 'waypoint':
        return float(arrival.get('waypoint_position_tolerance', .08))
    return float(arrival.get('final_position_tolerance', .05))


def rear_clear(scan, sensor_x, sensor_y, sensor_yaw, half_length, half_width):
    if not all(math.isfinite(v) for v in
               (sensor_x, sensor_y, sensor_yaw, half_length, half_width,
                scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)):
        return False
    if half_length <= 0 or half_width <= 0 or scan.angle_increment == 0:
        return False
    bins = set()
    for i, distance in enumerate(scan.ranges):
        angle = sensor_yaw+scan.angle_min+i*scan.angle_increment
        rear_angle = math.atan2(math.sin(angle-math.pi), math.cos(angle-math.pi))
        if abs(rear_angle) > math.radians(75):
            continue
        if not math.isfinite(distance) or not scan.range_min < distance <= scan.range_max:
            return False
        bins.add(min(29, int((rear_angle+math.radians(75))/math.radians(5))))
        x = sensor_x+distance*math.cos(angle)
        y = sensor_y+distance*math.sin(angle)
        # Check the inflated swept rear rectangle plus both rear corners.
        if -half_length-.12-.10 <= x <= .05 and abs(y) <= half_width+.10:
            return False
    return len(bins) == 30

def motion_progress(op, pose, wheel_distance, wheel_turn, now):
    """Require both mapped displacement and wheel feedback; detours count."""
    previous=op.get('progress_anchor')
    if previous is None:
        op['progress_anchor']=(list(pose),wheel_distance,wheel_turn)
        op['progress_at']=now
        return False
    anchor, distance, turn=previous
    moved=(math.hypot(pose[0]-anchor[0],pose[1]-anchor[1])>=.05
           and wheel_distance-distance>=.03)
    rotated=(abs(math.atan2(math.sin(pose[2]-anchor[2]),math.cos(pose[2]-anchor[2])))>=math.radians(5)
             and wheel_turn-turn>=math.radians(3))
    if moved or rotated:
        op['progress_anchor']=(list(pose),wheel_distance,wheel_turn)
        op['progress_at']=now
        return True
    return False

def path_progress(op, pose, wheel_distance, wheel_turn, now, path):
    """Track advancement along the route or improvement in its heading."""
    if len(path)<2:
        return motion_progress(op,pose,wheel_distance,wheel_turn,now)
    reference=op.setdefault('route_progress_reference',[list(p) for p in path])
    nearest=None
    arc=0.
    for a,b in zip(reference,reference[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=math.hypot(dx,dy)
        if length<1e-6:continue
        t=max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/(length*length)))
        point=(a[0]+t*dx,a[1]+t*dy)
        candidate=(math.hypot(pose[0]-point[0],pose[1]-point[1]),arc+t*length)
        if nearest is None or candidate[0]<nearest[0]:
            nearest=candidate
        arc+=length
    if nearest is None:
        return motion_progress(op,pose,wheel_distance,wheel_turn,now)
    ahead=nearest[1]+.45
    arc=0.;target=reference[-1]
    for a,b in zip(reference,reference[1:]):
        length=math.hypot(b[0]-a[0],b[1]-a[1])
        if length>1e-6 and arc+length>=ahead:
            t=(ahead-arc)/length
            target=[a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])]
            break
        arc+=length
    direction=math.atan2(target[1]-pose[1],target[0]-pose[0])
    angle=abs(math.atan2(math.sin(direction-pose[2]),math.cos(direction-pose[2])))
    old=op.get('route_motion_anchor')
    if old is None:
        op['route_motion_anchor']=(list(pose),wheel_distance,wheel_turn,nearest[1],angle)
        op['progress_at']=now
        return False
    anchor,dist,turn,along,best_angle=old
    displacement=math.hypot(pose[0]-anchor[0],pose[1]-anchor[1])
    advancing=displacement>=.05 and wheel_distance-dist>=.03 and nearest[1]-along>=.05
    detouring=displacement>=.15 and wheel_distance-dist>=.10
    aligning=best_angle-angle>=math.radians(5) and wheel_turn-turn>=math.radians(3)
    if advancing or detouring or aligning:
        op['progress_at']=now
        op['route_motion_anchor']=(list(pose),wheel_distance,wheel_turn,nearest[1],
                                   angle if advancing or detouring else min(best_angle,angle))
        if detouring and nearest[0]>.6:
            op['route_progress_reference']=[list(p) for p in path]
        return True
    return False
