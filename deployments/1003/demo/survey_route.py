"""Map-bound north-block sampling proposal, no driving and no map modification."""
from collections import deque
import json
import math

BOUNDS=(.1,5.1,-7.6,.3)  # +X is dock-forward (user's north), +Y left.


def north_grid_route(grid, spacing=.5, clearance=.28):
    if grid.map_id!='57609e1fda88052c715692582c8d743da48e69c308b8139c9483429e751f81e0':
        raise ValueError('底图版本已变化，北区边界和路线必须重新核对，未加载旧采样点')
    xmin,xmax,ymin,ymax=BOUNDS
    offsets=[(dx,dy) for dx in range(-6,7) for dy in range(-6,7)
             if math.hypot(dx,dy)*grid.resolution<=clearance]
    safe=set()
    for iy in range(grid.height):
        for ix in range(grid.width):
            x,y=grid.world(ix,iy)
            if not (xmin<=x<=xmax and ymin<=y<=ymax):continue
            if all(grid.value(ix+dx,iy+dy)==0 and not grid.forbidden(ix+dx,iy+dy) for dx,dy in offsets):
                safe.add((ix,iy))
    if not safe:raise ValueError('区域中没有足够净空的采样位置')
    start=min(safe,key=lambda c:math.hypot(*grid.world(*c)))
    def distances(source):
        out={source:0};q=deque([source])
        while q:
            x,y=q.popleft()
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nxt=(x+dx,y+dy)
                if nxt in safe and nxt not in out:out[nxt]=out[(x,y)]+1;q.append(nxt)
        return out
    reachable=distances(start)
    candidates=[]
    for i in range(10):
        for j in range(16):
            x=.35+spacing*i;y=.1-spacing*j;c=grid.cell_xy(x,y)
            try:grid.validate(x,y,clearance)
            except ValueError:continue
            if c in reachable:candidates.append((c,x,y))
    ordered=[];source=start;length=0
    while candidates:
        d=distances(source)
        item=min(candidates,key=lambda p:d[p[0]])
        length+=d[item[0]]*grid.resolution;ordered.append(item);source=item[0];candidates.remove(item)
    # Fixed north-facing sample pose avoids confounding map strength with vehicle yaw.
    points=[{'id':f'north-{round(x*100):04d}-{round(-y*100):04d}',
             'name':f'北区 {i+1:02d}','x':round(x,3),'y':round(y,3),'yaw':0.,'wait':7.,'enabled':True}
            for i,(_,x,y) in enumerate(ordered)]
    if not points:raise ValueError('北区没有可达采样点')
    return {'schema':1,'map_id':grid.map_id,'bounds':list(BOUNDS),'spacing_m':spacing,
            'clearance_m':clearance,'points':points,'planned_grid_path_length_m':round(length,2),
            'note':'仅已知可达白色区域；点位名义间隔0.5m，墙/禁区造成缺点；不是整个矩形的完整实测覆盖'}


def north_route(grid):
    """Few transit guides, continuous collection. Keep the left threshold closed."""
    if grid.map_id!='57609e1fda88052c715692582c8d743da48e69c308b8139c9483429e751f81e0':
        raise ValueError('地图变化，请重新核对回环边界')
    safe=set()
    for iy in range(grid.height):
        for ix in range(grid.width):
            x,y=grid.world(ix,iy)
            if not (.1<=x<=5.1 and -7.6<=y<=.3):continue
            try:grid.validate(x,y,.24)
            except ValueError:continue
            safe.add((ix,iy))
    def nearest(x,y):return min(safe,key=lambda c:math.hypot(grid.world(*c)[0]-x,grid.world(*c)[1]-y))
    def path(a,b):
        q=deque([a]);parent={a:None}
        while q:
            c=q.popleft()
            if c==b:
                result=[c]
                while parent[result[-1]] is not None:result.append(parent[result[-1]])
                return list(reversed(result))
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                n=(c[0]+dx,c[1]+dy)
                if n in safe and n not in parent:parent[n]=c;q.append(n)
        raise ValueError('回环通道不连通，未生成穿越障碍的路线')
    # Left-side threshold prevents a simple outer ring; explicitly return by right aisle.
    start=nearest(.35,-.15)
    anchors=[(.55,-6.8),(4.05,-6.7),(4.1,-.65),(3.45,-.15),
             (4.1,-.65),(4.05,-6.7),(.55,-6.8),(.35,-.15)]
    guides=[];preview=[list(grid.world(*start))];source=start
    for i,(x,y) in enumerate(anchors):
        cell=nearest(x,y);leg=path(source,cell);xy=grid.world(*cell)
        heading=math.atan2(leg[-1][1]-leg[-2][1],leg[-1][0]-leg[-2][0]) if len(leg)>1 else 0.
        guides.append({'id':f'north-loop-{i+1}','name':f'回环引导 {i+1}',
                       'x':xy[0],'y':xy[1],'yaw':heading,'wait':0.,'enabled':True})
        preview.extend([list(grid.world(*c)) for c in leg[1:]]);source=cell
    return {'schema':2,'map_id':grid.map_id,'bounds':list(BOUNDS),'profile':'moving_loop_demo',
        'points':guides,'preview_path':preview,'clearance_m':.24,
        'planned_grid_path_length_m':round((len(preview)-1)*grid.resolution,2),
        'note':'8个顺路引导点；沿途采集，不定点等样本、不统一朝向；左侧小坎保留禁行，受限处折返经右侧回桩'}
