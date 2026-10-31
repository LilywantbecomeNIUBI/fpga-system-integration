"""Offline ideal kinematic model with analytic ray/circle/wall ranges."""
from pathlib import Path
import json,math
from model import rpp,vfh,circular
ROOT=Path(__file__).resolve().parents[2]
def scan(pose,obstacles):
 x,y,theta=pose;out=[]
 for i in range(72):
  a=theta+math.radians(i*5+2.5);ux,uy=math.cos(a),math.sin(a);hits=[6000.0]
  for bound,origin,direction in ((-3000,x,ux),(4000,x,ux),(-3000,y,uy),(4000,y,uy)):
   if abs(direction)>1e-9:
    hit=(bound-origin)/direction
    if hit>0:hits.append(hit)
  for ox,oy,radius in obstacles:
   dx,dy=ox-x,oy-y;dot=dx*ux+dy*uy;disc=radius*radius-(dx*dx+dy*dy-dot*dot)
   if disc>=0 and dot>0:hits.append(max(1,dot-math.sqrt(disc)))
  out.append(int(min(hits)))
 return out

def simulate(obstacles,waypoints):
 pose=[0.,0.,0.];start=(0.,0.);index=0;previous_world=0.;speed=0.;binary=None;rows=[];min_clear=1e9
 for n in range(12000):
  x,y,theta=pose;goal=waypoints[index]
  pursuit=rpp(pose,start,goal,speed)
  if pursuit['done']:
   start=goal;index+=1
   if index==len(waypoints):return dict(done=True,seconds=n*.02,minimum_surface_clearance_mm=min_clear,trace=rows)
   continue
  target=pursuit['heading'];previous=(math.degrees(previous_world-theta)*100+18000)%36000-18000
  ranges=scan(pose,obstacles)
  ref=vfh(ranges,[True]*72,target,previous,radius=280,turn=int(pursuit['v']*1000/400),binary=binary);binary=ref['binary']
  retry=not ref['found']
  if retry:ref=vfh(ranges,[True]*72,target,previous,radius=280,turn=0,binary=binary);binary=ref['binary']
  v,w=0.,0.
  if ref['found'] and ref['front_min']>450:
   v=pursuit['v']*(.5 if ref['front_min']<1000 else 1)
   if retry or abs(ref['heading'])>4500:
    v=0
    if not ref['rotation_clear']:ref['found']=False
   if ref['found']:
    if circular(ref['heading'],target)<=250 and v>0:w=pursuit['w']*(v/pursuit['v'])
    else:w=ref['heading']*.349
    w=max(-400,min(400,w))
  previous_world=theta+math.radians(ref['heading']/100) if ref['found'] else previous_world
  dt=.02;pose=[x+v*math.cos(theta+w*.0005*dt)*dt,y+v*math.sin(theta+w*.0005*dt)*dt,theta+w*.001*dt];speed=v
  clearance=min([math.hypot(pose[0]-ox,pose[1]-oy)-radius for ox,oy,radius in obstacles] or [1e9]);min_clear=min(min_clear,clearance)
  rows.append([n*dt,*pose,v,w,index,clearance])
  if clearance<180:return dict(done=False,collision=True,seconds=n*dt,trace=rows)
 return dict(done=False,seconds=240,minimum_surface_clearance_mm=min_clear,trace=rows)

def run():
 cases=[('waypoints',[],[(1000,0),(1000,800),(1800,800)]),('box_front',[(700,0,120)],[(1500,0)]),('box_offset',[(700,150,120)],[(1500,0)])]
 result={};traces={}
 for name,obstacles,waypoints in cases:
  case=simulate(obstacles,waypoints);traces[name]=case.pop('trace');case.update(obstacles=obstacles,waypoints=waypoints);result[name]=case
  print(name,case,flush=True)
 folder=ROOT/'build/task4';(folder/'kinematic.json').write_text(json.dumps(dict(cases=result,ideal_kinematics=True,hardware_tested=False),indent=2))
 (folder/'kinematic_traces.json').write_text(json.dumps(traces))
 assert all(c['done'] and not c.get('collision') for c in result.values()),'Kinematic validation failed'
 return result
if __name__=='__main__':run()
