"""Independent geometry reference for the documented 72-sector VFH+ variant.

Uses floating geometry; quantization is explicit only at the sector boundary.
This is a project-specific reference, not a copy of controllerVFH.
"""
import math

def circular(a,b):
    d=abs(a-b)%36000
    return min(d,36000-d)

def vfh(ranges,known,target=0,previous=0,radius=300,turn=0,binary=None):
    if binary is None:binary=[True]*72
    binary=[True if not k or d<=1200 else False if d>=1400 else old for d,k,old in zip(ranges,known,binary)]
    blocked=[False]*72;left,right=9000,-9000
    for i,(d,k,b) in enumerate(zip(ranges,known,binary)):
        if not k:
            for j in (i,(i-1)%72,(i+1)%72):blocked[j]=True
        elif b:
            if d<=radius:blocked=[True]*72;continue
            ang=math.radians(i*5+2.5)
            # RTL truncates coordinates toward negative infinity (arithmetic shift).
            x=(d*round(math.cos(ang)*32768))//32768
            y=(d*round(math.sin(ang)*32768))//32768
            if turn and x*x+(y-turn if y>=0 else y+turn)**2<(turn+radius)**2:
                if y>0:left=min(left,i*500+250)
                if y<0:right=max(right,i*500-35750)
            # Conservative ceil(arcsin(R/d)/5) plus one sector for quantization.
            step=next((j for j in range(1,19) if d*round(math.sin(math.radians(j*5))*32768)>=radius*32768),18)
            spread=step+1
            for j in range(72):
                delta=abs(i-j);delta=min(delta,72-delta)
                if delta<=spread:blocked[j]=True
    for j in range(72):
        angle=j*500+250 if j<36 else j*500-35750
        if angle>=left or angle<=right:blocked[j]=True
    free=[not blocked[(i+54)%72] and right<i*500-8750<left for i in range(36)]
    candidates=[];i=0
    while i<36:
        if not free[i]:i+=1;continue
        first=i
        while i<36 and free[i]:i+=1
        last=i-1;width=i-first
        if width<=8:candidates.append((first+last)//2)
        else:
            a,b=first+4,last-4;candidates.extend((a,b))
            if a*500-9000<=target<=b*500-8501:
                candidates.append(min(b,max(a,(target+9000)//500)))
    costs=[5*circular(i*500-8750,target)+circular(i*500-8750,0)+3*circular(i*500-8750,previous) for i in candidates]
    chosen=candidates[costs.index(min(costs))] if costs else None
    front=[i for i in range(72) if i<6 or i>=66]
    return dict(binary=binary,blocked=sum(int(b)<<i for i,b in enumerate(blocked)),
                found=chosen is not None,heading=chosen*500-8750 if chosen is not None else None,
                cost=min(costs) if costs else 0xffffff,
                front_known=all(known[i] for i in front),rotation_clear=all(k and d>radius for k,d in zip(known,ranges)),
                front_min=min([ranges[i] for i in front if known[i]] or [65535]))

def rpp(pose,segment_start,goal,speed_mm_s,lookahead_min=300,lookahead_max=800,lookahead_ms=1000,vmax=120,wmax=400,accept=120):
    x,y,theta=pose;sx,sy=segment_start;gx,gy=goal
    distance=math.hypot(gx-x,gy-y)
    if distance<accept:return dict(done=True,v=0,w=0)
    dx,dy=gx-sx,gy-sy;length=math.hypot(dx,dy)
    if not length:raise ValueError('Zero-length unfinished segment')
    projection=max(0,min(length,((x-sx)*dx+(y-sy)*dy)/length))
    lookahead=max(lookahead_min,min(lookahead_max,abs(speed_mm_s)*lookahead_ms/1000))
    along=min(length,projection+lookahead)
    cx,cy=sx+dx*along/length-x,sy+dy*along/length-y
    bx=cx*math.cos(theta)+cy*math.sin(theta);by=-cx*math.sin(theta)+cy*math.cos(theta)
    dsq=bx*bx+by*by
    curve=vmax if by==0 else wmax*dsq/(2000*abs(by))
    speed=max(30,min(vmax,curve,distance*vmax/(2*lookahead_min)))
    return dict(done=False,v=speed,w=2*speed*by*1000/dsq,heading=math.degrees(math.atan2(by,bx))*100,carrot=(bx,by))
