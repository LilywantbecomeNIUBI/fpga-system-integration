"""Independent scalar LK oracle. No vendor libraries, cv2 optical flow, or RTL imports.

Coordinates right/down; signed Q7 (128 units/pixel). Integer mode models every
round/truncation; float mode isolates the quantization error of the hardware.
"""
import math

FRAC = 128


def texture(x, y):
    return max(0, min(255, round(128 + 32*math.sin(.47*x+.13*y)
                              + 31*math.cos(.21*x-.51*y)
                              + 22*math.sin(.73*x+.67*y))))


def image(width, height, dx=0, dy=0, flat=False, zoom=1):
    return [[128 if flat else texture((x-width/2)/zoom+width/2-dx,
                                     (y-height/2)/zoom+height/2-dy)
             for x in range(width)] for y in range(height)]


def divide_signed(n, d):
    return (-1 if n < 0 else 1) * (abs(n)//d)


def patch(a, b, cx, cy, seed=(0, 0), floating=False):
    h, w = len(a), len(a[0])
    su, sv = seed
    iu, iv = math.floor(su/128), math.floor(sv/128)
    wx, wy = cx-2+iu, cy-2+iv
    if cx < 3 or cy < 3 or cx >= w-3 or cy >= h-3 or wx < 0 or wy < 0 or wx+5 >= w or wy+5 >= h:
        return 0, 0, False
    fx, fy = su-iu*128, sv-iv*128
    aa = bb = cc = dd = ee = 0
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            x, y = cx+dx, cy+dy
            tx, ty = x+iu, y+iv
            gx = (a[y][x+1]-a[y][x-1])/2 if floating else (a[y][x+1]-a[y][x-1])//2
            gy = (a[y+1][x]-a[y-1][x])/2 if floating else (a[y+1][x]-a[y-1][x])//2
            mix = ((b[ty][tx]*(128-fx)+b[ty][tx+1]*fx)*(128-fy)
                   +(b[ty+1][tx]*(128-fx)+b[ty+1][tx+1]*fx)*fy)
            warped = mix/16384 if floating else mix//16384
            gt = a[y][x]-warped
            aa += gx*gx
            bb += gy*gy
            cc += gx*gy
            dd += gx*gt
            ee += gy*gt
    det = aa*bb-cc*cc
    if aa+bb < 512 or det <= 4096 or det <= (aa+bb)**2/64:
        return 0, 0, False
    if floating:
        du, dv = 128*(bb*dd-cc*ee)/det, 128*(aa*ee-cc*dd)/det
    else:
        du = divide_signed(128*(bb*dd-cc*ee), det)
        dv = divide_signed(128*(aa*ee-cc*dd), det)
    u, v = su+du, sv+dv
    good = abs(du) <= 512 and abs(dv) <= 512 and abs(u) <= 2048 and abs(v) <= 2048
    return (u, v, True) if good else (0, 0, False)


def grid(a, b, step=8, margin=8):
    return [(x, y, *patch(a, b, x, y)) for y in range(margin, len(a)-margin+1, step)
            for x in range(margin, len(a[0])-margin+1, step)]


def regional_counts(records, width=160, height=90):
    """V1 geometric risk proxy, independent of the temporal Safety FSM."""
    counts, risk = [0, 0, 0], [0, 0, 0]
    for x, y, u, v, good in records:
        if not good:
            continue
        region = 0 if x < width//3 else 1 if x < 2*width//3 else 2
        counts[region] += 1
        dx, dy = x-width//2, y-height//2
        dot = dx*u+dy*v
        if dot > 0 and dot >= 2*(dx*dx+dy*dy) and abs(u)+abs(v) >= 32:
            risk[region] += 1
    return counts, risk


def avoidance_decision(counts, risk, clear_count=0):
    if min(counts) < 4:
        return 0, True, 0, 0, 0
    danger = [r*4 >= n for r, n in zip(risk, counts)]
    if not any(danger):
        clear_count = min(2, clear_count+1)
        return (1, False, 0, 255, clear_count) if clear_count == 2 else (4, True, 0, 0, clear_count)
    if all(danger):
        return 4, True, 0, 0, 0
    if (danger[1] and not danger[0] and (danger[2] or risk[0] <= risk[2])) or (not danger[1] and danger[2] and not danger[0]):
        return 2, False, -1, 96, 0
    if not danger[2] and (danger[1] or danger[0]):
        return 3, False, 1, 96, 0
    return 1, False, 0, 64, 0


def downsample(a):
    return [[sum(a[y+j][x+i] for j in range(2) for i in range(2))//4
             for x in range(0, len(a[0]), 2)] for y in range(0, len(a), 2)]


def pyramid_patch(a, b, ca, cb, x, y, floating=False):
    u = v = 0
    for _ in range(3):
        u, v, good = patch(ca, cb, x//2, y//2, (u, v), floating=floating)
        if not good:
            return 0, 0, False
    u, v = u*2, v*2
    if abs(u) > 2048 or abs(v) > 2048:
        return 0, 0, False
    for _ in range(2):
        u, v, good = patch(a, b, x, y, (u, v), floating=floating)
        if not good:
            return 0, 0, False
    return u, v, True


def pyramid_grid(a, b, step=8, margin=8):
    ca, cb = downsample(a), downsample(b)
    return [(x, y, *pyramid_patch(a, b, ca, cb, x, y))
            for y in range(margin, len(a)-margin+1, step)
            for x in range(margin, len(a[0])-margin+1, step)]
