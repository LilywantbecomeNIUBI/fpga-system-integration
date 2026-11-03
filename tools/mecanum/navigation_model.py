"""Independent floating-point geometry oracle (not a translation of the RTL).

Filled oriented rectangle versus occupied square uses separating axes. Dense
poses cover motion between the hardware's 100 ms samples. No Nav2 code copied.
"""
import math


def intersects_cell(x, y, angle, front, back, width, cell_x, cell_y):
    c, s = math.cos(angle), math.sin(angle)
    center_offset = (front-back)/2
    hx, hy = (front+back)/2, width
    dx = (cell_x-64)*50+25-x-center_offset*c
    dy = (cell_y-64)*50+25-y-center_offset*s
    # Rectangle axes and grid-square axes are the four separating axes.
    return (abs(dx*c+dy*s) <= hx+25*(abs(c)+abs(s)) and
            abs(-dx*s+dy*c) <= hy+25*(abs(c)+abs(s)) and
            abs(dx) <= hx*abs(c)+hy*abs(s)+25 and
            abs(dy) <= hx*abs(s)+hy*abs(c)+25)


def collides(x, y, angle, sx, sy, sa, steps, obstacles,
             front=100, back=100, width=50):
    for k in range(max(1, (steps-1)*20+1)):
        t = k/20
        if any(intersects_cell(x+sx*t, y+sy*t, angle+sa*t,
                               front, back, width, cx, cy)
               for cx, cy in obstacles):
            return True
    return False


def ray_cells(x0, y0, x1, y1):
    """Bresenham to an unbounded endpoint, stopping at first grid exit."""
    dx, dy = abs(x1-x0), -abs(y1-y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx+dy
    while 0 <= x0 < 128 and 0 <= y0 < 128:
        yield x0, y0
        if (x0, y0) == (x1, y1):
            return
        twice = 2*err
        if twice >= dy:
            err += dy
            x0 += sx
        if twice <= dx:
            err += dx
            y0 += sy


def sweep_vectors():
    """Both exact collision and distant-clear cases, including rotation."""
    scenes = []
    for angle in (0, math.pi/6, -math.pi/3, math.radians(179), math.radians(-179)):
        for sx, sy, sa in ((0, 0, 0), (15, 0, 0), (0, -15, 0), (10, 10, .025)):
            steps = 20
            for cell in ((64, 64), (66, 64), (68, 65), (64, 60), (95, 95)):
                scenes.append((0, 0, round(angle*65536), sx, sy,
                               round(sa*65536), steps, *cell,
                               int(collides(0, 0, angle, sx, sy, sa, steps, [cell]))))
    return scenes
