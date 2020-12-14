import numpy as np
from numpy.linalg import det

# changed GRID to change the resolution of the line-finding algorithm:
# the algorithm looks at GRIDxGRID image created from lidar data.
GRID = 200
MAX_RANGE = 2.5 #2 # in meters, up to 3.5
CROSSBAR_LEN = int(15 * (GRID / 200.))
ZERO = ((0, 0), (0, 0))
CENTER = ((GRID / 2, GRID / 2), (GRID / 2, GRID / 2))
ROT90 = np.array([[np.cos(np.pi / 2), -np.sin(np.pi / 2)],
                  [np.sin(np.pi / 2),  np.cos(np.pi / 2)]])

'''
BEGIN LINE FINDING UTIL
'''

# returns length of line
def norm(line):
    p0, p1 = line
    return np.linalg.norm(np.array(p0) - np.array(p1))

# returns center point of line
def center(line):
    p0, p1 = line
    return (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2

# returns distance between centers of two lines
def dist(line1, line2):
    return norm((center(line1), center(line2)))

# returns normalized version of `x`
def unit(x):
    return x / np.linalg.norm(x)

# converts line with endpoints (p0, p1) to a unit vector pointing from p0 to p1
def line2vec(line):
    p0, p1 = line
    x = np.array([p1[0] - p0[0], p1[1] - p0[1]])
    return unit(x)

# this function from https://stackoverflow.com/a/13849249
def angle_between(v1, v2):
    # Returns the angle in radians between vectors 'v1' and 'v2'
    v1_u = unit(v1)
    v2_u = unit(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

# concatenates all parallel lines within a certain threshold
def parallel(lines, thres):
    lines = list(lines)
    toRemove = set()
    toAdd = set()
    for i in range(len(lines) - 1):
        line = lines[i]
        vec = line2vec(line)
        for j in range(len(lines)-i-1):
            other_line = lines[len(lines)-1-j]
            th = angle_between(vec, line2vec(other_line))
            if (-0.1 <= th and th <= 0.1) or (np.pi - 0.1 <= th and th <= np.pi + 0.1):
                #find shortest and max dist
                p0,p1 = line
                p2,p3 = other_line
                dist = []
                pairs = [(p0,p2), (p0, p3), (p1, p2), (p1, p3)]
                dist.append(norm((p0,p2)))
                dist.append(norm((p0,p3)))
                dist.append(norm((p1,p2)))
                dist.append(norm((p1,p3)))
                s = min(dist)
                ##print(s)
                l = np.argmax(dist)
                ##print(l)
                if s < thres:
                    newline = pairs[l]
                    theta = angle_between(vec, line2vec(newline))
                    # make sure new line is parallel to the two lines
                    if (-0.15 <= theta and theta <= 0.15) or (np.pi - 0.15 <= theta and theta <= np.pi + 0.15):
                        #print("parallel activate")
                        toAdd.add(newline)
                        toRemove.add(line)
                        toRemove.add(other_line)
    for i in toRemove:
        lines.remove(i)
    for i in toAdd:
        lines.append(i)
    return lines


# returns True if lines intersect; False otherwise
# (this function from https://stackoverflow.com/a/3842157)
def do_intersect(line1, line2):
    a, b = line1
    c, d = line2
    ax, ay = a
    bx, by = b
    cx, cy = c
    dx, dy = d

    s1 = np.sign(det([[ax-cx, bx-cx], [ay-cy, by-cy]]))
    s2 = np.sign(det([[ax-dx, bx-dx], [ay-dy, by-dy]]))
    s3 = np.sign(det([[cx-ax, dx-ax], [cy-ay, dy-ay]]))
    s4 = np.sign(det([[cx-bx, dx-bx], [cy-by, dy-by]]))

    out = s1 != s2 and s3 != s4

    if out:
        x = ( (ax*by-ay*bx)*(cx-dx)-(ax-bx)*(cx*dy-cy*dx) ) / ( (ax-bx)*(cy-dy)-(ay-by)*(cx-dx) )
        y = ( (ax*by-ay*bx)*(cy-dy)-(ay-by)*(cx*dy-cy*dx) ) / ( (ax-bx)*(cy-dy)-(ay-by)*(cx-dx) )
        return [x, y]
    else:
        return None 

# returns the bar that is ortho to the input line, with a certain length.
# centers of `line` and returned line are the same.
def mk_crossbar(line, length=CROSSBAR_LEN):
    x, y = center(line)
    vec = length * ROT90.dot(line2vec(line))
    p00 = x + vec[0]
    p01 = y + vec[1]
    p10 = x - vec[0]
    p11 = y - vec[1]

    return ((p00, p01), (p10, p11))

# returns this line, extended by DIST in each direction
# can also use `frac` to make line have new norm (curnorm * (1 + frac))
def extend(line, frac=None):
    DIST = 10000
    if frac:
        DIST = norm(line) * frac / 2

    x, y = line
    p00, p01 = x
    p10, p11 = y

    vec = line2vec(line)

    return ((p00 - DIST * vec[0], p01 - DIST * vec[1]),
            (p10 + DIST * vec[0], p11 + DIST * vec[1]))

# returns distance from line center to the robot
def dist_to_robot(line, tf):
    return dist(line, ((tf[0], tf[1]), (tf[0], tf[1])))

# adds a vector to each point of a line
def add(line, v0=ZERO, v1=ZERO):
    p0, p1 = line
    x0, y0 = p0
    x1, y1 = p1
    xv0, yv0 = v0
    xv1, yv1 = v1

    return ((x0 + xv0, y0 + yv0), (x1 + xv1, y1 + yv1))

# shifts a line along its ortho axis by a distance. `dir` should be -1 or 1
def shift(line, dist, dir=1):
    shift_vec = dist * dir * ROT90.dot(line2vec(line))
    shift_vec = (shift_vec[0], shift_vec[1])
    return add(line, v0=shift_vec, v1=shift_vec)

### DATA ASSOCIATION ###
# calc euc dist between centers and the area difference of the obs
def calc_sim(obs, prev):
    ad = np.abs(prev.area - obs.area)
    dist = (obs.center[0] - prev.mean[0])**2 + (obs.center[1] - prev.mean[3])**2
    return dist, ad

# calc a mapping of obstacles measured at the current timestep to the previous obstacles and sets the filters
def data_association(obs, prev, n):
    flag = False
    if not prev:
        return obs, flag

    # for o in obs:
    #     #print("observed center", o.mean)
    #     #print("observed area", o.area)

    # for o in prev:
    #     #print("kf center", o.mean)
    #     #print("kf area", o.area)

    dist_mat = np.zeros((len(obs), len(prev)))
    area_mat = np.zeros((len(obs), len(prev)))

    # from https://pdfs.semanticscholar.org/f5a2/bf3df3126d2923a617b977ec2b4e1c829a08.pdf
    for i, obst in enumerate(obs):
        for j, obst_p in enumerate(prev):
            dist, ad = calc_sim(obst, obst_p)
            dist_mat[i, j] = dist
            area_mat[i, j] = ad

    d_max = np.amax(dist_mat, axis=1) + 0.000001
    dist_mat = np.sqrt(dist_mat)
    dist_mat = dist_mat/d_max[:,None]

    a_max = np.amax(area_mat, axis=1)
    area_mat = area_mat/a_max[:,None]

    alpha = 0.9 # TUNE THIS
    beta = 1 - alpha

    cost_mat = alpha * dist_mat + beta * area_mat

    #print("COST:", cost_mat)
    m = np.argmin(cost_mat, axis=1)
    # obstacle at index i is associated with the previous obstacle at index m[i]
    #print("data_asso:", m)

    marked = set(range(n))
    toDel = list()
    for i in range(len(m)):
        # assign kf from prev to current obs
        obs[i].kf = prev[m[i]].kf
        obs[i].mean = prev[m[i]].mean
        obs[i].cov = prev[m[i]].cov

        # try to remove obs already assigned
        try:
            marked.remove(m[i])
        except: # if already removed, check which case
            if len(obs) <= len(prev): 
                toDel.append(obs[i])
                flag = True
            else:
                # allow both to affect kf, remove all but one obstacles pointing to that kf
                toDel.append(obs[i])

    for i in toDel:
        obs.remove(i)

    # leftover prev obstacles added to curr obs (since its assumed prev obs is the truth)
    #print("leftover", marked)
    for i in marked:
        obs.append(prev[i])

    return obs, flag







'''
END LINE FINDING UTIL
'''

'''
BEGIN LIDAR TO IMG CONVERSION
'''

def polar2xy(r, th):
    return (r * np.cos(th), r * np.sin(th))

# converts (x, y) coordinate to a specific (r, c) in a grid
def pt2vox(x, y):
    if np.isnan(x):
         x = 0
    if np.isnan(y):
         y = 0

    grid_x = int(max(min((x / MAX_RANGE) * (GRID // 2) + (GRID // 2), GRID - 1), 0))
    grid_y = int(max(min((y / MAX_RANGE) * (GRID // 2) + (GRID // 2), GRID - 1), 0))

    return grid_x, max(min(GRID - grid_y, GRID - 1), 0)

# coverts (r, c) in a grid to an (x, y) coordinate in meters
def vox2pt(vox):
    r, c = vox
    convert = lambda w: (w - (GRID / 2.)) * (2. * MAX_RANGE / GRID)

    return convert(r), -convert(c)

# constructs GRID x GRID numpy 2d array from polar lidar data
def lidar2img(data, tf):
    img = np.zeros((GRID, GRID))

    # extract data we need
    a_min = data.angle_min
    a_max = data.angle_max
    a_inc = data.angle_increment
    t_inc = data.time_increment
    r_min = data.range_min
    ranges = data.ranges
    intensities = data.intensities

    thetas = np.arange(a_min, a_max, a_inc) + tf[2]

    # construct img
    for r, th in zip(ranges, thetas):
        if r > MAX_RANGE:
            continue

        x, y = polar2xy(r, th)
        grid_x, grid_y = pt2vox(x, y)
        img[grid_y, grid_x] = 1.

    return img

'''
END LIDAR TO IMG CONVERSION
'''

# filters cluster by categorizing it as an object (true) or wall (false)
# cluster is a list of lines, tf is robot position
def cluster_filter(cluster, tf):
    ##print("BNUM LINES", len(cluster))

    if not cluster or len(cluster) == 0:
        return False

    # connects parallel lines together
    cluster = parallel(cluster, 0.1)

    # one long line is wall
    if len(cluster) == 1:
        line = cluster.pop()
        if norm(line) > 1:
            return False
        return True

    # wall if the robot is inside the angle formed by the two lines
    elif len(cluster) == 2:
        l1 = cluster.pop()
        l2 = cluster.pop()
        line1 = extend(l1, 0.5) 
        line2 = extend(l2, 0.5) 
        pt = do_intersect(line1, line2)
        if not pt:
            return True
        p0, p1 = l1
        
        p2, p3 = l2
        
        A = np.array([[p1[0]-pt[0], p3[0]-pt[0]], [p1[1]-pt[1], p3[1]-pt[1]]])
        b = np.array([tf[0]-pt[0], tf[1]-pt[1]])
        x = np.linalg.solve(A,b)
        if (x >= -0.5).all():
            return False
        return True

    #wall if >2 long lines
    else:
        ctr = 0
        for line in cluster:
            if norm(line) > 0.5:
                ctr = ctr + 1
                if ctr > 1:
                    return False
        return True
