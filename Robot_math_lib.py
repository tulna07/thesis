import numpy as np
import math



def intersection(x, y, radius, line_segments):
    '''
    find two different points where a line intersects with a circle
    '''

    is_pt = []
    p1x, p1y = line_segments[0]
    p2x, p2y = line_segments[1]
    dx, dy = np.subtract(p2x, p1x), np.subtract(p2y, p1y)
    a = dx ** 2 + dy ** 2
    b = 2 * (dx * (p1x - x) + dy * (p1y - y))
    c = (p1x - x) ** 2 + (p1y - y) ** 2 - radius ** 2
    discriminant = b ** 2 - 4 * a * c
    if discriminant > 0:
        t1 = (-b + discriminant ** 0.5) / (2 * a)
        t2 = (-b - discriminant ** 0.5) / (2 * a)
        pt1 = [dx * t1 + p1x, dy * t1 + p1y]
        pt2 = [dx * t2 + p1x, dy * t2 + p1y]
        is_pt = (pt1, pt2)

    return is_pt


def inside_line_segment(point, line_segment):
    '''
    check if a point is whether inside given line segment 
    return True if inside, otherwise return False
    using total distance to check, 
    '''
    dp1 = point_dist(point, line_segment[0])
    dp2 = point_dist(point, line_segment[1])
    dls = point_dist(line_segment[1], line_segment[0])
    dp = dp1 + dp2
    return math.isclose(dp, dls)


def rotate_vector_center(center, v, radian):
    '''
    rotate a vector with a angle of radians around center point
    '''
    vector_vc = np.subtract(v, center)
    r_vector_vc = rotate_vector(vector_vc, radian)
    result = np.add(center, r_vector_vc)
    return np.add(center, r_vector_vc)


def rotate_vector(v, radian):
    '''
    rotate vector with a angle of radians around (0,0)
    '''
    x, y = v
    rx = x * math.cos(radian) + y * math.sin(radian)
    ry = -x * math.sin(radian) + y * math.cos(radian)
    return rx, ry


def unit_vector(vector):
    '''
    Return the unit vector of the vector
    '''
    return vector / np.linalg.norm(vector)


def unsigned_angle(v1, v2):
    '''
    Find unsigned angle between two vectors
    '''
    usa = signed_angle(v1, v2)
    if usa < 0:
        usa = 2 * math.pi + usa
    return usa

def unsigned_angle(center, ptA, ptB):
    '''
    Find unsigned angle among 3 points (ptA- center- ptB)
    '''
    vector_a = np.subtract(ptA, center)
    vector_b = np.subtract(ptB, center)
    usa = signed_angle(vector_a, vector_b)
    if usa < 0:
        usa = 2 * math.pi + usa
    return usa

def unsigned_angle_xAxis(point):
    '''
    Find unsigned angle between point and x axis
    '''

    angle = math.atan2(point[1], point[0])
    if angle < 0:
        angle = 2 * math.pi + angle
    return angle


def signed_angle_xAxis(point):
    '''
    Finds signed angle between point and x axis
    return angle (+) for anti clockwise, and (-) for clockwise
    '''
    angle = math.atan2(point[1], point[0])
    # print (math.degrees(angle))
    return angle


def signed_angle(v1, v2):
    '''
    Finds angle between two vectors
    return angle (+) for anti clockwise, and (-) for clockwise
    '''
    angle = signed_angle_xAxis(v1)  # cal angle between vector (A) and ox axis
    v_b = rotate_vector(v2, angle)  # rotate vector B according to rotation_radians 
    return signed_angle_xAxis(v_b)


def get_angle_info(center, ptA, ptB):
    '''
    return angle, start edge , end edge (in anti-clockwise)
    '''

    vector_a = np.subtract(ptA, center)
    vector_b = np.subtract(ptB, center)

    angle = signed_angle(vector_a, vector_b)

    if angle < 0:
        vs = ptB  # start 
        ve = ptA  # end
    else:
        vs = ptA  # start
        ve = ptB  # end
    return angle, vs, ve


def inside_angle_area(point, center, ref_boundaries):
    ''' 
    check if a point is inside (ref[0]- center - ref[1]) area
    return True if inside
        additional code = 0, on the first of edge ref_boundaries
        additional code = 1, on the second of edge ref_boundaries
        additional code = 2, in closed area of ref_boundaries
    return Flase if outside
    using math.isclose to avoid the error of floating point computation
    '''
    vector_a = np.subtract(ref_boundaries[0], center)
    vector_b = np.subtract(ref_boundaries[1], center)
    vector_p = np.subtract(point, center)

    ref_angle = signed_angle(vector_a, vector_b)
    cpt_angle = signed_angle(vector_a, vector_p)
    diff_angle = abs(ref_angle - cpt_angle)
    rel_tol = 0.0000001
    ret_result = False  # return result
    ret_code = 0  # return cod3
    if abs(cpt_angle) < rel_tol:
        # print ("the point is on edge 0")
        ret_result = True
        ret_code = 0
    elif diff_angle < rel_tol:
        # print ("the point is on edge 1")
        ret_result = True
        ret_code = 1

    elif ref_angle * cpt_angle < 0:  # diff side
        # print ("the point is outside ref area")
        ret_result = False
    else:
        # compare angles in unsigned
        abs_angle_sight = abs(ref_angle)
        abs_angle_check_pt = abs(cpt_angle)
        if abs_angle_sight > abs_angle_check_pt:
            # print ("The point is in closed ref angle")
            ret_result = True
            ret_code = 2
        else:
            # print ("the point is outside ref area_1")
            ret_result = False
    return ret_result, ret_code


def inside_closed_angle_area(point, center, ref_boundaries):
    ''' 
    check if a check_pt is inside closed area of (ref[0]- center - ref[1])
    return True if inside (not boundary)
    return False if outside
    '''
    in_status, in_code = inside_angle_area(point, center, ref_boundaries)
    return in_status and in_code == 2


def center_triangle(triangle):
    '''
    return center of a triangle
    '''
    return np.mean(triangle, axis=0)


def center_triangles(triangles):
    ''' 
    return a list of center of triangles
    '''
    return [center_triangle(triangle) for triangle in triangles]


def inside_triangle(point, triangle):
    '''
    if point is at the edge of triangle: return true and code = 0,1,2
    if point is inside closed triangle: return true and code = 4
    if point is outside triangle: return false
    '''
    ptin_0, code0 = inside_angle_area(point, triangle[0], [triangle[1], triangle[2]])
    ptin_1, code1 = inside_angle_area(point, triangle[1], [triangle[0], triangle[2]])
    ptin = np.logical_and(ptin_0, ptin_1)
    if ptin:
        code = code0 + code1
    else:
        code = 0
    return ptin, code


def point_belong_triangle(point, triangle):
    '''
    return true if point is vertex of triangle
    otherwise return false
    '''
    result = belong_triangle(point, triangle)
    return np.sum(result) > 0


def belong_triangle(point, triangle):
    '''
    return true if the given points is one of triangle's vertices
    '''
    pdist_a = point_dist(triangle[0], point)
    pdist_b = point_dist(triangle[1], point)
    pdist_c = point_dist(triangle[2], point)
    at_a = math.isclose(pdist_a, 0)
    at_b = math.isclose(pdist_b, 0)
    at_c = math.isclose(pdist_c, 0)
    return at_a, at_b, at_c


def mutual_edge(triangleA, triangleB):
    '''
    return true if 2 triangles have a mutual edge
    '''
    check_a = belong_triangle(triangleA[0], triangleB)
    check_b = belong_triangle(triangleA[1], triangleB)
    check_c = belong_triangle(triangleA[2], triangleB)
    result = np.logical_or(check_a, check_b)
    result = np.logical_or(result, check_c)
    return np.sum(result) == 2


def get_pairs_of_triangles(triangles):
    '''
    return list of the edges of triangles
    '''
    edges = []
    for i in range(len(triangles) - 1):
        for j in range(i + 1, len(triangles)):
            if mutual_edge(triangles[i], triangles[j]):
                edges.append([i, j])
    return edges


def mid_point(P, Q):
    '''
    return mid point of 2 point Q, P
    '''
    x = (P[0] + Q[0]) / 2
    y = (P[1] + Q[1]) / 2
    return x, y


def line_from_points(P, Q):
    '''
    return a, b, c of line from point P and Q
    where ax + by = c
    '''
    a = Q[1] - P[1]
    b = P[0] - Q[0]
    c = a * (P[0]) + b * (P[1])
    return a, b, c


def belong_line(point, line):
    '''
    check if the point is whether belong line or not
    '''
    a, b, c = line_from_points(line[0], line[1])
    return math.isclose(a * point[0] + b * point[1], c)


def point_dist(p, q):
    '''
    calculate distance of 2 point
    '''
    return math.hypot(q[0] - p[0], q[1] - p[1])


def line_across(line1, line2):
    '''
    check whether 2 lines are cross each others or not
    return cross point if they are
    return None if not
    '''
    is_pt = line_intersection(line1, line2)
    ret_result = None
    if is_pt is not None and inside_line_segment(is_pt, line1) and inside_line_segment(is_pt, line2):
        ret_result = is_pt
    return ret_result


def line_intersection(line1, line2):
    '''
    return intersection point of 2 lines
    if that point does not exist, return none
    '''
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return None
    # raise Exception('lines do not intersect')
    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


def get_middle_direction(center, radius, points):
    '''
    given 2 points and a center, this function finds a direction where is from center to middle 
    of 2 points with length of radius
    '''
    mid_pt = mid_point(points[0], points[1])
    pt_is = intersection(center[0], center[1], radius, [center, mid_pt])

    if inside_line_segment(mid_pt, [pt_is[0], center]):
        return pt_is[0]
    else:
        return pt_is[1]


def get_index_true(array_idx_status):
    ''' 
    return index of true elements of array
    '''
    return np.where(array_idx_status)[0]


def get_index_false(array_idx_status):
    ''' 
    return index of False elements
    '''
    not_status = np.logical_not(array_idx_status)
    return np.where(not_status)[0]


def print_pairs(message_ID, pairs):
    print("{0}, len: {1}".format(message_ID, len(pairs)))
    for pair in pairs:
        print(pair[0], pair[1])


def print_point(message_ID, point_x, point_y):
    print(message_ID)
    for i in range(len(point_x)):
        print("{0} {1}".format(point_x[i], point_y[i]))


def print_cpairs(message_ID, cpairs):  # print circle pairs
    print("{0}, len: {1}".format(message_ID, len(cpairs)))
    for pairs in cpairs:
        print(pairs)


def approximately_num(num):
    return float(format(float(num), '.10f'))

def normal_vector(linesegment, robot_radius):
    ''' 
    this function is to find a  normal vector of a line segment
    return a normal vector has lengh fo robot_raidus and follows rule of clockwise with left-hand 
    '''
    # calculate normal vector
    # follow clockwise and left-hand rule
    n_vector = (-(linesegment[1][1] - linesegment[0][1]), (linesegment[1][0] - linesegment[0][0]))
    
    # anti-clockwise
    #n_vector = ((linesegment[1][1] - linesegment[0][1]), -(linesegment[1][0] - linesegment[0][0]))
    
    # return normal vector has length of robot's radius
    rn_vector = unit_vector(n_vector) * robot_radius
    return rn_vector


def cal_bisector(ptA, ptMid, ptB, robot_radius):
    '''
    this function calculates normal vector of a bisector of 2 vector (mid,A) and (mid,B)
    normal vector is in form of line segnments
    '''
    #print("AMB ", ptA, ptMid, ptB)
    vectorA = np.subtract(ptMid, ptA)
    vectorB = np.subtract(ptMid, ptB)
    u_vectorA = unit_vector(vectorA)
    u_vectorB = unit_vector(vectorB)
    vector_bisector = np.add(u_vectorA, u_vectorB)
    vector_bisector = unit_vector(vector_bisector) * robot_radius
    sa = signed_angle(vectorA, vectorB)
    if sa < 0:
        vector_bisector = - vector_bisector
    vector_bisector = np.add(ptMid, vector_bisector)
    vector_AB = np.subtract(u_vectorA, u_vectorB)
    limit_ls = np.add(vector_bisector, vector_AB)
    #print("___________", vector_bisector, limit_ls)
    line_segment_bs = [tuple(vector_bisector), tuple(limit_ls)]
    return line_segment_bs


def get_intersections_2circles(center_0, radius_0, center_1, radius_1):
    '''
    get intersection of 2 circles
    '''
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1
    x0, y0 = center_0
    x1, y1 = center_1
    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    # non intersecting
    if d > radius_0 + radius_1:
        return None
    # One circle within other
    if d < abs(radius_0 - radius_1):
        return None
    # coincident circles
    if d == 0 and radius_0 == radius_1:
        return None
    else:
        a = (radius_0 ** 2 - radius_1 ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(radius_0 ** 2 - a ** 2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return (x3, y3), (x4, y4)

''' check whether a point is inside a polygon or not using ray_tracing_method'''
def point_inside_polygons(pt, polygons):
    result = False
    for polygon in polygons:
        if ray_tracing_method(pt[0], pt[1], polygon):
            return True
    return False

# Ray tracing
def ray_tracing_method(x,y,poly):

    n = len(poly)
    inside = False

    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside


''' prepare name figure '''
def set_figure_name(map_name: str = "", range=0, start=0, goal=0, picking_strategy=0, ranking_function=0, RRTx=False, fig_type= None):
    if map_name.__contains__(".\\"):
        map_name = map_name[2]
    return_name =   "map{0}".format(map_name) + \
                    "_start_{0}_{1}".format(start[0],start[1]) + \
                    "_goal_{0}_{1}".format(goal[0],goal[1]) + \
                    "_range_{0}".format(range)
    if RRTx:
        return_name += "_RRTx"
    else:    
        return_name +=  "_{0}".format(ranking_function) +  \
                        "_{0}".format(picking_strategy)
    if fig_type == ".png":
        return_name += ".png"
        
    return return_name