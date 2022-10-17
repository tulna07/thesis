from Robot_math_lib import *
from Robot_draw_lib import *
from Program_config import *
from Obstacles import Obstacles
import numpy as np

rel_tol = 0.0000001

''' divide sights, which are in orderd R-C-R-C (C inside Reference sight), into smaller sights
'''
def divide_sight_R_C_R_C(center, A, B, C, D):  # line [A,C] for R, [B, D] for C, B inside AC , D outside [A,B]
    # divide sight into 2 different parts [R_C0_R]
    # check if D is right hand sight of A-C (meaning C is inside B-D )
    C_inside, _ = inside_angle_area(point=C, center=center, ref_boundaries=[B,D])
    if C_inside: # A-B-C-D
        BD_CO_point = line_intersection([B, D], [C, center])
    else:   # D-A-B-C
        BD_AO_point = line_intersection([B, D], [A, center])
    AC_BO_point = line_intersection([A, C], [B, center])
    
    if point_dist(center, AC_BO_point) < point_dist(center, B):  # A - C is closer center than B
        if C_inside:
            divided_sight = [[A, C], [BD_CO_point, D]]
        else:   # A inside
            divided_sight = [[A, C], [BD_AO_point, D]]
    else:
        if C_inside:
            divided_sight = [[A, AC_BO_point], [B, D]]
        else:   # A_inside
            divided_sight = [[AC_BO_point, C], [B, D]]


    return divided_sight


def divide_sight_R_C_C_R(center, r, c):  # divide  R-C-C-R into 3 parts
    is_c0R = line_intersection([center, c[0]], [r[0], r[1]])
    is_c1R = line_intersection([center, c[1]], [r[0], r[1]])
    if inside_line_segment(is_c0R, [r[0], is_c1R]):  # R0 isC0R isC1R R1
        divided_sight = [[r[0], is_c0R], [c[0], c[1]], [is_c1R, r[1]]]
    else:  # R0 isC1R isC0R R1
        divided_sight = [[r[0], is_c1R], [c[0], c[1]], [is_c0R, r[1]]]
    return divided_sight


def detect_blind_sight(center, ref_sight, check_sight):
    ''' 
    check sight  --> C0, C1
    reference sight -->  R0 R1 
    '''
    r = ref_sight
    c = check_sight

    r_blind = False  # true if reference sight is blind sight
    c_blind = False  # true if check_sight is blind sight
    d_sight = False  # true if there need to divide (ref, check) area in to 2 parts
    #                (ref_sub_part, common_ref/check_part, check_sub_path)
    #             then remove ref, check sight , and create 2 new sight

    divided_sight = []

    # get inside status of c1, c0
    c0_in, c0_code = inside_angle_area(c[0], center, r)
    c1_in, c1_code = inside_angle_area(c[1], center, r)
    #print("detect_blind_sight___status:", c0_in, c0_code, c1_in, c1_code, r, c)
    ''' 
        function inside_angle_area returns true if inside, false if outside
        if true, the additional code is provided,
            code = 0, in the 1st edge of area
            code = 1: in the 2nd edge of area
            code = 2: inside closed area
            
        reference sight fully coverages check sight
            -> if case
        reference sight and check sight have mutual edge: 
            -> elif pointC_0 == 0 or pointC_1 == 0
        reference sight and check sight have mutual area 
        # C0 inside, C1 outside
            -> elif pointC_0 > 0 and pointC_1 < 0:
        reference sight and check sight have mutual area 
        # C1 inside, C0 outside
            -->elif pointC_0 < 0 and pointC_1 > 0:
        reference sight and check sight are 2 distinct area 
            ---> else:   # both Check C0 C1 are outside
    '''
    if c0_in and c1_in:
        # check sight completely inside reference sight
        #print ("C1 inside, C2 inside _123")
        if c0_code <= 1 and c1_code == 2:  # mutual c0 while c1 inside
            # check if c1 is inside (r0, center r1)
            c1_in, _ = inside_angle_area(c[1], r[0], [center, r[1]])
            if c1_in:
                is_pt = line_intersection([center, c[1]], r)
                if c0_code == 0:
                    divided_sight = [[is_pt, r[1]], c]
                else:
                    divided_sight = [[r[0], is_pt], c]
                d_sight = True
            else:
                c_blind = True
        elif c0_code == 2 and c1_code <= 1:  # mutual c1 while c0 inside
            # check if c1 is inside (r0, center r1)
            c0_in, _ = inside_angle_area(c[0], r[0], [center, r[1]])
            if c0_in:
                is_pt = line_intersection([center, c[0]], r)
                if c1_code == 0:
                    divided_sight = [[is_pt, r[1]], c]
                else:
                    divided_sight = [[r[0], is_pt], c]
                d_sight = True
            else:
                c_blind = True
        elif (c0_code == 0 and c1_code == 1) or (c0_code == 1 and c1_code == 0):  # c1 and c2 are in r0 and r1 edges

            dp_cc = point_dist(center, c[0]) + point_dist(center, c[1])
            dp_cr = point_dist(center, r[0]) + point_dist(center, r[1])
            if dp_cc < dp_cr:  # c0 c1 is closer
                r_blind = True
            else:
                c_blind = True
        else:
            # if check line is closer to center than reference line
            #  then dividing reference into 3 parts Rs-C0, C0-C1, C1-Re
            # else true sight is reference sight
            cin, _ = inside_angle_area(c[0], r[0], [center, r[1]])
            if cin:
                d_sight = True
                divided_sight = divide_sight_R_C_C_R(center, r, c)
            else:
                c_blind = True

    elif (c0_in and c0_code < 2) or (c1_in and c1_code < 2):  # ether C0 or C1 is at the boundary segment
        '''
        check if check sight fully coverages ref_sight
        '''
        #print ("C1 inside, C2 inside _124")
        r0_in, _ = inside_angle_area(r[0], center, c)
        r1_in, _ = inside_angle_area(r[1], center, c)
        r0_inccc, _ = inside_angle_area(r[0], c[0], [center, c[1]])
        r1_inccc, _ = inside_angle_area(r[1], c[0], [center, c[1]])
        # print (r0_in, r1_in, r0_inccc, r1_inccc)
        if r0_in and r1_in:  # ref sight is inside check sight
            # print ("ref sight is inside check sight")
            # check if r is closer than check sight
            if r0_inccc and r1_inccc:  # reference sight is closer
                # print ("reference sight is closer")
                isR0C = line_intersection([center, r[0]], c)
                isR1C = line_intersection([center, r[1]], c)
                d_sight = True
                if not c0_in and c1_code == 0:  # c0 R1 R0=C1
                    # [Reference sight] - [C0: isR1C] in this order
                    divided_sight = [r, [c[0], isR1C]]
                elif not c0_in and c1_code == 1:  # c0 R0 R1=C1
                    # [Reference sight] - [C0: isR0C] in this order
                    divided_sight = [r, [c[0], isR0C]]
                elif not c1_in and c0_code == 0:  # c1 R1 R0=C0
                    # [Reference sight] - [C1: isR1C] in this order
                    divided_sight = [r, [c[1], isR1C]]
                elif not c1_in and c0_code == 1:  # c1 R0 R1=C1
                    # [Reference sight] - [C1: isR0C] in this order
                    divided_sight = [r, [c[1], isR0C]]
            else:
                # print ("reference sight is not closer")
                r_blind = True

    elif c0_in and not c1_in:
        ''' 
        check if R0 is inside area of [R1, C1]
        if R0 is outside then R1 is inside area of [R0, C1] for sure
        '''
        #print ("C1 inside, C2 outside _125")
        d_sight = True

        r0_in, _ = inside_angle_area(r[0], center, [r[1], c[1]])
        #print ("R0 in:", r0_in)
        if r0_in:  # R1 C0 (R0) C1
            divided_sight = divide_sight_R_C_R_C(center, r[1], c[0], r[0], c[1])
        else:  # R0 C0 (R1) C1
            divided_sight = divide_sight_R_C_R_C(center, r[0], c[0], r[1], c[1])

    elif not c0_in and c1_in:
        ''' check if R0 is inside area of [R1, C0]
            if R0 is outside then R1 is inside area of [R0, C0] for sure
        '''
        d_sight = True
        #print ("C1 outside, C2 inside _222")
        r0_in, _ = inside_angle_area(r[0], center, [r[1], c[0]])
        if r0_in:  # R1 C1 (R0) C0
            divided_sight = divide_sight_R_C_R_C(center, r[1], c[1], r[0], c[0])
        else:  # R0 C1 (R1) C0
            divided_sight = divide_sight_R_C_R_C(center, r[0], c[1], r[1], c[0])

    else:  # both Check C0 C1 are outside
        # print ("C1 outside, C2 outside _333")
        # print ("C1 outside, C2 outside _333")
        # Check if ref_sight is inside Check_sight,
        rin, _ = inside_angle_area(r[0], center, [c[1], c[0]])
        if rin:  # R inside C0- center - C1
            rin, _ = inside_angle_area(r[0], c[0], [center, c[1]])
            if rin:  # R inside angle C0- C1 - center
                d_sight = True
                divided_sight = divide_sight_R_C_C_R(center, c, r)
            else:  # R is outside
                r_blind = True

    return divided_sight, r_blind, c_blind, d_sight


def remove_blind_lss(center, b_lss):
    '''
    remove blind line segments in boundary line segments
    to get true closed sights
    '''
    closed_sights = b_lss

    i = 0  # start with index = 0
    while i < len(closed_sights) - 1:
        j = i + 1
        while j < len(closed_sights):

            #print ("__sight: ref{0} and check{1}".format(closed_sights[i],closed_sights[j]) )
            ds, r_blind, c_blind, d_sight = detect_blind_sight(center, closed_sights[i], closed_sights[j])
            # princlosed_sights(" [Local] divide sight", ds)
            #print ("status of sight: r {0}, c {1}, d {2}".format(r_blind, c_blind, d_sight))
            if d_sight:  # separate into 2/3 new sight
                closed_sights[i] = ds[0]
                closed_sights[j] = ds[1]

                if len(ds) == 3:
                    closed_sights.append(ds[2])
                i = i - 1
                break
            elif c_blind:  # remove check sight cause it's blind sight
                closed_sights.pop(j)
                j = j - 1
            elif r_blind:  # replace ref sight by check sight cause it's blind sight
                # update i so need to recheck
                closed_sights[i] = closed_sights[j]
                closed_sights.pop(j)
                i = i - 1
                break

            j += 1
        i += 1
    return closed_sights


def get_closed_sights_from_blss(center, b_lss):
    '''
    get closed sight from boundary line segments by remove blind line segments in blss
    '''
    closed_sights = []

    if len(b_lss) <= 1:  # if there is only 1 boundary line segment then it is closed_sights
        closed_sights = b_lss
    else:
        closed_sights = remove_blind_lss(center, b_lss)
    return closed_sights

''' find all intersection among robot_local (Circle) and obstacles,
return list of pairs of intersection points
'''
def get_local_obstacles_boundary(center, robot_vision, obstacles):
    local_obs_boundary = []
    for obstacle_part in obstacles:
        local_ob_boundary = get_local_obstacle_boundary(center, robot_vision, obstacle_part)
        local_obs_boundary.extend(local_ob_boundary)
    return local_obs_boundary


def get_local_obstacle_boundary(center, robot_vision, obstacles):
    ''' 
    find all boundary pairs among all obstacle line segments and circle 
    '''
    x, y = center
    b_lss = []
    for i in range(len(obstacles) - 1):
        ptA = obstacles[i]
        ptB = obstacles[i + 1]
        if belong_line(center, [ptA, ptB]): # robot stands too closed obstacles
            continue
        intersection_points = intersection(x, y, robot_vision, [ptA, ptB])

        if len(intersection_points) > 0:
            ''' find boundary pair between a single line segment and circle '''
            b_pts = []  # boundary points

            for point in intersection_points:
                pt_in = inside_line_segment(point, [ptA, ptB])
                # print ("inside_ls status *%# ", pt_in, " of point ", point)
                if pt_in:  # found intersection point is inside the line segment
                    b_pts.append(point)
                else:  # intersection point is not inside the line segment
                    ptA_in = inside_line_segment(ptA, intersection_points)

                    if ptA_in:  # found intersection point is inside the line segment
                        b_pts.append(ptA)
                    ptB_in = inside_line_segment(ptB, intersection_points)
                    if ptB_in:  # found intersection point is inside the line segment
                        b_pts.append(ptB)

            if len(b_pts) > 0:

                pd = point_dist(b_pts[0], b_pts[1])
                if not math.isclose(pd, 0):  # make sure p1 != p2
                    b_lss.append([b_pts[0], b_pts[1]])
    return b_lss

''' this function (by some how) get boundary points by Hoai_An's assumption theory'''
def get_boudary_points_theory(center, robot_vision, ob, goal):
    x, y = center
    # intersection points
    boundary_pts = []
    for obstacle_part in ob:
        for i in range(len(obstacle_part) - 1):
            ptA = obstacle_part[i]
            ptB = obstacle_part[i + 1]

            is_points = intersection(x, y, robot_vision, [ptA, ptB])

            if len(is_points) == 0:
                continue
            else:
                if is_points[0] != is_points[1]:
                    if inside_line_segment(is_points[0], [ptA, ptB]):
                        boundary_pts.append(is_points[0])
                    if inside_line_segment(is_points[1], [ptA, ptB]):
                        boundary_pts.append(is_points[1])
    return np.array(boundary_pts)

''' get open sights and its active point according to Hoai_An's assumption theory'''
def get_open_sights_in_active_arc_theory(arc_pts, parent_arc, robot_vision, ob, center):
    open_sights = []

    # when a goal is too close, not parent are will be provided
    # then no open_sights needed 
    if parent_arc is None:
        return open_sights, arc_pts

    # get all arc points including limited parent_arc points
    inside_Obstacle = [point_inside_polygons(arc_lim, ob) for arc_lim in parent_arc]
    outside_Obstacle = np.invert(inside_Obstacle)
    if len(arc_pts) != 0:
        arc_pts = np.vstack((arc_pts, parent_arc[outside_Obstacle]))
    else:
        arc_pts = parent_arc[outside_Obstacle]

    # sort all arc points by its angle delegation, which is rorated to align X-axis
    # get mid point parent arc
    pArc_mpt = mid_point(parent_arc[0], parent_arc[1])
    # calculate angle forming by (center -> midpt) and (X-axis)
    vector_mpt = np.subtract(pArc_mpt, center)
    vector_Xaxis_unit = [1,0]
    mpt_signed_angle = signed_angle(vector_Xaxis_unit, vector_mpt)
    # cat rotating angle to satify midpoint is along side Y-axis
    mpt_angle_diff = mpt_signed_angle - math.pi/2
        
    rotated_arc_pts = [rotate_vector_center(center, pt, mpt_angle_diff) for pt in arc_pts]
    rotated_arc_pts_vector = [np.subtract(pt, center) for pt in rotated_arc_pts]
    rotated_arc_pts_angle = np.array([signed_angle([1,0],vA) for vA in rotated_arc_pts_vector])
        
    # sort arc point accooding to its angle
    inds = rotated_arc_pts_angle.argsort()
    arc_pts = arc_pts[inds]

    for i in range(int(len(arc_pts)/2)):
        open_sight = init_open_cpair(center, robot_vision, arc_pts[i*2], arc_pts[i*2+1])
        open_sights.append(open_sight)

    return np.array(open_sights), arc_pts

def get_closed_sights(Robot, obstacles):
    # get boundary line segments where is limited by obstacles
    local_obs_boundary = get_local_obstacles_boundary(Robot.coordinate, Robot.vision_range, obstacles)
    if print_local_obstacles_boundary:
        print_pairs("local_obs_boundary", local_obs_boundary)

    closed_sights = get_closed_sights_from_blss(Robot.coordinate, local_obs_boundary)
    if print_closed_sights:
        print_pairs("print_closed_sights", closed_sights)

    return closed_sights


def inside_global_true_sight(pt, radius, traversal_path):
    result = [inside_local_true_sight(pt, x, radius, tsight) for x, tsight, _ in traversal_path]
    ret_result = np.sum(result) > 0
    # print ("inside global sight result: ", result, ", return :", ret_result)
    return ret_result


def inside_local_true_sight(pt, center, radius, t_sight):
    outside = True
    if point_dist(pt, center) < radius:  # inside vision area
        outside = False
        # check if pt is inside true sight angle
        inside_open_sight = True
        visible = False
        for ts_pair in t_sight:
            pt_in = inside_angle_area(pt, center, ts_pair)[0]
            if pt_in:  # inside angle of true sight
                inside_open_sight = False
                pt_in = inside_angle_area(pt, ts_pair[0], (center, ts_pair[1]))[0]
                if pt_in:
                    visible = True
                    # print ("found close ", pt, t_sight)
                    return True
                else:
                    # print ("found in blind sight ", pt, t_sight)
                    return False
        # print ("found open", pt, t_sight)
        return True
    else:
        # print ("Outside", pt)
        return False
    # return not outside and (inside_open_sight or visible)


def get_ref_csight_lss(center, radius, closed_sights):
    '''
    get close sight line segments where is intersection of close points and circle
    '''
    cspt0, cspt1 = closed_sights
    x, y = center

    cpoints = []
    # circle point 0
    cpt_is = intersection(x, y, radius, [cspt0, center])

    if inside_line_segment(cpt_is[0], [cspt0, center]):
        cpoints.append(cpt_is[0])
    else:
        cpoints.append(cpt_is[1])

    # circle point 1
    cpt_is = intersection(x, y, radius, [cspt1, center])
    if inside_line_segment(cpt_is[0], [cspt1, center]):
        cpoints.append(cpt_is[0])
    else:
        cpoints.append(cpt_is[1])

    return [cpoints[0], cpoints[1]]


def get_csight_lss(center, ref_csight_linesegments):
    '''
    from reference close sight line segments, extend nearby line segments into one
    '''
    result = []
    # clone new reference closed sight line segments
    rcsLSs = []
    rcsLSs.extend(ref_csight_linesegments)
    if len(rcsLSs) == 1:
        result = [[rcsLSs[0][0], rcsLSs[0][1], True]]
    else:
        i = 0
        while i < len(rcsLSs) - 1:
            r_cpairs = rcsLSs[i]  # reference pair
            j = i + 1
            while j < len(rcsLSs):
                c_cpairs = rcsLSs[j]
                result = mutual_point(r_cpairs, c_cpairs)
                # print (result)
                new_cpairs = []
                if result[0][0]:
                    new_cpairs = [r_cpairs[1], c_cpairs[1]]
                elif result[0][1]:
                    new_cpairs = [c_cpairs[0], r_cpairs[1]]
                elif result[1][0]:
                    new_cpairs = [r_cpairs[0], c_cpairs[1]]
                elif result[1][1]:
                    new_cpairs = [r_cpairs[0], c_cpairs[0]]

                if len(new_cpairs) > 0:
                    rcsLSs[i] = new_cpairs
                    rcsLSs.pop(j)
                    i = i - 1
                    break
                j = j + 1
            i = i + 1

        # once there is only one closed sight line segment
        # check if its whether true or fake
        if len(rcsLSs) == 1:
            ios = is_open_sight(center, rcsLSs[0], ref_csight_linesegments)
            if ios:  # csight line segment is true closed
                result = [[rcsLSs[0][0], rcsLSs[0][1], False]]
            else:  # csight line segment is complement
                result = [[rcsLSs[0][0], rcsLSs[0][1], True]]
        else:
            result = rcsLSs
    return result


def divide_open_cpair(center, inangle, vs, ve):
    return_pairs = []
    # print ("divide_open_cpair: ", math.degrees(inangle))
    angle = abs(inangle)
    if angle >= math.pi * 4 / 3:  # 240 degree
        
        if divide_6:
            r_angle = angle / 6
            # print ("divide into 3 parts, with angle = ", math.degrees(r_angle))
            v1 = rotate_vector_center(center, vs, -r_angle)
            v2 = rotate_vector_center(center, vs, -2 * r_angle)
            v3 = rotate_vector_center(center, vs, -3 * r_angle)
            v4 = rotate_vector_center(center, vs, -4 * r_angle)
            v5 = rotate_vector_center(center, vs, -5 * r_angle)
            return_pairs.append([vs, v1])
            return_pairs.append([v1, v2])
            return_pairs.append([v2, v3])
            return_pairs.append([v3, v4])
            return_pairs.append([v4, v5])
            return_pairs.append([v5, ve])
        else:   # default divide into 3 parts
            r_angle = angle / 3
            # print ("divide into 3 parts, with angle = ", math.degrees(r_angle))
            v1 = rotate_vector_center(center, vs, -r_angle)
            v2 = rotate_vector_center(center, vs, -2 * r_angle)
            return_pairs.append([vs, v1])
            return_pairs.append([v1, v2])
            return_pairs.append([v2, ve])
    elif angle >= math.pi * 2 / 3:  # 120 degree:
        # divide into 2 parts
        r_angle = angle / 2
        # print ("divide into 2 parts, with angle = ", math.degrees(r_angle))
        v1 = rotate_vector_center(center, vs, -r_angle)
        return_pairs.append([vs, v1])
        return_pairs.append([v1, ve])
    else:  # <= 120 degree
        return_pairs.append([vs, ve])
    return return_pairs


def divide_open_cpair_complement(center, cpair):
    return_pairs = []
    angle, vs, ve = get_angle_info(center, cpair[0], cpair[1])
    angle = 2 * math.pi - abs(angle)
    return_pairs = divide_open_cpair(center, angle, ve, vs)
    return return_pairs


def init_open_cpair(center, radius, ptS, ptE):
    # get a middle direction of a open sight, this direction is open point 
    oPt = get_middle_direction(center, radius, [ptS, ptE])
    return [ptS, ptE, oPt]


def get_osight_linesegments(center, radius, goal, close_cpairs):
    # declare open circle pairs for open sight
    o_cpairs = []

    # clone new close circle pairs
    c_cpairs = []
    c_cpairs.extend(close_cpairs)

    if len(c_cpairs) == 0:  # no obstacle detected
        # print ("No obstacle detected")
        # check if goal is at center
        if not math.isclose(point_dist(goal, center), 0):
            vector_cg_unit = unit_vector(np.subtract(goal, center))
        else:
            vector_cg_unit = [1, 0]

        vs = np.multiply(vector_cg_unit, radius) + center
        if divide_6:
            vs = rotate_vector_center(center, vs, math.pi / 6)
        else:
            vs = rotate_vector_center(center, vs, math.pi / 3)

        pairs_extend = divide_open_cpair_complement(center, [vs, vs])
        for pair in pairs_extend:
            o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))

    elif len(c_cpairs) == 1:  # only a obstacle detected
        # print ("only 1 obstacle detected")
        # print ("close cpairs", c_cpairs)
        if c_cpairs[0][2]:  # TRUE: True close detected then open_sight is its complement
            pairs_extend = divide_open_cpair_complement(center, c_cpairs[0])
        else:  # FALSE, this FAKE close sight is open_sight False
            # if open sight is 0 degree angle, then ignore it
            dist = point_dist(c_cpairs[0][0], c_cpairs[0][1])
            if math.isclose(dist, 0):
                pairs_extend = []
            else:
                angle, vs, ve = get_angle_info(center, c_cpairs[0][0], c_cpairs[0][1])
                pairs_extend = divide_open_cpair(center, angle, vs, ve)

        for pair in pairs_extend:
            o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))

    else:
        # print ("more than 2 obstacles detected")
        i = 0
        while i < len(c_cpairs) - 1:
            j = i + 1
            while j < len(c_cpairs):
                found, ridx, cidx = open_sight(center, i, j, c_cpairs)
                # print ("_Pair r {0} c {1}".format(c_cpairs[i], c_cpairs[i]) )
                # print ("__Open result {0}, index {1} {2}\n".format(found, ridx, cidx) )
                if found:
                    ptA = c_cpairs[i][ridx]
                    ptB = c_cpairs[j][cidx]
                    angle, vs, ve = get_angle_info(center, ptA, ptB)
                    pairs_extend = divide_open_cpair(center, angle, vs, ve)

                    for pair in pairs_extend:
                        o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))
                    c_cpairs[i] = [c_cpairs[i][1 - ridx], c_cpairs[j][1 - cidx]]
                    c_cpairs.pop(j)
                    continue  # begin new check with same j
                j = j + 1
            i = i + 1
            break

        if len(c_cpairs) > 0:  # the last 1
            ptA, ptB = c_cpairs[0]
            # get all close point, which are from close pairs, but ptA, ptB
            ios = is_open_sight(center, [ptA, ptB], close_cpairs)

            if ios:  # true open pair
                angle, vs, ve = get_angle_info(center, ptA, ptB)
                pairs_extend = divide_open_cpair(center, angle, vs, ve)
            else:
                pairs_extend = divide_open_cpair_complement(center, c_cpairs[0])

            for pair in pairs_extend:
                o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))

    return o_cpairs


def get_allpts_except(all_pairs, e_pairs):
    '''
    get all points, which are from pairs, but except points from except pair
    '''
    allPts = []
    for pair in all_pairs:
        allPts.append(pair[0])
        allPts.append(pair[1])
    allPts.remove(e_pairs[0])
    allPts.remove(e_pairs[1])
    return allPts


def is_open_sight(center, cpair, all_pairs):
    # print ("___Pair {0}, all points {1}".format(cpair, all_pairs))
    # print ("___Len checking point", len (all_pairs))
    all_pts = []
    for pair in all_pairs:
        all_pts.append(pair[0])
        all_pts.append(pair[1])
    # for point in all_pts:
    #    print ("___ points", point )
    in_status = [inside_closed_angle_area(pt, center, cpair) for pt in all_pts]

    ret_result = sum(in_status) == 0  # sum true == 0, all must outside

    # print ("____inside status: ", in_status, " -> result: ", ret_result)
    return ret_result


def is_open_sight_pairs(center, pt_i, pt_j, i, j, all_pairs):
    # i for ref, j for check
    r_cpairs = all_pairs[i]
    c_cpairs = all_pairs[j]
    ret_result = is_open_sight(center, [r_cpairs[pt_i], c_cpairs[pt_j]], all_pairs)

    # print ("_Check pair (r, c): {0} {1}, index i: {2} j: {3}, result: {4}".format(r_cpairs[pt_i],c_cpairs[pt_j], pt_i, pt_j,  ret_result) )

    return ret_result


def open_sight(center, i, j, close_cpairs):
    # i is always greater than j

    found = True
    ridx, cidx = (0, 0)
    # i first for ref, j second for check
    if is_open_sight_pairs(center, 0, 1, i, j, close_cpairs):
        ridx, cidx = (0, 1)
    elif is_open_sight_pairs(center, 1, 0, i, j, close_cpairs):
        ridx, cidx = (1, 0)
    elif is_open_sight_pairs(center, 1, 1, i, j, close_cpairs):
        ridx, cidx = (1, 1)
    elif is_open_sight_pairs(center, 0, 0, i, j, close_cpairs):
        ridx, cidx = (0, 0)
    else:
        found = False
    return found, ridx, cidx


def point_isclose(q, p):
    return point_dist(p, q) < rel_tol


def mutual_point(pts, ref_pts):
    dist_p0r0 = point_isclose(pts[0], ref_pts[0])
    dist_p0r1 = point_isclose(pts[0], ref_pts[1])
    dist_p1r0 = point_isclose(pts[1], ref_pts[0])
    dist_p1r1 = point_isclose(pts[1], ref_pts[1])

    result_0 = [dist_p0r0, dist_p0r1]
    result_1 = [dist_p1r0, dist_p1r1]
    return [result_0, result_1]


def get_open_sights(Robot, goal, closed_sights):
    # get reference closed line segments
    ref_csight_lss = [get_ref_csight_lss(Robot.coordinate, Robot.vision_range, c_sight) for c_sight in closed_sights]

    if print_ref_csight_line_segments:
        print("print_ref_csight_linesegments", ref_csight_lss)

    csights_lss = get_csight_lss(Robot.coordinate, ref_csight_lss)
    if print_csight_line_segments:
        print_cpairs("print_closed_linesegments", csights_lss)

    open_sights = get_osight_linesegments(Robot.coordinate, Robot.vision_range, goal, csights_lss)
    if print_open_sights:
        print_cpairs("print_open_sights", open_sights)

    return open_sights


def scan_around(Robot, obstacles: Obstacles, goal):
    '''
    this function is to scan obstacles 
    return true sight = (closed sights and open sights)
    '''
    if obstacles.enable_config_space:
        obstacles_data = obstacles.config_space
    else:
        obstacles_data = obstacles.obstacles
    closed_sights = get_closed_sights(Robot, obstacles_data)
    open_sights = get_open_sights(Robot, goal, closed_sights)
    return closed_sights, open_sights