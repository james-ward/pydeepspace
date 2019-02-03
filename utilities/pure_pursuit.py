import math
import numpy as np

# from magicbot import tunable


class PurePursuit:
    """
    Pure Pursuit controller for navigation with absolute waypoints
    Uses the method outlined here with some changes to be suitible for a swervedrive
    https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
    """

    # look_ahead = tunable(0) # m
    # speed_modifier = tunable(0)
    # ending_tolerance = tunable(0) # m

    def __init__(self, look_ahead):
        self.waypoints = []
        self.current_waypoint_number = 0
        self.look_ahead = look_ahead
        self.completed_path = False
        self.distance_traveled = 0
        self.trapezoid = True
        self.max_speed = 3  # <-- change to actuall max speed when avaliable

    def find_intersections(self, waypoint_start, waypoint_end, robot_position):
        """
        Find the intersection/s between our lookahead distance and path.
        http://mathworld.wolfram.com/Circle-LineIntersection.html
        NOTE: this will return the intersections in global co-ordinates
        """
        x_1, y_1 = waypoint_start[0], waypoint_start[1]
        x_2, y_2 = waypoint_end[0], waypoint_end[1]
        robot_x, robot_y, _ = robot_position
        x_2 -= robot_x
        x_1 -= robot_x
        y_2 -= robot_y
        y_1 -= robot_y
        segment_end = np.array((x_2, y_2))

        d_x = x_2 - x_1
        d_y = y_2 - y_1
        d_r = math.sqrt(d_x ** 2 + d_y ** 2)
        D = x_1 * y_2 - x_2 * y_1
        r = self.look_ahead
        discriminent = r ** 2 * d_r ** 2 - D ** 2

        if discriminent >= 0:  # if an intersection exists
            intersection_1 = np.zeros((2))
            intersection_2 = np.zeros((2))
            sqrt_discriminent = math.sqrt(discriminent)

            right_x = self.sgn(d_y) * d_x * sqrt_discriminent
            left_x = D * d_y
            right_y = abs(d_y) * sqrt_discriminent
            left_y = -1 * D * d_x
            denominator = d_r ** 2
            if denominator == 0:
                print("Pursuit: caught division by zero")
                return
            intersection_1[0] = (left_x + right_x) / denominator
            intersection_1[1] = (left_y + right_y) / denominator
            if discriminent == 0:  # if we are tangent to our path
                return intersection_1
            intersection_2[0] = (left_x - right_x) / denominator
            intersection_2[1] = (left_y - right_y) / denominator
            if np.linalg.norm((intersection_1) - (segment_end)) < np.linalg.norm(
                (intersection_2) - (segment_end)
            ):
                return intersection_1
            else:
                return intersection_2

        else:
            print("No intersection found")

    def build_path(self, waypoints):
        current_waypoint = 0
        """
        Take in a list of waypoints used to build a path.
        The waypoints must be a tuple (x, y, speed), this method will
        create waypoints with these co-ordinates and distance
        along the path from the start of the trajectory.
        """
        self.last_robot_x = waypoints[0][0]
        self.last_robot_y = waypoints[0][1]
        self.completed_path = False
        self.distance_traveled = 0
        self.waypoints = []
        waypoint_distance = 0
        previous_waypoint = waypoints[0]
        for waypoint in waypoints:
            x, y, speed = waypoint
            waypoint_distance += math.hypot(
                x - previous_waypoint[0], y - previous_waypoint[1]
            )
            previous_waypoint = waypoint
            self.waypoints.append((x, y, speed, waypoint_distance))
        self.current_waypoint_number = 0
        print(self.waypoints)
        if current_waypoint <= len(self.waypoints):
            end_point_x = (
                self.waypoints[current_waypoint + 1][0] - self.waypoints[current_waypoint][0]
            )
            end_point_y = (
                self.waypoints[current_waypoint + 1][1] - self.waypoints[current_waypoint][1]
            )
            angle = math.atan2(end_point_y, end_point_x)
            sin_x = math.sin(angle)
            cos_y = math.cos(angle)
            x, y = waypoints[current_waypoint][0], waypoints[current_waypoint][1]
            waypoint_distance = math.hypot(
                x - previous_waypoint[0], y - previous_waypoint[1]
            )

            if self.trapezoid:
                accel, decel = self.generate_traezoidal_distances(
                    self.max_speed, self.waypoints[current_waypoint][2], 0.5, 0.5
                )  # ^ enter actuall acceleration and decleration values when avaliable ^
                start_cruise = (
                    (sin_x * accel) + waypoints[current_waypoint][0],
                    (cos_y * accel) + waypoints[current_waypoint][1],
                )
                start_decel = (
                    (sin_x * decel) + waypoints[current_waypoint][0],
                    (cos_y * decel) + waypoints[current_waypoint][1],
                )
                self.waypoints.insert(
                    current_waypoint + 1,
                    (start_cruise[0], start_cruise[1], self.max_speed, waypoint_distance),
                )
                self.waypoints.insert(
                    current_waypoint + 2,
                    (start_decel[0], start_decel[1], self.max_speed, waypoint_distance),
                )
                current_waypoint += 3
                print("new waypoints")
                print("")
                print(waypoints)
                print("")
            else:
                dist_to_midpoint = self.generate_traezoidal_distances(
                    self.max_speed, self.waypoints[current_waypoint][2], 0.5, 0.5
                )  # ^ enter actuall acceleration and decleration values when avaliable ^
                midpoint = (
                    (sin_x * dist_to_midpoint) + waypoints[current_waypoint][0],
                    (cos_y * dist_to_midpoint) + waypoints[current_waypoint][1]
                )
                self.waypoints.insert(
                    current_waypoint + 1, (midpoint[0], midpoint[1], self.max_speed, waypoint_distance)
                )
                current_waypoint += 2

    def compute_direction(
        self, robot_position, segment_start, segment_end, distance_along_path
    ):
        """Find the goal_point and convert it to relative co-ordinates"""
        goal_point = self.find_intersections(segment_start, segment_end, robot_position)
        if goal_point is None:
            # if we cant find an intersection between the look_ahead and path
            # use the next waypoint as our goal point
            goal_point = segment_end[:2]
        # print(goal_point)
        if self.distance_traveled >= segment_end[3]:
            # if we have reached the end of our current segment
            self.current_waypoint_number += 1
            print("changed segment")
        goal_point = goal_point / np.linalg.norm(goal_point)
        return goal_point

    def sgn(self, number):
        """Returns the sign of a number, 0 is positive"""
        if number < 0:
            return -1
        else:
            return 1

    def distance_along_path(self, robot_position):
        """
        Find the robots position on the path using odometry.
        Every timestep, add the distance the robot has travelled to a
        running total used to check for waypoints.
        """
        robot_x, robot_y, _ = robot_position
        self.distance_traveled += math.hypot(
            robot_x - self.last_robot_x, robot_y - self.last_robot_y
        )
        self.last_robot_x = robot_x
        self.last_robot_y = robot_y
        # print(self.distance_traveled)
        return self.distance_traveled

    def find_speed(
        self,
        start_path_distance,
        end_path_distance,
        start_speed,
        end_speed,
        distance_along_path,
    ):
        """
        Find the how fast the robot should be moving at it's current point.
        """
        local_robot_distance = distance_along_path - start_path_distance
        local_end_distance = end_path_distance - start_path_distance
        speed_difference = end_speed - start_speed
        portion_path_completed = local_robot_distance / local_end_distance
        target_speed = speed_difference * portion_path_completed + start_speed
        return target_speed

    def find_velocity(self, robot_position):
        if self.current_waypoint_number >= len(self.waypoints) - 1:
            self.completed_path = True
            print("path completed")
            return None, None, None
        distance_along_path = self.distance_along_path(robot_position)
        segment_start = self.waypoints[self.current_waypoint_number]
        segment_end = self.waypoints[self.current_waypoint_number + 1]
        start_speed, start_distance = segment_start[2:]
        end_speed, end_distance = segment_end[2:]
        direction = self.compute_direction(
            robot_position, segment_start, segment_end, distance_along_path
        )
        speed = self.find_speed(
            start_distance, end_distance, start_speed, end_speed, distance_along_path
        )
        vx, vy = direction * speed
        heading = 0
        return vx, vy, heading

    def generate_traezoidal_distances(self, v, u, a, d):
        """Generate how far you have to travle to acceleration and deceleration for speed control.
        
        Args:
            v = final speed
            u = initial speed
            a = acceleration
            d = deceleration
        """
        segment_start = self.waypoints[self.current_waypoint_number]
        segment_end = self.waypoints[self.current_waypoint_number + 1]

        segment_distance = math.hypot(
            segment_end[0] - segment_start[0], segment_end[1] - segment_start[1]
        )

        d = abs(d)
        v_accel = (v ** 2 - u ** 2) / (2 * a)
        v_decel = (segment_distance - ((0.2 ** 2 - v ** 2) / (2 * d)))
        if v_decel < v_accel:
            self.trapezoid = False
            return (v_accel + v_decel) / 2
        else:
            self.trapezoid = True
            return v_accel, v_decel
