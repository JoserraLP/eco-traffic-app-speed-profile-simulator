import numpy as np

from speed_profile_simulator.path.path_model import PathModel
from speed_profile_simulator.static.constants import DELTA_T
from speed_profile_simulator.vehicle.veh_model import VehModel


def calculate_acceleration_factor(speed_t: float, speed_limit: float):
    """
    Calculate acceleration factor based on speed at a given instant and the limit

    :param speed_t: speed value at a given instant
    :type speed_t: float
    :param speed_limit: speed limit value
    :type speed_limit: float

    :return: acceleration factor
    """
    difference = abs(speed_t - speed_limit)
    if difference < 1:
        return 0.25
    elif difference < 3:
        return 0.5
    else:
        return 1


class VehicleMovement:
    """
    Class representing the relation between the route and a given vehicle
    """

    def __init__(self, route: PathModel, vehicle: VehModel):

        # Define number of segments of the route
        self._num_segments = len(route.segment_start_point)

        # Store both route and vehicle variables
        self._route = route
        self._vehicle = vehicle

        # Create variables related to segment end point, final speed and segment length
        self._segment_end_point = np.zeros(self._num_segments)
        self._final_speed = np.zeros(self._num_segments)
        self._segment_length = np.zeros(self._num_segments)

        # Create variables related to time, space, state, brake distance, speed, acceleration, calculated acceleration
        # and slope at a given instant
        self._time = [0]
        self._space = [0]
        self._state = [0]
        self._brake_distance = [0]
        self._speed = [0]
        self._acceleration = [0]
        self._acceleration_calc = [0]
        self._slope_t = [0]

        # Start time - General counter
        self._step = 1

        # Define acceleration
        self._ax_trac = {1: vehicle.ax_trac1, 2: vehicle.ax_trac2, 3: vehicle.ax_trac3}

    def perform_route(self):
        """
        Perform the route (by passing all of its sections) by using the vehicle being aware of its features and
        segment information as slopes.

        Initially, it calculates the slope in radians, the end point and the length of each segment.

        :return:
        """
        # Iterate over the segments
        for i in range(0, self._num_segments - 1):
            # Coincides with the limit speed of the following section
            if i < self._num_segments:
                # Segment end point
                self._segment_end_point[i] = self._route.segment_start_point[i + 1]
                # Final speed of the segment (m/s) is the next one
                self._final_speed[i] = self._route.speed_limit_km_h[i + 1] / 3.6

            # Last segment, arrival at destination
            else:
                # Segment end point
                self._segment_end_point[i] = self._route.total_distance
                # Final speed of the segment (m/s) is the actual one
                self._final_speed[i] = self._route.speed_limit_km_h[i] / 3.6

            # Segment length (m)
            self._segment_length[i] = self._segment_end_point[i] - self._route.segment_start_point[i]

            # Calculation of speed limit
            speed_limit = self._route.speed_limit_km_h[i] / 3.6

            # The kinematics of the vehicle is calculated at each simulation step.
            while self._space[self._step - 1] < self._segment_end_point[i]:
                # Braking distance calculation
                self._brake_distance.insert(self._step, (((self._final_speed[i]) ** 2 -
                                                          self._speed[self._step - 1] ** 2) /
                                                         (2 * self._vehicle.ax_brake)))

                # Determination of state (acceleration / constant speed / braking)
                # If it is in acceleration, state=1, so a=ax_trac
                # If it has reached the maximum speed of the section, state=0. In this case the speed is constant and
                # a=0.
                # If it must start braking, the state=-1, so a=ax_brake

                # Condition of speed greater than the final speed. In this case it makes sense that braking may exist.
                if self._speed[self._step - 1] >= self._final_speed[i]:
                    #  You do not have to brake, the current position indicates that you have not reached the braking
                    #  starting point.
                    if self._space[self._step - 1] < (self._segment_end_point[i] - self._brake_distance[self._step]):
                        # Definition of state-dependent acceleration

                        # If you do not reach the speed limit, you can speed up
                        if self._speed[self._step - 1] < speed_limit:
                            # Acceleration is smoothed, in case of being close to the speed limit.
                            factor = calculate_acceleration_factor(self._speed[self._step - 1], speed_limit)
                            # Calculate acceleration
                            if self._speed[self._step - 1] < (self._vehicle.v1_km_h / 3.6):
                                a = factor * self._ax_trac[1]
                            elif self._speed[self._step - 1] > (self._vehicle.v2_km_h / 3.6):
                                a = factor * self._ax_trac[3]
                            else:
                                a = factor * self._ax_trac[2]
                            # State = 1 -> acceleration
                            self._state.insert(self._step, 1)
                        else:
                            # If the speed limit of the section has been reached, the speed is kept constant,
                            # This implies that the acceleration is zero.
                            if self._speed[self._step - 1] > speed_limit:
                                # In this case you have to brake up to the maximum speed of the section
                                factor = calculate_acceleration_factor(self._speed[self._step - 1], speed_limit)
                                # Calculate acceleration
                                a = factor * self._vehicle.ax_brake
                                # State = -1 -> braking
                                self._state.insert(self._step, -1)
                            else:
                                # Acceleration equal to 0
                                a = 0
                                # State = 0 -> Maximum speed reached
                                self._state.insert(self._step, 0)
                    # It is in braking
                    else:
                        # Acceleration is smoothed, in case of being close to the speed limit.
                        factor = calculate_acceleration_factor(self._speed[self._step - 1], self._final_speed[i])
                        # Calculate acceleration
                        a = factor * self._vehicle.ax_brake
                        # State = -1 -> braking
                        self._state.insert(self._step, -1)
                # In case of speed(self._step)<final_speed: No braking
                else:
                    # Definition of acceleration as a function of sub-segment
                    if self._speed[self._step - 1] < speed_limit:
                        # Acceleration is smoothed, in case of being close to the speed limit.
                        factor = calculate_acceleration_factor(self._speed[self._step - 1], speed_limit)
                        # Calculate acceleration
                        if self._speed[self._step - 1] < (self._vehicle.v1_km_h / 3.6):
                            a = factor * self._ax_trac[1]
                        elif self._speed[self._step - 1] > (self._vehicle.v2_km_h / 3.6):
                            a = factor * self._ax_trac[3]
                        else:
                            a = factor * self._ax_trac[2]
                        # State = 1 -> acceleration
                        self._state.insert(self._step, 1)
                    # Speed limit case
                    else:
                        # Acceleration equal to 0
                        a = 0
                        # State = 0 -> Maximum speed reached
                        self._state.insert(self._step, 0)

                # Acceleration value is updated
                self._acceleration.insert(self._step, a)
                # A speed is calculated as a function of the acceleration imposed in the previous steps
                speed_calc = self._speed[self._step - 1] + a * DELTA_T
                # Speed is smoothed to avoid oscillations
                if abs(speed_calc - speed_limit) < 0.1:
                    self._speed.insert(self._step, speed_limit)
                else:
                    self._speed.insert(self._step, speed_calc)
                # Acceleration is calculated as a function of the smoothed speed
                self._acceleration_calc.insert(self._step, (self._speed[self._step] - self._speed[self._step - 1])
                                               / DELTA_T)
                # Also the space is calculated as a function of the smoothed speed
                self._space.insert(self._step,
                                   self._space[self._step - 1] + self._speed[self._step] * DELTA_T + 0.5 * a * (
                                           DELTA_T ** 2))
                # A ramp value is associated for each step
                self._slope_t.insert(self._step, self._route.slope[i])
                # Time is updated
                self._time.insert(self._step, self._time[self._step - 1] + DELTA_T)
                # Step is updated
                self._step += 1

    @property
    def num_segments(self):
        """Get the value of num_segments."""
        return self._num_segments

    @num_segments.setter
    def num_segments(self, value):
        """Set the value of num_segments."""
        self._num_segments = value

    @property
    def route(self):
        """Get the value of route."""
        return self._route

    @route.setter
    def route(self, value):
        """Set the value of route."""
        self._route = value

    @property
    def vehicle(self):
        """Get the value of vehicle."""
        return self._vehicle

    @vehicle.setter
    def vehicle(self, value):
        """Set the value of vehicle."""
        self._vehicle = value

    @property
    def segment_end_point(self):
        """Get the value of segment_end_point."""
        return self._segment_end_point

    @segment_end_point.setter
    def segment_end_point(self, value):
        """Set the value of segment_end_point."""
        self._segment_end_point = value

    @property
    def final_speed(self):
        """Get the value of final_speed."""
        return self._final_speed

    @final_speed.setter
    def final_speed(self, value):
        """Set the value of final_speed."""
        self._final_speed = value

    @property
    def segment_length(self):
        """Get the value of segment_length."""
        return self._segment_length

    @segment_length.setter
    def segment_length(self, value):
        """Set the value of segment_length."""
        self._segment_length = value

    @property
    def time(self):
        """Get the value of time."""
        return self._time

    @time.setter
    def time(self, value):
        """Set the value of time."""
        self._time = value

    @property
    def space(self):
        """Get the value of space."""
        return self._space

    @space.setter
    def space(self, value):
        """Set the value of space."""
        self._space = value

    @property
    def brake_distance(self):
        """Get the value of brake_distance."""
        return self._brake_distance

    @brake_distance.setter
    def brake_distance(self, value):
        """Set the value of brake_distance."""
        self._brake_distance = value

    @property
    def speed(self):
        """Get the value of speed."""
        return self._speed

    @speed.setter
    def speed(self, value):
        """Set the value of speed."""
        self._speed = value

    @property
    def acceleration(self):
        """Get the value of acceleration."""
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value):
        """Set the value of acceleration."""
        self._acceleration = value

    @property
    def acceleration_calc(self):
        """Get the value of acceleration_calc."""
        return self._acceleration_calc

    @acceleration_calc.setter
    def acceleration_calc(self, value):
        """Set the value of acceleration_calc."""
        self._acceleration_calc = value

    @property
    def state(self):
        """Get the value of state."""
        return self._state

    @state.setter
    def state(self, value):
        """Set the value of state."""
        self._state = value

    @property
    def slope_t(self):
        """Get the value of slope_t."""
        return self._slope_t

    @slope_t.setter
    def slope_t(self, value):
        """Set the value of slope_t."""
        self._slope_t = value

    @property
    def step(self):
        """Get the value of step."""
        return self._step

    @step.setter
    def step(self, value):
        """Set the value of step."""
        self._step = value

    @property
    def ax_trac(self):
        """Get the value of ax_trac."""
        return self._ax_trac

    @ax_trac.setter
    def ax_trac(self, value):
        """Set the value of ax_trac."""
        self._ax_trac = value
