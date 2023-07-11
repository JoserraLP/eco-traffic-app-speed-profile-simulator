import numpy as np
from scipy.interpolate import interp1d

from speed_profile_simulator.static.constants import ENGINE_PERFORMANCE, POWER_PERCENTAGE, DELTA_T
from speed_profile_simulator.vehicle.movement import VehicleMovement


class PowerEnergyEstimator:
    """
    Class for estimating the power and energy consumption for a given vehicle movement route
    """

    def __init__(self, vehicle_movement: VehicleMovement):
        # Retrieve vehicle movement number of steps
        self._step = vehicle_movement.step
        # Store vehicle movement
        self._vehicle_movement = vehicle_movement

        # Define traction force, average speed, power, instant energy [(W/s), (kW/h)], accumulated power (kW/h),
        # power percentage, performance, instant energy fuel (kW/h), accumulated energy fuel (kW/h), consumption,
        # total_consumption
        self._traction_force = np.zeros(self._step - 1)
        self._avg_speed = np.zeros(self._step - 1)
        self._power = np.zeros(self._step - 1)
        self._instant_energy_w_s = np.zeros(self._step - 1)
        self._instant_energy_kw_h = np.zeros(self._step - 1)
        self._accumulated_engine_energy_kw_h = np.zeros(self._step - 1)
        self._power_percentage = np.zeros(self._step - 1)
        self._performance = np.zeros(self._step - 1)
        self._instant_energy_fuel_kw_h = np.zeros(self._step - 1)
        self._instant_accumulated_energy_fuel_kw_h = np.zeros(self._step - 1)
        self._consumption = np.zeros(self._step - 1)
        self._consumption_total = np.zeros(self._step - 1)

        # Additional variable
        self._total_mass_981_001 = self._vehicle_movement.vehicle.total_mass * 9.81 * 0.01
        # Interpolation function based on power usage and engine performance 
        self._f = interp1d(POWER_PERCENTAGE, ENGINE_PERFORMANCE)

    def estimate_power_consumption(self):
        """
        Estimate power consumption based on the vehicle behavior and route

        :return:
        """
        # A loop is made that goes through all the steps, using the value of the variable "step".
        for j in range(1, self._step - 1):
            # Obtain speed in j
            speed_j = self._vehicle_movement.speed[j]
            # Obtain slope_t in j
            slope_t_j = self._vehicle_movement.slope_t[j]
            # Obtain acceleration in j
            acceleration_j = self._vehicle_movement.acceleration[j]
            # Calculate value of resistors together with speed
            ABC = self._vehicle_movement.vehicle.A + \
                  self._vehicle_movement.vehicle.B * speed_j + self._vehicle_movement.vehicle.C * (speed_j ** 2)
            # Calculate the force in Newtons, using the speed in m/s
            if self._vehicle_movement.state[j] == 1:
                # Considering acceleration
                self._traction_force[j] = ABC + self._total_mass_981_001 * slope_t_j \
                                          + self._vehicle_movement.vehicle.total_mass * acceleration_j
            elif self._vehicle_movement.state[j] == 0:
                # Regardless of acceleration
                self._traction_force[j] = ABC + self._total_mass_981_001 * slope_t_j
            elif -(ABC + self._total_mass_981_001 * slope_t_j) < self._vehicle_movement.vehicle.ax_brake:
                # Braking case
                self._traction_force[j] = ABC + self._total_mass_981_001 * slope_t_j - \
                                          self._vehicle_movement.vehicle.ax_brake * \
                                          self._vehicle_movement.vehicle.total_mass
            else:
                # Default to 0
                self._traction_force[j] = 0

            # Calculate average speed in m/s
            avg_speed_j = (speed_j + self._vehicle_movement.speed[j - 1]) / 2
            self._avg_speed[j] = avg_speed_j
            # Calculate power in W
            power_j = self._traction_force[j] * avg_speed_j
            self._power[j] = power_j

            # Energy at given instant
            # In W·s
            instant_energy_w_s_j = power_j * DELTA_T
            self._instant_energy_w_s[j] = instant_energy_w_s_j
            # In kW·h
            instant_energy_kw_h_j = instant_energy_w_s_j / 3600000
            self._instant_energy_kw_h[j] = instant_energy_kw_h_j

            # The accumulated energy delivered by the motor is calculated. It is a value that identifies the energy
            # consumption associated with the segment. It is independent of the engine, but depends on the dynamic
            # conditions of the vehicle, including its mass, driving resistances...
            self._accumulated_engine_energy_kw_h[j] = instant_energy_kw_h_j + \
                                                      self._accumulated_engine_energy_kw_h[j - 1]

            # Calculation of fuel consumption. For this purpose, the percentage of power is calculated. This value is
            # used to interpolate according to the performance values. It must be taken into account that the maximum
            # power is measured in kW
            power_percentage_j = 100 * power_j / (self._vehicle_movement.vehicle.p_max_kw * 1000)
            self._power_percentage[j] = power_percentage_j

            # Get performance interpolation
            performance_j = self._f(power_percentage_j)
            self._performance[j] = performance_j

            # Calculation of instantaneous fuel energy as a function of engine performance as a function of the power
            # demanded
            instant_energy_fuel_kw_h_j = instant_energy_kw_h_j / (performance_j / 100)
            self._instant_energy_fuel_kw_h[j] = instant_energy_fuel_kw_h_j
            # Store accumulated instantaneous fuel energy
            self._instant_accumulated_energy_fuel_kw_h[j] = instant_energy_fuel_kw_h_j + \
                                                            self._instant_accumulated_energy_fuel_kw_h[j - 1]

            # Conversion of fuel energy to liters
            consumption_j = instant_energy_fuel_kw_h_j * self._vehicle_movement.vehicle.l_diesel_por_kwh
            self._consumption[j] = consumption_j
            self._consumption_total[j] = self._consumption_total[j - 1] + consumption_j

    def get_final_results(self):
        """
        Get engine and fuel energy, alongside the consumption in liters by segment.
          
        :return: engine_energy_segment_kw_h, fuel_energy_segment_kw_h, 
        """
        # Valores finales
        # Salida de la función kW·h
        engine_energy_segment_kw_h = self._accumulated_engine_energy_kw_h[-1]
        fuel_energy_segment_kw_h = self._instant_accumulated_energy_fuel_kw_h[-1]
        consumption_segment_l = self._consumption_total[-1]
        return engine_energy_segment_kw_h, fuel_energy_segment_kw_h, consumption_segment_l

    @property
    def step(self):
        """Get the value of step."""
        return self._step

    @step.setter
    def step(self, value):
        """Set the value of step."""
        self._step = value

    @property
    def traction_force(self):
        """Get the value of traction_force."""
        return self._traction_force

    @traction_force.setter
    def traction_force(self, value):
        """Set the value of traction_force."""
        self._traction_force = value

    @property
    def avg_speed(self):
        """Get the value of avg_speed."""
        return self._avg_speed

    @avg_speed.setter
    def avg_speed(self, value):
        """Set the value of avg_speed."""
        self._avg_speed = value

    @property
    def power(self):
        """Get the value of power."""
        return self._power

    @power.setter
    def power(self, value):
        """Set the value of power."""
        self._power = value

    @property
    def instant_energy_w_s(self):
        """Get the value of instant_energy_w_s."""
        return self._instant_energy_w_s

    @instant_energy_w_s.setter
    def instant_energy_w_s(self, value):
        """Set the value of instant_energy_w_s."""
        self._instant_energy_w_s = value

    @property
    def instant_energy_kw_h(self):
        """Get the value of instant_energy_kw_h."""
        return self._instant_energy_kw_h

    @instant_energy_kw_h.setter
    def instant_energy_kw_h(self, value):
        """Set the value of instant_energy_kw_h."""
        self._instant_energy_kw_h = value

    @property
    def accumulated_engine_energy_kw_h(self):
        """Get the value of accumulated_engine_energy_kw_h."""
        return self._accumulated_engine_energy_kw_h

    @accumulated_engine_energy_kw_h.setter
    def accumulated_engine_energy_kw_h(self, value):
        """Set the value of accumulated_engine_energy_kw_h."""
        self._accumulated_engine_energy_kw_h = value

    @property
    def power_percentage(self):
        """Get the value of power_percentage."""
        return self._power_percentage

    @power_percentage.setter
    def power_percentage(self, value):
        """Set the value of power_percentage."""
        self._power_percentage = value

    @property
    def performance(self):
        """Get the value of performance."""
        return self._performance

    @performance.setter
    def performance(self, value):
        """Set the value of performance."""
        self._performance = value

    @property
    def instant_energy_fuel_kw_h(self):
        """Get the value of instant_energy_fuel_kw_h."""
        return self._instant_energy_fuel_kw_h

    @instant_energy_fuel_kw_h.setter
    def instant_energy_fuel_kw_h(self, value):
        """Set the value of instant_energy_fuel_kw_h."""
        self._instant_energy_fuel_kw_h = value
