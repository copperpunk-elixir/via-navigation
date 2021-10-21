defmodule ViaNavigation.Dubins.Shared.ModelSpec do
  defmacro climbout_distance_m, do: :climbout_distance_m
  defmacro climbout_agl_m, do: :climbout_agl_m
  defmacro climbout_speed_mps, do: :climbout_speed_mps
  defmacro cruise_speed_mps, do: :cruise_speed_mps
  defmacro flight_agl_range_m, do: :flight_agl_range_m
  defmacro flight_speed_range_mps, do: :flight_speed_range_mps
  defmacro gear_agl_m, do: :gear_agl
  defmacro landing_distances_agls_m, do: :landing_distances_agls_m
  defmacro landing_flaps_speed_mps, do: :landing_flaps_speed
  defmacro landing_speeds_mps, do: :landing_speeds_mps
  defmacro min_loiter_speed_mps, do: :min_loiter_speed_mps
  defmacro orbit_radius_m, do: :orbit_radius_m
  defmacro turn_rate_rps, do: :turn_rate_rps
  defmacro wp_dist_range_m, do: :wp_dist_range_m
  defmacro vehicle_type, do: :vehicle_type
  defmacro takeoff_flaps_speed_mps, do: :takeoff_flaps_speed
  defmacro takeoff_roll_m, do: :takeoff_roll_m
  defmacro wings_level_agl, do: :wings_level_agl

  @spec get_model_spec(binary()) :: map()
  def get_model_spec(model_type) do
    model = %{
      "Skyhawk" => %{
        vehicle_type() => "FixedWing",
        climbout_distance_m() => 1200,
        climbout_agl_m() => 100,
        climbout_speed_mps() => 35,
        cruise_speed_mps() => 45,
        flight_speed_range_mps() => {35, 45},
        flight_agl_range_m() => {100, 200},
        gear_agl_m() => 50,
        landing_distances_agls_m() => [{-1400, 100}, {-900, 100}, {100, 5}, {600, 0}],
        landing_flaps_speed_mps() => 34,
        landing_speeds_mps() => {40, 30},
        turn_rate_rps() => 0.08,
        takeoff_flaps_speed_mps() => 40,
        takeoff_roll_m() => 200,
        wings_level_agl() => 3,
        wp_dist_range_m() => {600, 1600}
      },
      "Cessna2m" => %{
        vehicle_type() => "FixedWing",
        climbout_distance_m() => 150,
        climbout_agl_m() => 30,
        climbout_speed_mps() => 12,
        cruise_speed_mps() => 14,
        flight_speed_range_mps() => {12, 18},
        flight_agl_range_m() => {50, 60},
        gear_agl_m() => 10,
        landing_distances_agls_m() => [{-150, 30}, {-10, 3}, {20, 1.5}, {50, 0}],
        landing_flaps_speed_mps() => 12,
        landing_speeds_mps() => {13, 10},
        min_loiter_speed_mps() => 12,
        orbit_radius_m() => 30,
        takeoff_flaps_speed_mps() => 14,
        takeoff_roll_m() => 10,
        turn_rate_rps() => 0.20,
        wings_level_agl() => 3,
        wp_dist_range_m() => {100, 200}
      }
      # "T28" => %{
      #   vehicle_type => "FixedWing",
      #   takeoff_roll => 30,
      #   climbout_distance => 200,
      #   climbout_height => 40,
      #   climbout_speed => 15,
      #   cruise_speed => 14,
      #   min_loiter_speed => 12,
      #   landing_distances_heights => [{-250, 40}, {-200, 40}, {-50, 3}, {1, 0}],
      #   landing_speeds => {15, 10},
      #   flight_speed_range => {15, 20},
      #   flight_agl_range => {50, 100},
      #   wp_dist_range => {200, 400},
      #   turn_rate => 0.30,
      #   orbit_radius => 25
      # },
      # "T28Z2m" => %{
      #   vehicle_type => "FixedWing",
      #   takeoff_roll => 30,
      #   climbout_distance => 200,
      #   climbout_height => 40,
      #   climbout_speed => 15,
      #   cruise_speed => 20,
      #   landing_distances_heights => [{-250, 40}, {-200, 40}, {-50, 3}, {1, 0}],
      #   landing_speeds => {15, 10},
      #   flight_speed_range => {15, 20},
      #   flight_agl_range => {50, 100},
      #   wp_dist_range => {200, 400},
      #   turn_rate => 0.80
      # },
      # "QuadX" => %{
      #   vehicle_type => "Multirotor",
      #   takeoff_roll => 5,
      #   climbout_distance => 25,
      #   climbout_height => 10,
      #   climbout_speed => 2,
      #   cruise_speed => 5,
      #   min_loiter_speed => 5,
      #   landing_distances_heights => [{-50, 7}, {-30, 5}, {-10, 1}, {0, 0.0}],
      #   landing_speeds => {3, 1.5},
      #   flight_speed_range => {5, 5},
      #   flight_agl_range => {5, 5},
      #   wp_dist_range => {100, 100},
      #   turn_rate => 0.25,
      #   orbit_radius => 10
      # },
      # "FerrariF1" => %{
      #   vehicle_type => "Car",
      #   cruise_speed => 3,
      #   flight_speed_range => {3, 3},
      #   wp_dist_range => {100, 100},
      #   turn_rate => 0.5,
      #   orbit_radius => 20
      # }
    }

    Map.get(model, model_type)
  end
end
