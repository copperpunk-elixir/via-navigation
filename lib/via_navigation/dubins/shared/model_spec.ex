defmodule ViaNavigation.Dubins.Shared.ModelSpec do
  defmacro climbout_distance_m, do: :climbout_distance_m
  defmacro climbout_height_m, do: :climbout_height_m
  defmacro climbout_speed_mps, do: :climbout_speed_mps
  defmacro cruise_speed_mps, do: :cruise_speed_mps
  defmacro flight_agl_range_m, do: :flight_agl_range_m
  defmacro flight_speed_range_mps, do: :flight_speed_range_mps
  defmacro landing_distances_heights_m, do: :landing_distances_heights_m
  defmacro landing_speeds_mps, do: :landing_speeds_mps
  defmacro min_loiter_speed_mps, do: :min_loiter_speed_mps
  defmacro planning_orbit_radius_m, do: :planning_orbit_radius_m
  defmacro planning_turn_rate_rps, do: :planning_turn_rate_rps
  defmacro wp_dist_range_m, do: :wp_dist_range_m
  defmacro vehicle_type, do: :vehicle_type
  defmacro takeoff_roll_m, do: :takeoff_roll_m

  @spec get_model_spec(binary()) :: map()
  def get_model_spec(model_type) do
    model = %{
      "Skyhawk" => %{
        vehicle_type() => "FixedWing",
        takeoff_roll_m() => 500,
        climbout_distance_m() => 1200,
        climbout_height_m() => 100,
        climbout_speed_mps() => 40,
        cruise_speed_mps() => 45,
        landing_distances_heights_m() => [{-1400, 100}, {-900, 100}, {100, 5}, {600, 0}],
        landing_speeds_mps() => {45, 35},
        flight_speed_range_mps() => {35, 45},
        flight_agl_range_m() => {100, 200},
        wp_dist_range_m() => {600, 1600},
        planning_turn_rate_rps() => 0.08
      },
      "CessnaZ2m" => %{
        vehicle_type() => "FixedWing",
        takeoff_roll_m() => 10,
        climbout_distance_m() => 150,
        climbout_height_m() => 30,
        climbout_speed_mps() => 12,
        cruise_speed_mps() => 14,
        min_loiter_speed_mps() => 12,
        landing_distances_heights_m() => [{-150, 30}, {-10, 3}, {20, 1.5}, {50, 0}],
        landing_speeds_mps() => {13, 10},
        flight_speed_range_mps() => {12, 18},
        flight_agl_range_m() => {50, 60},
        wp_dist_range_m() => {100, 200},
        planning_turn_rate_rps() => 0.20,
        planning_orbit_radius_m() => 30
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
      #   planning_turn_rate => 0.30,
      #   planning_orbit_radius => 25
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
      #   planning_turn_rate => 0.80
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
      #   planning_turn_rate => 0.25,
      #   planning_orbit_radius => 10
      # },
      # "FerrariF1" => %{
      #   vehicle_type => "Car",
      #   cruise_speed => 3,
      #   flight_speed_range => {3, 3},
      #   wp_dist_range => {100, 100},
      #   planning_turn_rate => 0.5,
      #   planning_orbit_radius => 20
      # }
    }

    Map.get(model, model_type)
  end
end
