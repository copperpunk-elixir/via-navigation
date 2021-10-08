defmodule Dubins.CreatePathTest do
  use ExUnit.Case
  require Logger
  alias ViaNavigation.Dubins

  test "Build Mission" do
    speed = 5
    dx = 100
    dy = 100
    latlon1 = ViaUtils.Location.new_degrees(45.0, -120.0, 100)
    latlon2 = ViaUtils.Location.location_from_point_with_dx_dy(latlon1, dx, 0)
    latlon3 = ViaUtils.Location.location_from_point_with_dx_dy(latlon1, dx, dy)
    latlon4 = ViaUtils.Location.location_from_point_with_dx_dy(latlon1, 0, dy)
    latlon5 = ViaUtils.Location.location_from_point_with_dx_dy(latlon1, 0, 0)

    wp1 = Dubins.Waypoint.new_flight(latlon1, speed, 0, "wp1")
    wp2 = Dubins.Waypoint.new_flight(latlon2, speed, 0, "wp2")
    wp3 = Dubins.Waypoint.new_flight(latlon3, speed, -:math.pi(), "wp3")
    wp4 = Dubins.Waypoint.new_flight(latlon4, speed, -:math.pi(), "wp4")
    wp5 = Dubins.Waypoint.new_flight(latlon5, speed, 0, "wp5")

    turn_rate = :math.pi()/10
    mission = ViaNavigation.new_mission("default", [wp1, wp2, wp3, wp4, wp5], turn_rate)
    path_follower_params = [k_path: 0.05, k_orbit: 2.0, chi_inf: 1.05, lookahead_dt: 0.5]
    route = ViaNavigation.calculate_route(mission, "Dubins", path_follower_params)
    Logger.debug(inspect(route))

    assert_in_delta(
      route.path_distance_m,
      2 * dx + 2 * dy + 4 * :math.pi() * speed / turn_rate,
      0.001
    )
  end
end
