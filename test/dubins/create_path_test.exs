defmodule Dubins.CreatePathTest do
  use ExUnit.Case
  require Logger
  require ViaNavigation.Dubins.Shared.PathFollowerValues, as: PFV
  alias ViaNavigation.Dubins

  test "Build Mission" do
    speed = 5
    turn_rate = :math.pi() / 10

    path_follower_params = [
      {PFV.k_path(), 0.05},
      {PFV.k_orbit(), 2.0},
      {PFV.chi_inf_rad(), 1.05},
      {PFV.lookahead_dt_s(), 0.5}
    ]

    dx = 100
    dy = 100

    Logger.debug("WP2-3, and WP4-5 should be should be Right-Right points")
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

    mission = ViaNavigation.new_mission("default", [wp1, wp2, wp3, wp4, wp5], turn_rate)
    route = ViaNavigation.calculate_route(mission, "Dubins", path_follower_params)
    # Logger.debug(inspect(route))

    assert_in_delta(
      route.path_distance_m,
      2 * (dx + dy) + 2 * speed / turn_rate * (:math.pi() - 2),
      0.001
    )

    Logger.debug("WP1-2, and WP3-4 should be Left-Left points")
    wp1 = Dubins.Waypoint.new_flight(latlon1, speed, :math.pi(), "wp1")
    wp2 = Dubins.Waypoint.new_flight(latlon4, speed, 0 / 2, "wp2")
    wp3 = Dubins.Waypoint.new_flight(latlon3, speed, 0, "wp3")
    wp4 = Dubins.Waypoint.new_flight(latlon2, speed, :math.pi(), "wp4")
    wp5 = Dubins.Waypoint.new_flight(latlon5, speed, :math.pi(), "wp5")
    mission = ViaNavigation.new_mission("default", [wp1, wp2, wp3, wp4, wp5], turn_rate)
    route = ViaNavigation.calculate_route(mission, "Dubins", path_follower_params)

    assert_in_delta(
      route.path_distance_m,
      2 * (dx + dy) + 2 * speed / turn_rate * (:math.pi() - 2),
      0.001
    )

    Logger.debug("Should be all Left-Right points")
    wp1 = Dubins.Waypoint.new_flight(latlon1, speed, 0, "wp1")
    wp2 = Dubins.Waypoint.new_flight(latlon2, speed, :math.pi() / 2, "wp2")
    wp3 = Dubins.Waypoint.new_flight(latlon3, speed, -:math.pi(), "wp3")
    wp4 = Dubins.Waypoint.new_flight(latlon4, speed, -:math.pi() / 2, "wp4")
    wp5 = Dubins.Waypoint.new_flight(latlon5, speed, 0, "wp5")
    mission = ViaNavigation.new_mission("default", [wp1, wp2, wp3, wp4, wp5], turn_rate)
    _route = ViaNavigation.calculate_route(mission, "Dubins", path_follower_params)

    Logger.debug("Should be all Right-Left points")
    wp1 = Dubins.Waypoint.new_flight(latlon1, speed, -:math.pi() / 2, "wp1")
    wp2 = Dubins.Waypoint.new_flight(latlon2, speed, 0, "wp2")
    wp3 = Dubins.Waypoint.new_flight(latlon3, speed, :math.pi() / 2, "wp3")
    wp4 = Dubins.Waypoint.new_flight(latlon4, speed, -:math.pi(), "wp4")
    wp5 = Dubins.Waypoint.new_flight(latlon5, speed, -:math.pi() / 2, "wp5")

    mission = ViaNavigation.new_mission("default", [wp1, wp2, wp3, wp4, wp5], turn_rate)
    route = ViaNavigation.calculate_route(mission, "Dubins", path_follower_params)

    Logger.debug("Dubins: #{inspect(Map.drop(route, [:config_points]))}")
    Process.sleep(50)
  end
end
