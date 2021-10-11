defmodule Dubins.CalculateGoalsTest do
  use ExUnit.Case
  require Logger
  alias ViaNavigation.Dubins
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaNavigation.Dubins.Shared.PathFollowerValues, as: PFV

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
    lat0 = ViaUtils.Math.deg2rad(45)
    lon0 = ViaUtils.Math.deg2rad(-120)
    alt0 = 100

    latlon1 = ViaUtils.Location.new(lat0, lon0, alt0)
    latlon2 = ViaUtils.Location.location_from_point_with_dx_dy(latlon1, dx, 0) |> Map.put(SVN.altitude_m, alt0 + 100)
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

    position_rrm =
      ViaUtils.Location.location_from_point_with_dx_dy(latlon1, dx/2, 0)
      |> Map.put(SVN.agl_m(), alt0)

    velocity_mps = %{SVN.groundspeed_mps() => 5.0, SVN.course_rad() => 0}
    {route, goals} = ViaNavigation.update_goals(route, position_rrm, velocity_mps)
    Logger.debug("goals: #{ViaUtils.Format.eftb_map(goals,4)}")
  end
end
