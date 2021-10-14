defmodule ViaNavigation do
  require Logger
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaUtils.Shared.GoalNames, as: SGN
  require ViaNavigation.Shared.MissionValues, as: MV

  @spec new_mission(binary(), binary(), list(), number()) :: struct()
  def new_mission(name, path_type, waypoints, turn_rate_rps) do
    ViaNavigation.Mission.new(name, path_type, waypoints, turn_rate_rps)
  end

  @spec calculate_route(struct(), list()) :: struct()
  def calculate_route(mission, path_follower_params) do
    %{MV.path_type() => path_type} = mission
    path_module = Module.concat(__MODULE__, String.to_atom(path_type))
    apply(path_module, :new, [mission, path_follower_params])
  end

  @spec update_goals(struct(), map(), map(), any(), any()) :: tuple()
  def update_goals(
        route,
        position_rrm,
        velocity_mps,
        takeoff_speed_mps \\ nil,
        wings_level_agl \\ nil
      ) do
    {route, goals, path_type} =
      apply(route.__struct__, :calculate_goals, [
        route,
        position_rrm,
        velocity_mps
      ])

    if Enum.empty?(goals) do
      {route, %{}}
    else
      %{SGN.altitude_m() => altitude_cmd, SGN.course_rad() => course_cmd} = goals

      %{
        SVN.course_rad() => course_rad,
        SVN.groundspeed_mps() => groundspeed_mps
      } = velocity_mps

      %{SVN.altitude_m() => altitude_m, SVN.agl_m() => agl_m} = position_rrm

      goals =
        case path_type do
          :flight ->
            goals

          :climbout ->
            %{takeoff_altitude_m: takeoff_altitude_m} = route
            agl_error = agl_error(altitude_cmd, takeoff_altitude_m, agl_m) |> max(0)
            altitude_cmd_from_agl = altitude_m + agl_error

            Map.put(goals, SGN.altitude_m(), altitude_cmd_from_agl)

          :ground ->
            %{takeoff_altitude_m: takeoff_altitude_m} = route

            if agl_m < wings_level_agl do
              sideslip_cmd =
                ViaUtils.Motion.turn_left_or_right_for_correction(course_cmd - course_rad)

              if groundspeed_mps < takeoff_speed_mps do
                Map.put(goals, SGN.altitude_m(), altitude_m)
              else
                agl_error = agl_error(altitude_cmd, takeoff_altitude_m, agl_m)
                altitude_cmd_from_agl = altitude_m + agl_error
                Map.put(goals, SGN.altitude_m(), altitude_cmd_from_agl)
              end
              |> Map.put(SGN.course_rad(), course_rad)
              |> Map.put(SGN.sideslip_rad(), sideslip_cmd)
            else
              goals
            end

          :landing ->
            %{landing_altitude_m: landing_altitude_m} = route

            agl_error = agl_error(altitude_cmd, landing_altitude_m, agl_m)
            altitude_cmd_from_agl = altitude_m + agl_error

            if agl_m < wings_level_agl do
              sideslip_cmd =
                ViaUtils.Motion.turn_left_or_right_for_correction(course_cmd - course_rad)

              Map.put(goals, SGN.course_rad(), course_rad)
              |> Map.put(SGN.sideslip_rad(), sideslip_cmd)
            else
              goals
            end
            |> Map.put(SGN.altitude_m(), altitude_cmd_from_agl)

          :approach ->
            %{landing_altitude_m: landing_altitude_m} = route

            agl_error = agl_error(altitude_cmd, landing_altitude_m, agl_m)
            altitude_cmd_from_agl = altitude_m + agl_error

            Map.put(goals, SGN.altitude_m(), altitude_cmd_from_agl)

          true ->
            raise "The #{inspect(path_type)} does not exist yet."
        end

      {route, goals}
    end
  end

  @spec agl_error(float(), float(), float()) :: float()
  def agl_error(altitude_cmd, landing_altitude, agl) do
    agl_cmd = altitude_cmd - landing_altitude
    agl_cmd - agl
  end
end
