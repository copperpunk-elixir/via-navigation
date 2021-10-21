defmodule ViaNavigation do
  require Logger
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaUtils.Shared.GoalNames, as: SGN
  require ViaNavigation.Shared.MissionValues, as: MV
  require ViaNavigation.Dubins.Shared.WaypointsValues, as: SWV
  require ViaNavigation.Dubins.Shared.ModelSpec, as: SMS

  @spec new_mission(binary(), binary(), list(), number(), map()) :: struct()
  def new_mission(name, path_type, waypoints, altitude_ref_m, model_spec) do
    ViaNavigation.Mission.new(name, path_type, waypoints, altitude_ref_m, model_spec)
  end

  @spec calculate_route(struct(), list()) :: struct()
  def calculate_route(mission, path_follower_params) do
    %{MV.path_type() => path_type} = mission
    path_module = Module.concat(__MODULE__, String.to_atom(path_type))
    apply(path_module, :new, [mission, path_follower_params])
  end

  @spec update_goals(struct(), map(), map()) :: tuple()
  def update_goals(route, position_rrm, velocity_mps) do
    {route, goals_pcl, path_type} =
      apply(route.__struct__, :calculate_goals, [
        route,
        position_rrm,
        velocity_mps
      ])

    if Enum.empty?(goals_pcl) do
      {route, %{}}
    else
      %{
        SGN.course_rad() => course_cmd,
        SGN.agl_m() => agl_cmd,
        SGN.groundspeed_mps() => groundspeed_cmd_mps
      } = goals_pcl

      %{SVN.course_rad() => course_rad} = velocity_mps
      %{SVN.altitude_m() => altitude_m, SVN.agl_m() => agl_m} = position_rrm
      %{model_spec: model_spec, altitude_ref_m: altitude_ref_m} = route

      %{
        SMS.takeoff_flaps_speed_mps() => takeoff_flaps_speed_mps,
        SMS.landing_flaps_speed_mps() => landing_flaps_speed_mps,
        SMS.gear_agl_m() => gear_agl_m,
        SMS.wings_level_agl() => wings_level_agl
      } = model_spec

      altitude_cmd =
        if path_type == SWV.agl_altitude() do
          agl_cmd - agl_m + altitude_m
        else
          agl_cmd + altitude_ref_m
        end

      Logger.debug("type: #{path_type}")

      # Logger.debug(
      #   "agl_cmd/agl: #{ViaUtils.Format.eftb(agl_cmd, 1)}/#{ViaUtils.Format.eftb(agl_m, 1)}"
      # )

      # Logger.debug(
      #   "alt_cmd/alt: #{ViaUtils.Format.eftb(altitude_cmd, 1)}/#{ViaUtils.Format.eftb(altitude_m, 1)}"
      # )

      {course_cmd, sideslip_cmd} =
        if agl_m < wings_level_agl do
          sideslip_cmd =
            ViaUtils.Motion.turn_left_or_right_for_correction(course_cmd - course_rad)

          {course_rad, sideslip_cmd}
          # Map.put(goals_pcl, SGN.course_rad(), course_rad)
          # |> Map.put(SGN.sideslip_rad(), sideslip_cmd)
        else
          {course_cmd, 0}
          # Map.put(goals_pcl, SGN.sideslip_rad(), 0)
        end

      goals_pcl = %{
        SGN.course_rad() => course_cmd,
        SGN.altitude_m() => altitude_cmd,
        SGN.groundspeed_mps() => groundspeed_cmd_mps,
        SGN.sideslip_rad() => sideslip_cmd
      }

      # goals_pcl =
      #   case path_type do
      #     SWV.approach() ->
      #       # %{landing_altitude_m: landing_altitude_m} = route

      #       agl_error = agl_error(altitude_cmd, landing_altitude_m, agl_m)
      #       altitude_cmd_from_agl = altitude_m + agl_error

      #     # Map.put(goals_pcl, SGN.altitude_m(), altitude_cmd_from_agl)

      #     SWV.climbout() ->
      #       # %{takeoff_altitude_m: takeoff_altitude_m} = route
      #       agl_error = agl_error(altitude_cmd, takeoff_altitude_m, agl_m) |> max(0)
      #       altitude_cmd_from_agl = altitude_m + agl_error

      #       Map.put(goals_pcl, SGN.altitude_m(), altitude_cmd_from_agl)

      #     SWV.flight() ->
      #       goals_pcl

      #     SWV.landing() ->
      #       %{landing_altitude_m: landing_altitude_m} = route

      #       agl_error = agl_error(altitude_cmd, landing_altitude_m, agl_m)
      #       altitude_cmd_from_agl = altitude_m + agl_error

      #       if agl_m < wings_level_agl do
      #         sideslip_cmd =
      #           ViaUtils.Motion.turn_left_or_right_for_correction(course_cmd - course_rad)

      #         Map.put(goals_pcl, SGN.course_rad(), course_rad)
      #         |> Map.put(SGN.sideslip_rad(), sideslip_cmd)
      #       else
      #         goals_pcl
      #       end
      #       |> Map.put(SGN.altitude_m(), altitude_cmd_from_agl)

      #     SWV.takeoff() ->
      #       %{takeoff_altitude_m: takeoff_altitude_m} = route

      #       if agl_m < wings_level_agl do
      #         sideslip_cmd =
      #           ViaUtils.Motion.turn_left_or_right_for_correction(course_cmd - course_rad)

      #         if groundspeed_mps < takeoff_speed_mps do
      #           Map.put(goals_pcl, SGN.altitude_m(), altitude_m)
      #         else
      #           agl_error = agl_error(altitude_cmd, takeoff_altitude_m, agl_m)
      #           altitude_cmd_from_agl = altitude_m + agl_error
      #           Map.put(goals_pcl, SGN.altitude_m(), altitude_cmd_from_agl)
      #         end
      #         |> Map.put(SGN.course_rad(), course_rad)
      #         |> Map.put(SGN.sideslip_rad(), sideslip_cmd)
      #       else
      #         goals_pcl
      #       end

      #     true ->
      #       raise "The #{inspect(path_type)} does not exist yet."
      #   end

      flaps =
        cond do
          groundspeed_cmd_mps < landing_flaps_speed_mps -> 1
          groundspeed_cmd_mps < takeoff_flaps_speed_mps -> 0.5
          true -> 0
        end

      Logger.debug(
        "GScmd/flaps: #{ViaUtils.Format.eftb(groundspeed_cmd_mps, 1)}/#{ViaUtils.Format.eftb(flaps, 1)}"
      )

      gear = if agl_m < gear_agl_m, do: 1, else: 0
      goals_any = %{SGN.flaps_scaled() => flaps, SGN.gear_scaled() => gear}
      # case path_type do
      #   SWV.approach() ->
      #     %{SGN.flaps_scaled() => 1, SGN.gear_scaled() => 1}

      #   SWV.climbout() ->
      #     gear = if agl_m < wings_level_agl, do: 1, else: 0
      #     %{SGN.flaps_scaled() => 0.5, SGN.gear_scaled() => gear}

      #   SWV.flight() ->
      #     %{SGN.flaps_scaled() => 0, SGN.gear_scaled() => 0}

      #   SWV.ground() ->
      #     %{SGN.flaps_scaled() => 0, SGN.gear_scaled() => 1}

      #   SWV.landing() ->
      #     %{SGN.flaps_scaled() => 1, SGN.gear_scaled() => 1}

      #   SWV.takeoff() ->
      #     %{SGN.flaps_scaled() => 0.5, SGN.gear_scaled() => 1}

      #   _other ->
      #     raise "The #{inspect(path_type)} does not exist yet."
      # end

      goals = %{SGN.current_pcl() => goals_pcl, SGN.any_pcl() => goals_any}
      {route, goals}
    end
  end

  @spec agl_error(float(), float(), float()) :: float()
  def agl_error(altitude_cmd, landing_altitude, agl) do
    agl_cmd = altitude_cmd - landing_altitude
    agl_cmd - agl
  end
end
