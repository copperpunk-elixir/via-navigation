defmodule ViaNavigation.Dubins do
  require Logger
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaUtils.Shared.GoalNames, as: SGN

  defstruct [
    :config_points,
    :current_cp_index,
    :current_path_case,
    :takeoff_altitude_m,
    :landing_altitude_m,
    :peripheral_control_allowed,
    :path_follower,
    :path_distance_m
  ]

  def new(mission, path_follower_params) do
    {config_points, path_distance} =
      ViaNavigation.Dubins.Utils.config_points_from_waypoints(
        mission.waypoints,
        mission.turn_rate_rps
      )

    # Logger.debug("path distance: #{path_distance}")
    current_cp = Enum.at(config_points, 0)
    current_path_case = Enum.at(current_cp.dubins.path_cases, 0)
    takeoff_altitude = Enum.at(config_points, 0) |> Map.get(:z2) |> Map.get(SVN.altitude_m())
    landing_altitude = Enum.at(config_points, -1) |> Map.get(:z2) |> Map.get(SVN.altitude_m())

    path_follower = ViaNavigation.Dubins.PathFollower.new(path_follower_params)

    %ViaNavigation.Dubins{
      config_points: config_points,
      current_cp_index: 0,
      current_path_case: current_path_case,
      takeoff_altitude_m: takeoff_altitude,
      landing_altitude_m: landing_altitude,
      peripheral_control_allowed: current_cp.peripheral_control_allowed,
      path_follower: path_follower,
      path_distance_m: path_distance
    }
  end

  def calculate_goals(state, position_rrm, velocity_mps) do
    state = update_position(state, position_rrm)
    %{current_path_case: current_path_case, path_follower: path_follower} = state

    if is_nil(current_path_case) do
      {state, %{}, nil}
    else
      %{SVN.groundspeed_mps() => groundspeed_mps, SVN.course_rad() => course_rad} = velocity_mps

      {speed_cmd, course_cmd, altitude_cmd} =
        ViaNavigation.Dubins.PathFollower.follow(
          path_follower,
          position_rrm,
          course_rad,
          groundspeed_mps,
          current_path_case
        )

      # , course_rad: course_cmd}
      goals = %{
        SVN.groundspeed_mps() => speed_cmd,
        SGN.altitude_m() => altitude_cmd,
        SGN.course_rad() => course_cmd,
        SGN.sideslip_rad() => 0
      }

      path_case_type = current_path_case.type
      {state, goals, path_case_type}
    end
  end

  def update_position(state, position) do
    # If peripheral cases are allowed, check for one, and process it if available

    current_case_index =
      if is_nil(state.current_path_case), do: -1, else: state.current_path_case.case_index

    temp_case_index =
      case state.current_cp_index do
        nil ->
          -1

        index ->
          # Logger.debug("cp_index/path_case_index: #{index}/#{state.current_path_case.case_index}")
          current_cp = Enum.at(state.config_points, index)

          ViaNavigation.Dubins.Utils.check_for_path_case_completion(
            position,
            current_cp,
            state.current_path_case
          )
      end

    {current_cp_index, current_path_case, peripheral_control_allowed} =
      case temp_case_index do
        -1 ->
          # Logger.error("No config points. Follow path_case if it exists")
          {nil, state.current_path_case, state.peripheral_control_allowed}

        5 ->
          # Completed this control point
          # if there is a goto, then go to it
          current_cp = Enum.at(state.config_points, state.current_cp_index)

          current_cp_index =
            case current_cp.goto_upon_completion do
              nil ->
                Logger.warn("no goto, move to cp_index: #{state.current_cp_index + 1}")
                cp_index = state.current_cp_index + 1

                if cp_index >= length(state.config_points) do
                  # No more waypoints
                  # Logging.Logger.save_log("mission_complete")
                  Logger.warn("Mission complete")
                  nil
                else
                  cp_index
                end

              wp_name ->
                index =
                  ViaUtils.Enum.index_for_embedded_value(state.config_points, :name, wp_name)

                Logger.warn("goto: #{index} for wp: #{wp_name}")
                index
            end

          case current_cp_index do
            nil ->
              {nil, nil, state.peripheral_control_allowed}

            index ->
              new_config_point = Enum.at(state.config_points, index)
              new_dubins = Map.get(new_config_point, :dubins)
              path_case_index = if new_dubins.skip_case_0 == true, do: 1, else: 0
              path_case = Enum.at(new_dubins.path_cases, path_case_index)
              Logger.debug("Next cp: #{new_config_point.name}")
              {index, path_case, new_config_point.peripheral_control_allowed}
          end

        index ->
          current_cp = Enum.at(state.config_points, state.current_cp_index)

          {state.current_cp_index, Enum.at(current_cp.dubins.path_cases, index),
           state.peripheral_control_allowed}
      end

    state = %{
      state
      | current_cp_index: current_cp_index,
        current_path_case: current_path_case,
        peripheral_control_allowed: peripheral_control_allowed
    }

    if temp_case_index != current_case_index do
      update_position(state, position)
    else
      state
    end
  end
end
