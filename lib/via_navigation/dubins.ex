defmodule ViaNavigation.Dubins do
  require Logger
  require ViaUtils.Shared.EstimationNames, as: EN

  defstruct [
    :config_points,
    :current_cp_index,
    :current_path_case,
    :takeoff_altitude_m,
    :landing_altitude_m,
    :peripheral_control_allowed,
    :path_follower
  ]

  def new(mission, path_follower_params) do
    {config_points, path_distance} =
      ViaNavigation.Dubins.Utils.config_points_from_waypoints(
        mission.waypoints,
        mission.vehicle_turn_rate_rps
      )

    Logger.debug("path distance: #{path_distance}")
    current_cp = Enum.at(config_points, 0)
    current_path_case = Enum.at(current_cp.dubins.path_cases, 0)
    takeoff_altitude = Enum.at(config_points, 0) |> Map.get(:z2) |> Map.get(:altitude)
    landing_altitude = Enum.at(config_points, -1) |> Map.get(:z2) |> Map.get(:altitude)

    path_follower = ViaNavigation.Dubins.PathFollower.new(path_follower_params)

    %ViaNavigation.Dubins{
      config_points: config_points,
      current_cp_index: 0,
      current_path_case: current_path_case,
      takeoff_altitude_m: takeoff_altitude,
      landing_altitude_m: landing_altitude,
      peripheral_control_allowed: current_cp.peripheral_control_allowed,
      path_follower: path_follower
    }
  end

  def calculate_goals(state, position, velocity) do
    state = update_position(state, position)
    current_path_case = state.current_path_case

    if is_nil(current_path_case) do
      nil
    else
      speed = Map.fetch!(velocity, EN.groundspeed_mps())
      course = Map.fetch!(velocity, EN.course_rad())

      {speed_cmd, course_cmd, altitude_cmd} =
        ViaNavigation.Dubins.PathFollower.follow(
          state.path_follower,
          position,
          course,
          speed,
          current_path_case
        )

      # , course_rad: course_cmd}
      goals = %{groundspeed_mps: speed_cmd, altitude_m: altitude_cmd, sideslip_rad: 0}
      path_case_type = current_path_case.type

      case path_case_type do
        :flight ->
          Map.put(goals, :course_rad, :course_cmd)

        :climbout ->
          agl_error = agl_error(altitude_cmd, state.takeoff_altitude, position.agl)
          altitude_cmd_from_agl = position.altitude + agl_error

          Map.put(goals, :course_rad, course_cmd)
          |> Map.put(:altitude, altitude_cmd_from_agl)

        :ground ->
          if position.agl < state.vehicle_agl_ground_threshold do
            sideslip_cmd = ViaUtils.Motion.turn_left_or_right_for_correction(course_cmd - course)

            if speed < state.vehicle_takeoff_speed do
              Map.put(goals, :altitude, position.altitude)
            else
              agl_error = agl_error(altitude_cmd, state.takeoff_altitude, position.agl)
              altitude_cmd_from_agl = position.altitude + agl_error
              Map.put(goals, :altitude, altitude_cmd_from_agl)
            end
            |> Map.put(:course_rad, course)
            |> Map.put(:sideslip, sideslip_cmd)
          else
            Map.put(goals, :course_rad, course_cmd)
          end

        :landing ->
          agl_error = agl_error(altitude_cmd, state.landing_altitude, position.agl)
          altitude_cmd_from_agl = position.altitude + agl_error

          if position.agl < state.vehicle_agl_ground_threshold do
            sideslip_cmd = ViaUtils.Motion.turn_left_or_right_for_correction(course_cmd - course)

            Map.put(goals, :course_rad, course)
            |> Map.put(:sideslip_rad, sideslip_cmd)
          else
            Map.put(goals, :course_rad, course_cmd)
          end
          |> Map.put(:altitude, altitude_cmd_from_agl)

        :approach ->
          agl_error = agl_error(altitude_cmd, state.landing_altitude, position.agl)
          altitude_cmd_from_agl = position.altitude + agl_error

          Map.put(goals, :course_tilt, course_cmd)
          |> Map.put(:altitude, altitude_cmd_from_agl)
      end
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
                Logger.debug("no goto, move to cp_index: #{state.current_cp_index + 1}")
                cp_index = state.current_cp_index + 1

                if cp_index >= length(state.config_points) do
                  # No more waypoints
                  # Logging.Logger.save_log("mission_complete")
                  Logger.debug("Mission complete")
                  nil
                else
                  cp_index
                end

              wp_name ->
                index =
                  ViaUtils.Enum.index_for_embedded_value(state.config_points, :name, wp_name)

                Logger.debug("goto: #{index} for wp: #{wp_name}")
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

  @spec agl_error(float(), float(), float()) :: float()
  def agl_error(altitude_cmd, landing_altitude, agl) do
    agl_cmd = altitude_cmd - landing_altitude
    agl_cmd - agl
  end
end
