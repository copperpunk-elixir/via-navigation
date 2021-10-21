defmodule ViaNavigation.Dubins.Mission.Builder do
  require Logger
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaNavigation.Dubins.Shared.ModelSpec, as: MS
  require ViaNavigation.Dubins.Shared.MissionValues, as: MV
  require ViaNavigation.Dubins.Shared.WaypointsValues, as: SWV

  @spec calculate_orbit_parameters(binary(), float()) :: tuple()
  def calculate_orbit_parameters(model_type, radius_m) do
    %{
      MS.turn_rate_rps() => planning_turn_rate_rps,
      MS.cruise_speed_mps() => cruise_speed_mps,
      MS.min_loiter_speed_mps() => min_loiter_speed_mps
    } = MS.get_model_spec(model_type)

    # Assume right turn if radius = 0
    direction = if radius_m >= 0, do: 1, else: -1
    radius_m = abs(radius_m)
    turn_rate = if radius_m != 0, do: cruise_speed_mps / radius_m, else: 0
    # Turn rate , Speed , Radius
    if turn_rate > planning_turn_rate_rps or turn_rate == 0 do
      speed = planning_turn_rate_rps * radius_m
      # Logger.warn("turn rate too high. new speed: #{speed}")
      cond do
        speed < min_loiter_speed_mps ->
          # Logger.warn("too slow")
          radius = min_loiter_speed_mps / planning_turn_rate_rps
          {planning_turn_rate_rps, min_loiter_speed_mps, radius * direction}

        speed > cruise_speed_mps ->
          # Logger.warn("too fast")
          radius = cruise_speed_mps / planning_turn_rate_rps
          {planning_turn_rate_rps, cruise_speed_mps, radius * direction}

        true ->
          {planning_turn_rate_rps, speed, radius_m * direction}
      end
    else
      {turn_rate, cruise_speed_mps, radius_m * direction}
    end
  end

  @spec set_waypoints(struct(), list()) :: struct()
  def set_waypoints(mission, waypoints) do
    %{mission | waypoints: waypoints}
  end

  @spec add_waypoint_at_index(struct(), struct(), integer()) :: struct()
  def add_waypoint_at_index(mission, waypoint, index) do
    waypoints =
      if index >= -1 do
        List.insert_at(mission.waypoints, index, waypoint)
      else
        Logger.debug("Index cannot be less than -1")
        mission.waypoints
      end

    %{mission | waypoints: waypoints}
  end

  @spec remove_waypoint_at_index(struct(), integer()) :: struct()
  def remove_waypoint_at_index(mission, index) do
    waypoints =
      if index >= -1 do
        List.delete_at(mission.waypoints, index)
      else
        Logger.debug("Index cannot be less than -1")
        mission.waypoints
      end

    %{mission | waypoints: waypoints}
  end

  @spec remove_all_waypoints(struct()) :: struct()
  def remove_all_waypoints(mission) do
    %{mission | waypoints: []}
  end

  @spec get_default_mission(binary()) :: struct()
  def get_default_mission(model_type) do
    speed = 0.8
    latlon1 = ViaUtils.Location.new_degrees(45.0, -120.0, 100)
    latlon2 = ViaUtils.Location.location_from_point_with_dx_dy(latlon1, 200, 20)
    latlon3 = ViaUtils.Location.location_from_point_with_dx_dy(latlon1, 0, 40)
    latlon4 = ViaUtils.Location.location_from_point_with_dx_dy(latlon1, 100, 30)
    latlon5 = ViaUtils.Location.location_from_point_with_dx_dy(latlon1, 100, -70)

    wp1 =
      ViaNavigation.Dubins.Waypoint.new_from_location(
        latlon1,
        speed,
        :math.pi() / 2,
        SWV.gps_altitude(),
        "wp1"
      )

    wp2 =
      ViaNavigation.Dubins.Waypoint.new_from_location(
        latlon2,
        speed,
        :math.pi() / 2,
        SWV.gps_altitude(),
        "wp2"
      )

    wp3 =
      ViaNavigation.Dubins.Waypoint.new_from_location(
        latlon3,
        speed,
        :math.pi() / 2,
        SWV.gps_altitude(),
        "wp3"
      )

    wp4 =
      ViaNavigation.Dubins.Waypoint.new_from_location(
        latlon4,
        speed,
        :math.pi(),
        SWV.gps_altitude(),
        "wp4"
      )

    wp5 =
      ViaNavigation.Dubins.Waypoint.new_from_location(
        latlon5,
        speed,
        0,
        SWV.gps_altitude(),
        "wp5",
        false,
        0
      )

    # vehicle_turn_rate = Configuration.Vehicle.Plane.Navigation.get_vehicle_limits(model_type)
    # |> Keyword.get(:vehicle_turn_rate)
    %{MS.turn_rate_rps() => planning_turn_rate} = MS.get_model_spec(model_type)

    ViaNavigation.Mission.new(
      "default",
      "Dubins",
      [wp1, wp2, wp3, wp4, wp5],
      latlon1.altitude_m,
      planning_turn_rate
    )
  end

  @spec get_takeoff_waypoints(struct(), float(), binary()) :: list()
  def get_takeoff_waypoints(start_position, course, model_type) do
    Logger.debug("start: #{ViaUtils.Location.to_string(start_position)}")

    %{
      MS.takeoff_roll_m() => takeoff_roll_distance,
      MS.climbout_distance_m() => climbout_distance,
      MS.climbout_agl_m() => climbout_agl,
      MS.climbout_speed_mps() => climbout_speed,
      MS.cruise_speed_mps() => cruise_speed,
      MS.vehicle_type() => vehicle_type
    } = MS.get_model_spec(model_type)

    # start_position = Map.put(start_position, SVN.altitude_m(), 0)

    takeoff_roll =
      ViaUtils.Location.location_from_point_with_distance_bearing(
        start_position,
        takeoff_roll_distance,
        course
      )

    %{SVN.altitude_m() => start_alt_m} = start_position

    climb_position =
      ViaUtils.Location.location_from_point_with_distance_bearing(
        start_position,
        climbout_distance,
        course
      )
      |> Map.put(SVN.altitude_m(), start_alt_m + climbout_agl)

    case vehicle_type do
      "FixedWing" ->
        wp0 =
          ViaNavigation.Dubins.Waypoint.new_from_location(
            start_position,
            climbout_speed,
            course,
            SWV.agl_altitude(),
            "Start"
          )

        wp1 =
          ViaNavigation.Dubins.Waypoint.new_from_location(
            takeoff_roll,
            climbout_speed,
            course,
            SWV.agl_altitude(),
            "takeoff"
          )

        wp2 =
          ViaNavigation.Dubins.Waypoint.new_from_location(
            climb_position,
            climbout_speed,
            course,
            SWV.agl_altitude(),
            "climbout"
          )

        [wp0, wp1, wp2]

      "Multirotor" ->
        # start_position = ViaUtils.Location.location_from_point_with_distance_bearing(start_position,5.0, course+:math.pi)
        start_position = Map.put(start_position, SVN.altitude_m(), 3.0)
        %{SVN.altitude_m() => takeoff_roll_alt_m} = takeoff_roll
        takeoff_roll = Map.put(takeoff_roll, SVN.altitude_m(), takeoff_roll_alt_m + 3.0)

        wp0 =
          ViaNavigation.Dubins.Waypoint.new_from_location(
            start_position,
            climbout_speed,
            course,
            SWV.agl_altitude(),
            "Start"
          )

        wp1 =
          ViaNavigation.Dubins.Waypoint.new_from_location(
            takeoff_roll,
            climbout_speed,
            course,
            SWV.agl_altitude(),
            "takeoff"
          )

        wp2 =
          ViaNavigation.Dubins.Waypoint.new_from_location(
            climb_position,
            cruise_speed,
            course,
            SWV.gps_altitude(),
            "climbout"
          )

        [wp0, wp1, wp2]
    end
  end

  @spec get_landing_waypoints(struct(), float(), binary()) :: list()
  def get_landing_waypoints(final_position, course, model_type) do
    %{
      MS.landing_distances_agls_m() => landing_distances_agls,
      MS.vehicle_type() => vehicle_type,
      MS.landing_speeds_mps() => landing_speeds,
      MS.cruise_speed_mps() => cruise_speed
    } = MS.get_model_spec(model_type)

    %{SVN.altitude_m() => fp_alt} = final_position

    landing_points =
      Enum.reduce(landing_distances_agls, [], fn {distance, height}, acc ->
        wp =
          ViaUtils.Location.location_from_point_with_distance_bearing(
            final_position,
            distance,
            course
          )
          |> Map.put(SVN.altitude_m(), fp_alt + height)

        acc ++ [wp]
      end)

    {approach_speed, touchdown_speed} = landing_speeds

    case vehicle_type do
      "FixedWing" ->
        [
          ViaNavigation.Dubins.Waypoint.new_from_location(
            Enum.at(landing_points, 0),
            approach_speed,
            course,
            SWV.agl_altitude(),
            "approach"
          )
        ] ++
          [
            ViaNavigation.Dubins.Waypoint.new_from_location(
              Enum.at(landing_points, 1),
              touchdown_speed,
              course,
              SWV.agl_altitude(),
              "descent"
            )
          ] ++
          [
            ViaNavigation.Dubins.Waypoint.new_from_location(
              Enum.at(landing_points, 2),
              touchdown_speed,
              course,
              SWV.agl_altitude(),
              "flare"
            )
          ] ++
          [
            ViaNavigation.Dubins.Waypoint.new_from_location(
              Enum.at(landing_points, 3),
              0,
              course,
              SWV.agl_altitude(),
              "touchdown"
            )
          ]

      "Multirotor" ->
        [
          ViaNavigation.Dubins.Waypoint.new_from_location(
            Enum.at(landing_points, 0),
            cruise_speed,
            course,
            SWV.agl_altitude(),
            "pre-approach"
          )
        ] ++
          [
            ViaNavigation.Dubins.Waypoint.new_from_location(
              Enum.at(landing_points, 1),
              approach_speed,
              course,
              SWV.agl_altitude(),
              "approach"
            )
          ] ++
          [
            ViaNavigation.Dubins.Waypoint.new_from_location(
              Enum.at(landing_points, 2),
              approach_speed,
              course,
              SWV.agl_altitude(),
              "descent"
            )
          ] ++
          [
            ViaNavigation.Dubins.Waypoint.new_from_location(
              Enum.at(landing_points, 3),
              touchdown_speed,
              course,
              SWV.agl_altitude(),
              "touchdown"
            )
          ]
    end

    # wp0 = ViaNavigation.Dubins.Waypoint.new_flight(Enum.at(landing_points,0), approach_speed, course, "pre-approach")
    # [wp0, wp1, wp2, wp3, wp4]
  end

  @spec get_complete_mission(
          binary(),
          binary(),
          binary(),
          binary(),
          integer(),
          struct(),
          struct()
        ) :: struct()
  def get_complete_mission(
        airport,
        runway,
        model_type,
        track_type,
        num_wps,
        start_position \\ nil,
        start_course \\ nil
      ) do
    %{MS.turn_rate_rps() => planning_turn_rate} = MS.get_model_spec(model_type)

    {start_position, start_course} =
      if is_nil(start_position) or is_nil(start_course) do
        get_runway_position_heading(airport, runway)
      else
        {start_position, start_course}
      end

    %{SVN.altitude_m() => altitude_ref_m} = start_position
    start_position = Map.put(start_position, SVN.altitude_m(), 0)

    takeoff_wps = get_takeoff_waypoints(start_position, start_course, model_type)
    starting_wp = Enum.at(takeoff_wps, 0)
    first_flight_wp = Enum.at(takeoff_wps, -1)

    flight_wps =
      case track_type do
        "none" ->
          if num_wps > 0 do
            get_random_waypoints(model_type, starting_wp, first_flight_wp, num_wps)
          else
            # %{SVN.altitude_m() => ground_altitude_m} = start_position

            %{
              SVN.course_rad() => course,
              SVN.groundspeed_mps() => speed,
              SVN.latitude_rad() => latitude,
              SVN.longitude_rad() => longitude,
              SVN.altitude_m() => altitude
            } = first_flight_wp

            pos =
              ViaUtils.Location.new(
                latitude,
                longitude,
                altitude
              )

            distance = 2 * speed / planning_turn_rate
            new_course = ViaUtils.Math.constrain_angle_to_compass(course + :math.pi())

            new_pos_temp =
              ViaUtils.Location.location_from_point_with_distance_bearing(
                pos,
                distance,
                course - :math.pi() / 2
              )

            new_pos =
              ViaUtils.Location.location_from_point_with_distance_bearing(
                new_pos_temp,
                50,
                new_course
              )

            new_wp =
              ViaNavigation.Dubins.Waypoint.new_from_location(
                new_pos,
                speed,
                new_course,
                SWV.gps_altitude(),
                "wp1",
                true
              )

            [new_wp]
          end

        type ->
          get_track_waypoints(airport, runway, type, model_type, true)
      end

    landing_wps = get_landing_waypoints(start_position, start_course, model_type)
    # Logger.debug(inspect(landing_wps))
    wps = takeoff_wps ++ flight_wps ++ landing_wps
    Logger.debug("Alt Ref: #{altitude_ref_m}")
    print_waypoints_relative(start_position, wps)

    ViaNavigation.Mission.new(
      "#{airport} - #{runway}: #{track_type}",
      "Dubins",
      wps,
      altitude_ref_m,
      MS.get_model_spec(model_type)
    )
  end

  @spec get_flight_mission(binary(), binary(), binary(), binary()) :: struct()
  def get_flight_mission(airport, runway, model_type, track_type) do
    {start_position, _} = get_runway_position_heading(airport, runway)
    %{SVN.altitude_m() => altitude_ref_m} = start_position
    wps = get_track_waypoints(airport, runway, track_type, model_type, true)
    print_waypoints_relative(Enum.at(wps, 0), wps)
    # %{MS.turn_rate_rps() => planning_turn_rate} = MS.get_model_spec(model_type)

    ViaNavigation.Mission.new(
      "#{airport} - #{runway}: #{track_type}",
      "Dubins",
      wps,
      altitude_ref_m,
      MS.get_model_spec(model_type)
    )
  end

  @spec get_landing_mission(binary(), binary(), binary()) :: struct()
  def get_landing_mission(airport, runway, model_type) do
    %{MS.turn_rate_rps() => planning_turn_rate} = MS.get_model_spec(model_type)

    {start_position, start_course} = get_runway_position_heading(airport, runway)
    %{SVN.altitude_m() => altitude_ref} = start_position
    start_position = Map.put(start_position, SVN.altitude_m(), 0)
    wps = get_landing_waypoints(start_position, start_course, model_type)
    print_waypoints_relative(start_position, wps)

    ViaNavigation.Mission.new(
      "#{airport} - #{runway}: landing",
      "Dubins",
      wps,
      altitude_ref,
      planning_turn_rate
    )
  end

  @spec add_current_position_to_mission(struct(), struct(), float(), float()) :: struct()
  def add_current_position_to_mission(mission, current_position, speed, course) do
    Logger.debug("add current position to mission")
    %{SVN.altitude_m() => altitude_m} = current_position
    wp_type = if altitude_m > 30, do: SWV.gps_altitude(), else: SWV.agl_altitude()

    current_wp =
      ViaNavigation.Dubins.Waypoint.new_from_location(
        current_position,
        speed,
        course,
        wp_type,
        "start"
      )

    Logger.debug("mission wps:")

    Enum.each(mission.waypoints, fn wp ->
      Logger.debug(ViaNavigation.Dubins.Waypoint.to_string(wp))
    end)

    Logger.debug("current wp: ")

    Logger.debug(ViaNavigation.Dubins.Waypoint.to_string(current_wp))

    %{
      MV.name() => name,
      MV.waypoints() => mission_waypoints,
      MV.turn_rate_rps() => turn_rate,
      MV.altitude_ref_m() => alt_ref
    } = mission

    wps = [current_wp] ++ mission_waypoints
    ViaNavigation.Mission.new(name, "Dubins", wps, alt_ref, turn_rate)
  end

  @spec get_track_waypoints(binary(), binary(), binary(), binary(), boolean()) :: list()
  def get_track_waypoints(airport, runway, track_type, model_type, loop) do
    %{MS.cruise_speed_mps() => wp_speed} = MS.get_model_spec(model_type)
    {origin, _runway_heading} = get_runway_position_heading(airport, runway)

    wps_relative = %{
      "flight_school" => %{
        "top_left" => {75, -150, 30},
        "top_right" => {75, 50, 30},
        "bottom_left" => {-175, -150, 30},
        "bottom_right" => {-175, 50, 30}
      }
    }

    wps_and_course_map = %{
      "racetrack_left" => [
        {"top_left", 180},
        {"bottom_left", 180},
        {"bottom_right", 0},
        {"top_right", 0}
      ],
      "racetrack_right" => [
        {"bottom_left", 0},
        {"top_left", 0},
        {"top_right", 180},
        {"bottom_right", 180}
      ],
      "hourglass_right" => [
        {"top_left", 0},
        {"top_right", 180},
        {"bottom_left", 180},
        {"bottom_right", 0}
      ],
      "hourglass_left" => [
        {"top_left", 180},
        {"bottom_right", 180},
        {"bottom_left", 0},
        {"top_right", 0}
      ]
    }

    reference_headings = %{
      "flight_school" => 180
    }

    wps_and_course = Map.get(wps_and_course_map, track_type)
    reference_heading = Map.get(reference_headings, airport) |> ViaUtils.Math.deg2rad()
    # Logger.debug("origin: #{ViaUtils.Location.to_string(origin)}")
    # Logger.debug("reference_heading: #{reference_heading}")
    wps =
      Enum.reduce(wps_and_course, [], fn {wp_name, rel_course}, acc ->
        Logger.debug("#{airport}/#{wp_name}")
        {rel_x, rel_y, rel_alt} = get_in(wps_relative, [airport, wp_name])
        {dx, dy} = ViaUtils.Math.rotate_point(rel_x, rel_y, reference_heading)
        # Logger.warn("relx/rely: #{rel_x}/#{rel_y}")
        # Logger.warn("dx/dy: #{dx}/#{dy}")
        lla =
          ViaUtils.Location.location_from_point_with_dx_dy(origin, dx, dy)
          |> Map.put(:altitude, rel_alt)

        # lla = ViaUtils.Location.new_degrees(lat, lon, alt)
        course =
          ViaUtils.Math.constrain_angle_to_compass(
            ViaUtils.Math.deg2rad(rel_course) + reference_heading
          )

        wp =
          ViaNavigation.Dubins.Waypoint.new_from_location(
            lla,
            wp_speed,
            course,
            SWV.gps_altitude(),
            "#{length(acc) + 1}",
            true
          )

        acc ++ [wp]
      end)

    first_wp = Enum.at(wps, 0)

    final_wp =
      if loop do
        %{first_wp | goto: first_wp.name}
      else
        first_wp
      end

    wps ++ [final_wp]
  end

  @spec get_lawnmower_mission(binary(), binary(), binary(), integer(), float(), float()) ::
          struct()
  def get_lawnmower_mission(airport, runway, model_type, num_rows, row_width, row_length) do
    %{MS.turn_rate_rps() => planning_turn_rate} = MS.get_model_spec(model_type)
    {origin, _runway_heading} = get_runway_position_heading(airport, runway)
    wps = get_lawnmower_waypoints(airport, runway, model_type, num_rows, row_width, row_length)

    ViaNavigation.Mission.new(
      "#{airport} - #{runway}: lawnmower",
      "Dubins",
      wps,
      origin.altitude_m,
      planning_turn_rate
    )
  end

  @spec get_lawnmower_waypoints(binary(), binary(), binary(), integer(), float(), float()) ::
          list()
  def get_lawnmower_waypoints(airport, runway, model_type, num_rows, row_width, row_length) do
    %{MS.turn_rate_rps() => planning_turn_rate, MS.cruise_speed_mps() => wp_speed} =
      MS.get_model_spec(model_type)

    radius = wp_speed / planning_turn_rate
    # Logger.info("radius: #{radius}")
    {origin, _runway_heading} = get_runway_position_heading(airport, runway)

    reference_headings = %{
      "cone_field" => 0
    }

    reference_heading = Map.get(reference_headings, airport) |> ViaUtils.Math.deg2rad()

    {_, wps} =
      Enum.reduce(0..(num_rows - 1), {reference_heading, []}, fn row, {current_heading, acc} ->
        Logger.debug("row/course: #{row}/#{ViaUtils.Format.eftb_deg(current_heading, 1)}")
        rel_y = row * row_width * :math.sin(reference_heading + :math.pi() / 2)
        rel_x1 = if rem(row, 2) == 1, do: row_length, else: 0.0
        rel_x2 = rel_x1 + (row_length + 1.5 * radius) * :math.cos(current_heading)
        {dx1, dy1} = ViaUtils.Math.rotate_point(rel_x1, rel_y, reference_heading)
        # {rel_x, rel_y, rel_alt} = get_in(wps_relative, [airport, wp_name])
        {dx2, dy2} = ViaUtils.Math.rotate_point(rel_x2, rel_y, reference_heading)
        # Logger.warn("relx/rely: #{rel_x}/#{rel_y}")
        # Logger.debug("dx1/dy1: #{dx1}/#{dy1}")
        # Logger.debug("dx2/dy2: #{dx2}/#{dy2}")
        lla_1 =
          ViaUtils.Location.location_from_point_with_dx_dy(origin, dx1, dy1)
          |> Map.put(:altitude, origin.altitude)

        lla_2 =
          ViaUtils.Location.location_from_point_with_dx_dy(origin, dx2, dy2)
          |> Map.put(:altitude, origin.altitude)

        # lla = ViaUtils.Location.new_degrees(lat, lon, alt)
        # course = ViaUtils.Math.constrain_angle_to_compass(current_heading + reference_heading)
        wp1 =
          ViaNavigation.Dubins.Waypoint.new_from_location(
            lla_1,
            wp_speed,
            current_heading,
            SWV.agl_altitude(),
            "#{length(acc) + 1}",
            true
          )

        wp2 =
          ViaNavigation.Dubins.Waypoint.new_from_location(
            lla_2,
            wp_speed,
            current_heading,
            SWV.agl_altitude(),
            "#{length(acc) + 2}",
            true
          )

        new_heading = ViaUtils.Math.constrain_angle_to_compass(current_heading + :math.pi())
        {new_heading, acc ++ [wp1, wp2]}
      end)

    first_wp = Enum.at(wps, 0)
    final_wp = %{first_wp | name: "end", goto: first_wp.name}
    wps ++ [final_wp]
  end

  @spec get_runway_position_heading(binary(), binary()) :: tuple()
  def get_runway_position_heading(airport, runway) do
    origin_heading = %{
      "seatac" => %{
        "34L" => {ViaUtils.Location.new_degrees(47.4407476, -122.3180652, 105.0), 0.0}
      },
      "montague" => %{
        "36L" => {ViaUtils.Location.new_degrees(41.76816, -122.50686, 802.0), 2.3},
        "18R" => {ViaUtils.Location.new_degrees(41.7689, -122.50682, 803.0), 182.3}
      },
      "flight_school" => %{
        "9L" => {ViaUtils.Location.new_degrees(41.76157, -122.48929, 1186.51), 180.0},
        "18L" => {ViaUtils.Location.new_degrees(41.76174, -122.48928, 1186.6), 180.0},
        "36R" => {ViaUtils.Location.new_degrees(41.76105, -122.48928, 1186.7), 0.0}
      },
      "boneyard" => %{
        "36R" => {ViaUtils.Location.new_degrees(42.18878, -122.08890, 0.2), 358.32}
      },
      "obstacle_course" => %{
        "36R" => {ViaUtils.Location.new_degrees(41.70676, -122.39755, 1800.4), 0.0}
      },
      "fpv_racing" => %{
        "27R" => {ViaUtils.Location.new_degrees(41.76302, -122.48963, 1186.6), 270}
      },
      "cone_field" => %{
        "18R" => {ViaUtils.Location.new_degrees(42.0, -120.0, 0.0), 180.0},
        "36L" => {ViaUtils.Location.new_degrees(41.76129, -122.49024, 1188.31), 0.0}
      }
    }

    {origin, heading} = get_in(origin_heading, [airport, runway])
    {origin, ViaUtils.Math.deg2rad(heading)}
  end

  @spec get_random_waypoints(binary(), struct(), struct(), integer(), boolean()) :: list()
  def get_random_waypoints(model_type, ground_wp, first_flight_wp, num_wps, loop \\ false) do
    %{
      MS.flight_speed_range_mps() => {min_flight_speed, max_flight_speed},
      MS.flight_agl_range_m() => {min_flight_agl, max_flight_agl},
      MS.wp_dist_range_m() => {min_wp_dist, max_wp_dist}
    } = MS.get_model_spec(model_type)

    %{SVN.groundspeed_mps() => ff_speed, SVN.course_rad() => ff_course} = first_flight_wp

    starting_wp =
      ViaNavigation.Dubins.Waypoint.new_from_location(
        first_flight_wp,
        ff_speed,
        ff_course,
        SWV.gps_altitude(),
        "wp0"
      )

    flight_speed_range = max_flight_speed - min_flight_speed
    flight_agl_range = max_flight_agl - min_flight_agl
    wp_dist_range = max_wp_dist - min_wp_dist

    wps =
      Enum.reduce(1..num_wps, [starting_wp], fn index, acc ->
        last_wp = hd(acc)
        dist = :rand.uniform() * wp_dist_range + min_wp_dist
        bearing = :rand.uniform() * 2 * :math.pi()
        course = :rand.uniform() * 2 * :math.pi()
        speed = :rand.uniform() * flight_speed_range + min_flight_speed
        %{SVN.altitude_m() => ground_alt} = ground_wp
        alt = :rand.uniform() * flight_agl_range + min_flight_agl + ground_alt
        # Logger.debug(ViaUtils.Location.to_string(last_wp))
        # Logger.debug("distance/bearing: #{dist}/#{ViaUtils.Math.rad2deg(bearing)}")
        new_pos =
          ViaUtils.Location.location_from_point_with_distance_bearing(last_wp, dist, bearing)
          |> Map.put(:altitude, alt)

        new_wp =
          ViaNavigation.Dubins.Waypoint.new_from_location(
            new_pos,
            speed,
            course,
            SWV.gps_altitude(),
            "wp#{index}",
            true
          )

        [new_wp | acc]
      end)
      |> Enum.reverse()
      |> Enum.drop(1)

    Logger.debug("before loop")
    # Enum.each(wps, fn wp ->
    #   Logger.debug("wp: #{wp.name}/#{wp.altitude}m")
    # end)
    if loop do
      first_wp = Enum.at(wps, 0)
      last_wp = %{first_wp | goto: starting_wp.name}
      wps ++ [last_wp]
    else
      wps
    end
  end

  # @spec encode(struct(), boolean(), boolean()) :: binary()
  # def encode(mission, confirm \\ false, display \\ false) do
  #   wps = Enum.reduce(mission.waypoints, [], fn (wp, acc) ->
  #     goto= if (wp.goto == ""), do: nil, else: wp.goto
  #     wp_proto = Navigation.Path.Protobuf.Mission.Waypoint.new([
  #     name: wp.name,
  #     latitude: wp.latitude,
  #     longitude: wp.longitude,
  #     altitude: wp.altitude,
  #     speed: wp.speed,
  #     course: wp.course,
  #     goto: goto,
  #     type: to_string(wp.type) |> String.upcase() |> String.to_atom(),
  #     peripheral_control_allowed: wp.peripheral_control_allowed
  #     ])
  #     acc ++ [wp_proto]
  #   end)
  #   mission_proto = Navigation.Path.Protobuf.Mission.new([
  #     name: mission.name,
  #     vehicle_turn_rate: mission.vehicle_turn_rate,
  #     waypoints: wps,
  #     confirm: confirm,
  #     display: display
  #   ])
  #   Navigation.Path.Protobuf.Mission.encode(mission_proto)
  # end

  @spec print_waypoints_relative(struct(), list()) :: atom()
  def print_waypoints_relative(start_position, wps) do
    %{SVN.latitude_rad() => start_lat, SVN.longitude_rad() => start_lon} = start_position

    Enum.each(wps, fn wp ->
      %{
        SVN.latitude_rad() => wp_lat,
        SVN.longitude_rad() => wp_lon,
        SVN.altitude_m() => wp_alt,
        name: wp_name,
        type: type
      } = wp

      {dx, dy} =
        ViaUtils.Location.dx_dy_between_points(
          start_lat,
          start_lon,
          wp_lat,
          wp_lon
        )

      Logger.debug(
        "wp: #{wp_name}/#{type}: (#{ViaUtils.Format.eftb(dx, 0)}, #{ViaUtils.Format.eftb(dy, 0)}, #{ViaUtils.Format.eftb(wp_alt, 0)})m"
      )
    end)
  end
end
