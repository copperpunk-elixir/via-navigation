defmodule ViaNavigation.Dubins.Utils do
  require Logger
  require ViaUtils.Constants, as: VC
  require ViaNavigation.Dubins.Shared.PathCaseValues, as: PCV
  @angle_constrain_deadband 0.001

  @spec check_for_path_case_completion(struct(), struct(), struct()) :: integer()
  def check_for_path_case_completion(position_rrm, current_cp, current_path_case) do
    %{PCV.plane_end_pos_rrm() => zi, PCV.plane_end_vector() => q, PCV.case_index() => case_index} =
      current_path_case

    {dx, dy} = ViaUtils.Location.dx_dy_between_points(zi, position_rrm)
    h = q.x * dx + q.y * dy
    h_pass = h >= 0
    # Logger.debug("index/h/h_pass: #{case_index}/#{h}/#{h_pass}")
    case case_index do
      0 ->
        if h_pass or current_cp.dubins.skip_case_0, do: 1, else: 0

      1 ->
        if h_pass, do: 2, else: 1

      2 ->
        if h_pass do
          if current_cp.dubins.skip_case_3, do: 4, else: 3
        else
          2
        end

      3 ->
        if h_pass, do: 4, else: 3

      4 ->
        if h_pass, do: 5, else: 4
    end
  end

  @spec config_points_from_waypoints(list(), float()) :: tuple()
  def config_points_from_waypoints(waypoints, vehicle_turn_rate) do
    num_wps = length(waypoints)
    Logger.debug("calculate new path with : #{num_wps} waypoints")
    wps_with_index = Enum.with_index(waypoints)

    Enum.reduce(wps_with_index, {[], 0}, fn {wp, index}, {cp_list, total_path_distance} ->
      # Logger.debug("index/max_index: #{index}/#{num_wps-1}")
      if index < num_wps - 1 do
        current_cp = ViaNavigation.Dubins.ConfigPoint.new(wp, vehicle_turn_rate)
        next_wp = Enum.at(waypoints, index + 1)
        next_cp = ViaNavigation.Dubins.ConfigPoint.new(next_wp, vehicle_turn_rate)

        current_cp = %{
          current_cp
          | end_speed_mps: next_cp.start_speed_mps,
            goto_upon_completion: next_wp.goto
        }

        current_cp =
          find_shortest_path_between_config_points(current_cp, next_cp)
          |> set_dubins_parameters()

        {cp_list ++ [current_cp], total_path_distance + current_cp.path_distance_m}
      else
        {cp_list, total_path_distance}
      end
    end)
  end

  @spec find_shortest_path_between_config_points(struct(), struct()) :: struct()
  def find_shortest_path_between_config_points(current_cp, next_cp) do
    # Logger.debug("current/next: #{inspect(current_cp)}/#{inspect(next_cp)}")
    # current_cp
    path_config_points = [
      right_right_path(current_cp, next_cp),
      right_left_path(current_cp, next_cp),
      left_right_path(current_cp, next_cp),
      left_left_path(current_cp, next_cp)
    ]

    cp =
      Enum.reject(path_config_points, &(&1.path_distance_m < 0))
      |> Enum.sort(&(&1.path_distance_m < &2.path_distance_m))
      |> Enum.at(0)

    Logger.warn("Path turn directions: #{orbit_directions_for_path(cp)}")

    if is_nil(cp) do
      Logger.error("No valid paths available")
      nil
    else
      q3 =
        ViaUtils.Math.Vector.new(:math.cos(next_cp.course_rad), :math.sin(next_cp.course_rad), 0)

      theta1 = ViaUtils.Math.constrain_angle_to_compass(current_cp.course_rad)
      theta2 = :math.atan2(cp.q1.y, cp.q1.x) |> ViaUtils.Math.constrain_angle_to_compass()
      skip_case_0 = can_skip_case(theta1, theta2, cp.start_direction)

      # Logger.debug("theta1/theta/skip0?: #{ViaUtils.Format.eftb_deg(theta1,2)}/#{ViaUtils.Format.eftb_deg(theta2,2)}/#{skip_case_0}")

      theta1 = :math.atan2(cp.q1.y, cp.q1.x) |> ViaUtils.Math.constrain_angle_to_compass()
      theta2 = :math.atan2(q3.y, q3.x) |> ViaUtils.Math.constrain_angle_to_compass()
      skip_case_3 = can_skip_case(theta1, theta2, cp.end_direction)

      # Logger.debug("theta1/theta/skip3?: #{ViaUtils.Format.eftb_deg(theta1,2)}/#{ViaUtils.Format.eftb_deg(theta2,2)}/#{skip_case_3}")
      # Logger.debug("start/radius: #{current_cp.start_radius_m}/#{next_cp.start_radius_m}")
      %{
        cp
        | start_radius_m: current_cp.start_radius_m,
          end_radius_m: next_cp.start_radius_m,
          z3: next_cp.pos,
          q3: q3,
          dubins: %{cp.dubins | skip_case_0: skip_case_0, skip_case_3: skip_case_3}
      }
    end
  end

  @spec set_dubins_parameters(struct()) :: struct()
  def set_dubins_parameters(cp) do
    path_case_0 = ViaNavigation.Dubins.PathCase.new_orbit(0, cp.type)

    path_case_0 = %{
      path_case_0
      | PCV.groundspeed_mps() => cp.start_speed_mps,
        PCV.orbit_center_rrm() => cp.cs,
        PCV.orbit_radius_m() => cp.start_radius_m,
        PCV.turn_direction() => cp.start_direction,
        PCV.plane_end_vector() => ViaUtils.Math.Vector.reverse(cp.q1),
        PCV.plane_end_pos_rrm() => cp.z1
    }

    path_case_1 = %{
      path_case_0
      | PCV.case_index() => 1,
        PCV.plane_end_vector() => cp.q1
    }

    path_case_2 = ViaNavigation.Dubins.PathCase.new_line(2, cp.type)

    path_case_2 = %{
      path_case_2
      | PCV.groundspeed_mps() => cp.end_speed_mps,
        PCV.straight_begin_pos_rrm() => cp.z1,
        PCV.plane_end_vector() => cp.q1,
        PCV.plane_end_pos_rrm() => cp.z2
    }

    path_case_3 = ViaNavigation.Dubins.PathCase.new_orbit(3, cp.type)

    path_case_3 = %{
      path_case_3
      | PCV.groundspeed_mps() => cp.end_speed_mps,
        PCV.orbit_center_rrm() => cp.ce,
        PCV.orbit_radius_m() => cp.end_radius_m,
        PCV.turn_direction() => cp.end_direction,
        PCV.plane_end_vector() => ViaUtils.Math.Vector.reverse(cp.q3),
        PCV.plane_end_pos_rrm() => cp.z3
    }

    path_case_4 = %{
      path_case_3
      | PCV.case_index() => 4,
        PCV.plane_end_vector() => cp.q3
    }

    path_cases = [path_case_0, path_case_1, path_case_2, path_case_3, path_case_4]
    %{cp | dubins: %{cp.dubins | path_cases: path_cases}}
  end

  @spec can_skip_case(float(), float(), integer()) :: boolean()
  def can_skip_case(theta1, theta2, direction) do
    if abs(theta1 - theta2) < 0.0175 or abs(theta1 - theta2) > 6.2657 do
      true
    else
      theta_diff =
        if direction < 0 do
          theta1 = if theta1 < theta2, do: theta1 + VC.two_pi(), else: theta1
          theta1 - theta2
        else
          theta2 = if theta2 < theta1, do: theta2 + VC.two_pi(), else: theta2
          theta2 - theta1
        end

      if theta_diff < VC.pi_2(), do: true, else: false
    end
  end

  @spec right_right_path(struct(), struct()) :: struct()
  def right_right_path(cp1, cp2) do
    radius1 = cp1.start_radius_m
    radius2 = cp2.start_radius_m
    # Right Start
    crs =
      ViaUtils.Location.location_from_point_with_distance_bearing(
        cp1.pos,
        radius1,
        cp1.course_rad + VC.pi_2()
      )

    # Right End
    cre =
      ViaUtils.Location.location_from_point_with_distance_bearing(
        cp2.pos,
        radius2,
        cp2.course_rad + VC.pi_2()
      )

    {dx, dy} = ViaUtils.Location.dx_dy_between_points(crs, cre)
    ell = ViaUtils.Math.hypot(dx, dy)
    # Logger.debug("ell/r1/r2: #{ell}/#{radius1}/#{radius2}")
    # Logger.debug("dx/dy: #{dx}/#{dy}")

    # Logger.debug(
    #   "crs/cre: #{ViaUtils.Location.to_string(crs)}/#{ViaUtils.Location.to_string(cre)}"
    # )

    if ell > abs(radius1 - radius2) do
      gamma =
        if dy == 0 do
          -VC.pi_2()
        else
          -:math.atan(dx / dy)
        end

      beta = :math.asin((radius2 - radius1) / ell)
      alpha = gamma - beta
      # Logger.debug("alpha/beta/gamma: #{alpha}/#{beta}/#{gamma}")
      a3 = ViaUtils.Location.location_from_point_with_distance_bearing(crs, radius1, alpha)
      a4 = ViaUtils.Location.location_from_point_with_distance_bearing(cre, radius2, alpha)
      cs_to_p3 = ViaUtils.Location.dx_dy_between_points(crs, a3)
      p3_to_p4 = ViaUtils.Location.dx_dy_between_points(a3, a4)
      cross_a = ViaUtils.Math.cross_product(p3_to_p4, cs_to_p3)

      {line_start, line_end, v2} =
        if cross_a < 0 do
          line_start = a3
          line_end = a4
          {line_start, line_end, alpha}
        else
          line_start =
            ViaUtils.Location.location_from_point_with_distance_bearing(crs, -radius1, alpha)

          line_end =
            ViaUtils.Location.location_from_point_with_distance_bearing(cre, -radius2, alpha)

          {line_start, line_end, :math.pi() + alpha}
        end

      {lsle_dx, lsle_dy} = ViaUtils.Location.dx_dy_between_points(line_start, line_end)
      s1 = ViaUtils.Math.hypot(lsle_dx, lsle_dy)
      s2 =
        ViaUtils.Math.constrain_angle_to_compass_with_deadband(
          v2 - (cp1.course_rad - VC.pi_2()),
          @angle_constrain_deadband
        )
        |> Kernel.*(radius1)
        |> Kernel.abs()

      s3 =
        ViaUtils.Math.constrain_angle_to_compass_with_deadband(
          cp2.course_rad - VC.pi_2() - v2,
          @angle_constrain_deadband
        )
        |> Kernel.*(radius2)
        |> Kernel.abs()

      path_distance = s1 + s2 + s3

      # angle1 = v2 - (cp1.course_rad - VC.pi_2())
      # angle2 = cp2.course_rad - VC.pi_2() - v2
      # # Logger.debug(
      #   "s2 angle: #{angle1}/#{ViaUtils.Math.constrain_angle_to_compass_with_deadband(angle1, 0.001)}"
      # )

      # Logger.debug(
      #   "s3 angle: #{angle2}/#{ViaUtils.Math.constrain_angle_to_compass_with_deadband(angle2, 0.001)}"
      # )

      # Logger.debug("cp crs 1/2/v2 #{cp1.course_rad}/#{cp2.course_rad}/#{v2}")
      # Logger.debug("RR s1/s2/s3/tot: #{s1}/#{s2}/#{s3}/#{path_distance}")

      q1 =
        ViaUtils.Math.Vector.new(
          lsle_dx / s1,
          lsle_dy / s1,
          (line_end.altitude_m - line_start.altitude_m) / s1
        )

      %{
        cp1
        | cs: crs,
          start_direction: 1,
          ce: cre,
          end_direction: 1,
          q1: q1,
          z1: line_start,
          z2: line_end,
          path_distance_m: path_distance
      }
    else
      %ViaNavigation.Dubins.ConfigPoint{path_distance_m: -1}
    end
  end

  @spec right_left_path(struct(), struct()) :: struct()
  def right_left_path(cp1, cp2) do
    radius1 = cp1.start_radius_m
    radius2 = cp2.start_radius_m
    # Right Start
    crs =
      ViaUtils.Location.location_from_point_with_distance_bearing(
        cp1.pos,
        radius1,
        cp1.course_rad + VC.pi_2()
      )

    # Left End
    cle =
      ViaUtils.Location.location_from_point_with_distance_bearing(
        cp2.pos,
        radius2,
        cp2.course_rad - VC.pi_2()
      )

    {dx, dy} = ViaUtils.Location.dx_dy_between_points(crs, cle)
    xL = ViaUtils.Math.hypot(dx, dy)

    if xL >= radius1 + radius2 do
      xL1 = xL * radius1 / (radius1 + radius2)
      xL2 = xL * radius2 / (radius1 + radius2)
      straight1 = xL1 * xL1 - radius1 * radius1
      straight2 = xL2 * xL2 - radius2 * radius2
      v = ViaUtils.Location.angle_between_points(crs, cle)
      # Logger.debug("v: #{v}")
      v2 = v - VC.pi_2() + :math.asin((radius1 + radius2) / xL)
      # Logger.debug("v2: #{v2}")
      s1 = :math.sqrt(straight1) + :math.sqrt(straight2)

      s2 =
        ViaUtils.Math.constrain_angle_to_compass_with_deadband(
          VC.two_pi() + ViaUtils.Math.constrain_angle_to_compass(v2) -
            ViaUtils.Math.constrain_angle_to_compass(cp1.course_rad - VC.pi_2()),
          @angle_constrain_deadband
        )
        |> Kernel.*(radius1)
        |> Kernel.abs()

      s3 =
        ViaUtils.Math.constrain_angle_to_compass_with_deadband(
          VC.two_pi() + ViaUtils.Math.constrain_angle_to_compass(v2 + :math.pi()) -
            ViaUtils.Math.constrain_angle_to_compass(cp2.course_rad + VC.pi_2()),
          @angle_constrain_deadband
        )
        |> Kernel.*(radius2)
        |> Kernel.abs()

      path_distance = s1 + s2 + s3
      # Logger.debug("RL s1/s2/s3/tot: #{s1}/#{s2}/#{s3}/#{path_distance}")

      q1 =
        ViaUtils.Math.Vector.new(
          :math.cos(v2 + VC.pi_2()),
          :math.sin(v2 + VC.pi_2()),
          (cle.altitude_m - crs.altitude_m) / s1
        )

      z1 = ViaUtils.Location.location_from_point_with_distance_bearing(crs, radius1, v2)

      z2 =
        ViaUtils.Location.location_from_point_with_distance_bearing(cle, radius2, v2 + :math.pi())

      %{
        cp1
        | cs: crs,
          start_direction: 1,
          ce: cle,
          end_direction: -1,
          q1: q1,
          z1: z1,
          z2: z2,
          path_distance_m: path_distance
      }
    else
      %ViaNavigation.Dubins.ConfigPoint{path_distance_m: -1}
    end
  end

  @spec left_right_path(struct(), struct()) :: struct()
  def left_right_path(cp1, cp2) do
    radius1 = cp1.start_radius_m
    radius2 = cp2.start_radius_m
    # Left Start
    cls =
      ViaUtils.Location.location_from_point_with_distance_bearing(
        cp1.pos,
        radius1,
        cp1.course_rad - VC.pi_2()
      )

    # Right End
    cre =
      ViaUtils.Location.location_from_point_with_distance_bearing(
        cp2.pos,
        radius2,
        cp2.course_rad + VC.pi_2()
      )

    {dx, dy} = ViaUtils.Location.dx_dy_between_points(cls, cre)
    xL = ViaUtils.Math.hypot(dx, dy)

    if xL >= radius1 + radius2 do
      xL1 = xL * radius1 / (radius1 + radius2)
      xL2 = xL * radius2 / (radius1 + radius2)
      straight1 = xL1 * xL1 - radius1 * radius1
      straight2 = xL2 * xL2 - radius2 * radius2
      v = ViaUtils.Location.angle_between_points(cls, cre)
      # Logger.debug("v: #{v}")
      v2 = :math.acos((radius1 + radius2) / xL)
      # Logger.debug("v2: #{v2}")
      s1 = :math.sqrt(straight1) + :math.sqrt(straight2)

      s2 =
        ViaUtils.Math.constrain_angle_to_compass_with_deadband(
          VC.two_pi() + ViaUtils.Math.constrain_angle_to_compass(cp1.course_rad + VC.pi_2()) -
            ViaUtils.Math.constrain_angle_to_compass(v + v2),
          @angle_constrain_deadband
        )
        |> Kernel.*(radius1)
        |> Kernel.abs()

      s3 =
        ViaUtils.Math.constrain_angle_to_compass_with_deadband(
          VC.two_pi() + ViaUtils.Math.constrain_angle_to_compass(cp2.course_rad - VC.pi_2()) -
            ViaUtils.Math.constrain_angle_to_compass(v + v2 - :math.pi()),
          @angle_constrain_deadband
        )
        |> Kernel.*(radius2)
        |> Kernel.abs()

      path_distance = s1 + s2 + s3
      # Logger.debug("LR s1/s2/s3/tot: #{s1}/#{s2}/#{s3}/#{path_distance}")

      q1 =
        ViaUtils.Math.Vector.new(
          :math.cos(v + v2 - VC.pi_2()),
          :math.sin(v + v2 - VC.pi_2()),
          (cre.altitude_m - cls.altitude_m) / s1
        )

      z1 = ViaUtils.Location.location_from_point_with_distance_bearing(cls, radius1, v + v2)

      z2 =
        ViaUtils.Location.location_from_point_with_distance_bearing(
          cre,
          radius2,
          v + v2 - :math.pi()
        )

      %{
        cp1
        | cs: cls,
          start_direction: -1,
          ce: cre,
          end_direction: 1,
          q1: q1,
          z1: z1,
          z2: z2,
          path_distance_m: path_distance
      }
    else
      %ViaNavigation.Dubins.ConfigPoint{path_distance_m: -1}
    end
  end

  @spec left_left_path(struct(), struct()) :: struct()
  def left_left_path(cp1, cp2) do
    radius1 = cp1.start_radius_m
    radius2 = cp2.start_radius_m
    # Left Start
    cls =
      ViaUtils.Location.location_from_point_with_distance_bearing(
        cp1.pos,
        radius1,
        cp1.course_rad - VC.pi_2()
      )

    # Left End
    cle =
      ViaUtils.Location.location_from_point_with_distance_bearing(
        cp2.pos,
        radius2,
        cp2.course_rad - VC.pi_2()
      )

    {dx, dy} = ViaUtils.Location.dx_dy_between_points(cls, cle)
    ell = ViaUtils.Math.hypot(dx, dy)

    if ell > abs(radius1 - radius2) do
      gamma =
        if dy == 0 do
          -VC.pi_2()
        else
          -:math.atan(dx / dy)
        end

      beta = :math.asin((radius2 - radius1) / ell)
      alpha = gamma + beta
      a3 = ViaUtils.Location.location_from_point_with_distance_bearing(cls, radius1, alpha)
      a4 = ViaUtils.Location.location_from_point_with_distance_bearing(cle, radius2, alpha)
      cs_to_p3 = ViaUtils.Location.dx_dy_between_points(cls, a3)
      p3_to_p4 = ViaUtils.Location.dx_dy_between_points(a3, a4)
      cross_a = ViaUtils.Math.cross_product(p3_to_p4, cs_to_p3)

      {line_start, line_end, v2} =
        if cross_a > 0 do
          line_start = a3
          line_end = a4
          {line_start, line_end, :math.pi() + alpha}
        else
          line_start =
            ViaUtils.Location.location_from_point_with_distance_bearing(cls, -radius1, alpha)

          line_end =
            ViaUtils.Location.location_from_point_with_distance_bearing(cle, -radius2, alpha)

          {line_start, line_end, alpha}
        end

      {lsle_dx, lsle_dy} = ViaUtils.Location.dx_dy_between_points(line_start, line_end)
      s1 = ViaUtils.Math.hypot(lsle_dx, lsle_dy)

      s2 =
        ViaUtils.Math.constrain_angle_to_compass_with_deadband(
          cp1.course_rad - VC.pi_2() - v2,
          @angle_constrain_deadband
        )
        |> Kernel.*(radius1)
        |> Kernel.abs()

      s3 =
        ViaUtils.Math.constrain_angle_to_compass_with_deadband(
          v2 - (cp2.course_rad - VC.pi_2()),
          @angle_constrain_deadband
        )
        |> Kernel.*(radius2)
        |> Kernel.abs()

      path_distance = s1 + s2 + s3
      # Logger.debug("LL s1/s2/s3/tot: #{s1}/#{s2}/#{s3}/#{path_distance}")

      q1 =
        ViaUtils.Math.Vector.new(
          lsle_dx / s1,
          lsle_dy / s1,
          (cle.altitude_m - cls.altitude_m) / s1
        )

      %{
        cp1
        | cs: cls,
          start_direction: -1,
          ce: cle,
          end_direction: -1,
          q1: q1,
          z1: line_start,
          z2: line_end,
          path_distance_m: path_distance
      }
    else
      %ViaNavigation.Dubins.ConfigPoint{path_distance_m: -1}
    end
  end

  def orbit_directions_for_path(config_point) do
    start_dir = if config_point.start_direction == 1, do: "Right", else: "Left"
    end_dir = if config_point.end_direction == 1, do: "Right", else: "Left"
    "#{start_dir}-#{end_dir}"
  end
end
