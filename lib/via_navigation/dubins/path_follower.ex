defmodule ViaNavigation.Dubins.PathFollower do
  require Logger
  require ViaUtils.Constants, as: VC
  require ViaNavigation.Dubins.Shared.PathCaseValues, as: PCV
  require ViaNavigation.Dubins.Shared.PathFollowerValues, as: PFV
  require ViaNavigation.Dubins.Shared.WaypointsValues, as: SWV
  require ViaUtils.Shared.ValueNames, as: SVN
  # @pi_2 1.5708#79633267948966
  # @two_pi 6.2832#185307179586

  defstruct [PFV.k_path(), PFV.k_orbit(), PFV.chi_inf_rad_two_over_pi(), PFV.lookahead_dt_s()]

  @spec new(list) :: struct()
  def new(config) do
    new(
      Keyword.fetch!(config, PFV.k_path()),
      Keyword.fetch!(config, PFV.k_orbit()),
      Keyword.fetch!(config, PFV.chi_inf_rad()),
      Keyword.fetch!(config, PFV.lookahead_dt_s())
    )
  end

  @spec new(float(), float(), float(), float()) :: struct()
  def new(k_path, k_orbit, chi_inf_rad, lookahead_dt_s) do
    %ViaNavigation.Dubins.PathFollower{
      PFV.k_path() => k_path,
      PFV.k_orbit() => k_orbit,
      PFV.chi_inf_rad_two_over_pi() => chi_inf_rad * 2.0 / :math.pi(),
      PFV.lookahead_dt_s() => lookahead_dt_s
    }
  end

  @spec follow(struct(), struct(), number(), number(), struct()) :: tuple()
  def follow(path_follower, position_rrm, course_rad, speed_mps, path_case) do
    if Map.fetch!(path_case, PCV.flag()) == ViaNavigation.Dubins.PathCase.line_flag() do
      %{
        PFV.k_path() => k_path,
        PFV.chi_inf_rad_two_over_pi() => chi_inf_rad_two_over_pi,
        PFV.lookahead_dt_s() => lookahead_dt_s
      } = path_follower

      position_rrm = get_lookahead_position(position_rrm, speed_mps, course_rad, lookahead_dt_s)

      %{
        PCV.type() => case_type,
        PCV.straight_begin_pos_rrm() => straight_begin_pos_rrm,
        PCV.plane_end_pos_rrm() => plane_end_pos_rrm,
        PCV.plane_end_vector() => plane_end_vector,
        PCV.groundspeed_mps() => v_cmd
      } = path_case

      %{SVN.altitude_m() => straight_begin_alt} = straight_begin_pos_rrm

      %{SVN.altitude_m() => plane_end_alt} = plane_end_pos_rrm

      {dx, dy} = ViaUtils.Location.dx_dy_between_points(straight_begin_pos_rrm, position_rrm)

      # Logger.debug(
      #   "start/end/curr alt: #{ViaUtils.Format.eftb(straight_begin_alt, 1)}/#{ViaUtils.Format.eftb(plane_end_alt, 1)}/#{ViaUtils.Format.eftb(position_rrm.altitude_m, 1)}"
      # )

      # Logger.debug("post look: #{dx}/#{dy}")
      # Add lookahead

      %{x: qx, y: qy, z: qz} = plane_end_vector
      temp_vector = qx * dy - qy * dx
      si1 = dx + qy * temp_vector
      si2 = dy - qx * temp_vector

      # Logger.debug("r.alt/ q.z / si1 / si2: #{straight_begin_pos_rrm.altitude_m}/#{qz}/#{si1}/#{si2}")

      d_altitude_cmd =
        if case_type == SWV.agl_altitude() and plane_end_alt < straight_begin_alt do
          landing_distance =
            ViaUtils.Location.dx_dy_between_points(straight_begin_pos_rrm, plane_end_pos_rrm)
            |> ViaUtils.Math.hypot()

          d_alt_landing = plane_end_alt - straight_begin_alt

          landing_distance_travelled = ViaUtils.Math.hypot(si1, si2)

          Logger.debug(
            "landing: #{ViaUtils.Format.eftb(landing_distance_travelled, 1)}/#{ViaUtils.Format.eftb(landing_distance, 1)}/#{ViaUtils.Format.eftb(d_alt_landing, 1)}"
          )

          0.5 * d_alt_landing *
            (:math.cos(:math.pi() * (1.0 - landing_distance_travelled / landing_distance)) + 1)
        else
          qz * ViaUtils.Math.hypot(si1, si2) / ViaUtils.Math.hypot(qx, qy)
        end

      # Logger.debug("d_alt: #{d_altitude_cmd}")
      altitude_cmd = straight_begin_alt + d_altitude_cmd

      chi_q = :math.atan2(qy, qx)

      chi_q =
        cond do
          chi_q - course_rad < -:math.pi() -> chi_q + VC.two_pi()
          chi_q - course_rad > :math.pi() -> chi_q - VC.two_pi()
          true -> chi_q
        end

      sin_chi_q = :math.sin(chi_q)
      cos_chi_q = :math.cos(chi_q)

      # e_px = cos_chi_q*dx + sin_chi_q*dy
      e_py = -sin_chi_q * dx + cos_chi_q * dy

      course_cmd =
        (chi_q - chi_inf_rad_two_over_pi * :math.atan(k_path * e_py))
        |> ViaUtils.Math.constrain_angle_to_compass()

      # Logger.debug("e_py/course_cmd: #{ViaUtils.Format.eftb(e_py,2)}/#{ViaUtils.Format.eftb_deg(course_cmd,1)}")
      # d_course = ViaUtils.Motion.turn_left_or_right_for_correction(course_cmd - course_rad)

      # Logger.debug(
      #   "e_py/course_cmd: #{ViaUtils.Format.eftb(e_py, 3)}/#{ViaUtils.Format.eftb_deg(d_course, 2)}"
      # )

      {v_cmd, course_cmd, altitude_cmd}
    else
      %{
        PFV.k_orbit() => k_orbit,
        PFV.lookahead_dt_s() => lookahead_dt_s
      } = path_follower

      %{
        PCV.case_index() => case_index,
        PCV.turn_direction() => turn_direction,
        PCV.orbit_center_rrm() => orbit_center_rrm,
        PCV.orbit_radius_m() => orbit_radius_m,
        PCV.plane_end_pos_rrm() => plane_end_pos_rrm,
        PCV.groundspeed_mps() => v_cmd
      } = path_case

      altitude_cmd = Map.fetch!(orbit_center_rrm, SVN.altitude_m())

      distance_to_z1 =
        if is_nil(plane_end_pos_rrm) do
          1_000_000_000
        else
          ViaUtils.Location.dx_dy_between_points(plane_end_pos_rrm, position_rrm)
          |> ViaUtils.Math.hypot()
        end

      # Only use lookahead position if we are far enough away from the transition to the next path case
      lookahead_position =
        if abs(distance_to_z1) > speed_mps * 1.0 or case_index == 0 or
             case_index == 3 do
          get_lookahead_position(position_rrm, speed_mps, course_rad, lookahead_dt_s)
        else
          position_rrm
        end

      {dx, dy} = ViaUtils.Location.dx_dy_between_points(orbit_center_rrm, lookahead_position)
      # Logger.debug("post look: #{dx}/#{dy}")
      orbit_d = ViaUtils.Math.hypot(dx, dy)
      phi = :math.atan2(dy, dx)

      phi =
        cond do
          phi - course_rad < -:math.pi() -> phi + VC.two_pi()
          phi - course_rad > :math.pi() -> phi - VC.two_pi()
          true -> phi
        end

      course_cmd =
        (phi +
           turn_direction *
             (VC.pi_2() +
                :math.atan(k_orbit * (orbit_d - orbit_radius_m) / orbit_radius_m)))
        |> ViaUtils.Math.constrain_angle_to_compass()

      # e_py = orbit_d - path_case.rho

      # Logger.debug("orbit_d/rho: #{ViaUtils.Format.eftb(orbit_d,2)}/#{ViaUtils.Format.eftb(path_case.rho,2)}")
      # d_course = ViaUtils.Motion.turn_left_or_right_for_correction(course_cmd - course_rad)

      # Logger.debug(
      #   "e_py/course_cmd: #{ViaUtils.Format.eftb(e_py, 2)}/#{ViaUtils.Format.eftb_deg(d_course, 1)}"
      # )

      # Logger.debug("#{ViaUtils.Format.eftb_deg(course_cmd, 1)}/#{ViaUtils.Format.eftb_deg(course, 1)}")
      {v_cmd, course_cmd, altitude_cmd}
    end
  end

  @spec get_lookahead_position(struct(), float(), float(), float()) :: struct()
  def get_lookahead_position(position_rrm, speed_mps, course_rad, dt_s) do
    look_dx = speed_mps * dt_s * :math.cos(course_rad)
    look_dy = speed_mps * dt_s * :math.sin(course_rad)
    ViaUtils.Location.location_from_point_with_dx_dy(position_rrm, look_dx, look_dy)
  end
end
