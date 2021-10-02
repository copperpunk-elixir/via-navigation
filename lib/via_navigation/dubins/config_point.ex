defmodule ViaNavigation.Dubins.ConfigPoint do
  require Logger

  defstruct [
    :name,
    :pos,
    :start_direction,
    :end_direction,
    :cs,
    :ce,
    :q1,
    :q3,
    :z1,
    :z2,
    :z3,
    :path_distance_m,
    :course_rad,
    :start_speed_mps,
    :end_speed_mps,
    :start_radius_m,
    :end_radius_m,
    :dubins,
    :goto_upon_completion,
    :type,
    :peripheral_control_allowed
  ]

  @spec new(struct(), number()) :: struct()
  def new(waypoint, vehicle_turn_rate_rps) do
    radius = if (waypoint.groundspeed_mps < 0.5), do: 1000.0, else: waypoint.groundspeed_mps/vehicle_turn_rate_rps
    # Logger.debug("new waypoint. speed/turn_rate/radius: #{waypoint.speed}/#{vehicle_turn_rate}/#{radius}")
    %ViaNavigation.Dubins.ConfigPoint{
      name: waypoint.name,
      pos: ViaUtils.Location.new(waypoint.latitude_rad, waypoint.longitude_rad, waypoint.altitude_m),
      start_speed_mps: waypoint.groundspeed_mps,
      course_rad: waypoint.course_rad,
      start_radius_m: radius,
      dubins: ViaNavigation.Dubins.Path.new(),
      type: waypoint.type,
      peripheral_control_allowed: waypoint.peripheral_control_allowed
    }
  end

  @spec to_string(struct()) :: binary()
  def to_string(cp) do
    line1 = "cp #{inspect(cp.name)}: #{ViaUtils.Location.to_string(cp.pos)}"
    line2 = "Speed/Course/Radius: #{ViaUtils.Format.eftb(cp.start_speed_mps, 1)}/#{ViaUtils.Format.eftb_deg(cp.course_rad,1)}/#{ViaUtils.Format.eftb(cp.start_radius_m,1)}"
    line1 <> ", " <> line2
  end
end
