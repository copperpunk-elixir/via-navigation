defmodule ViaNavigation.Dubins.Waypoint do
  require Logger
  require ViaUtils.Shared.ValueNames, as: SVN
  require ViaNavigation.Dubins.Shared.WaypointsValues, as: SWV

  defstruct [
    SVN.latitude_rad(),
    SVN.longitude_rad(),
    SVN.groundspeed_mps(),
    SVN.course_rad(),
    SVN.altitude_m(),
    :name,
    :type,
    :goto,
    :peripheral_control_allowed
  ]

  @spec new(
          number(),
          number(),
          number(),
          number(),
          number(),
          atom(),
          binary(),
          any(),
          boolean()
        ) :: struct()
  def new(
        latitude,
        longitude,
        altitude,
        speed,
        course,
        type,
        name \\ "",
        goto \\ nil,
        peripheral_control_allowed \\ false
      ) do
    %ViaNavigation.Dubins.Waypoint{
      SVN.latitude_rad() => latitude,
      SVN.longitude_rad() => longitude,
      SVN.altitude_m() => altitude,
      SVN.groundspeed_mps() => speed,
      SVN.course_rad() => course,
      name: name,
      type: type,
      goto: goto,
      peripheral_control_allowed: peripheral_control_allowed
    }
  end

  @spec new_from_location(struct(), number(), number(), atom(), binary(), any(), boolean()) ::
          struct()
  def new_from_location(
        location,
        speed,
        course,
        type,
        name \\ "",
        goto \\ nil,
        peripheral_control_allowed \\ false
      ) do
    %{
      SVN.latitude_rad() => latitude_rad,
      SVN.longitude_rad() => longitude_rad,
      SVN.altitude_m() => altitude_m
    } = location

    new(
      latitude_rad,
      longitude_rad,
      altitude_m,
      speed,
      course,
      type,
      name,
      goto,
      peripheral_control_allowed
    )
  end

  @spec new_flight(struct(), number(), number(), binary(), any()) :: struct()
  def new_flight(lla, speed, course, name \\ "", goto \\ nil) do
    new_from_location(lla, speed, course, SWV.flight(), name, goto, false)
  end

  @spec new_flight_peripheral(struct(), number(), number(), binary(), any()) :: struct()
  def new_flight_peripheral(lla, speed, course, name \\ "", goto \\ nil) do
    new_from_location(lla, speed, course, SWV.flight(), name, goto, true)
  end

  @spec new_ground(struct(), number(), number(), binary(), any()) :: struct()
  def new_ground(lla, speed, course, name \\ "", goto \\ nil) do
    new_from_location(lla, speed, course, SWV.ground(), name, goto, false)
  end

  @spec new_ground_peripheral(struct(), number(), number(), binary(), any()) :: struct()
  def new_ground_peripheral(lla, speed, course, name \\ "", goto \\ nil) do
    new_from_location(lla, speed, course, SWV.ground(), name, goto, true)
  end

  @spec new_climbout(struct(), number(), number(), binary(), any()) :: struct()
  def new_climbout(lla, speed, course, name \\ "", goto \\ nil) do
    new_from_location(lla, speed, course, SWV.climbout(), name, goto)
  end

  @spec new_landing(struct(), number(), number(), binary(), any()) :: struct()
  def new_landing(lla, speed, course, name \\ "", goto \\ nil) do
    new_from_location(lla, speed, course, SWV.landing(), name, goto)
  end

  @spec new_approach(struct(), number(), number(), binary(), any()) :: struct()
  def new_approach(lla, speed, course, name \\ "", goto \\ nil) do
    new_from_location(lla, speed, course, SWV.approach(), name, goto)
  end

  @spec new_takeoff(struct(), number(), number(), binary(), any()) :: struct()
  def new_takeoff(lla, speed, course, name \\ "", goto \\ nil) do
    new_from_location(lla, speed, course, SWV.takeoff(), name, goto)
  end

  @spec new_takeoff_peripheral(struct(), number(), number(), binary(), any()) :: struct()
  def new_takeoff_peripheral(lla, speed, course, name \\ "", goto \\ nil) do
    new_from_location(lla, speed, course, SWV.takeoff(), name, goto, true)
  end


  @spec to_string(struct()) :: binary()
  def to_string(wp) do
    %{
      SVN.latitude_rad() => lat,
      SVN.longitude_rad() => lon,
      SVN.altitude_m() => alt,
      SVN.groundspeed_mps() => speed,
      SVN.course_rad() => course,
      name: name
    } = wp

    lla = ViaUtils.Location.new(lat, lon, alt)
    line1 = "wp #{inspect(name)}: #{ViaUtils.Location.to_string(lla)}"

    line2 =
      "Speed/Course: #{ViaUtils.Format.eftb(speed, 1)}/#{ViaUtils.Format.eftb_deg(course, 1)}"

    line1 <> ", " <> line2
  end
end
