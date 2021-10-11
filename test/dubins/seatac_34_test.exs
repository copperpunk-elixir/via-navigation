defmodule Dubins.Seatac34Test do
  use ExUnit.Case
  require Logger
  require ViaNavigation.Dubins.Shared.MissionValues, as: MV

  test "Seatac 34" do
    mission = ViaNavigation.Dubins.Mission.Prebuilt.get_seatac_34L(0)
    %{MV.waypoints() => waypoints} = mission

    Enum.each(waypoints, fn wp ->
      Logger.debug(ViaNavigation.Dubins.Waypoint.to_string(wp))
    end)
  end
end
