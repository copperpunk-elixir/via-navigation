defmodule Dubins.Montague36Test do
  use ExUnit.Case
  require Logger
  require ViaNavigation.Dubins.Shared.MissionValues, as: MV

  test "Montague 36" do
    mission = ViaNavigation.Dubins.Mission.Prebuilt.get_montague_36L(0)
    %{MV.waypoints() => waypoints} = mission

    Enum.each(waypoints, fn wp ->
      Logger.debug(ViaNavigation.Dubins.Waypoint.to_string(wp))
    end)
  end
end
