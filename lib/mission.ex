defmodule ViaNavigation.Mission do
  require ViaNavigation.Dubins.Shared.MissionValues, as: MV
  defstruct [MV.name(), MV.waypoints(), MV.turn_rate_rps()]

  @spec new(binary(), list(), number()) :: struct()
  def new(name, waypoints, turn_rate_rps) do
    %ViaNavigation.Mission{
      MV.name() => name,
      MV.waypoints() => waypoints,
      MV.turn_rate_rps() => turn_rate_rps
    }
  end
end
