defmodule ViaNavigation.Mission do
  require ViaNavigation.Dubins.Shared.MissionValues, as: MV
  defstruct [MV.name(), MV.path_type(), MV.waypoints(), MV.turn_rate_rps()]

  @spec new(binary(), binary(), list(), number()) :: struct()
  def new(name, path_type, waypoints, turn_rate_rps) do
    %ViaNavigation.Mission{
      MV.name() => name,
      MV.path_type() => path_type,
      MV.waypoints() => waypoints,
      MV.turn_rate_rps() => turn_rate_rps
    }
  end
end
