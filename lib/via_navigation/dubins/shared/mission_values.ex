defmodule ViaNavigation.Dubins.Shared.MissionValues do
  require ViaNavigation.Shared.MissionValues, as: MV
  defmacro name(), do: MV.name()
  defmacro turn_rate_rps(), do: :turn_rate_rps
  defmacro path_type(), do: MV.path_type()
  defmacro waypoints(), do: MV.waypoints()
end
