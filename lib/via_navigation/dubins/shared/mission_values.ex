defmodule ViaNavigation.Dubins.Shared.MissionValues do
  require ViaNavigation.Shared.MissionValues, as: MV

  defmacro altitude_ref_m, do: MV.altitude_ref_m()
  defmacro model_spec, do: MV.model_spec()
  defmacro name(), do: MV.name()
  defmacro path_type(), do: MV.path_type()
  defmacro turn_rate_rps(), do: :turn_rate_rps
  defmacro waypoints(), do: MV.waypoints()
end
