defmodule ViaNavigation.Mission do
  require ViaNavigation.Dubins.Shared.MissionValues, as: MV

  defstruct [MV.name(), MV.path_type(), MV.waypoints(), MV.model_spec(), MV.altitude_ref_m()]

  @spec new(binary(), binary(), list(), number(), map()) :: struct()
  def new(name, path_type, waypoints, altitude_ref, model_spec) do

    %ViaNavigation.Mission{
      MV.name() => name,
      MV.path_type() => path_type,
      MV.waypoints() => waypoints,
      MV.model_spec() => model_spec,
      MV.altitude_ref_m() => altitude_ref
    }
  end
end
