defmodule ViaNavigation.Mission do
  defstruct [:name, :waypoints, :turn_rate_rps]

  @spec new_mission(binary(), list(), number()) :: struct()
  def new_mission(name, waypoints, turn_rate_rps) do
    %ViaNavigation.Mission{
      name: name,
      waypoints: waypoints,
      turn_rate_rps: turn_rate_rps
    }
  end
end
