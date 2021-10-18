defmodule ViaNavigation.Dubins.Shared.WaypointsValues do
  defmacro approach(), do: :approach
  defmacro climbout(), do: :climbout
  defmacro flight(), do: :flight
  defmacro ground(), do: :ground
  defmacro landing(), do: :landing
  defmacro takeoff(), do: :takeoff
end
