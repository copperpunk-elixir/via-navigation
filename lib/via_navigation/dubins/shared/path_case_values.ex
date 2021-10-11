defmodule ViaNavigation.Dubins.Shared.PathCaseValues do
  defmacro case_index(), do: :case_index
  defmacro flag(), do: :flag
  defmacro groundspeed_mps, do: :v_des
  defmacro orbit_center_rrm(), do: :c
  defmacro orbit_radius_m(), do: :rho
  defmacro plane_end_pos_rrm, do: :zi
  defmacro plane_end_vector(), do: :q
  defmacro straight_begin_pos_rrm(), do: :r
  defmacro turn_direction(), do: :turn_direction
  defmacro type(), do: :type
end
