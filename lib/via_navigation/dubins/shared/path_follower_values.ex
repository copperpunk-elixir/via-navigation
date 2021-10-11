defmodule ViaNavigation.Dubins.Shared.PathFollowerValues do
  defmacro k_path(), do: :k_path
  defmacro k_orbit(), do: :k_orbit
  defmacro chi_inf_rad(), do: :chi_inf_rad
  defmacro chi_inf_rad_two_over_pi(), do: :chi_inf_two_over_pi
  defmacro lookahead_dt_s(), do: :lookahead_dt_s
end
