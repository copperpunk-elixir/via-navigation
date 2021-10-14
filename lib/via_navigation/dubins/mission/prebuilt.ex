defmodule ViaNavigation.Dubins.Mission.Prebuilt do
  require Logger

  @spec get_seatac_34L(integer()) :: struct()
  def get_seatac_34L(num_wps \\ 0) do
    ViaNavigation.Dubins.Mission.Builder.get_complete_mission(
      "seatac",
      "34L",
      "Skyhawk",
      "none",
      num_wps,
      true
    )
  end

  @spec get_flight_school_18L(integer()) :: struct()
  def get_flight_school_18L(num_wps \\ 0) do
    ViaNavigation.Dubins.Mission.Builder.get_complete_mission(
      "flight_school",
      "18L",
      "CessnaZ2m",
      "none",
      num_wps,
      true
    )
  end



@spec get_montague_36L(integer()) :: struct()
  def get_montague_36L(num_wps \\ 0) do
    ViaNavigation.Dubins.Mission.Builder.get_complete_mission(
      "montague",
      "36L",
      "CessnaZ2m",
      "none",
      num_wps,
      true
    )
  end

end
