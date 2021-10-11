defmodule ViaNavigation.Dubins.PathCase do
  require Logger
  require ViaNavigation.Dubins.Shared.PathCaseValues, as: PCV

  @line_flag 1
  @orbit_flag 2
  defstruct [
    PCV.type(),
    PCV.flag(),
    PCV.case_index(),
    PCV.groundspeed_mps(),
    PCV.straight_begin_pos_rrm(),
    PCV.orbit_center_rrm(),
    PCV.orbit_radius_m(),
    PCV.turn_direction(),
    PCV.plane_end_vector(),
    PCV.plane_end_pos_rrm()
  ]

  @spec new_line(integer(), atom()) :: struct()
  def new_line(case_index, type) do
    %ViaNavigation.Dubins.PathCase{
      PCV.flag() => @line_flag,
      PCV.case_index() => case_index,
      PCV.type() => type
    }
  end

  @spec new_orbit(integer(), atom()) :: struct()
  def new_orbit(case_index, type) do
    %ViaNavigation.Dubins.PathCase{
      PCV.flag() => @orbit_flag,
      PCV.case_index() => case_index,
      PCV.type() => type
    }
  end

  @spec line_flag() :: integer()
  def line_flag() do
    @line_flag
  end

  @spec orbit_flag() :: integer()
  def orbit_flag() do
    @orbit_flag
  end
end
