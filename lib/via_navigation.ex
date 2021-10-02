defmodule ViaNavigation do

  @spec calculate_route(struct(), binary()) :: struct()
  def calculate_route(mission, path_type) do
    path_module = Module.concat(__MODULE__, String.to_atom(path_type))
    apply(path_module, :calculate_route, [mission])
  end
end
