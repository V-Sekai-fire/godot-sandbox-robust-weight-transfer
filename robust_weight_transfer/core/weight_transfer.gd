extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")
func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	vmcall("run_tests")
	var quad_mesh_source: QuadMesh = QuadMesh.new()
	var quad_mesh_target: BoxMesh = BoxMesh.new()
	var source_arrays: Array = quad_mesh_source.surface_get_arrays(0)
	var target_arrays: Array = quad_mesh_target.surface_get_arrays(0)
	
	var matched: Array = []
	var target_weights: Array = []
	vmcall("find_matches_closest_surface_mesh", source_arrays, target_arrays, 0, matched, target_weights)
	print(matched)
	print(target_weights)

func _print(line) -> void:
	print(line)
