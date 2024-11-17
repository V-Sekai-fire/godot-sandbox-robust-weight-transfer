extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")


func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	vmcall("run_tests")
	var quad_mesh_source = QuadMesh.new()
	print(quad_mesh_source.get_surface_count())
	var quad_mesh_target = QuadMesh.new()
	print(quad_mesh_target.get_surface_count())
	var result = vmcall("find_matches_closest_surface_mesh", quad_mesh_source, quad_mesh_target)
	print(result)
	
func _print(line) -> void:
	print(line)
