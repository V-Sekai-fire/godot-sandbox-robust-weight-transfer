extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")


func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	vmcall("run_tests")
	#var quad_mesh_source = QuadMesh.new()
	#var quad_mesh_target = QuadMesh.new()
	#for surface_index in range(quad_mesh_source.get_surface_count()):
		#var source_array = quad_mesh_source.surface_get_arrays(surface_index)
		#var target_array = quad_mesh_target.surface_get_arrays(surface_index)
		#var result = vmcall("find_matches_closest_surface_mesh", source_array, target_array)
		#print(result)
	
func _print(line) -> void:
	print(line)
