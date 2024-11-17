extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")
func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	vmcall("run_tests")
	var quad_mesh_source: QuadMesh = QuadMesh.new()
	var quad_mesh_target: BoxMesh = BoxMesh.new()
	var source_arrays: Array = quad_mesh_source.surface_get_arrays(0)
	var source_vertices_array: Array = source_arrays[Mesh.ARRAY_VERTEX].slice(0)
	var source_triangles_array: Array = source_arrays[Mesh.ARRAY_INDEX].slice(0)
	var source_normals_array: Array = source_arrays[Mesh.ARRAY_NORMAL].slice(0)
	var target_arrays: Array = quad_mesh_target.surface_get_arrays(0)
	var target_vertices_array: Array = target_arrays[Mesh.ARRAY_VERTEX].slice(0)
	var target_triangles_array: Array = target_arrays[Mesh.ARRAY_INDEX].slice(0)
	var target_normals_array: Array = target_arrays[Mesh.ARRAY_NORMAL].slice(0)
	
	print("Source vertices array size: ", source_vertices_array.size())
	print("Source triangles array size: ", source_triangles_array.size())
	print("Source normals array size: ", source_normals_array.size())
	print("Target vertices array size: ", target_vertices_array.size())
	print("Target triangles array size: ", target_triangles_array.size())
	print("Target normals array size: ", target_normals_array.size())
	
	var matched: Array = []
	var target_weights: Array = []
	vmcall("find_matches_closest_surface_mesh", source_vertices_array, source_triangles_array, source_normals_array, target_vertices_array, target_triangles_array, target_normals_array, 0, matched, target_weights)
	print(matched)
	print(target_weights)

func _print(line) -> void:
	print(line)
