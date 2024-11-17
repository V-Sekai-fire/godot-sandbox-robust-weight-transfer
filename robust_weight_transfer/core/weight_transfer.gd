extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")


func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	vmcall("run_tests")
	var array: Array = []
	vmcall("find_matches_closest_surface_mesh", QuadMesh.new(), BoxMesh.new(), array)


func _print(line) -> void:
	print(line)
