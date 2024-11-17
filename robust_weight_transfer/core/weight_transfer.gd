extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")


func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	vmcall("run_tests")
	var array: Array = []
	var ok = vmcall("find_matches_closest_surface_mesh", SphereMesh.new(), SphereMesh.new(), array)
	print(ok)
	print(array)

func _print(line) -> void:
	print(line)
