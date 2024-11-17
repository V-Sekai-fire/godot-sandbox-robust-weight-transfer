extends Sandbox

const program: ELFScript = preload("../robust_weight_transfer.elf")


func _ready() -> void:
	set_program(program)
	set_redirect_stdout(_print)
	vmcall("run_tests")
	
func _print(line) -> void:
	print(line)
