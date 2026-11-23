extends SceneTree
## Bakes what the Void demo cannot recompute bit-exactly from Templar Valley's
## Godot sources (branch m1/terrain-pass at 1356930): the greybox prop
## placements of spike/tools/gen_cells.gd, which depend on Godot's
## RandomNumberGenerator and hash(Vector2i). Also prints golden samples of the
## Godot noise and height function for the engine's noise_fnl tests.
##
## Run in an empty project (needs only this file):
##   godot --headless --path <dir> -s bake_valley.gd -- <out.csv>
## The CSV has one prop per line: kind,x,y,z,sx,sy,sz (metres, Godot axes:
## y up, -z north). kind: b = building (box sx*sy*sz, roof slab on top),
## t = tree (height sy; 0.35 m trunk, 3.5 m canopy), c = clutter crate (1 m).

const CELL: float = 96.0
const HALF_EXTENT: float = 1024.0

static var _noise: FastNoiseLite

static func height_at(x: float, z: float) -> float:
	if _noise == null:
		_noise = FastNoiseLite.new()
		_noise.seed = 1165
		_noise.frequency = 0.004
		_noise.fractal_type = FastNoiseLite.FRACTAL_FBM
		_noise.fractal_octaves = 4
	return 8.0 + 150.0 * pow(absf(x) / HALF_EXTENT, 1.5) + _noise.get_noise_2d(x, z) * 18.0

func _ground(origin: Vector3, rng: RandomNumberGenerator) -> Vector3:
	var x: float = origin.x + rng.randf_range(2, CELL - 2)
	var z: float = origin.z + rng.randf_range(2, CELL - 2)
	return Vector3(x, height_at(x, z), z)

func _init() -> void:
	var args: PackedStringArray = OS.get_cmdline_user_args()
	var out: String = args[0] if args.size() > 0 else "valley_props.csv"
	var f: FileAccess = FileAccess.open(out, FileAccess.WRITE)
	f.store_line("kind,x,y,z,sx,sy,sz")
	var half: int = ceili(HALF_EXTENT / CELL)
	var n: int = 0
	for cx: int in range(-half, half):
		for cz: int in range(-half, half):
			var coord: Vector2i = Vector2i(cx, cz)
			var origin: Vector3 = Vector3(coord.x * CELL, 0.0, coord.y * CELL)
			var rng: RandomNumberGenerator = RandomNumberGenerator.new()
			rng.seed = hash(coord)
			var center: Vector3 = origin + Vector3(CELL * 0.5, 0, CELL * 0.5)
			var village: bool = Vector2(center.x, center.z).length() < 320.0
			var ridge: bool = absf(center.x) > 600.0
			var buildings: int = 22 if village else (0 if ridge else rng.randi_range(0, 3))
			var trees: int = 20 if village else (70 if ridge else 45)
			var clutter: int = 60 if village else 12
			for i: int in range(buildings):
				var size: Vector3 = Vector3(rng.randf_range(5, 10), rng.randf_range(4, 7), rng.randf_range(5, 9))
				var p: Vector3 = _ground(origin, rng)
				f.store_line("b,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f" % [p.x, p.y, p.z, size.x, size.y, size.z])
				n += 1
			for i: int in range(trees):
				var p: Vector3 = _ground(origin, rng)
				var h: float = rng.randf_range(6, 12)
				f.store_line("t,%.3f,%.3f,%.3f,3.5,%.3f,3.5" % [p.x, p.y, p.z, h])
				n += 1
			for i: int in range(clutter):
				var p: Vector3 = _ground(origin, rng)
				f.store_line("c,%.3f,%.3f,%.3f,1,1,1" % [p.x, p.y, p.z])
				n += 1
	f.close()
	print("props ", n, " -> ", out)
	# golden samples for tests/test_noise_fnl.odin
	var hn: FastNoiseLite = FastNoiseLite.new()
	hn.seed = 1165; hn.frequency = 0.004; hn.fractal_octaves = 4
	var en: FastNoiseLite = FastNoiseLite.new()
	en.seed = 2207; en.frequency = 0.01; en.fractal_type = FastNoiseLite.FRACTAL_NONE
	var pn: FastNoiseLite = FastNoiseLite.new()
	pn.seed = 3313; pn.frequency = 0.006; pn.fractal_type = FastNoiseLite.FRACTAL_NONE
	var pts: Array[Vector2] = [Vector2(0, 0), Vector2(120, -562), Vector2(-731.25, 404.5), Vector2(1023, -1023), Vector2(13.7, 999.1), Vector2(-0.3, -0.7), Vector2(555.5, -77.25), Vector2(-1024, 1024)]
	for p: Vector2 in pts:
		print("GOLD %.4f %.4f h=%.9f n=%.9f e=%.9f p=%.9f" % [p.x, p.y, height_at(p.x, p.y), hn.get_noise_2d(p.x, p.y), en.get_noise_1d(p.y), pn.get_noise_2d(p.x, p.y)])
	quit(0)
