# VoidEngine Makefile
# Built with Odin + SDL2

ODIN := odin
OUT_DIR := .

.PHONY: all build shmup demo puzzle void3d templar3d shared check test clean run run-demo run-puzzle run-void3d run-void3d-vk run-templar3d vk-shaders

all: build

build: shmup demo puzzle void3d templar3d

shmup:
	$(ODIN) build examples/shmup -out:$(OUT_DIR)/shmup -debug

demo:
	$(ODIN) build examples/demo -out:$(OUT_DIR)/demo -debug

puzzle:
	$(ODIN) build examples/puzzle -out:$(OUT_DIR)/puzzle -debug

void3d: vk-shaders
	$(ODIN) build examples/void3d -out:$(OUT_DIR)/void3d -debug

templar3d:
	$(ODIN) build examples/templar3d -out:$(OUT_DIR)/templar3d -debug

vk-shaders:
	@mkdir -p examples/void3d/assets/shaders
	@cd src/core/shaders && for f in vk_*.vert vk_*.frag; do \
		stage=vert; [ "$${f##*.}" = "frag" ] && stage=frag; \
		glslc -fshader-stage=$$stage $$f -o ../../../examples/void3d/assets/shaders/$$f.spv || exit 1; \
	done

shared:
	$(ODIN) build src/core -build-mode:shared -out:$(OUT_DIR)/voidengine.so -debug

check:
	$(ODIN) check src/core -no-entry-point
	$(ODIN) check examples/shmup -no-entry-point
	$(ODIN) check examples/demo -no-entry-point
	$(ODIN) check examples/puzzle -no-entry-point
	$(ODIN) check examples/void3d -no-entry-point
	$(ODIN) check examples/templar3d -no-entry-point

test:
	@echo "Running engine tests..."
	$(ODIN) test tests/ -out:$(OUT_DIR)/voidengine-test

clean:
	rm -f shmup demo puzzle void3d templar3d voidengine.so voidengine-test

run: shmup
	./shmup

run-demo: demo
	./demo

run-puzzle: puzzle
	./puzzle

run-void3d: void3d
	./void3d --backend gl

run-templar3d: templar3d
	./templar3d

run-void3d-vk: void3d
	./void3d --backend vk

bench-void3d: void3d
	./void3d --backend gl --bench 600
	./void3d --backend vk --bench 600
