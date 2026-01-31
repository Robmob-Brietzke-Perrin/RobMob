.PHONY: all build run-sim format clean doc help


all: help

run-sim: build
	@echo "launching project simulation..."
	source install/setup.bash
	ros2 launch maquette_bringup gz_house.launch.py

build:
	@echo "building project..."
	colcon build --symlink-install

format:
	@echo "Formatting C++ (google like clang-format)..."
	find src -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i -style=file
	@echo "Formatting Python (black)..."
	black src

clean:
	@echo "Cleaning up..."
	rm -rf build/ install/ log/
	rm -rf doc/html doc/latex

doc:
	@echo "Generating Doxygen documentation..."
	doxygen Doxyfile
	@echo "Done! Open doc/html/index.html in a browser to view."

help:
	@echo "Usage:"
	@echo "  make build         : Build the project"
	@echo "  make run-sim       : Build and run the simulation"
	@echo "  make format        : Auto-format C++ files using clang-format"
	@echo "  make clean         : Remove all build artifacts"
