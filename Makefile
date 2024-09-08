game: *
	cmake --build build --target game

configure: *
	cmake -B build -S . -G Ninja
	cp ./build/compile_commands.json .
